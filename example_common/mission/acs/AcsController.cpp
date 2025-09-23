#include "AcsController.h"

#include <cmath>
#include <cstring>

#include "fsfw/returnvalues/returnvalue.h"
#include "fsfw/serviceinterface/ServiceInterfaceStream.h"

AcsController::AcsController(object_id_t oid, std::array<object_id_t, 4> rwOids)
    : SystemObject(oid), rwOids_(rwOids) {
  lastRpmCmd_.fill(0);
}

ReturnValue_t AcsController::initialize() {
  // Resolve IPC store
  ipcStore_ = ObjectManager::instance()->get<StorageManagerIF>(objects::IPC_STORE);
  if (ipcStore_ == nullptr) {
    sif::error << "AcsController: IPC_STORE not available" << std::endl;
    return returnvalue::FAILED;
  }

  // Resolve RW command queues
  for (size_t i = 0; i < rwOids_.size(); ++i) {
    if (rwOids_[i] == objects::NO_OBJECT) {
      rwMq_[i] = MessageQueueIF::NO_QUEUE;
      continue;
    }
    auto* dh = ObjectManager::instance()->get<DeviceHandlerIF>(rwOids_[i]);
    if (dh == nullptr) {
      sif::warning << "AcsController: RW oid 0x" << std::hex << rwOids_[i] << std::dec
                   << " not found" << std::endl;
      rwMq_[i] = MessageQueueIF::NO_QUEUE;
    } else {
      rwMq_[i] = dh->getCommandQueue();
    }
  }

  // Initialize timing
  (void)Clock::getUptime(&lastUptimeMs_);
  return returnvalue::OK;
}

ReturnValue_t AcsController::performOperation(uint8_t) {
  // --- compute dt ---
  uint32_t nowMs = 0;
  (void)Clock::getUptime(&nowMs);
  float dt = (nowMs >= lastUptimeMs_) ? (nowMs - lastUptimeMs_) * 1e-3f : loopPeriodHintSec_;
  if (dt <= 0.0f || dt > 1.0f) {
    dt = loopPeriodHintSec_;
  }
  lastUptimeMs_ = nowMs;

  // --- 1) Guidance: commanded body rates (rad/s) ---
  // For now: zero rates (hold attitude). You can extend Guidance later.
  float wCmd[3] = {0.0f, 0.0f, 0.0f};
  guidance_.update(dt);
  guidance_.getRateCmd(wCmd);

  // --- 2) Estimator: measured body rates (rad/s) ---
  float w[3] = {0.0f, 0.0f, 0.0f};
  estimator_.update(dt);
  estimator_.getBodyRate(w);

  // --- 3) Controller: D-only rate control tau_des = -Kd (w - wCmd) ---
  float tauDes[3];
  for (int i = 0; i < 3; ++i) {
    float err = (w[i] - wCmd[i]);
    float tau = -kd_[i] * err;
    tauDes[i] = clamp(tau, -tauMax_[i], +tauMax_[i]);
  }

  // --- 4) Allocation: distribute to wheel torques ---
  float tauWheel[4] = {0, 0, 0, 0};
  allocator_.solve(tauDes, tauWheel);

  // --- 5) Torque -> RPM update (speed interface fallback) ---
  for (size_t i = 0; i < rwOids_.size(); ++i) {
    if (rwMq_[i] == MessageQueueIF::NO_QUEUE) {
      continue;
    }
    // delta Omega = tau / J * dt ; convert to RPM and apply slew limit
    float dOmega_rps = (tauWheel[i] / jWheel_[i]) * dt;  // rad/s
    float dRpm = radpsToRpm(dOmega_rps);
    float target = static_cast<float>(lastRpmCmd_[i]) + dRpm;

    // slew limit
    float maxStep = maxSlewRpmPerS_ * dt;
    float step = clamp(target - lastRpmCmd_[i], -maxStep, +maxStep);
    int newRpm = static_cast<int>(std::lround(lastRpmCmd_[i] + step));

    // clamp absolute RPM
    newRpm = clamp(newRpm, -maxRpm_, +maxRpm_);

    // send only if changed
    if (newRpm != lastRpmCmd_[i]) {
      lastRpmCmd_[i] = static_cast<int16_t>(newRpm);
      sendRpmToWheel(i, lastRpmCmd_[i]);
    }
  }

  return returnvalue::OK;
}

void AcsController::sendRpmToWheel(size_t idx, int16_t rpm) {
  if (ipcStore_ == nullptr || rwMq_[idx] == MessageQueueIF::NO_QUEUE) {
    return;
  }
  store_address_t sid{};
  uint8_t* p = nullptr;
  if (ipcStore_->getFreeElement(&sid, sizeof(rpm), &p) != returnvalue::OK) {
    return;
  }
  std::memcpy(p, &rpm, sizeof(rpm));

  // CMD_SET_SPEED = 0x01 in your ReactionWheelsHandler
  CommandMessage msg;
  ActionMessage::setCommand(&msg, 0x01, sid);
  MessageQueueSenderIF::sendMessage(rwMq_[idx], &msg, MessageQueueMessage::MAX_MESSAGE_SIZE);
}
