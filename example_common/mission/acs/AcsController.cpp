#include "AcsController.h"
#include <cmath>
#include <limits>

// FSFW datapool
#include "fsfw/datapoollocal/LocalPoolVariable.h"
// Objects IDs (contains IPC_STORE)
#include "commonObjects.h"
// Our RW handler header for command IDs + Pool IDs
#include "fsfw/devicehandlers/ReactionWheelsHandler.h"
#include "fsfw/ipc/CommandMessage.h"
#include "fsfw/devicehandlers/DeviceHandlerMessage.h"
#include "fsfw/devicehandlers/RwProtocol.h"

namespace { constexpr float PI_F = 3.14159265358979323846f; }

AcsController::AcsController(object_id_t objectId,
                             std::array<object_id_t, 4> wheelIds,
                             const acs::Config& cfg)
    : SystemObject(objectId),
      cfg_(cfg),
      rwIds_(wheelIds),
      estimator_(cfg),
      guidance_(),
      allocator_(cfg) {
  estimator_.setAxes(cfg_.Bcols);
  allocator_.setAxes(cfg_.Bcols);
  dtMs_ = cfg_.timing.dtMs;

  // Create own small message queue (depth 3)
  myQueue_ = QueueFactory::instance()->createMessageQueue(3);
  // Get IPC store to pass payloads to device handlers
  ipcStore_ = ObjectManager::instance()->get<StorageManagerIF>(objects::IPC_STORE);
  if (ipcStore_ == nullptr) {
    sif::error << "ACS: IPC store not available!" << std::endl;
  }
}

ReturnValue_t AcsController::performOperation(uint8_t) {
  const float dt = cfg_.timing.dt();

  // 1) Read measured torques from RW handler(s)
  std::array<float, 4> tauW_meas = {0, 0, 0, 0};
  for (int i = 0; i < 4; ++i) {
    const object_id_t wid = rwIds_[i];
    if (wid == 0) {
      tauW_meas[i] = 0.0f;
      continue;
    }

    using PoolIds = ReactionWheelsHandler::PoolIds;
    // Read HK torque as int16_t (mNm), remote read-only (no commit)
    LocalPoolVariable<int16_t> torqueVar(wid, static_cast<lp_id_t>(PoolIds::HK_TORQUE_mNm));
    if (torqueVar.read() == returnvalue::OK && torqueVar.isValid()) {
      tauW_meas[i] = static_cast<float>(torqueVar.value);  // [mNm]
    } else {
      // Fallback until HK valid: use last commanded
      tauW_meas[i] = tauWheelCmd_[i];
#ifdef ACS_VERBOSE
      sif::warning << "ACS: RW" << i
                   << " HK torque invalid -> fallback to commanded." << std::endl;
#endif
    }
  }

  // 2) Estimator
  estimator_.step(tauW_meas);

  const float* wMeas = estimator_.omegaMeas();
  const float* qTrue = estimator_.quatTrue();

  // 3) Guidance
  guidance_.update(dt);
  float wRef[3] = {0.0f, 0.0f, 0.0f};

  // 4) Quaternion error
  auto qRef = guidance_.getTargetQuat();
  float qc[4];
  quatConj_(qTrue, qc);
  float qerr[4];
  quatMul_(qRef.data(), qc, qerr);
  if (qerr[0] < 0.0f) {
    for (int i = 0; i < 4; i++) qerr[i] = -qerr[i];
  }
  qErr_ = {qerr[0], qerr[1], qerr[2], qerr[3]};

  // 5) PD control
  float e[3] = {2.0f * qerr[1], 2.0f * qerr[2], 2.0f * qerr[3]};
  float tauDes[3] = {Kp_[0] * e[0] + Kd_[0] * (wRef[0] - wMeas[0]),
                     Kp_[1] * e[1] + Kd_[1] * (wRef[1] - wMeas[1]),
                     Kp_[2] * e[2] + Kd_[2] * (wRef[2] - wMeas[2])};
  float prev[3] = {tauDes_[0], tauDes_[1], tauDes_[2]};
  rateLimit3_(tauDes, prev, cfg_.limits.torqueRateLimit_mNm_per_s, dt);
  clamp3_(tauDes, cfg_.limits.torqueMax_mNm);
  tauDes_ = {tauDes[0], tauDes[1], tauDes[2]};

  // 6) Allocation
  float tauW[4];
  allocator_.solve(tauDes, tauW);
  for (int i = 0; i < 4; i++) {
    if (tauW[i] > cfg_.limits.torqueMax_mNm) tauW[i] = cfg_.limits.torqueMax_mNm;
    if (tauW[i] < -cfg_.limits.torqueMax_mNm) tauW[i] = -cfg_.limits.torqueMax_mNm;
  }
  tauWheelCmd_ = {tauW[0], tauW[1], tauW[2], tauW[3]};

  // 7) Send to RW(s) if enabled
  if (enabled_.load()) {
    sendWheelTorques_(tauWheelCmd_);
  }

  return returnvalue::OK;
}

AcsDiagSnapshot AcsController::getDiag() const {
  AcsDiagSnapshot s;
  s.version = 1;
  s.enabled = enabled_.load();
  s.qRef = guidance_.getTargetQuat();
  const float* qt = estimator_.quatTrue();
  const float* wt = estimator_.omegaTrue();
  const float* wm = estimator_.omegaMeas();
  s.qTrue = {qt[0], qt[1], qt[2], qt[3]};
  s.wTrue = {wt[0], wt[1], wt[2]};
  s.wMeas = {wm[0], wm[1], wm[2]};
  s.Kp = Kp_;
  s.Kd = Kd_;
  s.tauDes = tauDes_;
  s.tauWheelCmd = tauWheelCmd_;
  s.dtMs = dtMs_;
  return s;
}

// --- helpers ---

void AcsController::quatConj_(const float q[4], float qc[4]) {
  qc[0] = q[0];
  qc[1] = -q[1];
  qc[2] = -q[2];
  qc[3] = -q[3];
}

void AcsController::quatMul_(const float a[4], const float b[4], float out[4]) {
  out[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  out[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  out[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  out[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

void AcsController::clamp3_(float v[3], float maxAbs) {
  for (int i = 0; i < 3; i++) {
    if (v[i] > maxAbs) v[i] = maxAbs;
    if (v[i] < -maxAbs) v[i] = -maxAbs;
  }
}

void AcsController::rateLimit3_(float v[3], const float vPrev[3], float rateLimitPerSec, float dt) {
  for (int i = 0; i < 3; i++) {
    float dv = v[i] - vPrev[i];
    float maxDv = rateLimitPerSec * dt;
    if (dv > maxDv) v[i] = vPrev[i] + maxDv;
    if (dv < -maxDv) v[i] = vPrev[i] - maxDv;
  }
}

void AcsController::sendWheelTorques_(const std::array<float, 4>& tauWheel_mNm) {
  if (ipcStore_ == nullptr || myQueue_ == nullptr) {
    return;
  }

  for (int i = 0; i < 4; i++) {
    const object_id_t wid = rwIds_[i];
    if (wid == 0) {
      continue;
    }

    // Resolve device handler and its command queue
    auto* dh = ObjectManager::instance()->get<DeviceHandlerIF>(wid);
    if (dh == nullptr) {
#ifdef ACS_VERBOSE
      sif::warning << "ACS: RW device 0x" << std::hex << wid << std::dec << " not found."
                   << std::endl;
#endif
      continue;
    }

    // Per-wheel safety clamp (in mNm) before converting to int16
    float tq = tauWheel_mNm[i];
    const float LIM = cfg_.limits.torqueMax_mNm;
    if (tq >  LIM) tq =  LIM;
    if (tq < -LIM) tq = -LIM;

    // Convert float mNm -> int16 mNm (rounded) and guard against overflow
    int32_t rounded = static_cast<int32_t>(std::lround(tq));
    if (rounded > std::numeric_limits<int16_t>::max()) rounded = std::numeric_limits<int16_t>::max();
    if (rounded < std::numeric_limits<int16_t>::min()) rounded = std::numeric_limits<int16_t>::min();
    const int16_t torque_mNm = static_cast<int16_t>(rounded);

    // Build wire frame for this wheel (RAW command)
    uint8_t frame[RwProtocol::CMD_LEN] = {};
    const size_t total = RwProtocol::buildSetTorque(frame, sizeof(frame), torque_mNm);
    if (total == 0) {
#ifdef ACS_VERBOSE
      sif::warning << "ACS: RwProtocol::buildSetTorque failed for idx " << i << std::endl;
#endif
      continue;
    }

    // Put frame into IPC store
    store_address_t storeId{};
    const ReturnValue_t res = ipcStore_->addData(&storeId, frame, total);
    if (res != returnvalue::OK) {
#ifdef ACS_VERBOSE
      sif::warning << "ACS: IPC store addData failed, skipping RAW torque cmd for RW idx "
                   << i << std::endl;
#endif
      continue;
    }

    // Send RAW command to device handler
    CommandMessage cmd;
    DeviceHandlerMessage::setDeviceHandlerRawCommandMessage(&cmd, storeId);
    MessageQueueSenderIF::sendMessage(dh->getCommandQueue(), &cmd, myQueue_->getId());
  }
}
