#include "AcsController.h"

#include <cmath>
#include <limits>

// FSFW datapool
#include "fsfw/datapoollocal/LocalPoolVariable.h"
// Objects IDs (contains IPC_STORE)
#include "commonObjects.h"
// Device handler messaging
#include "fsfw/devicehandlers/DeviceHandlerMessage.h"
#include "fsfw/devicehandlers/ReactionWheelsHandler.h"
#include "fsfw/devicehandlers/RwProtocol.h"
#include "fsfw/ipc/CommandMessage.h"
#include "fsfw/ipc/MessageQueueSenderIF.h"
#include "fsfw/serviceinterface/ServiceInterfaceStream.h"
#include "fsfw/action/ActionMessage.h"

namespace {
constexpr float PI_F = 3.14159265358979323846f;

// Merker der zuletzt gesendeten int16-Torques je Rad (für Anti-Spam/Deadband)
int16_t gLastSentInt16[4] = {0, 0, 0, 0};
}

// ========================= DEBUG / FORCE TORQUE SWITCHES =========================
#ifndef ACS_FORCE_TORQUE_ENABLE
#define ACS_FORCE_TORQUE_ENABLE 0
#endif
#ifndef ACS_FORCE_TORQUE_MNM
#define ACS_FORCE_TORQUE_MNM 0  // mNm
#endif
#ifndef ACS_FORCE_TORQUE_WHEEL_MASK
#define ACS_FORCE_TORQUE_WHEEL_MASK 0x1 // Bit0..Bit3 für RW0..RW3
#endif
// ================================================================================

// ==== Verkehrsreduktion / Hygiene ====
#ifndef ACS_MIN_CMD_ABS_MNM
#define ACS_MIN_CMD_ABS_MNM 0.05f
#endif

AcsController::AcsController(object_id_t objectId, std::array<object_id_t, 4> wheelIds,
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

  myQueue_ = QueueFactory::instance()->createMessageQueue(3);
  ipcStore_ = ObjectManager::instance()->get<StorageManagerIF>(objects::IPC_STORE);
  if (ipcStore_ == nullptr) {
    sif::error << "ACS: IPC store not available!" << std::endl;
  }
}

ReturnValue_t AcsController::performOperation(uint8_t) {
  const float dt = cfg_.timing.dt();

  // --- Kante erkennen: enabled (an/aus) ---
  const bool enabledNow = enabled_.load();
  static bool prevEnabled = false;

  if (!enabledNow) {
    // Beim Übergang von an -> aus: einmalig STOP an alle Räder + Outputs nullen
    if (prevEnabled) {
      for (int i = 0; i < 4; ++i) {
        const object_id_t wid = rwIds_[i];
        if (wid == 0) continue;
        auto* dh = ObjectManager::instance()->get<DeviceHandlerIF>(wid);
        if (dh == nullptr) continue;

        CommandMessage stopCmd;
        ActionMessage::setCommand(&stopCmd,
          static_cast<ActionId_t>(ReactionWheelsHandler::CMD_STOP),
          store_address_t{});
        (void)MessageQueueSenderIF::sendMessage(dh->getCommandQueue(), &stopCmd, myQueue_->getId());
      }
      // Diagnose-/Output-Werte zurücksetzen, damit nächste TM konsistent ist
      tauDes_ = {0.f, 0.f, 0.f};
      tauWheelCmd_ = {0.f, 0.f, 0.f, 0.f};
      for (auto& v : gLastSentInt16) v = 0;
    }
    prevEnabled = false;
    return returnvalue::OK;  // absolut nichts rechnen/senden, wenn aus!
  }

  // Ab hier: Controller ist an
  prevEnabled = true;

  // 1) Read measured torques from RW handler(s)
  std::array<float, 4> tauW_meas = {0, 0, 0, 0};
  for (int i = 0; i < 4; ++i) {
    const object_id_t wid = rwIds_[i];
    if (wid == 0) {
      tauW_meas[i] = 0.0f;
      continue;
    }
    using PoolIds = ReactionWheelsHandler::PoolIds;
    LocalPoolVariable<int16_t> torqueVar(wid, static_cast<lp_id_t>(PoolIds::HK_TORQUE_mNm));
    if (torqueVar.read() == returnvalue::OK && torqueVar.isValid()) {
      tauW_meas[i] = static_cast<float>(torqueVar.value);  // [mNm]
    } else {
      tauW_meas[i] = tauWheelCmd_[i];
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
  float tauDes_pre[3] = {
      Kp_[0] * e[0] + Kd_[0] * (wRef[0] - wMeas[0]),
      Kp_[1] * e[1] + Kd_[1] * (wRef[1] - wMeas[1]),
      Kp_[2] * e[2] + Kd_[2] * (wRef[2] - wMeas[2])
  };
  float tauDes[3] = {tauDes_pre[0], tauDes_pre[1], tauDes_pre[2]};
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

#if ACS_FORCE_TORQUE_ENABLE
  {
    std::array<float, 4> forced{0.f, 0.f, 0.f, 0.f};
    const float F = static_cast<float>(ACS_FORCE_TORQUE_MNM);
    if (ACS_FORCE_TORQUE_WHEEL_MASK & 0x1) forced[0] = F;
    if (ACS_FORCE_TORQUE_WHEEL_MASK & 0x2) forced[1] = F;
    if (ACS_FORCE_TORQUE_WHEEL_MASK & 0x4) forced[2] = F;
    if (ACS_FORCE_TORQUE_WHEEL_MASK & 0x8) forced[3] = F;
    for (int i = 0; i < 4; ++i) {
      if (forced[i] > cfg_.limits.torqueMax_mNm) forced[i] = cfg_.limits.torqueMax_mNm;
      if (forced[i] < -cfg_.limits.torqueMax_mNm) forced[i] = -cfg_.limits.torqueMax_mNm;
    }
    tauWheelCmd_ = forced;
  }
#endif

  // 7) Senden (mit Deadband/Zero-Spam)
  sendWheelTorques_(tauWheelCmd_);
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

  // Bei disabled immer 0 reporten, um TM konsistent zu halten.
  if (!s.enabled) {
    s.tauDes = {0.f, 0.f, 0.f};
    s.tauWheelCmd = {0.f, 0.f, 0.f, 0.f};
  } else {
    s.tauDes = tauDes_;
    s.tauWheelCmd = tauWheelCmd_;
  }
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
    sif::warning << "ACS: sendWheelTorques_: no ipcStore or myQueue" << std::endl;
    return;
  }

  for (int i = 0; i < 4; i++) {
    const object_id_t wid = rwIds_[i];
    if (wid == 0) continue;

    auto* dh = ObjectManager::instance()->get<DeviceHandlerIF>(wid);
    if (dh == nullptr) {
      sif::warning << "ACS: RW device 0x" << std::hex << wid << std::dec << " not found." << std::endl;
      continue;
    }

    // Clamp + Umrechnung float mNm -> int16 mNm
    float tq = tauWheel_mNm[i];
    const float LIM = cfg_.limits.torqueMax_mNm;
    if (tq >  LIM) tq =  LIM;
    if (tq < -LIM) tq = -LIM;

    // Deadband: unterhalb Schwellwert, nur einmalig 0 senden (wenn vorher != 0)
    if (std::fabs(tq) < ACS_MIN_CMD_ABS_MNM) {
      if (gLastSentInt16[i] != 0) {
        store_address_t storeId{};
        const uint8_t payloadZero[2] = {0, 0};
        if (ipcStore_->addData(&storeId, payloadZero, sizeof(payloadZero)) == returnvalue::OK) {
          CommandMessage cmd;
          ActionMessage::setCommand(&cmd,
            static_cast<ActionId_t>(ReactionWheelsHandler::CMD_SET_TORQUE),
            /* FIX: richtig ist store_address_t */ storeId);
          (void)MessageQueueSenderIF::sendMessage(dh->getCommandQueue(), &cmd, myQueue_->getId());
          gLastSentInt16[i] = 0;
        }
      }
      continue;
    }

    int32_t rounded = static_cast<int32_t>(std::lround(tq));
    if (rounded > std::numeric_limits<int16_t>::max()) rounded = std::numeric_limits<int16_t>::max();
    if (rounded < std::numeric_limits<int16_t>::min()) rounded = std::numeric_limits<int16_t>::min();
    const int16_t torque_mNm = static_cast<int16_t>(rounded);

    if (torque_mNm == gLastSentInt16[i]) {
      continue; // nichts Neues
    }

    store_address_t storeId{};
    const uint8_t payload[2] = {
      static_cast<uint8_t>((torque_mNm >> 8) & 0xFF),
      static_cast<uint8_t>( torque_mNm       & 0xFF)
    };
    if (ipcStore_->addData(&storeId, payload, sizeof(payload)) != returnvalue::OK) {
      continue;
    }

    CommandMessage cmd;
    ActionMessage::setCommand(&cmd,
        static_cast<ActionId_t>(ReactionWheelsHandler::CMD_SET_TORQUE),
        storeId);

    const auto mq = dh->getCommandQueue();
    if (MessageQueueSenderIF::sendMessage(mq, &cmd, myQueue_->getId()) == returnvalue::OK) {
      gLastSentInt16[i] = torque_mNm;
    }
  }
}
