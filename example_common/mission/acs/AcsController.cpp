#include "AcsController.h"

#include <cmath>
#include <limits>
#include <iomanip>
#include <algorithm>

// FSFW datapool
#include "fsfw/datapoollocal/LocalPoolVariable.h"
// Objects IDs (contains IPC_STORE)
#include "commonObjects.h"
// Device handler messaging
#include "fsfw/devicehandlers/DeviceHandlerIF.h"
#include "fsfw/devicehandlers/ReactionWheelsHandler.h"
#include "fsfw/ipc/CommandMessage.h"
#include "fsfw/ipc/MessageQueueSenderIF.h"
#include "fsfw/action/ActionMessage.h"
#include "fsfw/serviceinterface/ServiceInterfaceStream.h"

// Print every X control loop iterations (e.g. X=25 @ 5 Hz ≈ 5 s)
#ifndef ACS_LOG_ATT_DIV
#define ACS_LOG_ATT_DIV 10
#endif

namespace {
constexpr float PI_F = 3.14159265358979323846f;

// Last-sent int16 torques per wheel (anti-spam / hysteresis)
int16_t gLastSentInt16[4] = {0, 0, 0, 0};

// ======== Controller (Variant A) defaults ========
// Outer loop: attitude P -> rate reference (rad/s)
#ifndef ACS_KATT_X
#define ACS_KATT_X 0.04f
#endif
#ifndef ACS_KATT_Y
#define ACS_KATT_Y 0.04f
#endif
#ifndef ACS_KATT_Z
#define ACS_KATT_Z 0.04f
#endif
#ifndef ACS_WREF_MAX_X
#define ACS_WREF_MAX_X 0.02f  // ~1.15 deg/s
#endif
#ifndef ACS_WREF_MAX_Y
#define ACS_WREF_MAX_Y 0.02f
#endif
#ifndef ACS_WREF_MAX_Z
#define ACS_WREF_MAX_Z 0.02f
#endif

// Inner loop: rate-PI gains (mNm/(rad/s)) and (mNm/rad)
#ifndef ACS_KPW_X
#define ACS_KPW_X 100.0f
#endif
#ifndef ACS_KPW_Y
#define ACS_KPW_Y 100.0f
#endif
#ifndef ACS_KPW_Z
#define ACS_KPW_Z 100.0f
#endif

#ifndef ACS_KIW_X
#define ACS_KIW_X 20.0f
#endif
#ifndef ACS_KIW_Y
#define ACS_KIW_Y 20.0f
#endif
#ifndef ACS_KIW_Z
#define ACS_KIW_Z 20.0f
#endif

// Anti-windup back-calculation gain (1/s): i += (Ki*e + K_aw*(tau_sat - tau_unsat)) * dt
#ifndef ACS_KAW
#define ACS_KAW 2.0f
#endif

// 1st-order low-pass on measured omega (Hz). 0 => disabled
#ifndef ACS_OMEGA_LP_FC_HZ
#define ACS_OMEGA_LP_FC_HZ 1.0f
#endif

// Keep <= per-step increment to avoid eating small commands.
// For 5 Hz and torqueRateLimit=5 mNm/s -> 1 mNm per step; 0.25–0.5 is a good start.
#ifndef ACS_MIN_CMD_ABS_MNM
#define ACS_MIN_CMD_ABS_MNM 0.25f
#endif

// Debug prints
#ifndef ACS_VERBOSE
#define ACS_VERBOSE 1
#endif

// ======== Persistent controller state (single instance) ========
// English: PI integrator state [mNm] per axis, and low-pass filtered omega [rad/s]
float gIstate[3]   = {0.f, 0.f, 0.f};
float gOmegaF[3]   = {0.f, 0.f, 0.f};
bool  gStatesInit  = false;  // (re)initialize on enable-edge

// Quaternion -> yaw(Z), pitch(Y), roll(X) [radians]
inline void quatToYpr(const float q[4], float& yaw, float& pitch, float& roll) {
  // roll (x-axis rotation)
  const float sinr_cosp = 2.0f * (q[0]*q[1] + q[2]*q[3]);
  const float cosr_cosp = 1.0f - 2.0f * (q[1]*q[1] + q[2]*q[2]);
  roll = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  const float sinp = 2.0f * (q[0]*q[2] - q[3]*q[1]);
  if (std::fabs(sinp) >= 1.0f) {
    pitch = std::copysign(PI_F / 2.0f, sinp);  // clamp to 90°
  } else {
    pitch = std::asin(sinp);
  }

  // yaw (z-axis rotation)
  const float siny_cosp = 2.0f * (q[0]*q[3] + q[1]*q[2]);
  const float cosy_cosp = 1.0f - 2.0f * (q[2]*q[2] + q[3]*q[3]);
  yaw = std::atan2(siny_cosp, cosy_cosp);
}
} // namespace

// ========================= DEBUG / FORCE TORQUE SWITCHES =========================
#ifndef ACS_FORCE_TORQUE_ENABLE
#define ACS_FORCE_TORQUE_ENABLE 0
#endif
#ifndef ACS_FORCE_TORQUE_MNM
#define ACS_FORCE_TORQUE_MNM 0  // mNm
#endif
#ifndef ACS_FORCE_TORQUE_WHEEL_MASK
#define ACS_FORCE_TORQUE_WHEEL_MASK 0x1 // Bit0..Bit3 for RW0..RW3
#endif
// ================================================================================

AcsController::AcsController(object_id_t objectId, std::array<object_id_t, 4> wheelIds,
                             const acs::Config& cfg)
    : SystemObject(objectId),
      cfg_(cfg),
      rwIds_(wheelIds),
      estimator_(cfg),
      guidance_(),
      allocator_(cfg) {
  // Ensure axes (B matrix) propagated to submodules
  estimator_.setAxes(cfg_.Bcols);
  allocator_.setAxes(cfg_.Bcols);
  dtMs_ = cfg_.timing.dtMs;

  // Map legacy Kp_/Kd_ to new parameters for TM visibility:
  // Kp_ -> attitude gains (1/s), Kd_ -> rate-P gains (mNm/(rad/s))
  Kp_ = {ACS_KATT_X, ACS_KATT_Y, ACS_KATT_Z};
  Kd_ = {ACS_KPW_X,  ACS_KPW_Y,  ACS_KPW_Z};

  myQueue_ = QueueFactory::instance()->createMessageQueue(3);
  ipcStore_ = ObjectManager::instance()->get<StorageManagerIF>(objects::IPC_STORE);
  if (ipcStore_ == nullptr) {
    sif::error << "ACS: IPC store not available!" << std::endl;
  }
}

ReturnValue_t AcsController::performOperation(uint8_t) {
  const float dt = cfg_.timing.dt();

  // --- Edge detect: enabled (on/off) ---
  const bool enabledNow = enabled_.load();
  static bool prevEnabled = false;

  if (!enabledNow) {
    // On transition from ON -> OFF: send STOP once to all wheels and clear outputs
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
      // Reset diagnostic/output values so next TM is consistent
      tauDes_ = {0.f, 0.f, 0.f};
      tauWheelCmd_ = {0.f, 0.f, 0.f, 0.f};
      for (auto& v : gLastSentInt16) v = 0;

      // Reset controller states
      gIstate[0]=gIstate[1]=gIstate[2]=0.f;
      gOmegaF[0]=gOmegaF[1]=gOmegaF[2]=0.f;
      gStatesInit = false;
    }
    prevEnabled = false;
    return returnvalue::OK;  // do absolutely nothing if disabled
  }

  // From here: controller is enabled
  // On enable-edge, initialize filter state with current measurements to avoid a transient.
  if (!prevEnabled || !gStatesInit) {
    gIstate[0]=gIstate[1]=gIstate[2]=0.f;
    gOmegaF[0]=gOmegaF[1]=gOmegaF[2]=0.f;
    gStatesInit = true;
  }
  prevEnabled = true;

  // 1) Read measured wheel torques from RW handler(s) [mNm]
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
      // Fallback: assume last commanded (best-effort if device HK not available)
      tauW_meas[i] = tauWheelCmd_[i];
    }
  }

  // 2) Estimator step (wheel torque -> body rates/attitude); no external torques here
  estimator_.step(tauW_meas, /*tauExt_mNm=*/std::array<float,3>{0.f,0.f,0.f});

  const float* wMeas = estimator_.omegaMeas(); // [rad/s]
  const float* qTrue = estimator_.quatTrue();

  // Initialize low-pass with first available measurement, once
  if (gOmegaF[0]==0.f && gOmegaF[1]==0.f && gOmegaF[2]==0.f) {
    gOmegaF[0]=wMeas[0]; gOmegaF[1]=wMeas[1]; gOmegaF[2]=wMeas[2];
  }

  // 3) Guidance (update and get reference rate if any)
  guidance_.update(dt);
  float wCmdFeed[3] = {0.0f, 0.0f, 0.0f};
  guidance_.getRateCmd(wCmdFeed); // currently zeros

  // 4) Quaternion error qerr = qRef * conj(qTrue) (Hamilton product), ensure shortest path
  auto qRef = guidance_.getTargetQuat();
  float qc[4];
  quatConj_(qTrue, qc);
  float qerr[4];
  quatMul_(qRef.data(), qc, qerr);
  if (qerr[0] < 0.0f) {
    for (int i = 0; i < 4; i++) qerr[i] = -qerr[i];
  }
  qErr_ = {qerr[0], qerr[1], qerr[2], qerr[3]};

  // --- Periodic attitude log (every ACS_LOG_ATT_DIV loops) ---
  static uint32_t attLogCtr = 0;
  if (++attLogCtr >= ACS_LOG_ATT_DIV) {
    attLogCtr = 0;

    // YPR for reference and true attitude (deg)
    float yawR, pitchR, rollR, yawT, pitchT, rollT;
    quatToYpr(qRef.data(), yawR, pitchR, rollR);
    quatToYpr(qTrue,       yawT, pitchT, rollT);
    const float rad2deg = 180.0f / PI_F;

    // Small-angle attitude error magnitude (deg), based on vector part of qerr
    const float evec[3] = {2.0f*qerr[1], 2.0f*qerr[2], 2.0f*qerr[3]};
    const float eNorm   = std::sqrt(evec[0]*evec[0] + evec[1]*evec[1] + evec[2]*evec[2]);
    const float angDeg  = 2.0f * std::asin(std::min(1.0f, 0.5f * eNorm)) * rad2deg;

    sif::info << std::fixed << std::setprecision(2)
              << "ATT ref[YPRdeg]=[" << yawR*rad2deg << "," << pitchR*rad2deg << "," << rollR*rad2deg << "] "
              << "true[YPRdeg]=["    << yawT*rad2deg << "," << pitchT*rad2deg << "," << rollT*rad2deg << "] "
              << "errAngle=" << angDeg << " deg"
              << std::endl;
  }

  // ===== Variant A: Attitude-P -> rate reference (per-axis clamp) =====
  // Small-angle equivalent vector error: e = 2 * qerr_vec (dimension ~ rad for small angles)
  const float e[3] = {2.0f * qerr[1], 2.0f * qerr[2], 2.0f * qerr[3]};

  float wRef[3] = {
      ACS_KATT_X * e[0] + wCmdFeed[0],
      ACS_KATT_Y * e[1] + wCmdFeed[1],
      ACS_KATT_Z * e[2] + wCmdFeed[2]
  };
  // Clamp rate reference
  if (wRef[0] >  ACS_WREF_MAX_X) wRef[0] =  ACS_WREF_MAX_X;
  if (wRef[0] < -ACS_WREF_MAX_X) wRef[0] = -ACS_WREF_MAX_X;
  if (wRef[1] >  ACS_WREF_MAX_Y) wRef[1] =  ACS_WREF_MAX_Y;
  if (wRef[1] < -ACS_WREF_MAX_Y) wRef[1] = -ACS_WREF_MAX_Y;
  if (wRef[2] >  ACS_WREF_MAX_Z) wRef[2] =  ACS_WREF_MAX_Z;
  if (wRef[2] < -ACS_WREF_MAX_Z) wRef[2] = -ACS_WREF_MAX_Z;

  // ===== Low-pass filter on measured omega =====
  if (ACS_OMEGA_LP_FC_HZ > 0.f) {
    const float alpha = 1.0f - std::exp(-2.0f * PI_F * ACS_OMEGA_LP_FC_HZ * dt);
    gOmegaF[0] = gOmegaF[0] + alpha * (wMeas[0] - gOmegaF[0]);
    gOmegaF[1] = gOmegaF[1] + alpha * (wMeas[1] - gOmegaF[1]);
    gOmegaF[2] = gOmegaF[2] + alpha * (wMeas[2] - gOmegaF[2]);
  } else {
    gOmegaF[0] = wMeas[0]; gOmegaF[1] = wMeas[1]; gOmegaF[2] = wMeas[2];
  }

  // Rate error
  const float er[3] = { wRef[0] - gOmegaF[0], wRef[1] - gOmegaF[1], wRef[2] - gOmegaF[2] };

  // ===== Inner loop: per-axis PI with anti-windup back-calculation =====
  const float Kpw[3] = {ACS_KPW_X, ACS_KPW_Y, ACS_KPW_Z};
  const float Kiw[3] = {ACS_KIW_X, ACS_KIW_Y, ACS_KIW_Z};

  // Unsaturated PI torque (pre-clamp), and clamped version for AW feedback
  float tauUnsat[3] = {
      Kpw[0] * er[0] + gIstate[0],
      Kpw[1] * er[1] + gIstate[1],
      Kpw[2] * er[2] + gIstate[2]
  };
  float tauClamped[3] = {tauUnsat[0], tauUnsat[1], tauUnsat[2]};
  const float tauMax = cfg_.limits.torqueMax_mNm;
  clamp3_(tauClamped, tauMax);

  // Anti-windup: back-calculation towards the clamped value
  for (int i = 0; i < 3; ++i) {
    const float aw = ACS_KAW * (tauClamped[i] - tauUnsat[i]); // drives i-state to respect clamp
    gIstate[i] += (Kiw[i] * er[i] + aw) * dt;
  }

  // Use the clamped PI output as pre-rate-limit desired torque
  float tauDes_pre[3] = {tauClamped[0], tauClamped[1], tauClamped[2]};

  // Apply rate limit and final clamp per-axis
  float tauDes[3] = {tauDes_pre[0], tauDes_pre[1], tauDes_pre[2]};
  const float prev[3] = {tauDes_[0], tauDes_[1], tauDes_[2]};
  rateLimit3_(tauDes, prev, cfg_.limits.torqueRateLimit_mNm_per_s, dt);
  clamp3_(tauDes, tauMax);
  tauDes_ = {tauDes[0], tauDes[1], tauDes[2]};

#if ACS_VERBOSE
  {
    const float eNorm = std::sqrt(e[0]*e[0] + e[1]*e[1] + e[2]*e[2]);
    const float angDeg = 2.0f * std::asin(std::min(1.0f, 0.5f * eNorm)) * 180.0f / PI_F;
    sif::info << std::fixed << std::setprecision(3)
              << "ACS dbg: |e|=" << angDeg << " deg, "
              << "wRef=[" << wRef[0] << "," << wRef[1] << "," << wRef[2] << "] rad/s, "
              << "wF=["   << gOmegaF[0] << "," << gOmegaF[1] << "," << gOmegaF[2] << "] rad/s, "
              << "er=["   << er[0] << "," << er[1] << "," << er[2] << "] rad/s, "
              << "tauPI_unsat=[" << tauUnsat[0] << "," << tauUnsat[1] << "," << tauUnsat[2] << "] mNm, "
              << "tauPI_sat=["   << tauClamped[0] << "," << tauClamped[1] << "," << tauClamped[2] << "] mNm, "
              << "tauDes=["      << tauDes[0] << "," << tauDes[1] << "," << tauDes[2] << "] mNm"
              << std::endl;
  }
#endif

  // 6) Allocation: body torque -> wheel torques [mNm]
  float tauW[4] = {0,0,0,0};
  allocator_.solve(tauDes, tauW);

  // Extra safety clamp
  for (int i = 0; i < 4; i++) {
    if (tauW[i] > tauMax) tauW[i] = tauMax;
    if (tauW[i] < -tauMax) tauW[i] = -tauMax;
  }
  tauWheelCmd_ = {tauW[0], tauW[1], tauW[2], tauW[3]};

#if ACS_FORCE_TORQUE_ENABLE
  {
    // Force specific torques for bring-up (masked)
    std::array<float, 4> forced{0.f, 0.f, 0.f, 0.f};
    const float F = static_cast<float>(ACS_FORCE_TORQUE_MNM);
    if (ACS_FORCE_TORQUE_WHEEL_MASK & 0x1) forced[0] = F;
    if (ACS_FORCE_TORQUE_WHEEL_MASK & 0x2) forced[1] = F;
    if (ACS_FORCE_TORQUE_WHEEL_MASK & 0x4) forced[2] = F;
    if (ACS_FORCE_TORQUE_WHEEL_MASK & 0x8) forced[3] = F;
    for (int i = 0; i < 4; ++i) {
      if (forced[i] >  tauMax) forced[i] =  tauMax;
      if (forced[i] < -tauMax) forced[i] = -tauMax;
    }
    tauWheelCmd_ = forced;
  }
#endif

  // 7) Send wheel torques with deadband & hysteresis
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

  // For TM compatibility: expose attitude P gains in Kp, and rate-P gains in Kd.
  s.Kp = {ACS_KATT_X, ACS_KATT_Y, ACS_KATT_Z};
  s.Kd = {ACS_KPW_X,  ACS_KPW_Y,  ACS_KPW_Z};

  // For disabled report zeros to keep TM consistent.
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
    if (v[i] >  maxAbs) v[i] =  maxAbs;
    if (v[i] < -maxAbs) v[i] = -maxAbs;
  }
}

void AcsController::rateLimit3_(float v[3], const float vPrev[3], float rateLimitPerSec, float dt) {
  for (int i = 0; i < 3; i++) {
    const float dv = v[i] - vPrev[i];
    const float maxDv = rateLimitPerSec * dt;
    if (dv >  maxDv) v[i] = vPrev[i] + maxDv;
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

    // Clamp + convert float [mNm] -> int16 [mNm]
    float tq = tauWheel_mNm[i];
    const float LIM = cfg_.limits.torqueMax_mNm;
    if (tq >  LIM) tq =  LIM;
    if (tq < -LIM) tq = -LIM;

    // Deadband: below threshold, send one-time zero if last != 0; else skip
    if (std::fabs(tq) < ACS_MIN_CMD_ABS_MNM) {
      if (gLastSentInt16[i] != 0) {
        store_address_t storeId{};
        const uint8_t payloadZero[2] = {0, 0};
        if (ipcStore_->addData(&storeId, payloadZero, sizeof(payloadZero)) == returnvalue::OK) {
          CommandMessage cmd;
          ActionMessage::setCommand(&cmd,
            static_cast<ActionId_t>(ReactionWheelsHandler::CMD_SET_TORQUE),
            storeId);
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

    // Hysteresis: do not re-send identical values
    if (torque_mNm == gLastSentInt16[i]) {
      continue; // nothing new
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
