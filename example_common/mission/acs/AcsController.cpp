#include "AcsController.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>

#include "fsfw/datapoollocal/LocalPoolVariable.h"
#include "commonObjects.h"
#include "RwHandler/ReactionWheelsHandler.h"
#include "fsfw/ipc/CommandMessage.h"
#include "fsfw/ipc/MessageQueueSenderIF.h"
#include "fsfw/action/ActionMessage.h"
#include "fsfw/serviceinterface/ServiceInterfaceStream.h"
#include "fsfw/timemanager/Clock.h" 

/*
 * AcsController.h – Attitude Control System (ACS)
 *
 * AcsController computes body torque commands from attitude/rate errors and maps
 * them to reaction wheels. Implements a cascaded P/PI controller with anti-windup.
 *
 * Features:
 *  - Takes attitude target from Guidance and gyro input from Estimator to calculate body
 *    torque commands:
 *      - Outer loop: Attitude-P -> rate reference (per axis)
 *      - Inner loop: Rate-PI with anti-windup (back-calculation) and rate low-pass
 *  - Allocation: Body torque -> RW torques using Axes3x4 and wheelMask
 *  - Limits: Rate/torque clamps, deadband, saturation handling
 *
 * Notes:
 *  - Tuning and limits in AcsConfig.h
 *  - Interfaces with RW DeviceHandlers via FSFW message queues
 *
 */

namespace {
constexpr float PI_F = 3.14159265358979323846f;

// Last-sent int16 torques per wheel (anti-spam)
int16_t gLastSentInt16[4] = {0, 0, 0, 0};

// Initialisation of controller state
float gIstate[3]  = {0.f, 0.f, 0.f}; // PI integrator state [mNm]
float gOmegaF[3]  = {0.f, 0.f, 0.f}; // LPF body rates [rad/s]
bool  gInitStates = false;           // One-time init guard

// Convert quaternion to yaw(Z), pitch(Y), roll(X) in radians
static inline void quatToYpr(const float q[4], float& yaw, float& pitch, float& roll) {
  // roll (x-axis rotation)
  const float sinr_cosp = 2.0f * (q[0]*q[1] + q[2]*q[3]);
  const float cosr_cosp = 1.0f - 2.0f * (q[1]*q[1] + q[2]*q[2]);
  roll = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  const float sinp = 2.0f * (q[0]*q[2] - q[3]*q[1]);
  if (std::fabs(sinp) >= 1.0f) {
    pitch = std::copysign(PI_F / 2.0f, sinp); // clamp at +/- 90 deg
  } else {
    pitch = std::asin(sinp);
  }

  // yaw (z-axis rotation)
  const float siny_cosp = 2.0f * (q[0]*q[3] + q[1]*q[2]);
  const float cosy_cosp = 1.0f - 2.0f * (q[2]*q[2] + q[3]*q[3]);
  yaw = std::atan2(siny_cosp, cosy_cosp);
}
} // namespace

// Construct controller and wire submodules
AcsController::AcsController(object_id_t objectId, std::array<object_id_t, 4> wheelIds,
                             const acs::Config& cfg)
    : SystemObject(objectId),
      cfg_(cfg),
      rwIds_(wheelIds),
      estimator_(cfg),
      guidance_(),
      allocator_(cfg) {
  // Propagate axes to estimator and allocator
  estimator_.setAxes(cfg_.Bcols);
  allocator_.setAxes(cfg_.Bcols);

  // Cache dt in ms for TM
  dtMs_ = cfg_.timing.dtMs;

  // Map config gains to TM-visible fields (Kp = attitude-P, Kd = rate-P)
  Kp_ = {cfg_.ctrl.outer.kAttX, cfg_.ctrl.outer.kAttY, cfg_.ctrl.outer.kAttZ};
  Kd_ = {cfg_.ctrl.inner.KpwX,  cfg_.ctrl.inner.KpwY,  cfg_.ctrl.inner.KpwZ};

  // Setup IPC resources
  myQueue_ = QueueFactory::instance()->createMessageQueue(3);
  ipcStore_ = ObjectManager::instance()->get<StorageManagerIF>(objects::IPC_STORE);
  if (ipcStore_ == nullptr) {
    sif::error << "ACS: IPC store not available!" << std::endl;
  }
}

// Task entry: one closed-loop step
ReturnValue_t AcsController::performOperation(uint8_t) {
  const float dt = cfg_.timing.dt();

  // Detect enable edge to safely stop wheels and reset state
  const bool enabledNow = enabled_.load();
  static bool prevEnabled = false;

  // Disabled path: optionally send STOP once on falling edge and clear state
  if (!enabledNow) {
    if (prevEnabled) {
      // Send STOP to all wheels exactly once on ON->OFF transition
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

      // Clear outputs and internal states for a clean restart
      tauDes_ = {0.f, 0.f, 0.f};
      tauWheelCmd_ = {0.f, 0.f, 0.f, 0.f};
      for (auto& v : gLastSentInt16) v = 0;
      gIstate[0]=gIstate[1]=gIstate[2]=0.f;
      gOmegaF[0]=gOmegaF[1]=gOmegaF[2]=0.f;
      gInitStates = false;
    }
    prevEnabled = false;
    return returnvalue::OK; // do nothing when disabled
  }

  // Enabled path: initialize internal states once after enable
  if (!prevEnabled || !gInitStates) {
    gIstate[0] = gIstate[1] = gIstate[2] = 0.f;
    gOmegaF[0] = gOmegaF[1] = gOmegaF[2] = 0.f;
    gInitStates = true;
  }
  prevEnabled = true;

  // Read measured wheel torques [mNm] from handlers for estimator input
  std::array<float, 4> tauW_meas = {0, 0, 0, 0};
  for (int i = 0; i < 4; ++i) {
    const object_id_t wid = rwIds_[i];
    if (wid == 0) continue;
    using PoolIds = ReactionWheelsHandler::PoolIds;
    LocalPoolVariable<int16_t> torqueVar(wid, static_cast<lp_id_t>(PoolIds::HK_TORQUE_mNm));
    if (torqueVar.read() == returnvalue::OK && torqueVar.isValid()) {
      tauW_meas[i] = static_cast<float>(torqueVar.value); // device HK
    } else {
      tauW_meas[i] = tauWheelCmd_[i]; // fallback to last command
    }
  }

  // Estimator step: Propagate true state and produce measured omega
  estimator_.step(tauW_meas, /*tauExt_mNm=*/std::array<float,3>{0.f,0.f,0.f});
  const float* wMeas = estimator_.omegaMeas(); // [rad/s] to controller
  const float* qTrue = estimator_.quatTrue();  // true attitude for logging

  // Initialize LPF once from first available measurement to avoid jump
  if (gOmegaF[0]==0.f && gOmegaF[1]==0.f && gOmegaF[2]==0.f) {
    gOmegaF[0]=wMeas[0]; gOmegaF[1]=wMeas[1]; gOmegaF[2]=wMeas[2];
  }


  // Compute attitude error quaternion: qerr = qRef * conj(qTrue)
  auto qRef = guidance_.getTargetQuat();
  float qc[4];
  quatConj_(qTrue, qc);
  float qerr[4];
  quatMul_(qRef.data(), qc, qerr);
  if (qerr[0] < 0.0f) for (int i=0;i<4;i++) qerr[i] = -qerr[i]; // shortest path
  qErr_ = {qerr[0], qerr[1], qerr[2], qerr[3]};

  // Periodic attitude log (every cfg_.debug.logEveryN loops)
  static uint32_t attCtr = 0;
  if (cfg_.debug.logEveryN > 0 && ++attCtr >= cfg_.debug.logEveryN) {
    attCtr = 0;

    // Convert both to YPR for human-readable logs
    float yawR, pitchR, rollR, yawT, pitchT, rollT;
    quatToYpr(qRef.data(), yawR, pitchR, rollR);
    quatToYpr(qTrue,       yawT, pitchT, rollT);

    const float rad2deg = 180.0f / PI_F;

    // Small-angle error magnitude from vector part (deg)
    const float evec[3] = {2.0f*qerr[1], 2.0f*qerr[2], 2.0f*qerr[3]};
    const float eNorm   = std::sqrt(evec[0]*evec[0] + evec[1]*evec[1] + evec[2]*evec[2]);
    const float angDeg  = 2.0f * std::asin(std::min(1.0f, 0.5f * eNorm)) * rad2deg;

    sif::info << std::fixed << std::setprecision(2)
              << "ATT ref[YPRdeg]=[" << yawR*rad2deg << "," << pitchR*rad2deg << "," << rollR*rad2deg << "] "
              << "true[YPRdeg]=["    << yawT*rad2deg << "," << pitchT*rad2deg << "," << rollT*rad2deg << "] "
              << "errAngle=" << angDeg << " deg"
              << std::endl;
  }

  // Outer loop: attitude P -> rate reference (per-axis clamp)
  const float e[3] = {2.0f*qerr[1], 2.0f*qerr[2], 2.0f*qerr[3]};  // small-angle vector error
  float wRef[3] = {
      cfg_.ctrl.outer.kAttX * e[0],
      cfg_.ctrl.outer.kAttY * e[1],
      cfg_.ctrl.outer.kAttZ * e[2]
  };
  wRef[0] = std::clamp(wRef[0], -cfg_.ctrl.outer.wRefMaxX, cfg_.ctrl.outer.wRefMaxX);
  wRef[1] = std::clamp(wRef[1], -cfg_.ctrl.outer.wRefMaxY, cfg_.ctrl.outer.wRefMaxY);
  wRef[2] = std::clamp(wRef[2], -cfg_.ctrl.outer.wRefMaxZ, cfg_.ctrl.outer.wRefMaxZ);

  // Low-pass filter on measured omega
  if (cfg_.ctrl.inner.omegaLpFcHz > 0.f) {
    const float alpha = 1.0f - std::exp(-2.0f * PI_F * cfg_.ctrl.inner.omegaLpFcHz * dt);
    gOmegaF[0] = gOmegaF[0] + alpha * (wMeas[0] - gOmegaF[0]);
    gOmegaF[1] = gOmegaF[1] + alpha * (wMeas[1] - gOmegaF[1]);
    gOmegaF[2] = gOmegaF[2] + alpha * (wMeas[2] - gOmegaF[2]);
  } else {
    gOmegaF[0]=wMeas[0]; gOmegaF[1]=wMeas[1]; gOmegaF[2]=wMeas[2];
  }

  // Inner loop: per-axis PI with anti-windup back-calculation
  const float er[3] = { wRef[0]-gOmegaF[0], wRef[1]-gOmegaF[1], wRef[2]-gOmegaF[2] };

  // Unsaturated PI output
  float tauUnsat[3] = {
    cfg_.ctrl.inner.KpwX * er[0] + gIstate[0],
    cfg_.ctrl.inner.KpwY * er[1] + gIstate[1],
    cfg_.ctrl.inner.KpwZ * er[2] + gIstate[2]
  };

  // Apply torque clamp before rate limit (for anti-windup feedback)
  float tauClamped[3] = {tauUnsat[0], tauUnsat[1], tauUnsat[2]};
  clamp3_(tauClamped, cfg_.limits.torqueMax_mNm);

  // Anti-windup: drive integrator so that tau follows the clamp
  for (int i=0;i<3;i++) {
    const float aw = cfg_.ctrl.inner.Kaw * (tauClamped[i] - tauUnsat[i]);
    const float Ki = (i==0) ? cfg_.ctrl.inner.KiwX : (i==1) ? cfg_.ctrl.inner.KiwY : cfg_.ctrl.inner.KiwZ;
    gIstate[i] += (Ki * er[i] + aw) * dt;
  }

  // Pre-rate-limit desired torque
  float tauDes_pre[3] = {tauClamped[0], tauClamped[1], tauClamped[2]};

  // Per-axis torque rate limit and final clamp
  float tauDes[3] = {tauDes_pre[0], tauDes_pre[1], tauDes_pre[2]};
  const float prev[3] = {tauDes_[0], tauDes_[1], tauDes_[2]};
  rateLimit3_(tauDes, prev, cfg_.limits.torqueRateLimit_mNm_per_s, dt);
  clamp3_(tauDes, cfg_.limits.torqueMax_mNm);
  tauDes_ = {tauDes[0], tauDes[1], tauDes[2]};

  // Debug line every 10 control-loop iterations
#if ACS_VERBOSE
  static uint32_t dbgCtr = 0;
  if (++dbgCtr >= 10) {
    dbgCtr = 0;
    const float eNorm  = std::sqrt(e[0]*e[0] + e[1]*e[1] + e[2]*e[2]);
    const float angDeg = 2.0f * std::asin(std::min(1.0f, 0.5f * eNorm)) * 180.0f / PI_F;
    sif::info << std::fixed << std::setprecision(3)
              << "ACS dbg: |e|=" << angDeg << " deg, "
              << "wRef=[" << wRef[0] << "," << wRef[1] << "," << wRef[2] << "] rad/s, "
              << "wF=["   << gOmegaF[0] << "," << gOmegaF[1] << "," << gOmegaF[2] << "] rad/s, "
              << "er=["   << er[0] << "," << er[1] << "," << er[2] << "] rad/s, "
              << "tauPI_unsat=[" << tauUnsat[0] << "," << tauUnsat[1] << "," << tauUnsat[2] << "] mNm, "
              << "tauPI_sat=["   << tauClamped[0] << "," << tauClamped[1] << "," << tauClamped[2] << "] mNm, "
              << "tauDes=["      << tauDes_[0] << "," << tauDes_[1] << "," << tauDes_[2] << "] mNm"
              << std::endl;
  }
#endif

  // Allocation: body torque -> per-wheel torques [mNm]
  float tauW[4] = {0,0,0,0};
  allocator_.solve(tauDes, tauW);

  // Optional bring-up: override with constant torques per mask
  if (cfg_.force.enable) {
    std::array<float,4> forced{0.f,0.f,0.f,0.f};
    if (cfg_.force.wheelMask & 0x1) forced[0] = cfg_.force.torque_mNm;
    if (cfg_.force.wheelMask & 0x2) forced[1] = cfg_.force.torque_mNm;
    if (cfg_.force.wheelMask & 0x4) forced[2] = cfg_.force.torque_mNm;
    if (cfg_.force.wheelMask & 0x8) forced[3] = cfg_.force.torque_mNm;
    for (int i=0;i<4;i++) {
      forced[i] = std::clamp(forced[i], -cfg_.limits.torqueMax_mNm, cfg_.limits.torqueMax_mNm);
    }
    tauWheelCmd_ = forced;
  } else {
    // Safety clamp (allocator already clamps, keep defensive)
    for (int i=0;i<4;i++) {
      tauW[i] = std::clamp(tauW[i], -cfg_.limits.torqueMax_mNm, cfg_.limits.torqueMax_mNm);
    }
    tauWheelCmd_ = {tauW[0], tauW[1], tauW[2], tauW[3]};
  }

  // Send wheel torques with deadband and hysteresis to reduce bus traffic
  sendWheelTorques_(tauWheelCmd_);

  return returnvalue::OK;
}

// Build diagnostic snapshot for TM
AcsDiagSnapshot AcsController::getDiag() const {
  AcsDiagSnapshot s;
  s.version = 1;
  s.enabled = enabled_.load();

  // Attitudes and rates
  s.qRef = guidance_.getTargetQuat();
  const float* qt = estimator_.quatTrue();
  const float* wt = estimator_.omegaTrue();
  const float* wm = estimator_.omegaMeas();
  s.qTrue = {qt[0], qt[1], qt[2], qt[3]};
  s.wTrue = {wt[0], wt[1], wt[2]};
  s.wMeas = {wm[0], wm[1], wm[2]};

  // Expose controller gains for ground visibility
  s.Kp = {cfg_.ctrl.outer.kAttX, cfg_.ctrl.outer.kAttY, cfg_.ctrl.outer.kAttZ};
  s.Kd = {cfg_.ctrl.inner.KpwX,  cfg_.ctrl.inner.KpwY,  cfg_.ctrl.inner.KpwZ};

  // Torques (report zeros when disabled)
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

// -------- Attitude snapshot for TM 220/133 ----------
void AcsController::getAttitudeTM(AttitudeTM& out) const {
  // Fetch reference and true attitude
  const auto qRef = guidance_.getTargetQuat();
  const float* qTrue = estimator_.quatTrue();

  // Convert both to YPR in radians
  float yawR, pitchR, rollR, yawT, pitchT, rollT;
  quatToYpr(qRef.data(), yawR, pitchR, rollR);
  quatToYpr(qTrue,       yawT, pitchT, rollT);

  // Compute small-angle error magnitude from vector part of qerr
  float qc[4];
  quatConj_(qTrue, qc);
  float qerr[4];
  quatMul_(qRef.data(), qc, qerr);
  if (qerr[0] < 0.0f) for (int i=0;i<4;i++) qerr[i] = -qerr[i]; // shortest path


  const float rad2deg = 180.0f / PI_F;
  const float evec[3] = {2.0f*qerr[1], 2.0f*qerr[2], 2.0f*qerr[3]};
  const float eNorm   = std::sqrt(evec[0]*evec[0] + evec[1]*evec[1] + evec[2]*evec[2]);
  const float angDeg  = 2.0f * std::asin(std::min(1.0f, 0.5f * eNorm)) * rad2deg;

  // Fill output (degrees)
  out.refYawDeg   = yawR   * rad2deg;
  out.refPitchDeg = pitchR * rad2deg;
  out.refRollDeg  = rollR  * rad2deg;

  out.trueYawDeg   = yawT   * rad2deg;
  out.truePitchDeg = pitchT * rad2deg;
  out.trueRollDeg  = rollT  * rad2deg;

  out.errAngleDeg = angDeg;

  // Timestamp
  (void)Clock::getUptime(&out.timestampMs);
}

// --- helpers ---
void AcsController::quatConj_(const float q[4], float qc[4]) {
  qc[0] = q[0]; qc[1] = -q[1]; qc[2] = -q[2]; qc[3] = -q[3];
}
void AcsController::quatMul_(const float a[4], const float b[4], float out[4]) {
  out[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
  out[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
  out[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
  out[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}
void AcsController::clamp3_(float v[3], float maxAbs) {
  for (int i=0;i<3;i++) v[i] = std::clamp(v[i], -maxAbs, maxAbs);
}
void AcsController::rateLimit3_(float v[3], const float vPrev[3], float rateLimitPerSec, float dt) {
  for (int i=0;i<3;i++) {
    const float dv = v[i] - vPrev[i];
    const float maxDv = rateLimitPerSec * dt;
    if (dv >  maxDv) v[i] = vPrev[i] + maxDv;
    if (dv < -maxDv) v[i] = vPrev[i] - maxDv;
  }
}

// Send torque commands to wheel handlers with small-value deadband and
// "do not resend same value" hysteresis to reduce traffic.
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

    // Clamp to device limit (float)
    float tq = std::clamp(tauWheel_mNm[i], -cfg_.limits.torqueMax_mNm, cfg_.limits.torqueMax_mNm);

    // Deadband: if tiny torque and last sent was non-zero, send one-time zero
    if (std::fabs(tq) < cfg_.deadband.minCmdAbs_mNm) {
      if (gLastSentInt16[i] != 0) {
        store_address_t storeId{};
        const uint8_t payloadZero[2] = {0, 0}; // big-endian int16(0)
        if (ipcStore_->addData(&storeId, payloadZero, sizeof(payloadZero)) == returnvalue::OK) {
          CommandMessage cmd;
          ActionMessage::setCommand(&cmd,
              static_cast<ActionId_t>(ReactionWheelsHandler::CMD_SET_TORQUE),
              storeId);
          (void)MessageQueueSenderIF::sendMessage(dh->getCommandQueue(), &cmd, myQueue_->getId());
          gLastSentInt16[i] = 0; // remember last sent value
        }
      }
      continue; // skip sending a small non-zero torque
    }

    // Quantize to int16 mNm
    int32_t rounded = static_cast<int32_t>(std::lround(tq));
    if (rounded > std::numeric_limits<int16_t>::max()) rounded = std::numeric_limits<int16_t>::max();
    if (rounded < std::numeric_limits<int16_t>::min()) rounded = std::numeric_limits<int16_t>::min();
    const int16_t torque_mNm = static_cast<int16_t>(rounded);

    // Hysteresis: do not resend identical values
    if (torque_mNm == gLastSentInt16[i]) {
      continue;
    }

    // Pack big-endian and send as Action command
    store_address_t storeId{};
    const uint8_t payload[2] = {
      static_cast<uint8_t>((torque_mNm >> 8) & 0xFF),
      static_cast<uint8_t>( torque_mNm       & 0xFF)
    };
    if (ipcStore_->addData(&storeId, payload, sizeof(payload)) != returnvalue::OK) {
      continue; // store full; skip this cycle
    }

    CommandMessage cmd;
    ActionMessage::setCommand(&cmd,
        static_cast<ActionId_t>(ReactionWheelsHandler::CMD_SET_TORQUE),
        storeId);

    if (MessageQueueSenderIF::sendMessage(dh->getCommandQueue(), &cmd, myQueue_->getId()) == returnvalue::OK) {
      gLastSentInt16[i] = torque_mNm; // update last-sent on success
    }
  }
}
