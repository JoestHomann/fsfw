// example_common/mission/acs/AcsController.cpp
#include "AcsController.h"
#include <cmath>

#include "fsfw/serviceinterface/ServiceInterfaceStream.h"

AcsController::AcsController(object_id_t objectId, std::array<object_id_t, 4> wheelIds)
    : SystemObject(objectId), rwIds_(wheelIds) {
  // Defaults for demo
  kd_[0] = kd_[1] = kd_[2] = 0.05f;
  dtMs_ = 50;
}

ReturnValue_t AcsController::performOperation(uint8_t) {
  // In a real system, you'd read sensors, estimate attitude/rates,
  // compute desired torque, allocate to wheels, and send setpoints.
  // Here we just keep some moving demo numbers if enabled.
  if (enabled_.load(std::memory_order_relaxed)) {
    updateInternalDemo_();
    // Note: sending to wheels is intentionally omitted in this minimal scaffold.
    // You could look up the ReactionWheelsHandler objects and send RPM/TORQUE here.
  }
  return returnvalue::OK;
}

void AcsController::fillDiagSnapshot(AcsDiagSnapshot& out) const {
  out.version = 1;
  out.enabled = enabled_.load(std::memory_order_relaxed);
  for (int i = 0; i < 3; ++i) {
    out.kd[i]     = kd_[i];
    out.tauDes[i] = tauDes_[i];
  }
  for (int i = 0; i < 4; ++i) {
    out.tauWheel[i] = tauWheel_[i];
  }
  out.dt_ms = dtMs_;
}

void AcsController::setKd(float kx, float ky, float kz) {
  kd_[0] = kx; kd_[1] = ky; kd_[2] = kz;
}

void AcsController::updateInternalDemo_() {
  // Very small placeholder dynamics just to have non-zero values
  // This can be replaced by your real pipeline later.
  static float t = 0.0f;
  t += 0.05f; // assuming 20 Hz

  tauDes_[0] = 2.0f * std::sin(t);
  tauDes_[1] = 1.5f * std::cos(0.5f * t);
  tauDes_[2] = 1.0f * std::sin(0.3f * t + 1.0f);

  // Simple allocator demo: distribute Z torque on wheel 0..3 evenly,
  // X and Y on first two for illustration (mNm units).
  tauWheel_[0] = 0.5f * tauDes_[2] + 0.7f * tauDes_[0];
  tauWheel_[1] = 0.5f * tauDes_[2] + 0.7f * tauDes_[1];
  tauWheel_[2] = 0.5f * tauDes_[2] - 0.7f * tauDes_[0];
  tauWheel_[3] = 0.5f * tauDes_[2] - 0.7f * tauDes_[1];
}
