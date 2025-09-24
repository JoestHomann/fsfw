#include "Guidance.h"
#include <cmath>

Guidance::Guidance() = default;

void Guidance::setTargetQuat(const std::array<float,4>& q_ref) {
  qRef_ = q_ref;
  normalize_(qRef_);
}

void Guidance::update(float /*dt*/) {
  // For "hold" mode nothing to do; extend for time-varying profiles if needed.
}

void Guidance::getRateCmd(float outWcmd[3]) const {
  // Zero reference body rates (rate damping around attitude hold)
  outWcmd[0] = 0.0f; outWcmd[1] = 0.0f; outWcmd[2] = 0.0f;
}

void Guidance::normalize_(std::array<float,4>& q) {
  float n = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  if (n > 1e-12f) {
    float inv = 1.0f / n;
    for (auto& v : q) v *= inv;
  } else {
    // Fallback to identity quaternion if an invalid target is passed
    q = {1.0f, 0.0f, 0.0f, 0.0f};
  }
}
