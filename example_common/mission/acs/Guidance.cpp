#include "Guidance.h"
#include <cmath>

// Constructor: default initialization (identity target)
Guidance::Guidance() = default;

// Set target attitude and normalize to unit length
void Guidance::setTargetQuat(const std::array<float,4>& q_ref) {
  qRef_ = q_ref;
  normalize_(qRef_);
}

// Advance guidance by one control step
// Hold-attitude mode: no operation
void Guidance::update(float /*dt*/) {
  // no-op for fixed target
}

// Provide rate command in BODY frame [rad/s]
// Hold-attitude mode: zero rate reference
void Guidance::getRateCmd(float outWcmd[3]) const {
  outWcmd[0] = 0.0f;
  outWcmd[1] = 0.0f;
  outWcmd[2] = 0.0f;
}

// Normalize quaternion in place; fallback to identity if norm is tiny
void Guidance::normalize_(std::array<float,4>& q) {
  float n = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  if (n > 1e-12f) {
    float inv = 1.0f / n;
    for (auto& v : q) v *= inv;
  } else {
    q = {1.0f, 0.0f, 0.0f, 0.0f};
  }
}
