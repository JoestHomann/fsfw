#pragma once

#include <array>

// Minimal allocator mapping desired body torque to wheel torques.
// For the 1-wheel demo, we map Z-axis torque to wheel #0, others 0.
// Extend to a full B-matrix pseudo-inverse when you add more wheels.

class RwAllocator {
 public:
  RwAllocator() = default;

  // tauDes[3] -> tauWheel[4]
  void solve(const float tauDes[3], float tauWheel[4]) const {
    // Trivial demo mapping:
    tauWheel[0] = -tauDes[2]; // assume wheel 0 roughly aligned with +Z body
    tauWheel[1] = 0.0f;
    tauWheel[2] = 0.0f;
    tauWheel[3] = 0.0f;
  }
};
