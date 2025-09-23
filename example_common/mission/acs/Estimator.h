#pragma once

// Minimal estimator: returns body rates (rad/s).
// For now, zeros; later you can feed actual gyro/attitude estimates.

class Estimator {
 public:
  void update(float /*dt*/) {}
  void getBodyRate(float outW[3]) const {
    outW[0] = 0.0f; outW[1] = 0.0f; outW[2] = 0.0f;
  }
};
