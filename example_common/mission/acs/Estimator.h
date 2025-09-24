#pragma once
#include <array>
#include "AcsConfig.h"

// Estimator: propagates true attitude/rate and synthesizes gyro measurements
// - State: q_true (unit quaternion), omega_true (rad/s), gyro bias (rad/s)
// - Input: wheel torques [mNm] and optional external torque [mNm]
// - Output: omega_meas (for controller) and q_true / omega_true (for HK)

class Estimator {
 public:
  // Construct with static config (timing, inertia, gyro noise, axes, limits)
  explicit Estimator(const acs::Config& cfg);

  // Set RW axes matrix B (3x4, column-major). Overrides cfg.Bcols if needed.
  void setAxes(const acs::Axes3x4& Bcols);

  // Run one propagation step
  // tauW_mNm: wheel torques [mNm] (as measured / commanded)
  // tauExt_mNm: external body torque [mNm] (default 0,0,0)
  void step(const std::array<float,4>& tauW_mNm,
            const std::array<float,3>& tauExt_mNm = {0.0f, 0.0f, 0.0f});

  // Accessors for housekeeping
  // True body rates [rad/s]
  const float* omegaTrue() const { return omega_; }
  // Measured body rates [rad/s] after bias + noise + optional BLWN
  const float* omegaMeas() const { return omegaMeas_; }
  // True attitude quaternion (unit length)
  const float* quatTrue()  const { return q_; }

  // Reset full state (q, omega, bias, filters)
  void reset(const float q_init[4], const float w_init[3]);

 private:
  // Copy of static configuration (timing, inertia, noise, etc.)
  acs::Config  cfg_{};
  // RW axes matrix (3x4, column-major b1|b2|b3|b4)
  acs::Axes3x4 B_{};

  // True attitude quaternion (w,x,y,z), unit length
  float q_[4]{};
  // True body rates [rad/s]
  float omega_[3]{};
  // Gyro bias state [rad/s]
  float bias_[3]{};
  // Band-limited white noise state for gyro [rad/s]
  float blwn_[3]{};
  // Synthesized gyro measurement [rad/s]
  float omegaMeas_[3]{};

  // Draw from standard normal distribution (Box–Muller)
  static float normalRand_();
  // Normalize quaternion in place; on tiny norm, set to identity
  static void  quatNormalize_(float q[4]);
};
