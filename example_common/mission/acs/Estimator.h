#pragma once
#include <array>
#include "AcsConfig.h"

/**
 * Propagation + gyro measurement simulation.
 * State: q_true (unit quaternion), omega_true (rad/s), gyro bias (rad/s).
 * Input: measured wheel torques [mNm] (from RW status incl. BLWN).
 * Output: omega_meas for controller; q_true/omega_true for HK.
 */
class Estimator {
 public:
  explicit Estimator(const acs::Config& cfg);

  void setAxes(const acs::Axes3x4& Bcols);

  // One propagation step. tauW_mNm: wheel torques [mNm]. Optional external torque [mNm].
  void step(const std::array<float,4>& tauW_mNm,
            const std::array<float,3>& tauExt_mNm = {0.0f,0.0f,0.0f});

  const float* omegaTrue() const { return omega_; }
  const float* omegaMeas() const { return omegaMeas_; }
  const float* quatTrue()  const { return q_; }

  void reset(const float q_init[4], const float w_init[3]);

 private:
  acs::Config  cfg_;
  acs::Axes3x4 B_;

  float q_[4];
  float omega_[3];
  float bias_[3];
  float blwn_[3];
  float omegaMeas_[3];

  static float normalRand_(); // unit Gaussian
  static void  quatNormalize_(float q[4]);
};
