// example_common/mission/acs/RwAllocator.h
#pragma once
#include <array>
#include "AcsConfig.h"

/**
 * Maps desired body torque [mNm] to per-wheel torques [mNm] using pseudo-inverse of B.
 * B is 3x4 (columns are unit wheel axes in body frame).
 */
class RwAllocator {
 public:
  explicit RwAllocator(const acs::Config& cfg);

  void setAxes(const acs::Axes3x4& Bcols);
  const acs::Axes3x4& axes() const { return B_; }

  // tauDes [mNm] -> tauWheel [mNm]; simple min-norm solution via B+ = B^T (B B^T)^{-1}
  void solve(const float tauDes[3], float tauWheel[4]) const;

 private:
  acs::Config  cfg_;
  acs::Axes3x4 B_;
};

