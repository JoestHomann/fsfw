#pragma once
#include <array>
#include "AcsConfig.h"

// Reaction-wheel allocator: maps desired body torque to per-wheel torques
// - Input  : tauDes[3]  body torque in mNm (x,y,z)
// - Output : tauWheel[4] wheel torques in mNm (w1..w4)
// - Method : minimum-norm solution using pseudo-inverse  B+ = B^T (B B^T)^-1
//            where B is 3x4, columns are unit wheel axes in body frame.
// - Notes  : Implementation guards against near-singular B B^T and will fall
//            back to putting torque on the best-aligned wheel in that case.

class RwAllocator {
 public:
  // Construct with static ACS config (copies B from cfg by default)
  explicit RwAllocator(const acs::Config& cfg);

  // Update wheel axes matrix (3x4, column-major: b1|b2|b3|b4)
  void setAxes(const acs::Axes3x4& Bcols);

  // Read back current axes matrix
  const acs::Axes3x4& axes() const { return B_; }

  // Compute wheel torques from desired body torque
  // tauDes [mNm] -> tauWheel [mNm]
  // Uses min-norm pseudo-inverse; see .cpp for singularity fallback.
  void solve(const float tauDes[3], float tauWheel[4]) const;

 private:
  acs::Config  cfg_;  // static parameters (limits, timing, etc.)
  acs::Axes3x4 B_;    // 3x4 wheel axes matrix in body frame (column-major)
};
