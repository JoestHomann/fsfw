// example_common/mission/acs/RwAllocator.cpp
#include "RwAllocator.h"
#include <algorithm>
#include <cmath>

/*
 * RwAllocator.h - Reaction-Wheel Torque Allocator
 *
 *   Maps calculated body torque to per-wheel torques using 
 *   the reaction-wheel axes matrix.
 *
 */



RwAllocator::RwAllocator(const acs::Config& cfg) : cfg_(cfg), B_(cfg.Bcols) {}

void RwAllocator::setAxes(const acs::Axes3x4& Bcols) { B_ = Bcols; }

void RwAllocator::solve(const float tauDes[3], float tauWheel[4]) const {
#if ACS_ALLOC_SINGLE_AXIS_DEBUG
  // Debug path: Map body X torque to wheel 1 directly; others off.
  // Wheel torque acts opposite on body torque by convention.
  tauWheel[0] = -tauDes[0];
  tauWheel[1] = 0.0f;
  tauWheel[2] = 0.0f;
  tauWheel[3] = 0.0f;

  // Clamp to device limit
  const float lim = cfg_.limits.torqueMax_mNm;
  for (int i = 0; i < 4; ++i) {
    tauWheel[i] = std::clamp(tauWheel[i], -lim, lim);
  }
  return;
#endif

  // Desired body torque (input). Wheel torque produces opposite body torque.
  const float y[3] = { -tauDes[0], -tauDes[1], -tauDes[2] };

  // Compute M = B * B^T (3x3)
  float M[9] = {0};            // row-major: [0..2]=row0, [3..5]=row1, [6..8]=row2
  for (int i = 0; i < 4; ++i) {
    const float bx = B_[0 + 3*i];
    const float by = B_[1 + 3*i];
    const float bz = B_[2 + 3*i];
    M[0] += bx*bx; M[1] += bx*by; M[2] += bx*bz;
    M[3] += by*bx; M[4] += by*by; M[5] += by*bz;
    M[6] += bz*bx; M[7] += bz*by; M[8] += bz*bz;
  }

  // Invert M with adjugate. Guard against near-singularity.
  const float det =
      M[0]*(M[4]*M[8] - M[5]*M[7])
    - M[1]*(M[3]*M[8] - M[5]*M[6])
    + M[2]*(M[3]*M[7] - M[4]*M[6]);

  if (std::fabs(det) < 1e-6f) {
    // Fallback: put torque on the most aligned wheel axis (simple projection)
    int idx = 0; float best = 0.0f;
    for (int i = 0; i < 4; ++i) {
      const float proj = std::fabs(B_[0+3*i]*y[0] + B_[1+3*i]*y[1] + B_[2+3*i]*y[2]);
      if (proj > best) { best = proj; idx = i; }
    }
    for (int i = 0; i < 4; ++i) {
      const float bx = B_[0+3*i], by = B_[1+3*i], bz = B_[2+3*i];
      const float dot = bx*y[0] + by*y[1] + bz*y[2];  // keep sign
      tauWheel[i] = (i == idx) ? dot : 0.0f;
    }
  } else {
    // Compute Minv = (B B^T)^-1
    const float invDet = 1.0f / det;
    float Minv[9];
    Minv[0] =  (M[4]*M[8]-M[5]*M[7]) * invDet;
    Minv[1] = -(M[1]*M[8]-M[2]*M[7]) * invDet;
    Minv[2] =  (M[1]*M[5]-M[2]*M[4]) * invDet;
    Minv[3] = -(M[3]*M[8]-M[5]*M[6]) * invDet;
    Minv[4] =  (M[0]*M[8]-M[2]*M[6]) * invDet;
    Minv[5] = -(M[0]*M[5]-M[2]*M[3]) * invDet;
    Minv[6] =  (M[3]*M[7]-M[4]*M[6]) * invDet;
    Minv[7] = -(M[0]*M[7]-M[1]*M[6]) * invDet;
    Minv[8] =  (M[0]*M[4]-M[1]*M[3]) * invDet;

    // v = Minv * y
    const float vx = Minv[0]*y[0] + Minv[1]*y[1] + Minv[2]*y[2];
    const float vy = Minv[3]*y[0] + Minv[4]*y[1] + Minv[5]*y[2];
    const float vz = Minv[6]*y[0] + Minv[7]*y[1] + Minv[8]*y[2];

    // tauWheel = B^T * v
    for (int i = 0; i < 4; ++i) {
      const float bx = B_[0 + 3*i], by = B_[1 + 3*i], bz = B_[2 + 3*i];
      tauWheel[i] = bx*vx + by*vy + bz*vz;
    }
  }

  // Clamp per-wheel torque to device limit
  const float lim = cfg_.limits.torqueMax_mNm;
  for (int i = 0; i < 4; ++i) {
    tauWheel[i] = std::clamp(tauWheel[i], -lim, lim);
  }
}
