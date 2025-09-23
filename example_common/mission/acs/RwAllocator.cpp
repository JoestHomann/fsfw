// example_common/mission/acs/RwAllocator.cpp
#include "RwAllocator.h"
#include <cmath>

RwAllocator::RwAllocator(const acs::Config& cfg) : cfg_(cfg), B_(cfg.Bcols) {}

void RwAllocator::setAxes(const acs::Axes3x4& Bcols) { B_ = Bcols; }

void RwAllocator::solve(const float tauDes[3], float tauWheel[4]) const {
  // Compute y = -(tauDes) because wheel torque acts opposite on body
  float y[3] = {-tauDes[0], -tauDes[1], -tauDes[2]};

  // Compute M = B B^T (3x3)
  float M[9] = {0};
  for (int i=0;i<4;i++) {
    float bx = B_[0+3*i], by = B_[1+3*i], bz = B_[2+3*i];
    M[0] += bx*bx; M[1] += bx*by; M[2] += bx*bz;
    M[3] += by*bx; M[4] += by*by; M[5] += by*bz;
    M[6] += bz*bx; M[7] += bz*by; M[8] += bz*bz;
  }

  // Invert M (3x3). Guard against near-singularity.
  float det = M[0]*(M[4]*M[8]-M[5]*M[7]) - M[1]*(M[3]*M[8]-M[5]*M[6]) + M[2]*(M[3]*M[7]-M[4]*M[6]);
  if (std::fabs(det) < 1e-6f) {
    // Fallback: put everything on the "best" aligned wheel (max projection)
    int idx=0; float best=0.0f;
    for (int i=0;i<4;i++) {
      float proj = std::fabs(B_[0+3*i]*y[0]+B_[1+3*i]*y[1]+B_[2+3*i]*y[2]);
      if (proj>best) { best=proj; idx=i; }
    }
    for (int i=0;i<4;i++) tauWheel[i] = (i==idx) ? (B_[0+3*i]*y[0]+B_[1+3*i]*y[1]+B_[2+3*i]*y[2]) : 0.0f;
    return;
  }
  float invDet = 1.0f/det;
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

  // Compute tauWheel = B^T Minv y
  // v = Minv * y
  float vx = Minv[0]*y[0] + Minv[1]*y[1] + Minv[2]*y[2];
  float vy = Minv[3]*y[0] + Minv[4]*y[1] + Minv[5]*y[2];
  float vz = Minv[6]*y[0] + Minv[7]*y[1] + Minv[8]*y[2];
  for (int i=0;i<4;i++) {
    float bx = B_[0+3*i], by = B_[1+3*i], bz = B_[2+3*i];
    tauWheel[i] = bx*vx + by*vy + bz*vz;
  }
}
