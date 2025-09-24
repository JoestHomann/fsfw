#include "Estimator.h"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace { constexpr float PI_F = 3.14159265358979323846f; }

Estimator::Estimator(const acs::Config& cfg)
    : cfg_(cfg),
      B_(cfg.Bcols),
      q_{1,0,0,0},
      omega_{0,0,0},
      bias_{0,0,0},
      blwn_{0,0,0},
      omegaMeas_{0,0,0} {}

void Estimator::setAxes(const acs::Axes3x4& Bcols) { B_ = Bcols; }

void Estimator::step(const std::array<float,4>& tauW_mNm, const std::array<float,3>& tauExt_mNm) {
  // Map wheel torques to body torque: tau_body = -B * tauWheel  (units: mNm -> N·m)
  float tauB[3] = {0,0,0};
  for (int i = 0; i < 4; ++i) {
    const float tw_Nm = 1e-3f * tauW_mNm[i];
    tauB[0] -= tw_Nm * B_[0 + 3*i];
    tauB[1] -= tw_Nm * B_[1 + 3*i];
    tauB[2] -= tw_Nm * B_[2 + 3*i];
  }
  // Add external torques (mNm -> N·m)
  tauB[0] += 1e-3f * tauExt_mNm[0];
  tauB[1] += 1e-3f * tauExt_mNm[1];
  tauB[2] += 1e-3f * tauExt_mNm[2];

  const float Ts = cfg_.timing.dt();
  const float Jx = cfg_.inertia.Jx, Jy = cfg_.inertia.Jy, Jz = cfg_.inertia.Jz;

  // Rigid body rates dynamics (2nd-order RK)
  auto f = [&](const float w[3], float out[3]) {
    const float Jw[3] = {Jx*w[0], Jy*w[1], Jz*w[2]};
    const float cross[3] = {
        w[1]*Jw[2] - w[2]*Jw[1],
        w[2]*Jw[0] - w[0]*Jw[2],
        w[0]*Jw[1] - w[1]*Jw[0]
    };
    out[0] = (tauB[0] - cross[0]) / Jx;
    out[1] = (tauB[1] - cross[1]) / Jy;
    out[2] = (tauB[2] - cross[2]) / Jz;
  };
  float k1[3]; f(omega_, k1);
  const float wmid[3] = {
      omega_[0] + 0.5f*Ts*k1[0],
      omega_[1] + 0.5f*Ts*k1[1],
      omega_[2] + 0.5f*Ts*k1[2]
  };
  float k2[3]; f(wmid, k2);
  omega_[0] += Ts * k2[0];
  omega_[1] += Ts * k2[1];
  omega_[2] += Ts * k2[2];

  // Attitude kinematics (quaternion, passive/spacecraft-to-inertial)
  const float wx = omega_[0], wy = omega_[1], wz = omega_[2];
  const float dq[4] = {
      -0.5f*( q_[1]*wx + q_[2]*wy + q_[3]*wz ),
       0.5f*( q_[0]*wx + q_[2]*wz - q_[3]*wy ),
       0.5f*( q_[0]*wy - q_[1]*wz + q_[3]*wx ),
       0.5f*( q_[0]*wz + q_[1]*wy - q_[2]*wx )
  };
  for (int i = 0; i < 4; ++i) q_[i] += Ts * dq[i];
  quatNormalize_(q_);

  // Gyro measurement model: white + bias random walk + optional BLWN shaping
  const float d2r  = PI_F / 180.0f;
  const float sigG = (cfg_.gyro.sigmaG_dps_sqrtHz * d2r) * std::sqrt(1.0f / Ts);
  const float sigB = (cfg_.gyro.sigmaB_dps_sqrtHz * d2r) * std::sqrt(Ts);

  for (int i = 0; i < 3; ++i) {
    bias_[i]  += sigB * normalRand_();
  }

  float meas[3] = {
      omega_[0] + bias_[0] + sigG * normalRand_(),
      omega_[1] + bias_[1] + sigG * normalRand_(),
      omega_[2] + bias_[2] + sigG * normalRand_()
  };

  if (cfg_.gyro.fc_blwn_hz > 0.0f) {
    const float alpha = 1.0f - std::exp(-2.0f * PI_F * cfg_.gyro.fc_blwn_hz * Ts);
    for (int i = 0; i < 3; ++i) {
      blwn_[i]     = blwn_[i] + alpha * (meas[i] - blwn_[i]);
      omegaMeas_[i] = blwn_[i];
    }
  } else {
    for (int i = 0; i < 3; ++i) omegaMeas_[i] = meas[i];
  }
}

void Estimator::reset(const float q_init[4], const float w_init[3]) {
  for (int i = 0; i < 4; ++i) q_[i] = q_init[i];
  for (int i = 0; i < 3; ++i) {
    omega_[i]    = w_init[i];
    bias_[i]     = 0.0f;
    blwn_[i]     = 0.0f;
    omegaMeas_[i]= w_init[i];
  }
  quatNormalize_(q_);
}

// --- private helpers ---

float Estimator::normalRand_() {
  // Box–Muller using a tiny LCG for repeatability (no <random> dependency)
  static uint32_t s = 0x12345678u;
  auto urand = [&](){
    s = 1664525u*s + 1013904223u;
    return (s >> 8) * (1.0f / 16777216.0f); // [0,1)
  };
  float u1 = urand(); if (u1 < 1e-7f) u1 = 1e-7f;
  float u2 = urand();
  return std::sqrt(-2.0f * std::log(u1)) * std::cos(6.28318530718f * u2);
}

void Estimator::quatNormalize_(float q[4]) {
  const float n2 = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
  if (n2 > 1e-24f) {
    const float inv = 1.0f / std::sqrt(n2);
    q[0] *= inv; q[1] *= inv; q[2] *= inv; q[3] *= inv;
  } else {
    q[0] = 1.0f; q[1] = q[2] = q[3] = 0.0f;
  }
}
