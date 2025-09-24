#include "Estimator.h"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace { constexpr float PI_F = 3.14159265358979323846f; }

// Constructor: copy static config, set B matrix, initialize states
Estimator::Estimator(const acs::Config& cfg)
    : cfg_(cfg),
      B_(cfg.Bcols),
      q_{1,0,0,0},          // unit quaternion (w,x,y,z)
      omega_{0,0,0},        // true body rates [rad/s]
      bias_{0,0,0},         // gyro bias state [rad/s]
      blwn_{0,0,0},         // band-limited noise state [rad/s]
      omegaMeas_{0,0,0} {}  // synthesized gyro measurement [rad/s]

// Update RW axes matrix (3x4, column-major b1|b2|b3|b4)
void Estimator::setAxes(const acs::Axes3x4& Bcols) { B_ = Bcols; }

// Run one propagation step
// tauW_mNm: wheel torques [mNm], tauExt_mNm: external body torque [mNm]
void Estimator::step(const std::array<float,4>& tauW_mNm, const std::array<float,3>& tauExt_mNm) {
  // Map wheel torques to body torque: tau_body = -B * tauWheel  (units: mNm -> N*m)
  float tauB[3] = {0,0,0};
  for (int i = 0; i < 4; ++i) {
    const float tw_Nm = 1e-3f * tauW_mNm[i];     // mNm -> N*m
    tauB[0] -= tw_Nm * B_[0 + 3*i];
    tauB[1] -= tw_Nm * B_[1 + 3*i];
    tauB[2] -= tw_Nm * B_[2 + 3*i];
  }
  // Add external torques (mNm -> N*m)
  tauB[0] += 1e-3f * tauExt_mNm[0];
  tauB[1] += 1e-3f * tauExt_mNm[1];
  tauB[2] += 1e-3f * tauExt_mNm[2];

  const float Ts = cfg_.timing.dt();             // sample time [s]
  const float Jx = cfg_.inertia.Jx, Jy = cfg_.inertia.Jy, Jz = cfg_.inertia.Jz;

  // Rigid body rate dynamics (Runge Kutta 2)
  auto f = [&](const float w[3], float out[3]) {
    const float Jw[3] = {Jx*w[0], Jy*w[1], Jz*w[2]};
    // omega cross (J*omega)
    const float cross[3] = {
        w[1]*Jw[2] - w[2]*Jw[1],
        w[2]*Jw[0] - w[0]*Jw[2],
        w[0]*Jw[1] - w[1]*Jw[0]
    };
    // Euler equations: J * domega = tau - omega x (J*omega)
    out[0] = (tauB[0] - cross[0]) / Jx;
    out[1] = (tauB[1] - cross[1]) / Jy;
    out[2] = (tauB[2] - cross[2]) / Jz;
  };
  float k1[3]; f(omega_, k1);                    // slope at current state
  const float wmid[3] = {                        // mid-step state
      omega_[0] + 0.5f*Ts*k1[0],
      omega_[1] + 0.5f*Ts*k1[1],
      omega_[2] + 0.5f*Ts*k1[2]
  };
  float k2[3]; f(wmid, k2);                      // slope at mid-step
  omega_[0] += Ts * k2[0];                       // integrate omega
  omega_[1] += Ts * k2[1];
  omega_[2] += Ts * k2[2];

  // Attitude kinematics (quaternion, passive frame)
  const float wx = omega_[0], wy = omega_[1], wz = omega_[2];
  const float dq[4] = {
      -0.5f*( q_[1]*wx + q_[2]*wy + q_[3]*wz ),
       0.5f*( q_[0]*wx + q_[2]*wz - q_[3]*wy ),
       0.5f*( q_[0]*wy - q_[1]*wz + q_[3]*wx ),
       0.5f*( q_[0]*wz + q_[1]*wy - q_[2]*wx )
  };
  for (int i = 0; i < 4; ++i) q_[i] += Ts * dq[i]; // integrate quaternion
  quatNormalize_(q_);                              // keep unit length

  // Gyro measurement model
  // With GyroNoise = 0 this reduces to omegaMeas = omegaTrue.
  const float d2r  = PI_F / 180.0f;                // deg/s -> rad/s
  const float sigG = (cfg_.gyro.sigmaG_dps_sqrtHz * d2r) * std::sqrt(1.0f / Ts); // white noise
  const float sigB = (cfg_.gyro.sigmaB_dps_sqrtHz * d2r) * std::sqrt(Ts);        // bias RW

  // Bias random walk
  for (int i = 0; i < 3; ++i) {
    bias_[i] += sigB * normalRand_();
  }

  // Additive noise on rates
  float meas[3] = {
      omega_[0] + bias_[0] + sigG * normalRand_(),
      omega_[1] + bias_[1] + sigG * normalRand_(),
      omega_[2] + bias_[2] + sigG * normalRand_()
  };

  // Optional band-limited white noise (1st order low-pass on white noise)
  if (cfg_.gyro.fc_blwn_hz > 0.0f) {
    const float alpha = 1.0f - std::exp(-2.0f * PI_F * cfg_.gyro.fc_blwn_hz * Ts);
    for (int i = 0; i < 3; ++i) {
      blwn_[i]      = blwn_[i] + alpha * (meas[i] - blwn_[i]);
      omegaMeas_[i] = blwn_[i];
    }
  } else {
    for (int i = 0; i < 3; ++i) omegaMeas_[i] = meas[i];
  }
}

// Reset full estimator state (q, omega, bias, noise states, meas)
void Estimator::reset(const float q_init[4], const float w_init[3]) {
  for (int i = 0; i < 4; ++i) q_[i] = q_init[i];
  for (int i = 0; i < 3; ++i) {
    omega_[i]     = w_init[i];
    bias_[i]      = 0.0f;
    blwn_[i]      = 0.0f;
    omegaMeas_[i] = w_init[i];
  }
  quatNormalize_(q_);
}

// --- private helpers ---

// Draw one sample from N(0,1) using Box-Muller with a small LCG
static float urand01_(uint32_t& s) {
  s = 1664525u*s + 1013904223u;
  return (s >> 8) * (1.0f / 16777216.0f); // [0,1)
}

float Estimator::normalRand_() {
  static uint32_t s = 0x12345678u;               // deterministic seed for repeatability
  float u1 = urand01_(s); if (u1 < 1e-7f) u1 = 1e-7f; // avoid log(0)
  float u2 = urand01_(s);
  return std::sqrt(-2.0f * std::log(u1)) * std::cos(6.28318530718f * u2);
}

// Normalize quaternion in place; if norm underflows, set identity
void Estimator::quatNormalize_(float q[4]) {
  const float n2 = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
  if (n2 > 1e-24f) {
    const float inv = 1.0f / std::sqrt(n2);
    q[0] *= inv; q[1] *= inv; q[2] *= inv; q[3] *= inv;
  } else {
    q[0] = 1.0f; q[1] = q[2] = q[3] = 0.0f;
  }
}
