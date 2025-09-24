#pragma once

#include <array>
#include <cstdint>

/**
 * Central ACS configuration used by controller, estimator and allocator.
 * Keep comments in English.
 */
namespace acs {

struct InertiaDiag {
  float Jx{};
  float Jy{};
  float Jz{};
};

struct GyroNoise {
  // White noise density [deg/s/√Hz]
  float sigmaG_dps_sqrtHz{0.0f};
  // Bias random-walk density [deg/s/√s] (per √s step)
  float sigmaB_dps_sqrtHz{0.0f};
  // Optional BLWN cutoff [Hz]; 0 => disabled
  float fc_blwn_hz{0.0f};
};

struct Limits {
  // Max per-axis body torque magnitude [mNm]
  float torqueMax_mNm{5.0f};
  // Max torque rate (per axis) [mNm/s] – controller will enforce per step
  float torqueRateLimit_mNm_per_s{5.0f}; // -> 1.0 mNm per 200 ms step @ 5 Hz
};

struct Timing {
  // Controller step in milliseconds; used for scheduling and discretization
  uint32_t dtMs{200};
  constexpr float dt() const { return 1e-3f * static_cast<float>(dtMs); }
};

// 3x4 RW axes matrix in column-major (b1|b2|b3|b4), body frame, unit vectors
using Axes3x4 = std::array<float, 12>;

struct Config {
  Timing     timing{};
  InertiaDiag inertia{};
  Axes3x4    Bcols{};
  Limits     limits{};
  GyroNoise  gyro{};
};

// Default config values – adjust in your mission code if needed.
inline constexpr Config DEFAULT{
    /* timing  */ Timing{200},                         // 5 Hz controller step
    /* inertia */ InertiaDiag{0.8f, 0.8f, 0.8f},
    /* Bcols   */
    // Column-major: (b1.x, b1.y, b1.z,  b2.x, b2.y, b2.z,  b3.x, b3.y, b3.z,  b4.x, b4.y, b4.z)
    // Debug layout: wheel 1..3 align with body axes; wheel 4 disabled.
    Axes3x4{{
      1.0f, 0.0f, 0.0f,   // b1 -> +X
      0.0f, 1.0f, 0.0f,   // b2 -> +Y
      0.0f, 0.0f, 1.0f,   // b3 -> +Z
      0.0f, 0.0f, 0.0f    // b4 -> unused
    }},
    /* limits  */ Limits{5.0f, 5.0f},
    /* gyro    */ GyroNoise{0.0f, 0.0f, 0.0f}
};

} // namespace acs
