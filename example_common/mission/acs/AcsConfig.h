// example_common/mission/acs/AcsConfig.h
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
  float sigmaG_dps_sqrtHz{0.03f};
  // Bias random-walk density [deg/s/√s] (per √s step)
  float sigmaB_dps_sqrtHz{5e-4f};
  // Optional BLWN cutoff [Hz]; 0 => disabled
  float fc_blwn_hz{25.0f};
};

struct Limits {
  // Max per-axis body torque magnitude [mNm]
  float torqueMax_mNm{20.0f};
  // Max torque rate (per axis) [mNm/s]
  float torqueRateLimit_mNm_per_s{200.0f};
};

struct Timing {
  // Controller step in milliseconds; used for scheduling and discretization
  uint32_t dtMs{50};
  constexpr float dt() const { return 1e-3f * static_cast<float>(dtMs); }
};

// 3x4 RW axes matrix in column-major (b1|b2|b3|b4), body frame, unit vectors
using Axes3x4 = std::array<float, 12>;

struct Config {
  Timing   timing{};
  InertiaDiag inertia{};
  Axes3x4  Bcols{};
  Limits   limits{};
  GyroNoise gyro{};
};

// Default config values – adjust in your mission code if needed.
inline constexpr Config DEFAULT{
    /* timing  */ Timing{50},
    /* inertia */ InertiaDiag{0.02f, 0.02f, 0.03f},
    /* Bcols   */ Axes3x4{{
        1.0f, 0.0f, 0.0f,   // b1
        0.0f, 1.0f, 0.0f,   // b2
        0.0f, 0.0f, 1.0f,   // b3
        0.0f, 0.0f, 1.0f    // b4 (duplicate Z for demo; replace with real axis)
    }},
    /* limits  */ Limits{20.0f, 200.0f},
    /* gyro    */ GyroNoise{0.03f, 5e-4f, 25.0f}
};

} // namespace acs
