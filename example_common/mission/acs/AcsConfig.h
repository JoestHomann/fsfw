#pragma once

#include <array>
#include <cstdint>

/*
 * AcsConfig.h - ACS Configuration
 * 
 *   Central ACS configuration used by AcsController, Estimator and RwAllocator.
 *   Defines physical parameters, control tuning, limits, and timing.
 * 
 */


namespace acs {

// ============================== RW Configuration =======================================
// Defines the orientation of the RWs in the body frame.
// Each column is a unit vector of a RW spin axis.
// Current configuration represents a pyramid array (Geshnizjani, p. 22)
using Axes3x4 = std::array<float, 12>;

inline constexpr float INV_SQRT3 = 0.5773502691896258f; // = approx. 1/sqrt(3)

inline constexpr Axes3x4 B_AXES_IDENTITY{
  1.0f, 0.0f, 0.0f,                    // RW1
  0.0f, 1.0f, 0.0f,                    // RW2
  0.0f, 0.0f, 1.0f,                    // RW3
  INV_SQRT3, INV_SQRT3, INV_SQRT3      // RW4
};

// ======================== Plant & Sensor Models =============================

// Body inertia (diagonal) in kg*m^2
struct InertiaDiag {
  float Jx{0.8f};  // Inertia around X
  float Jy{0.8f};  // Inertia around Y
  float Jz{0.8f};  // Inertia around Z
};

// Gyro noise model used by the estimator
struct GyroNoise {
  float sigmaG_dps_sqrtHz{0.0f}; // White noise density [deg/s/sqrt(Hz)]
  float sigmaB_dps_sqrtHz{0.0f}; // Bias random-walk [deg/s/sqrt(s)] (per sqrt(s) step)
  float fc_blwn_hz{0.0f};        // 1st-order BLWN cutoff [Hz]
}; // Idea from Chapter 3.5.1 in Geshnizjani, p. 76ff

// Physical / command limits enforced by the controller
struct Limits {
  float torqueMax_mNm{5.0f};              // max body torque per axis [mNm]
  float torqueRateLimit_mNm_per_s{5.0f};  // max body torque change per axis [mNm/s]
  // Example: at 5 Hz (dt = 0.2 s), 5 mNm/s -> 1.0 mNm increment per step.
};

// Control loop timing
struct Timing {
  uint32_t dtMs{200}; // controller step time [ms] (5 Hz)
  constexpr float dt() const { return 1e-3f * static_cast<float>(dtMs); } // step [s]
};

// =========================== Controller Tuning ==============================

// Outer loop P: Attitude error -> rate reference (per axis)
struct OuterP {
  float kAttX{0.08f};  // attitude gain X [1/s]
  float kAttY{0.08f};  // attitude gain Y [1/s]
  float kAttZ{0.08f};  // attitude gain Z [1/s]
  float wRefMaxX{0.04f}; // clamp wRef.x [rad/s]
  float wRefMaxY{0.04f}; // clamp wRef.y [rad/s]
  float wRefMaxZ{0.04f}; // clamp wRef.z [rad/s]
};

// Inner loop PI: Rate error -> torque command (anti-windup + omega LPF)
struct InnerPI {
  float KpwX{100.0f};  // Rate-P gain X [mNm/(rad/s)]
  float KpwY{100.0f};  // Rate-P gain Y [mNm/(rad/s)]
  float KpwZ{100.0f};  // Rate-P gain Z [mNm/(rad/s)]
  float KiwX{20.0f};   // Rate-I gain X [mNm/rad]
  float KiwY{20.0f};   // Rate-I gain Y [mNm/rad]
  float KiwZ{20.0f};   // Rate-I gain Z [mNm/rad]
  float Kaw{2.0f};     // Anti-windup back-calc gain [1/s]
  float omegaLpFcHz{1.0f}; // omega-meas 1st-order LPF cutoff [Hz]
};

// Outgoing command deadband
struct Deadband {
  float minCmdAbs_mNm{0.25f}; // Min absolute torque command [mNm]
};

// Debug & logging verbosity
struct Debug {
  uint32_t logEveryN{11};  // Print one-line attitude log every N loops
  bool     verbose{false}; // Extra PI debug per loop (verbose)
};

// Forced wheel torque (overrides RwAllocator when enabled)
struct ForceTorque {
  bool    enable{false};    // If true, bypass allocator and send fixed torques
  float   torque_mNm{0.0f}; // Fixed torque magnitude [mNm]
  uint8_t wheelMask{0x00};  // 0x00 = none, 0x0F = all wheels, bit per wheel
};

// Group controller parameters (outer P for attitude, inner PI for rate)
struct Control {
  OuterP outer{}; // P: Attitude error -> rate reference (per axis)
  InnerPI   inner{}; // PI: Rate error -> torque command (anti-windup + omega LPF)
};

// ============================== Config struct ===================================
// Full ACS configuration shared by controller, estimator and allocator
struct Config {
  Timing      timing{};               // dt = 200 ms (5 Hz)
  InertiaDiag inertia{};              // Jx=Jy=Jz=0.8 kg*m^2
  Axes3x4     Bcols{B_AXES_IDENTITY}; // default axes (identity + unused 4th)
  Limits      limits{};               // 5 mNm, 5 mNm/s
  GyroNoise   gyro{};                 // ideal gyro (no noise)
  Control     ctrl{};                 // outer + inner loop gains
  Deadband    deadband{};             // min torque command = X mNm
  Debug       debug{};                // log every 11 loops, non-verbose
  ForceTorque force{};                // forced torque disabled
};

// Aggregated default instance (uses the in-class member defaults above)
inline constexpr Config DEFAULT{};

} // namespace acs

// Debug on/off switch (set to 1 to enable verbose logging)
#ifndef ACS_VERBOSE
#define ACS_VERBOSE 0
#endif