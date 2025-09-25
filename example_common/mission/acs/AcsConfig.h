/**
 * Central ACS configuration used by controller, estimator and allocator.
 */

#pragma once

#include <array>
#include <cstdint>

namespace acs {

// ============================== Types =======================================
// Column-major reaction wheel axes matrix (3x4): (b1 | b2 | b3 | b4).
// Each column is a unit vector of a wheel spin axis in the BODY frame.
using Axes3x4 = std::array<float, 12>;

// Default axes layout (single source of truth):
// b1 -> +X, b2 -> +Y, b3 -> +Z, b4 unused (0-vector)
// This is handy for bring-up; missions can override Bcols in their Config.
inline constexpr Axes3x4 B_AXES_IDENTITY{
  1.0f, 0.0f, 0.0f,  // b1
  0.0f, 1.0f, 0.0f,  // b2
  0.0f, 0.0f, 1.0f,  // b3
  0.0f, 0.0f, 0.0f   // b4 (unused)
};

// ======================== Plant & Sensor Models =============================

// Principal body inertia (diagonal) in kg*m^2
struct InertiaDiag {
  float Jx{0.8f};  // inertia around +X
  float Jy{0.8f};  // inertia around +Y
  float Jz{0.8f};  // inertia around +Z
};

// Simple gyro noise model used by the estimator
struct GyroNoise {
  float sigmaG_dps_sqrtHz{0.0f}; // white noise density [deg/s/sqrt(Hz)]
  float sigmaB_dps_sqrtHz{0.0f}; // bias random-walk [deg/s/sqrt(s)] (per sqrt(s) step)
  float fc_blwn_hz{0.0f};        // 1st-order BLWN cutoff [Hz]; 0 -> disabled
};

// Physical / command limits enforced by the controller
struct Limits {
  float torqueMax_mNm{5.0f};              // max |tau_body| per axis [mNm]
  float torqueRateLimit_mNm_per_s{5.0f};  // max d(tau)/dt per axis [mNm/s]
  // Example: at 5 Hz (dt = 0.2 s), 5 mNm/s -> 1.0 mNm increment per step.
};

// Control loop timing
struct Timing {
  uint32_t dtMs{200}; // controller step [ms] (5 Hz)
  constexpr float dt() const { return 1e-3f * static_cast<float>(dtMs); } // step [s]
};

// =========================== Controller Tuning ==============================

// Outer loop: attitude P -> rate reference (per axis)
// wRef = kAtt * e  with small-angle attitude error e = 2*q_err_vec, clamped to wRefMax*
struct OuterLoop {
  float kAttX{0.08f};  // attitude gain X [1/s]
  float kAttY{0.08f};  // attitude gain Y [1/s]
  float kAttZ{0.08f};  // attitude gain Z [1/s]
  float wRefMaxX{0.04f}; // clamp |wRef.x| [rad/s]  (~ 2.3 deg/s)
  float wRefMaxY{0.04f}; // clamp |wRef.y| [rad/s]
  float wRefMaxZ{0.04f}; // clamp |wRef.z| [rad/s]
};

// Inner loop: rate PI with anti-windup and omega low-pass
// tau_unsat = Kpw * (wRef - w) + i ; i' = Kiw * (wRef - w) + Kaw * (tau_sat - tau_unsat)
struct InnerPI {
  float KpwX{100.0f};  // rate-P gain X [mNm/(rad/s)]
  float KpwY{100.0f};  // rate-P gain Y [mNm/(rad/s)]
  float KpwZ{100.0f};  // rate-P gain Z [mNm/(rad/s)]
  float KiwX{20.0f};   // rate-I gain X [mNm/rad]
  float KiwY{20.0f};   // rate-I gain Y [mNm/rad]
  float KiwZ{20.0f};   // rate-I gain Z [mNm/rad]
  float Kaw{2.0f};     // anti-windup back-calc gain [1/s]
  float omegaLpFcHz{1.0f}; // omega-meas 1st-order LPF cutoff [Hz]; 0 -> disable LPF
};

// Outgoing command hygiene / deadbands
struct Hygiene {
  float minCmdAbs_mNm{0.25f}; // skip tiny |tau_wheel|; send one-time zero if needed
};

// Debug & logging verbosity
struct Debug {
  uint32_t logEveryN{11}; // print one-line attitude log every N loops (0 -> off)
  bool     verbose{false}; // extra PI debug per loop (verbose)
};

// Optional forced wheel torque for bring-up (overrides allocator when enabled)
struct ForceTorque {
  bool    enable{false};    // if true, bypass allocator and send fixed torques
  float   torque_mNm{0.0f}; // fixed torque magnitude [mNm]
  uint8_t wheelMask{0x00};  // bit 0..3 select wheels 0..3
};

// Group controller parameters
struct Control {
  OuterLoop outer{}; // attitude -> rate ref
  InnerPI   inner{}; // rate PI
};

// ============================== Top-Level ===================================
// Full ACS configuration shared by controller, estimator and allocator
// All members have conservative defaults; override only what you need
struct Config {
  Timing      timing{};               // dt = 200 ms (5 Hz)
  InertiaDiag inertia{};              // Jx=Jy=Jz=0.8 kg*m^2
  Axes3x4     Bcols{B_AXES_IDENTITY}; // default axes (identity + unused 4th)
  Limits      limits{};               // 5 mNm, 5 mNm/s
  GyroNoise   gyro{};                 // ideal gyro (no noise)
  Control     ctrl{};                 // outer + inner loop gains
  Hygiene     hygiene{};              // min command = 0.25 mNm
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