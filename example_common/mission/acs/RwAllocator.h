#pragma once
#include <array>
#include "AcsConfig.h"

/*
 * RwAllocator.h - Reaction-Wheel Torque Allocator
 *
 *   Maps calculated body torque to per-wheel torques using
 *   the reaction-wheel axes matrix.
 *
 */
class RwAllocator {
 public:
  // Construct with static ACS config (copies B from cfg by default)
  explicit RwAllocator(const acs::Config& cfg);

  // Update wheel axes matrix 
  void setAxes(const acs::Axes3x4& Bcols);

  // Read back current axes matrix
  const acs::Axes3x4& axes() const { return B_; }

  // Compute wheel torques from desired body torque
  void solve(const float tauDes[3], float tauWheel[4]) const;

 private:
  acs::Config  cfg_;  // static parameters (limits, timing, etc.)
  acs::Axes3x4 B_;    // RW axes matrix
};


//Debug switch to route all torque to wheel 1 only (x-axis)
#ifndef ACS_ALLOC_SINGLE_AXIS_DEBUG
#define ACS_ALLOC_SINGLE_AXIS_DEBUG 0
#endif