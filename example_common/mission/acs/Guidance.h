#pragma once

#include <array>

/*
 * Guidance.h - Attitude Guidance
 *
 *  Holds/updates the attitude reference quaternion for the ACS and
 *  provides a rate reference in BODY frame.
 *
 */

class Guidance {
 public:
  Guidance();

  // Set target attitude. Non-unit input will be normalized.
  void setTargetQuat(const std::array<float,4>& q_ref);

  // Read back current target quaternion (unit length)
  const std::array<float,4>& getTargetQuat() const { return qRef_; }

  // Provide rate command in BODY frame [rad/s]
  void getRateCmd(float outWcmd[3]) const;

 private:
  std::array<float,4> qRef_{ {1.0f, 0.0f, 0.0f, 0.0f} }; // identity quaternion

  // Normalize quaternion in-place; if norm is tiny, set to identity
  static void normalize_(std::array<float,4>& q);
};
