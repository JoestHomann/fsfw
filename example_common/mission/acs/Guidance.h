// example_common/mission/acs/Guidance.h
#pragma once
#include <array>

// Guidance module: holds target attitude and optional rate feed-forward

class Guidance {
 public:
  Guidance();

  // Set target attitude. Non-unit input will be normalized.
  void setTargetQuat(const std::array<float,4>& q_ref);

  // Read back current target quaternion (unit length)
  const std::array<float,4>& getTargetQuat() const { return qRef_; }

  // Advance guidance by one control step
  // For hold attitude mode this is not used, can be extended
  void update(float dt);

  // Provide rate command in BODY frame [rad/s]
  void getRateCmd(float outWcmd[3]) const;

 private:
  std::array<float,4> qRef_{ {1.0f, 0.0f, 0.0f, 0.0f} }; // identity quaternion

  // Normalize quaternion in-place; if norm is tiny, set to identity
  static void normalize_(std::array<float,4>& q);
};
