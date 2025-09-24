// example_common/mission/acs/Guidance.h
#pragma once
#include <array>

/**
 * Holds the target attitude 
 */
class Guidance {
 public:
  Guidance();

  void setTargetQuat(const std::array<float,4>& q_ref);
  const std::array<float,4>& getTargetQuat() const { return qRef_; }

  void update(float dt);
  void getRateCmd(float outWcmd[3]) const;

 private:
  std::array<float,4> qRef_{ {1.0f,0.0f,0.0f,0.0f} };
  static void normalize_(std::array<float,4>& q);
};
