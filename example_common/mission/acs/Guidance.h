#pragma once

// Minimal guidance: provides commanded body rates (rad/s).
// Start with zeros; extend to slews or target spins later.

class Guidance {
 public:
  void update(float /*dt*/) {}
  void getRateCmd(float outWcmd[3]) const {
    outWcmd[0] = 0.0f; outWcmd[1] = 0.0f; outWcmd[2] = 0.0f;
  }
};
