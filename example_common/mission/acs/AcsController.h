// example_common/mission/acs/AcsController.h
#pragma once

#include <array>
#include <atomic>
#include <cstdint>

#include "fsfw/objectmanager/SystemObject.h"
#include "fsfw/returnvalues/returnvalue.h"
#include "fsfw/tasks/ExecutableObjectIF.h"

/**
 * Minimal ACS controller scaffolding:
 * - Holds enable flag.
 * - Provides a diagnostic snapshot for typed TM.
 * - Periodic performOperation() can be extended later (guidance/estimator/controller/allocator).
 */
struct AcsDiagSnapshot {
  uint8_t version{1};     // typed TM format version
  bool    enabled{false};
  float   kd[3]{0.0f, 0.0f, 0.0f};
  float   tauDes[3]{0.0f, 0.0f, 0.0f};    // desired body torques [mNm]
  float   tauWheel[4]{0.0f, 0.0f, 0.0f, 0.0f}; // per-wheel torque [mNm]
  uint32_t dt_ms{50};     // control step [ms]
};

class AcsController : public SystemObject, public ExecutableObjectIF {
 public:
  AcsController(object_id_t objectId, std::array<object_id_t, 4> wheelIds);

  // ExecutableObjectIF
  ReturnValue_t performOperation(uint8_t opCode = 0) override;

  // Enable/disable ACS control loop
  void setEnabled(bool en) { enabled_.store(en, std::memory_order_relaxed); }
  bool isEnabled() const { return enabled_.load(std::memory_order_relaxed); }

  // Fill diagnostic snapshot (thread-safe copy)
  void fillDiagSnapshot(AcsDiagSnapshot& out) const;

  // Optionally expose a way to update controller gains
  void setKd(float kx, float ky, float kz);

 private:
  std::array<object_id_t, 4> rwIds_{};
  std::atomic<bool> enabled_{false};

  // Simple internal state (would be updated by your real control pipeline)
  float kd_[3]{0.0f, 0.0f, 0.0f};
  float tauDes_[3]{0.0f, 0.0f, 0.0f};
  float tauWheel_[4]{0.0f, 0.0f, 0.0f};
  uint32_t dtMs_{50}; // example 20 Hz

  // Helper to simulate/update some values (placeholder)
  void updateInternalDemo_();
};
