#pragma once

#include <array>
#include <cstdint>

#include "fsfw/objectmanager/SystemObject.h"
#include "fsfw/tasks/ExecutableObjectIF.h"
#include "fsfw/objectmanager/ObjectManager.h"
#include "fsfw/devicehandlers/DeviceHandlerIF.h"
#include "fsfw/action/ActionMessage.h"
#include "fsfw/ipc/MessageQueueSenderIF.h"
#include "fsfw/storagemanager/StorageManagerIF.h"
#include "fsfw/timemanager/Clock.h"

#include "Guidance.h"
#include "Estimator.h"
#include "RwAllocator.h"

// Minimal ACS controller that drives ReactionWheelsHandler via CMD_SET_SPEED (0x01).
// It implements a rate-PD controller (D-only by default) and a trivial allocator.
// This is intentionally simple

class AcsController : public SystemObject, public ExecutableObjectIF {
 public:
  AcsController(object_id_t oid, std::array<object_id_t, 4> rwOids);

  // SystemObject hook: resolve dependencies (queues, stores)
  ReturnValue_t initialize() override;

  // ExecutableObjectIF: called each task cycle
  ReturnValue_t performOperation(uint8_t opCode) override;

 private:
  // Config (tune as needed)
  float loopPeriodHintSec_ = 0.05f;     // If dt cannot be computed, fall back to this
  float kd_[3]             = {0.01f, 0.01f, 0.01f};  // simple D gains [Nms]
  float tauMax_[3]         = {0.03f, 0.03f, 0.03f};  // axis torque clamp [N·m]
  float jWheel_[4]         = {1.0e-3f, 1.0e-3f, 1.0e-3f, 1.0e-3f}; // wheel inertia [kg m^2]
  int16_t maxRpm_          = 4000;      // match your handler limit
  float maxSlewRpmPerS_    = 500.0f;    // simple slew limiter

  // RW endpoints
  std::array<object_id_t, 4> rwOids_{};
  std::array<MessageQueueId_t, 4> rwMq_{};
  std::array<int16_t, 4> lastRpmCmd_{};

  // Stores
  StorageManagerIF* ipcStore_{nullptr};

  // Simple submodules
  Guidance guidance_;
  Estimator estimator_;
  RwAllocator allocator_;

  // Timing
  uint32_t lastUptimeMs_{0};

  // Helpers
  static inline float clamp(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi ? hi : v);
  }
  static inline int16_t clampRpm(int v, int16_t lo, int16_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return static_cast<int16_t>(v);
  }
  static inline float radpsToRpm(float w) { return w * 60.0f / (2.0f * 3.1415926535f); }

  void sendRpmToWheel(size_t idx, int16_t rpm);
};
