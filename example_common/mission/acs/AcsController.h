#pragma once

#include <array>
#include <atomic>
#include <cstdint>

#include "fsfw/objectmanager/SystemObject.h"
#include "fsfw/returnvalues/returnvalue.h"
#include "fsfw/tasks/ExecutableObjectIF.h"
#include "fsfw/serviceinterface/ServiceInterfaceStream.h"

#include "fsfw/ipc/QueueFactory.h"
#include "fsfw/ipc/MessageQueueIF.h"
#include "fsfw/ipc/MessageQueueSenderIF.h"
#include "fsfw/objectmanager/ObjectManager.h"
#include "fsfw/storagemanager/StorageManagerIF.h"
#include "fsfw/storagemanager/storeAddress.h"
#include "fsfw/devicehandlers/DeviceHandlerIF.h"

#include "AcsConfig.h"
#include "Estimator.h"
#include "Guidance.h"
#include "RwAllocator.h"

// ACS high level controller
// - Drives body torque commands based on attitude error and rate feedback
// - Allocates body torque to reaction wheels
// - Exchanges TC/TM with wheel device handlers via FSFW message queues

// Forward declare to avoid include cycles
class ReactionWheelsHandler;

// Snapshot struct for typed TM (diagnostics)
struct AcsDiagSnapshot {
  uint8_t version{1};                        // TM version
  bool enabled{false};                       // controller enable flag

  std::array<float,4> qRef{ {1,0,0,0} };     // target attitude (unit quaternion)
  std::array<float,4> qTrue{ {1,0,0,0} };    // estimator true attitude
  std::array<float,3> wTrue{ {0,0,0} };      // true body rates [rad/s]
  std::array<float,3> wMeas{ {0,0,0} };      // measured body rates to controller [rad/s]

  std::array<float,3> Kp{ {0,0,0} };         // exposed as attitude gains (outer loop)
  std::array<float,3> Kd{ {0,0,0} };         // exposed as rate P-gains (inner loop)

  std::array<float,3> tauDes{ {0,0,0} };     // desired body torque [mNm]
  std::array<float,4> tauWheelCmd{ {0,0,0,0} }; // commanded wheel torques [mNm]

  uint32_t dtMs{50};                         // control step in ms (for HK)
};

class AcsController : public SystemObject, public ExecutableObjectIF {
 public:
  // Construct controller with 4 wheel object IDs and static config
  AcsController(object_id_t objectId,
                std::array<object_id_t, 4> wheelIds,
                const acs::Config& cfg = acs::DEFAULT);

  // Called by task each cycle. Runs estimation, control, allocation and actuation.
  ReturnValue_t performOperation(uint8_t opCode) override;

  // Enable or disable closed-loop control
  void enable(bool on) { enabled_.store(on); }

  // Set new target attitude (normalized inside Guidance)
  void setTargetAttitude(const std::array<float,4>& q_ref) { guidance_.setTargetQuat(q_ref); }

  // Keep for TM/config interfaces (maps to visible fields only)
  void setGains(const std::array<float,3>& Kp, const std::array<float,3>& Kd) { Kp_ = Kp; Kd_ = Kd; }

  // Build diagnostic snapshot for typed TM
  AcsDiagSnapshot getDiag() const;

 private:
  // Static config and wiring
  acs::Config cfg_;                           // tuning and limits
  std::array<object_id_t, 4> rwIds_{};        // wheel device object IDs
  std::atomic<bool> enabled_{false};          // controller on/off flag

  // Submodules
  Estimator   estimator_;                     // propagates attitude and simulates gyro
  Guidance    guidance_;                      // holds target attitude and rate feed-forward
  RwAllocator allocator_;                     // maps body torque to wheel torques

  // Exposed copies for TM (mapped from cfg_.ctrl)
  std::array<float,3> Kp_ = {2.0f, 2.0f, 2.0f}; // attitude gains (outer loop)
  std::array<float,3> Kd_ = {0.5f, 0.5f, 0.5f}; // rate P-gains   (inner loop)

  // Last commands and errors
  std::array<float,3>  tauDes_{ {0,0,0} };      // desired body torque [mNm]
  std::array<float,4>  tauWheelCmd_{ {0,0,0,0} }; // last wheel commands [mNm]
  std::array<float,4>  qErr_{ {1,0,0,0} };      // last attitude error quaternion
  uint32_t             dtMs_{50};               // cached dt in ms

  // Messaging resources (FSFW)
  MessageQueueIF*    myQueue_{nullptr};         // own queue to receive replies if needed
  StorageManagerIF*  ipcStore_{nullptr};        // shared store for action payloads

  // Math helpers
  static void quatConj_(const float q[4], float qc[4]);          // conjugate quaternion
  static void quatMul_(const float a[4], const float b[4], float out[4]); // Hamilton product
  static void clamp3_(float v[3], float maxAbs);                 // clamp vector to +/- maxAbs
  void rateLimit3_(float v[3], const float vPrev[3],
                   float rateLimitPerSec, float dt);             // per-axis rate limit

  // Send torque commands to wheels (deadband and hysteresis handled inside)
  void sendWheelTorques_(const std::array<float,4>& tauWheel_mNm);
};
