#pragma once

#include <array>
#include <atomic>
#include <cstdint>

#include "fsfw/objectmanager/SystemObject.h"
#include "fsfw/returnvalues/returnvalue.h"
#include "fsfw/tasks/ExecutableObjectIF.h"
#include "fsfw/serviceinterface/ServiceInterfaceStream.h"

// NEW: messaging + storage + device IF
#include "fsfw/ipc/QueueFactory.h"
#include "fsfw/ipc/MessageQueueIF.h"
#include "fsfw/ipc/MessageQueueSenderIF.h"
#include "fsfw/objectmanager/ObjectManager.h"
#include "fsfw/storagemanager/StorageManagerIF.h"
#include "fsfw/storagemanager/storeAddress.h"
#include "fsfw/devicehandlers/DeviceHandlerIF.h"
#include "fsfw/devicehandlers/DeviceHandlerMessage.h"

#include "AcsConfig.h"
#include "Estimator.h"
#include "Guidance.h"
#include "RwAllocator.h"

// Forward declare to avoid include cycles
class ReactionWheelsHandler;

struct AcsDiagSnapshot {
  uint8_t version{1};
  bool enabled{false};
  std::array<float,4> qRef{ {1,0,0,0} };
  std::array<float,4> qTrue{ {1,0,0,0} };
  std::array<float,3> wTrue{ {0,0,0} };
  std::array<float,3> wMeas{ {0,0,0} };
  std::array<float,3> Kp{ {0,0,0} };
  std::array<float,3> Kd{ {0,0,0} };
  std::array<float,3> tauDes{ {0,0,0} };        // body [mNm]
  std::array<float,4> tauWheelCmd{ {0,0,0,0} }; // wheel [mNm]
  uint32_t dtMs{50};
};

class AcsController : public SystemObject, public ExecutableObjectIF {
 public:
  AcsController(object_id_t objectId,
                std::array<object_id_t, 4> wheelIds,
                const acs::Config& cfg = acs::DEFAULT);

  ReturnValue_t performOperation(uint8_t opCode) override;

  void enable(bool on) { enabled_.store(on); }
  void setTargetAttitude(const std::array<float,4>& q_ref) { guidance_.setTargetQuat(q_ref); }

  void setGains(const std::array<float,3>& Kp, const std::array<float,3>& Kd) { Kp_ = Kp; Kd_ = Kd; }

  AcsDiagSnapshot getDiag() const;

 private:
  acs::Config cfg_;
  std::array<object_id_t, 4> rwIds_{};
  std::atomic<bool> enabled_{false};

  Estimator   estimator_;
  Guidance    guidance_;
  RwAllocator allocator_;

  std::array<float,3> Kp_ = {2.0f, 2.0f, 2.0f};
  std::array<float,3> Kd_ = { 0.5f,  0.5f,  0.5f};
  std::array<float,3> tauDes_{ {0,0,0} };
  std::array<float,4> tauWheelCmd_{ {0,0,0,0} };
  std::array<float,4> qErr_{ {1,0,0,0} };
  uint32_t dtMs_{50};

  // messaging resources
  MessageQueueIF*    myQueue_{nullptr};
  StorageManagerIF*  ipcStore_{nullptr};

  // Helpers
  static void quatConj_(const float q[4], float qc[4]);
  static void quatMul_(const float a[4], const float b[4], float out[4]);
  static void clamp3_(float v[3], float maxAbs);
  void rateLimit3_(float v[3], const float vPrev[3], float rateLimitPerSec, float dt);

  // RW hookup
  void sendWheelTorques_(const std::array<float,4>& tauWheel_mNm);
};

// ---- Debug / tuning macros ----
#ifndef ACS_VERBOSE
#define ACS_VERBOSE 0
#endif

#ifndef ACS_DEBUG_EVERY
// Wie oft Debugzeilen gedruckt werden (alle N Zyklen)
#define ACS_DEBUG_EVERY 10
#endif

#ifndef ACS_TORQUE_MIN_CMD_MNM
// Mindest-Befehl pro Rad in mNm vor der Rundung auf int16 (0 => deaktiviert)
#define ACS_TORQUE_MIN_CMD_MNM 1.0f
#endif