#pragma once

#include "commonObjects.h"
#include "fsfw/storagemanager/StorageManagerIF.h"
#include "fsfw/storagemanager/storeAddress.h"
#include "fsfw/tmtcservices/CommandingServiceBase.h"

#include <unordered_map>

// English comments!
// PUS Service 220 for Reaction Wheel & ACS control.
//
// Telecommands (TC):
//   1   = SET_SPEED       (AppData: RW_OID(4) | i16 rpm)
//   2   = STOP            (AppData: RW_OID(4))
//   3   = STATUS          (AppData: RW_OID(4))
//   4   = SET_TORQUE      (AppData: RW_OID(4) | i16 torque_mNm)
//   10  = SET_MODE        (AppData: TARGET_OID(4) | u8 mode | u8 submode)
//   140 = ACS_SET_ENABLE  (AppData: ACS_OID(4) | u8 enable)
//   141 = ACS_SET_TARGET  (AppData: ACS_OID(4) | f32 q[4] big-endian)
//
// Telemetry (TM):
//   220/130 = TM_STATUS       (AppData from parsed wheel STATUS)
//   220/132 = TM_ACS_HK_TYPED (AppData: ACS_OID(4) | u8 ver | u8 enabled |
//                              float Kd[3] | float tauDes[3] | float tauWheelCmd[4] | u32 dtMs)


class RwPusService : public CommandingServiceBase {
 public:
  // Subservice IDs used by this PUS service
  enum Subservice : uint8_t {
    SET_SPEED       = 1,
    STOP            = 2,
    STATUS          = 3,
    SET_TORQUE      = 4,
    SET_MODE        = 10,
    TM_STATUS       = 130,
    TM_ACS_HK_TYPED = 132,
    ACS_SET_ENABLE  = 140,
    ACS_SET_TARGET  = 141
  };

  // Constructor
  // apid        : APID of the TM/TC space packets
  // serviceId   : PUS service number (220 here)
  // numParallel : max number of parallel commands this service tracks
  // timeoutSec  : command timeout in seconds
  RwPusService(object_id_t objectId, uint16_t apid, uint8_t serviceId = 220,
               uint8_t numParallelCommands = 8, uint16_t commandTimeoutSeconds = 5);

  // Resolve stores and finish base initialization
  ReturnValue_t initialize() override;

 protected:
  // Check whether a subservice is supported
  ReturnValue_t isValidSubservice(uint8_t subservice) override;

  // Route TC to the correct queue/object
  // For ACS local ops, returns NO_QUEUE to handle in prepareCommand()
  ReturnValue_t getMessageQueueAndObject(uint8_t subservice, const uint8_t* tcData, size_t tcLen,
                                         MessageQueueId_t* id, object_id_t* objectId) override;

  // Build device/local command from TC payload and set state if multi-step
  ReturnValue_t prepareCommand(CommandMessage* message, uint8_t subservice, const uint8_t* tcData,
                               size_t tcLen, uint32_t* state, object_id_t objectId) override;

  // Handle replies, progress state machine, and decide completion
  ReturnValue_t handleReply(const CommandMessage* reply, Command_t previousCommand, uint32_t* state,
                            CommandMessage* next, object_id_t objectId, bool* isStep) override;

  // Parse RW status payload from store and emit TM_STATUS (220/130)
  ReturnValue_t handleDataReplyAndEmitTm(store_address_t sid, object_id_t objectId);

  // Handle unrequested replies and emit TM if possible
  void handleUnrequestedReply(CommandMessage* reply) override;

  // Emit typed ACS HK (220/132) for the given ACS object
  ReturnValue_t emitAcsTypedHk(object_id_t acsObjectId);

 private:
  // Storage managers (resolved in initialize)
  StorageManagerIF* ipcStore = nullptr;
  StorageManagerIF* tmStore  = nullptr;
  StorageManagerIF* tcStore  = nullptr;

  // Map sender queue -> object id (used to attribute unrequested replies)
  std::unordered_map<MessageQueueId_t, object_id_t> qidToObj_{};

  // Fallback object id if sender mapping is not known yet
  object_id_t lastTargetObjectId_{objects::NO_OBJECT};
};

// Debug on/off switch (set to 1 to enable verbose logging)
#ifndef RW_PUS_VERBOSE
#define RW_PUS_VERBOSE 0
#endif
