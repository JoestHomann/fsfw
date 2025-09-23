// fsfw/tmtcservices/RwPusService.h
#pragma once

#include "commonObjects.h"
#include "fsfw/storagemanager/StorageManagerIF.h"
#include "fsfw/storagemanager/storeAddress.h"
#include "fsfw/tmtcservices/CommandingServiceBase.h"

#include <unordered_map>

/**
 * English comments!
 * PUS Service 220 for Reaction Wheel & ACS control.
 *
 * Telecommands (TC):
 *   1   = SET_SPEED      (AppData: RW_OID(4) | i16 rpm)
 *   2   = STOP           (AppData: RW_OID(4))
 *   3   = STATUS         (AppData: RW_OID(4))
 *   4   = SET_TORQUE     (AppData: RW_OID(4) | i16 torque_mNm)   // NEW
 *   10  = SET_MODE       (AppData: TARGET_OID(4) | u8 mode | u8 submode)
 *   140 = ACS_SET_ENABLE (AppData: ACS_OID(4) | u8 enable)       // NEW
 *
 * Telemetry (TM):
 *   220/130 = TM_STATUS (legacy wheel status typed by raw-parser in Python tool)
 *   220/132 = TM_ACS_HK_TYPED  (AppData: ACS_OID(4) | u8 ver | u8 enabled |
 *                               float kd[3] | float tauDes[3] | float tauWheel[4] | u32 dt_ms)
 */
class RwPusService : public CommandingServiceBase {
 public:
  enum Subservice : uint8_t {
    SET_SPEED       = 1,
    STOP            = 2,
    STATUS          = 3,
    SET_TORQUE      = 4,    // NEW
    SET_MODE        = 10,
    TM_STATUS       = 130,
    TM_ACS_HK_TYPED = 132,  // NEW
    ACS_SET_ENABLE  = 140   // NEW
  };

  RwPusService(object_id_t objectId, uint16_t apid, uint8_t serviceId = 220,
               uint8_t numParallelCommands = 8, uint16_t commandTimeoutSeconds = 5);

  ReturnValue_t initialize() override;

 protected:
  // CommandingServiceBase hooks
  ReturnValue_t isValidSubservice(uint8_t subservice) override;
  ReturnValue_t getMessageQueueAndObject(uint8_t subservice, const uint8_t* tcData, size_t tcLen,
                                         MessageQueueId_t* id, object_id_t* objectId) override;

  ReturnValue_t prepareCommand(CommandMessage* message, uint8_t subservice, const uint8_t* tcData,
                               size_t tcLen, uint32_t* state, object_id_t objectId) override;

  ReturnValue_t handleReply(const CommandMessage* reply, Command_t previousCommand, uint32_t* state,
                            CommandMessage* next, object_id_t objectId, bool* isStep) override;

  // Parse RW status payload from store and emit TM_STATUS (220/130).
  ReturnValue_t handleDataReplyAndEmitTm(store_address_t sid, object_id_t objectId);

  // Handle late/unrequested data replies by trying to parse & emit TM.
  void handleUnrequestedReply(CommandMessage* reply) override;

  // Emit typed ACS HK (220/132) for given ACS object, returns OK on success.
  ReturnValue_t emitAcsTypedHk(object_id_t acsObjectId);

 private:
  // Stores (resolved in initialize())
  StorageManagerIF* ipcStore = nullptr;
  StorageManagerIF* tmStore = nullptr;
  StorageManagerIF* tcStore = nullptr;

  // Route unrequested replies robustly: sender queue -> object id
  std::unordered_map<MessageQueueId_t, object_id_t> qidToObj_{};

  // Fallback if sender mapping is not known yet
  object_id_t lastTargetObjectId_{objects::NO_OBJECT};
};

// Debug On/Off switch (set to 1 to enable debug output)
#ifndef RW_PUS_VERBOSE
#define RW_PUS_VERBOSE 0
#endif
