#pragma once

#include "commonObjects.h"
#include "fsfw/storagemanager/StorageManagerIF.h"
#include "fsfw/storagemanager/storeAddress.h"
#include "fsfw/tmtcservices/CommandingServiceBase.h"

#include <unordered_map>

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
//   220/131 = TM_STATUS_TYPED (v1)
//             AppData: ver(1)=1 | oid(4) | speed(i16) | torque(i16) | running(u8)
//                       | flags(u16) | err(u16) | crcCnt(u32) | malCnt(u32) | tsMs(u32) | sample(u16)
//   220/132 = TM_ACS_HK_TYPED
//   220/133 = TM_ATT_YPR_TYPED

class RwPusService : public CommandingServiceBase {
 public:
  enum Subservice : uint8_t {
    SET_SPEED        = 1,
    STOP             = 2,
    STATUS           = 3,
    SET_TORQUE       = 4,
    SET_MODE         = 10,
    TM_STATUS_TYPED  = 131,
    TM_ACS_HK_TYPED  = 132,
    TM_ATT_YPR_TYPED = 133,
    ACS_SET_ENABLE   = 140,
    ACS_SET_TARGET   = 141
  };

  RwPusService(object_id_t objectId, uint16_t apid, uint8_t serviceId = 220,
               uint8_t numParallelCommands = 8, uint16_t commandTimeoutSeconds = 5);

  ReturnValue_t initialize() override;

 protected:
  ReturnValue_t isValidSubservice(uint8_t subservice) override;

  ReturnValue_t getMessageQueueAndObject(uint8_t subservice, const uint8_t* tcData, size_t tcLen,
                                         MessageQueueId_t* id, object_id_t* objectId) override;

  ReturnValue_t prepareCommand(CommandMessage* message, uint8_t subservice, const uint8_t* tcData,
                               size_t tcLen, uint32_t* state, object_id_t objectId) override;

  ReturnValue_t handleReply(const CommandMessage* reply, Command_t previousCommand, uint32_t* state,
                            CommandMessage* next, object_id_t objectId, bool* isStep) override;

  ReturnValue_t handleDataReplyAndEmitTm(store_address_t sid, object_id_t objectId);

  void handleUnrequestedReply(CommandMessage* reply) override;

  ReturnValue_t emitAcsTypedHk(object_id_t acsObjectId);

  // -------- periodic activities (typed RW + typed attitude TM) -------------
  void doPeriodicOperation() override;
  ReturnValue_t emitAttYprTm(object_id_t acsObjectId);

 private:
  // Build and send TM 220/131 directly from RW local HK dataset
  ReturnValue_t emitRwTypedFromHk(object_id_t rwObjectId);

  // Helper: check if new data is available (age < maxAgeMs)
  bool isRwHkFresh_(object_id_t rwOid, uint32_t maxAgeMs = 400) const;

  // Storage managers
  StorageManagerIF* ipcStore = nullptr;
  StorageManagerIF* tmStore  = nullptr;
  StorageManagerIF* tcStore  = nullptr;

  // Map sender queue -> object id
  std::unordered_map<MessageQueueId_t, object_id_t> qidToObj_{};

  object_id_t lastTargetObjectId_{objects::NO_OBJECT};

  // -------- lightweight periodic state -------------------------------------
  object_id_t rwPollOid_{objects::RW_HANDLER};   // RW device to report (updated on TC)
  object_id_t acsPollOid_{objects::RW_ACS_CTRL}; // ACS object to report
  uint32_t    serviceTick_{0};
  uint16_t    rwSample_{0};
  uint16_t    attSample_{0};
  bool        acsEnabledCached_{false};

  static constexpr uint32_t RW_TYPED_EVERY_N = 25; // emit 220/131 every N service calls
};

// Debug on/off switch (set to 1 to enable verbose logging)
#ifndef RW_PUS_VERBOSE
#define RW_PUS_VERBOSE 0
#endif
