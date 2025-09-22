#pragma once

#include "commonObjects.h"
#include "fsfw/storagemanager/StorageManagerIF.h"
#include "fsfw/storagemanager/storeAddress.h"
#include "fsfw/tmtcservices/CommandingServiceBase.h"
#include "fsfw/ipc/MessageQueueIF.h"  // for MessageQueueId_t

#include <unordered_map>

/**
 * Minimal PUS service for a reaction wheel commander.
 *
 * Subservices:
 *   1 = SET_SPEED (int16 RPM, big-endian in AppData)
 *   2 = STOP
 *   3 = STATUS
 *  10 = SET_MODE (mode, submode)
 *
 * AppData convention (all TCs):
 *   [0..3]  Destination object ID (big-endian)
 *   [4..]   Subservice-specific payload
 *
 * Note: Device reply on the wire is STATUS_LEN bytes with CRC-16/CCITT:
 *   [AB,10,spdH,spdL,torH,torL,running,crcH,crcL]
 */
class RwPusService : public CommandingServiceBase {
 public:
  enum Subservice : uint8_t {
    SET_SPEED = 1,
    STOP = 2,
    STATUS = 3,
    SET_MODE = 10,
    TM_STATUS = 130,
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

  // Parse RW status payload from store and emit TM_STATUS.
  ReturnValue_t handleDataReplyAndEmitTm(store_address_t sid, object_id_t objectId);

  // Handle late/unrequested data replies by resolving sender via qid->obj map.
  void handleUnrequestedReply(CommandMessage* reply) override;

 private:
  // Stores (resolved in initialize())
  StorageManagerIF* ipcStore = nullptr;
  StorageManagerIF* tmStore = nullptr;
  StorageManagerIF* tcStore = nullptr;

  // Robust routing for unrequested replies: sender queue -> object id
  std::unordered_map<MessageQueueId_t, object_id_t> qidToObj_{};

  // Multi-device support: object id -> sender queue
  std::unordered_map<object_id_t, MessageQueueId_t> objToQid_{};

  // Optional per-object status cache for quick access/re-broadcasts
  struct RwStatusCache {
    int16_t  speedRpm{0};
    int16_t  torqueMnM{0};
    uint8_t  running{0};
    uint32_t timestampMs{0};  // local uptime when parsed
  };
  std::unordered_map<object_id_t, RwStatusCache> lastStatus_{};

  // Helpers
  void rememberMapping(object_id_t obj, MessageQueueId_t q);
  MessageQueueId_t getQidFor(object_id_t obj) const;
  object_id_t resolveObjFromSender(MessageQueueId_t sender) const;
};

// Debug On/Off switch (set to 1 to enable debug output)
#ifndef RW_PUS_VERBOSE
#define RW_PUS_VERBOSE 0
#endif
