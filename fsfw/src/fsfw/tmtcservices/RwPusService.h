// RwPusService.h
#pragma once

#include <unordered_map>

#include "commonObjects.h"
#include "fsfw/ipc/MessageQueueIF.h"  // for MessageQueueId_t
#include "fsfw/storagemanager/StorageManagerIF.h"
#include "fsfw/storagemanager/storeAddress.h"
#include "fsfw/tmtcservices/CommandingServiceBase.h"

/**
 * Minimal PUS service for a reaction wheel commander.
 *
 * Subservices:
 *   1 = SET_SPEED (int16 RPM, big-endian in AppData)
 *   2 = STOP
 *   3 = STATUS
 *  10 = SET_MODE (mode, submode)
 *
 * Telemetry:
 * 130 = TM_STATUS (legacy, compact)
 * 131 = TM_STATUS_TYPED (versioned, with quality counters)
 *
 * AppData convention for TCs:
 *   [0..3]  Destination object ID (big-endian)
 *   [4..]   Subservice-specific payload
 *
 * Typed TM (131) v1 payload (big-endian):
 *   ver(1)=1 | objectId(4) | speedRpm(2) | torque_mNm(2) | running(1)
 *   | flags(2) | error(2) | crcErrCnt(4) | malformedCnt(4) | timestampMs(4) | sampleCnt(2)
 */
class RwPusService : public CommandingServiceBase {
 public:
  enum Subservice : uint8_t {
    SET_SPEED = 1,
    STOP = 2,
    STATUS = 3,
    SET_MODE = 10,
    TM_STATUS = 130,
    TM_STATUS_TYPED = 131,  // NEW
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

  // Parse RW status payload from store and emit TM (130 legacy + 131 typed).
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

  // Per-object status cache (optional use)
  struct RwStatusCache {
    int16_t speedRpm{0};
    int16_t torqueMnM{0};
    uint8_t running{0};
    uint32_t timestampMs{0};  // local uptime when parsed
  };
  std::unordered_map<object_id_t, RwStatusCache> lastStatus_{};

  // Per-object quality counters for typed TM
  struct RwStats {
    uint32_t crcErrCnt{0};
    uint32_t malformedCnt{0};
    uint16_t sampleCnt{0};
  };
  std::unordered_map<object_id_t, RwStats> stats_{};

  // Helpers
  void rememberMapping(object_id_t obj, MessageQueueId_t q);
  MessageQueueId_t getQidFor(object_id_t obj) const;
  object_id_t resolveObjFromSender(MessageQueueId_t sender) const;
};

// Debug On/Off switch (set to 1 to enable debug output)
#ifndef RW_PUS_VERBOSE
#define RW_PUS_VERBOSE 0
#endif

// On/Off switch for legacy TM 220/130 (Set to 0 to disable sending the legacy compact status
// packet)
#ifndef RW_PUS_ENABLE_LEGACY_130
#define RW_PUS_ENABLE_LEGACY_130 0
#endif