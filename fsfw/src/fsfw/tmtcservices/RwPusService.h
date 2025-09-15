#pragma once

#include "fsfw/tmtcservices/CommandingServiceBase.h"
#include "fsfw/storagemanager/StorageManagerIF.h"

// NEW: Accept raw device replies directly in this service
#include "fsfw/devicehandlers/AcceptsDeviceResponsesIF.h"
#include "fsfw/ipc/MessageQueueIF.h"

/**
 * Minimal PUS service for a reaction wheel commander.
 * Subservices:
 *   1 = SET_SPEED (int16 RPM, big-endian in AppData)
 *   2 = STOP
 *   3 = STATUS
 *  10 = SET_MODE (mode, submode)
 *
 * AppData convention (all TCs):
 *   [0..3]  Destination object ID (big-endian)
 *   [4..]   Subservice-specific payload
 */
class RwPusService : public CommandingServiceBase, public AcceptsDeviceResponsesIF {
 public:
  enum Subservice : uint8_t {
    SET_SPEED = 1,
    STOP      = 2,
    STATUS    = 3,
    SET_MODE  = 10,
    TM_STATUS = 130,
  };

  RwPusService(object_id_t objectId, uint16_t apid, uint8_t serviceId = 220,
               uint8_t numParallelCommands = 8, uint16_t commandTimeoutSeconds = 5);

  ReturnValue_t initialize() override;

  // AcceptsDeviceResponsesIF: return the queue where device handlers shall send raw replies.
  // This routes REPLY_RAW_REPLY (and similar) to this service instead of PUS Service 2.
  MessageQueueId_t getDeviceQueue() override { return commandQueue->getId(); }

 protected:
  // CommandingServiceBase hooks
  ReturnValue_t isValidSubservice(uint8_t subservice) override;
  ReturnValue_t getMessageQueueAndObject(uint8_t subservice, const uint8_t* tcData,
                                         size_t tcLen, MessageQueueId_t* id,
                                         object_id_t* objectId) override;

  ReturnValue_t prepareCommand(CommandMessage* message, uint8_t subservice,
                               const uint8_t* tcData, size_t tcLen, uint32_t* state,
                               object_id_t objectId) override;

  ReturnValue_t handleReply(const CommandMessage* reply, Command_t previousCommand,
                            uint32_t* state, CommandMessage* next, object_id_t objectId,
                            bool* isStep) override;

 private:
  StorageManagerIF* ipcStore = nullptr;
};
