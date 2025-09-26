#pragma once

#include <cstdint>
#include <unordered_map>
#include "commonObjects.h"
#include "fsfw/storagemanager/StorageManagerIF.h"
#include "fsfw/storagemanager/storeAddress.h"
#include "fsfw/tmtcservices/CommandingServiceBase.h"

/*
 * RwPusService.h - PUS service for Reaction Wheels and ACS
 *
 *  RwPusService (Service 220) handles TM/TC to and from RW DeviceHandlers and the ACS controller.
 *  Converts TCs to DH actions/mode commands, parse replies, and send typed TM.
 *
 * Telecommands (TC):
 *   1   = SET_SPEED
 *   2   = STOP
 *   3   = STATUS
 *   4   = SET_TORQUE
 *   10  = SET_MODE
 *   140 = ACS_SET_ENABLE
 *   141 = ACS_SET_TARGET
 *
 * Telemetry (TM):
 *   220/131 = TM_STATUS_TYPED
 *   220/132 = TM_ACS_HK_TYPED
 *   220/133 = TM_ATT_YPR_TYPED
 *
 *  - Joest Homann
 */

// -------- Service definition -------------------------------------------------
class RwPusService : public CommandingServiceBase {
 public:
  // Map PUS subservice codes (TC/TM identifiers)
  enum Subservice : uint8_t {
    // Telecommands
    SET_SPEED        = 1,
    STOP             = 2,
    STATUS           = 3,
    SET_TORQUE       = 4,
    SET_MODE         = 10,
    ACS_SET_ENABLE   = 140,
    ACS_SET_TARGET   = 141,

    // Telemetry (typed)
    TM_STATUS_TYPED  = 131,
    TM_ACS_HK_TYPED  = 132,
    TM_ATT_YPR_TYPED = 133
  };

  // -------- Construction -----------------------------------------------------
  RwPusService(object_id_t objectId, uint16_t apid, uint8_t serviceId = 220,
               uint8_t numParallelCommands = 8, uint16_t commandTimeoutSeconds = 5);

  // -------- Initialization ---------------------------------------------------
  ReturnValue_t initialize() override;

 protected:
  // -------- Subservice filter ------------------------------------------------
  ReturnValue_t isValidSubservice(uint8_t subservice) override;

  // -------- Target resolution ------------------------------------------------
  // Decode target OID from TC and return target queue/object
  ReturnValue_t getMessageQueueAndObject(uint8_t subservice, const uint8_t* tcData, size_t tcLen,
                                         MessageQueueId_t* id, object_id_t* objectId) override;

  // -------- TC translation ---------------------------------------------------
  // Convert TC AppData to FSFW messages (+ optional state)
  ReturnValue_t prepareCommand(CommandMessage* message, uint8_t subservice, const uint8_t* tcData,
                               size_t tcLen, uint32_t* state, object_id_t objectId) override;

  // -------- Reply routing ----------------------------------------------------
  // Handle replies, advance state, and complete TCs
  ReturnValue_t handleReply(const CommandMessage* reply, Command_t previousCommand, uint32_t* state,
                            CommandMessage* next, object_id_t objectId, bool* isStep) override;

  // Parse stored raw reply, emit typed 220/131
  ReturnValue_t handleDataReplyAndEmitTm(store_address_t sid, object_id_t objectId);

  // Handle spontaneous/periodic DH replies (e.g., periodic STATUS)
  void handleUnrequestedReply(CommandMessage* reply) override;

  // -------- Typed TM emitters ------------------------------------------------
  ReturnValue_t emitAcsTypedHk(object_id_t acsObjectId);

  // Periodic hook (typed RW + typed attitude TM)
  void doPeriodicOperation() override;
  ReturnValue_t emitAttYprTm(object_id_t acsObjectId);

 private:
  // -------- Typed RW HK (220/131) -------------------------------------------
  // Build and send TM 220/131 from RW local HK dataset
  ReturnValue_t emitRwTypedFromHk(object_id_t rwObjectId);

  // -------- Check for new data -----------------------------------------------
  // Return true if RW HK timestamp age <= maxAgeMs
  bool isRwHkNew(object_id_t rwOid, uint32_t maxAgeMs = 400) const;

  // -------- Stores & routing -------------------------------------------------
  StorageManagerIF* ipcStore = nullptr;   //TC->DH handover


  // Map sender queue -> object id (identify RW for unrequested replies)
  std::unordered_map<MessageQueueId_t, object_id_t> qidToObj_{};

  // Last addressed object id (fallback when no mapping exists)
  object_id_t lastTargetObjectId_{objects::NO_OBJECT};

  // -------- Lightweight periodic state --------------------------------------
  object_id_t rwPollOid_{objects::RW_HANDLER};   // RW to report data from
  object_id_t acsPollOid_{objects::RW_ACS_CTRL}; // ACS to report data from
  uint32_t    serviceTick_{0};                   // Counter for TM periodicity
  uint16_t    rwSample_{0};                      // RW typed TM sample counter
  uint16_t    attSample_{0};                     // Attitude typed TM sample counter
  bool        acsEnabledCached_{false};          // Gate for 220/133 output

  // Emit RW typed TM every N serviceTick_
  static constexpr uint32_t RW_TYPED_EVERY_N = 25;
};

// -------- Debug switch -------------------------------------------------------
// Set to 1 to enable verbose logging (hex dumps, traces)
#ifndef RW_PUS_VERBOSE
#define RW_PUS_VERBOSE 0
#endif
