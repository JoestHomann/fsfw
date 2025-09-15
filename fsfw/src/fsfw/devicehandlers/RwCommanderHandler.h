#pragma once

#include <cstdint>
#include <cstddef>
#include <array>
#include <cstring>

#include "fsfw/devicehandlers/DeviceHandlerBase.h"
#include "fsfw/devicehandlers/DeviceHandlerIF.h"
#include "fsfw/datapoollocal/StaticLocalDataSet.h"
#include "fsfw/datapoollocal/LocalPoolVector.h"
#include "fsfw/datapoollocal/localPoolDefinitions.h"
#include "fsfw/datapoollocal/LocalDataPoolManager.h"
#include "fsfw/datapool/PoolEntry.h"
#include "fsfw/action/HasActionsIF.h"
#include "fsfw/returnvalues/returnvalue.h"
#include "fsfw/serviceinterface/ServiceInterfaceStream.h"

/**
 * Reaction Wheel Commander Handler (UART, binary protocol).
 *
 * Direct-reply flow (FSFW-official):
 * - Map TC STATUS with hasDifferentReplyId=true and provide a reply dataset.
 * - In interpretDeviceReply(), fill dataset and call handleDeviceTm(replySet, REPLY_STATUS_TC).
 * - Base forwards as DeviceHandlerMessage::REPLY_DIRECT_COMMAND_DATA to the requester (RwPusService).
 * - Polling uses a separate virtual reply id and does NOT forward.
 */
class RwCommanderHandler : public DeviceHandlerBase {
 public:
  // Wire protocol framing
  static constexpr uint8_t START_BYTE_CMD   = 0xAA;
  static constexpr uint8_t START_BYTE_REPLY = 0xAB;

  // Device commands (wire)
  static constexpr DeviceCommandId_t CMD_SET_SPEED   = 0x01;
  static constexpr DeviceCommandId_t CMD_STOP        = 0x02;
  static constexpr DeviceCommandId_t CMD_STATUS      = 0x03;

  // Internal pseudo command for periodic status poll
  static constexpr DeviceCommandId_t CMD_STATUS_POLL = 0x83;

  // Wire reply discriminator (second byte on the line is always 0x10)
  static constexpr uint8_t REPLY_STATUS_WIRE = 0x10;

  // Distinguish TC vs POLL on FSFW side with two different reply ids
  static constexpr DeviceCommandId_t REPLY_STATUS_TC   = 0x10; // forwarded to service
  static constexpr DeviceCommandId_t REPLY_STATUS_POLL = 0x11; // log only

  // Local pool IDs
  enum class PoolIds : lp_id_t {
    RAW_REPLY = 1  // LocalPoolVector<uint8_t, 8>
  };

  // Dataset containing raw 8-byte device reply
  class RwRawReplySet final : public StaticLocalDataSet<1> {
   public:
    LocalPoolVector<uint8_t, 8> raw;

    explicit RwRawReplySet(HasLocalDataPoolIF* owner)
        : StaticLocalDataSet<1>(owner, 0),  // setId unused here
          raw(owner, static_cast<lp_id_t>(PoolIds::RAW_REPLY), this) {}
  };

  RwCommanderHandler(object_id_t objectId, object_id_t comIF, CookieIF* cookie);

  // DeviceHandlerBase overrides
  void doStartUp() override;
  void doShutDown() override;
  ReturnValue_t performOperation(uint8_t opCode) override;
  void modeChanged() override;

  ReturnValue_t buildNormalDeviceCommand(DeviceCommandId_t* id) override;
  ReturnValue_t buildTransitionDeviceCommand(DeviceCommandId_t* id) override;
  ReturnValue_t buildCommandFromCommand(DeviceCommandId_t deviceCommand,
                                        const uint8_t* data, size_t len) override;

  void fillCommandAndReplyMap() override;

  ReturnValue_t scanForReply(const uint8_t* start, size_t len,
                             DeviceCommandId_t* foundId, size_t* foundLen) override;

  ReturnValue_t interpretDeviceReply(DeviceCommandId_t id, const uint8_t* packet) override;

  uint32_t getTransitionDelayMs(Mode_t from, Mode_t to) override;
  ReturnValue_t initializeLocalDataPool(localpool::DataPool& localDataPoolMap,
                                        LocalDataPoolManager& poolManager) override;
  
  //ReturnValue_t executeAction(ActionId_t, MessageQueueId_t, const uint8_t*, size_t) override;

 private:
  // Helpers
  static uint8_t crc8(const uint8_t* data, size_t len);

  // TX buffer
  uint8_t txBuf[8] = {0};

  // Polling control
  uint8_t  statusPollDivider = 5;  // send STATUS every N handler ticks
  uint8_t  statusPollCnt     = 0;
  uint32_t warmupCycles      = 1;
  uint32_t warmupCnt         = 0;

  // Track last outgoing frame source to demultiplex reply IDs
  bool lastSentWasPoll = false;

  // Optional: briefly pause polls after external TCs
  uint8_t pollSnooze = 0;
  static constexpr uint8_t POLL_SNOOZE_CYCLES = 5;

  int16_t lastTargetRpm = 0;

  // Dataset instance for direct-reply forwarding
  RwRawReplySet replySet{this};
};
