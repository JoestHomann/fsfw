#pragma once

#include <cstddef>
#include <cstdint>
#include <array>

#include "fsfw/devicehandlers/DeviceHandlerBase.h"
#include "fsfw/devicehandlers/DeviceHandlerIF.h"
#include "fsfw/action/HasActionsIF.h"
#include "commonObjects.h"

// Local data pool / dataset for "direct command data"
#include "fsfw/datapoollocal/StaticLocalDataSet.h"
#include "fsfw/datapoollocal/LocalPoolVector.h"
#include "fsfw/datapoollocal/localPoolDefinitions.h"
#include "fsfw/datapoollocal/LocalDataPoolManager.h"
#include "fsfw/datapool/PoolEntry.h"

/**
 * Minimal device handler for a reaction wheel simulator.
 *
 * UART protocol (binary):
 *   TX command  (5 bytes):  0xAA <id> <hi> <lo> <crc8>
 *   RX reply    (8 bytes):  0xAB 0x10 <spdH spdL> <torH torL> <running> <crc8>
 *
 * This handler demonstrates the "FSFW-official" way to return a direct command
 * reply: We set hasDifferentReplyId=true and provide a reply data set in
 * insertInCommandAndReplyMap(). In interpretDeviceReply(), we fill that data
 * set and call handleDeviceTm(). The base will forward it as
 * DeviceHandlerMessage::REPLY_DIRECT_COMMAND_DATA to the commander.
 */
class RwCommanderHandler : public DeviceHandlerBase {
 public:
  // Device protocol framing
  static constexpr uint8_t START_BYTE_CMD    = 0xAA;
  static constexpr uint8_t START_BYTE_REPLY  = 0xAB;

  // Device commands
  static constexpr DeviceCommandId_t CMD_SET_SPEED   = 0x01;
  static constexpr DeviceCommandId_t CMD_STOP        = 0x02;
  static constexpr DeviceCommandId_t CMD_STATUS      = 0x03;

  // Internal helper command for periodic polling (no on-wire ID)
  static constexpr DeviceCommandId_t CMD_STATUS_POLL = 0x83;

  // Device reply IDs (different from command IDs)
  static constexpr DeviceCommandId_t REPLY_STATUS    = 0x10;

  // Pool IDs for the direct-reply dataset (we store the raw 8-byte frame)
  enum class PoolIds : lp_id_t {
    RAW_REPLY = 1  // LocalPoolVector<uint8_t, 8>
  };

  /** Small dataset that holds the raw 8-byte device reply.
   *  We use LocalPoolVector so the base can serialize it and send it as DATA_REPLY.
   */
  class RwRawReplySet final : public StaticLocalDataSet<1> {
   public:
    LocalPoolVector<uint8_t, 8> raw;

    explicit RwRawReplySet(HasLocalDataPoolIF* owner)
        : StaticLocalDataSet<1>(owner, 0 /*setId not used for TM here*/), raw(owner, static_cast<lp_id_t>(PoolIds::RAW_REPLY), this) {}
  };

  RwCommanderHandler(object_id_t objectId, object_id_t comIF, CookieIF* cookie);

  // DeviceHandlerBase interface
  void doStartUp() override;
  void doShutDown() override;

  ReturnValue_t buildNormalDeviceCommand(DeviceCommandId_t* id) override;
  ReturnValue_t buildTransitionDeviceCommand(DeviceCommandId_t* id) override;

  ReturnValue_t buildCommandFromCommand(DeviceCommandId_t deviceCommand, const uint8_t* commandData,
                                        size_t commandDataLen) override;

  ReturnValue_t scanForReply(const uint8_t* start, size_t len, DeviceCommandId_t* foundId,
                             size_t* foundLen) override;

  ReturnValue_t interpretDeviceReply(DeviceCommandId_t id, const uint8_t* packet) override;

  void fillCommandAndReplyMap() override;

  ReturnValue_t performOperation(uint8_t opCode) override;

  // Callback without parameters
  void modeChanged() override;

  uint32_t getTransitionDelayMs(Mode_t from, Mode_t to) override;

  ReturnValue_t initializeLocalDataPool(localpool::DataPool& localDataPoolMap,
                                        LocalDataPoolManager& poolManager) override;

  // Optional actions interface passthrough
  ReturnValue_t executeAction(ActionId_t actionId, MessageQueueId_t commandedBy,
                              const uint8_t* data, size_t size) override;

 private:
  // Helpers
  static uint8_t crc8(const uint8_t* data, size_t len);

  // Shared small TX buffer
  uint8_t txBuf[8] = {0};

  // Polling control
  uint8_t  statusPollDivider = 100;   // e.g. send STATUS every N handler ticks (set as you like)
  uint8_t  statusPollCnt     = 0;
  uint32_t warmupCycles      = 1;   // skip first cycles after port open
  uint32_t warmupCnt         = 0;

  int16_t lastTargetRpm = 0;

  // Dataset instance for direct-command reply path
  RwRawReplySet replySet{this};

  // Pause polls for a few handler cycles after sending any external TC
  uint8_t pollSnooze = 0;
  static constexpr uint8_t POLL_SNOOZE_CYCLES = 5;
};
