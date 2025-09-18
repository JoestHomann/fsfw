#pragma once

#include "fsfw/datapoollocal/LocalDataPoolManager.h"
#include "fsfw/datapoollocal/LocalDataSet.h"
#include "fsfw/datapoollocal/LocalPoolVector.h"
#include "fsfw/devicehandlers/DeviceHandlerBase.h"
#include "fsfw/ipc/MessageQueueIF.h"
#include "fsfw/returnvalues/returnvalue.h"

class ReactionWheelsHandler : public DeviceHandlerBase {
 public:
  // On-wire constants
  static constexpr uint8_t START_BYTE_CMD = 0xAA;
  static constexpr uint8_t START_BYTE_REPLY = 0xAB;
  static constexpr uint8_t REPLY_STATUS_WIRE = 0x10;

  // Internal command IDs
  enum DeviceCmd : DeviceCommandId_t {
    CMD_SET_SPEED = 0x01,
    CMD_STOP = 0x02,
    CMD_STATUS = 0x03,        // action-driven request (same wire frame)
    CMD_STATUS_POLL = 0x1003  // device command used for both periodic and TC-driven polls
  };

  // Unified internal reply ID (matches scanForReply classification)
  enum ReplyId : DeviceCommandId_t { REPLY_STATUS_POLL = 0x2002 };

  // Local pool IDs / DataSet ID
  enum class PoolIds : lp_id_t { RAW_REPLY = 1 };
  static constexpr uint32_t DATASET_ID = 0xCA;

  struct RwReplySet : public LocalDataSet {
    LocalPoolVector<uint8_t, 8> raw;
    explicit RwReplySet(HasLocalDataPoolIF* owner)
        : LocalDataSet(owner, DATASET_ID, 1),
          raw(owner->getObjectId(), static_cast<lp_id_t>(PoolIds::RAW_REPLY), this) {}
  };

  ReactionWheelsHandler(object_id_t objectId, object_id_t comIF, CookieIF* cookie);

  // DeviceHandlerBase interface
  void doStartUp() override;
  void doShutDown() override;
  void modeChanged() override;

  ReturnValue_t buildNormalDeviceCommand(DeviceCommandId_t* id) override;
  ReturnValue_t buildTransitionDeviceCommand(DeviceCommandId_t* id) override;
  ReturnValue_t buildCommandFromCommand(DeviceCommandId_t deviceCommand, const uint8_t* data,
                                        size_t len) override;

  ReturnValue_t scanForReply(const uint8_t* start, size_t len, DeviceCommandId_t* foundId,
                             size_t* foundLen) override;

  ReturnValue_t interpretDeviceReply(DeviceCommandId_t id, const uint8_t* packet) override;

  uint32_t getTransitionDelayMs(Mode_t from, Mode_t to) override;

  ReturnValue_t initializeLocalDataPool(localpool::DataPool& localDataPoolMap,
                                        LocalDataPoolManager& poolManager) override;

  void fillCommandAndReplyMap() override;

  ReturnValue_t executeAction(ActionId_t actionId, MessageQueueId_t commandedBy,
                              const uint8_t* data, size_t size) override;

 private:
  // Drain UART RX quickly in a non-blocking fashion to drop stale bytes
  ReturnValue_t drainRxNow();

  // CRC8 helper (same polynomial as device)
  static uint8_t crc8(const uint8_t* data, size_t len);

  // Compact TX buffer for all commands (STATUS/SET/STOP are 5 bytes)
  uint8_t txBuf[5] = {};

  // Simple warm-up before entering NORMAL
  uint32_t warmupCnt{0};
  static constexpr uint32_t warmupCycles{2};

  // Periodic polling divider (incremented each PST cycle)
  uint32_t statusPollCnt{0};
  static constexpr uint32_t statusPollDivider{100};

  // Short backoff ticks after TC to avoid immediate re-poll
  uint32_t pollSnooze{0};

  // Set true when a TC STATUS has been issued; next valid frame will be routed as DATA_REPLY
  bool pendingTcStatusTm{false};

  // Queue to route a TC data reply back to the requester (PUS service)
  MessageQueueId_t pendingTcStatusReportedTo{MessageQueueIF::NO_QUEUE};

  // Most recent raw reply (8-byte wire frame)
  RwReplySet replySet{this};

  // Last commanded target RPM (for reference/diagnostics)
  int16_t lastTargetRpm{0};
};
