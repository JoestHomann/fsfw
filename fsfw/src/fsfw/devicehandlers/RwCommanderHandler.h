#pragma once

#include "fsfw/devicehandlers/DeviceHandlerBase.h"
#include "fsfw/datapoollocal/LocalDataSet.h"
#include "fsfw/datapoollocal/LocalPoolVector.h"
#include "fsfw/datapoollocal/LocalDataPoolManager.h"
#include "fsfw/returnvalues/returnvalue.h"

class RwCommanderHandler : public DeviceHandlerBase {
 public:
  // On-wire constants
  static constexpr uint8_t START_BYTE_CMD    = 0xAA;
  static constexpr uint8_t START_BYTE_REPLY  = 0xAB;
  static constexpr uint8_t REPLY_STATUS_WIRE = 0x10;

  // Internal command IDs
  enum DeviceCmd : DeviceCommandId_t {
    CMD_SET_SPEED   = 0x01,
    CMD_STOP        = 0x02,
    CMD_STATUS      = 0x03,
    CMD_STATUS_POLL = 0x1003
  };

  // Internal reply IDs
  enum ReplyId : DeviceCommandId_t {
    REPLY_STATUS_TC   = 0x2001,
    REPLY_STATUS_POLL = 0x2002
  };

  // Local pool IDs / DataSet ID
  enum class PoolIds : lp_id_t { RAW_REPLY = 1 };
  static constexpr uint32_t DATASET_ID = 0xCA;

  struct RwReplySet : public LocalDataSet {
    LocalPoolVector<uint8_t, 8> raw;

    explicit RwReplySet(HasLocalDataPoolIF* owner)
        // LocalDataSet(hkOwner, setId, maxSizeOfSet)
        : LocalDataSet(owner, DATASET_ID, 1),
          // LocalPoolVector(poolOwnerObjectId, poolId, dataSet)
          // (RW-Mode ist default, daher kein PoolVariableIF nötig)
          raw(owner->getObjectId(),
              static_cast<lp_id_t>(PoolIds::RAW_REPLY),
              this) {}
  };

  RwCommanderHandler(object_id_t objectId, object_id_t comIF, CookieIF* cookie);

  void doStartUp() override;
  void doShutDown() override;
  ReturnValue_t performOperation(uint8_t opCode) override;
  void modeChanged() override;

  ReturnValue_t buildNormalDeviceCommand(DeviceCommandId_t* id) override;
  ReturnValue_t buildTransitionDeviceCommand(DeviceCommandId_t* id) override;
  ReturnValue_t buildCommandFromCommand(DeviceCommandId_t deviceCommand,
                                        const uint8_t* data, size_t len) override;

  ReturnValue_t scanForReply(const uint8_t* start, size_t len,
                             DeviceCommandId_t* foundId, size_t* foundLen) override;

  ReturnValue_t interpretDeviceReply(DeviceCommandId_t id,
                                     const uint8_t* packet) override;

  uint32_t getTransitionDelayMs(Mode_t from, Mode_t to) override;

  ReturnValue_t initializeLocalDataPool(localpool::DataPool& localDataPoolMap,
                                        LocalDataPoolManager& poolManager) override;

  void fillCommandAndReplyMap() override;

  ReturnValue_t executeAction(ActionId_t actionId, MessageQueueId_t commandedBy,
                              const uint8_t* data, size_t size) override;

 private:
  static uint8_t crc8(const uint8_t* data, size_t len);

  uint8_t txBuf[5] = {};

  uint32_t warmupCnt{0};
  static constexpr uint32_t warmupCycles{2};

  uint32_t statusPollCnt{0};
  static constexpr uint32_t statusPollDivider{3};

  uint32_t pollSnooze{0};
  static constexpr uint32_t POLL_SNOOZE_CYCLES{3};

  bool lastSentWasPoll{false};
  bool pendingTcStatusTm{false};

  RwReplySet replySet{this};
  int16_t lastTargetRpm{0};
};
