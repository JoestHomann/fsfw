#pragma once

#include "RwProtocol.h"

#include "fsfw/datapoollocal/LocalDataPoolManager.h"
#include "fsfw/datapoollocal/LocalDataSet.h"
#include "fsfw/datapoollocal/LocalPoolVector.h"
#include "fsfw/datapoollocal/LocalPoolVariable.h"
#include "fsfw/devicehandlers/DeviceHandlerBase.h"
#include "fsfw/ipc/MessageQueueIF.h"
#include "fsfw/returnvalues/returnvalue.h"
#include "fsfw/container/SharedRingBuffer.h"

#include "fsfw/parameters/ParameterHelper.h"
#include "fsfw/parameters/ParameterWrapper.h"

class ReactionWheelsHandler : public DeviceHandlerBase {
 public:
  // Internal command IDs
  enum DeviceCmd : DeviceCommandId_t {
    CMD_SET_SPEED   = 0x01,
    CMD_STOP        = 0x02,
    CMD_STATUS      = 0x03,        // action-driven request (same wire frame as poll)
    CMD_STATUS_POLL = 0x1003       // used for periodic and TC-driven polls
  };

  // Unified internal reply ID
  enum ReplyId : DeviceCommandId_t { REPLY_STATUS_POLL = 0x2002 };

  // Parameters (Domain = 0x42)
  static constexpr uint8_t PARAM_DOMAIN = 0x42;
  enum class ParamId : uint8_t {
    MAX_RPM        = 1,  // int16
    MAX_SLEW_RPM_S = 2,  // uint16 (optional enforcement)
    POLL_DIVIDER   = 3   // uint32 (divider for periodic STATUS polls)
  };

  // Local pool IDs
  enum class PoolIds : lp_id_t {
    RAW_REPLY     = 1,
    HK_SPEED_RPM  = 2,
    HK_TORQUE_mNm = 3,
    HK_RUNNING    = 4,
    HK_FLAGS      = 5,
    HK_ERROR      = 6
  };

  // Dataset IDs
  static constexpr uint32_t DATASET_ID_RAW = 0xCA;
  static constexpr uint32_t DATASET_ID_HK  = 0xCB;

  // RAW reply dataset (wire frame)
  struct RwReplySet : public LocalDataSet {
    // 9-byte wire reply with CRC16:
    // [0]=AB, [1]=0x10 (STATUS), [2..3]=speed, [4..5]=torque, [6]=running, [7..8]=crc16
    LocalPoolVector<uint8_t, RwProtocol::STATUS_LEN> raw;
    explicit RwReplySet(HasLocalDataPoolIF* owner)
        : LocalDataSet(owner, DATASET_ID_RAW, 1),
          raw(owner->getObjectId(), static_cast<lp_id_t>(PoolIds::RAW_REPLY), this) {}
  };

  // Housekeeping dataset (parsed, user-friendly variables)
  struct RwHkSet : public LocalDataSet {
    LocalPoolVariable<int16_t>  speedRpm;
    LocalPoolVariable<int16_t>  torque_mNm;
    LocalPoolVariable<uint8_t>  running;
    LocalPoolVariable<uint16_t> flags;
    LocalPoolVariable<uint16_t> error;
    static constexpr uint16_t HK_VAR_COUNT = 5;
    explicit RwHkSet(HasLocalDataPoolIF* owner)
        : LocalDataSet(owner, DATASET_ID_HK, HK_VAR_COUNT),
          speedRpm(owner->getObjectId(), static_cast<lp_id_t>(PoolIds::HK_SPEED_RPM), this),
          torque_mNm(owner->getObjectId(), static_cast<lp_id_t>(PoolIds::HK_TORQUE_mNm), this),
          running(owner->getObjectId(), static_cast<lp_id_t>(PoolIds::HK_RUNNING), this),
          flags(owner->getObjectId(), static_cast<lp_id_t>(PoolIds::HK_FLAGS), this),
          error(owner->getObjectId(), static_cast<lp_id_t>(PoolIds::HK_ERROR), this) {}
  };

  // Transition delays (tune as needed; non-zero helps sequencing)
  static constexpr uint32_t RW_DELAY_OFF_TO_ON_MS    = 100;
  static constexpr uint32_t RW_DELAY_ON_TO_NORMAL_MS = 100;

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

  // Parameters (override from HasParametersIF via DeviceHandlerBase)
  ReturnValue_t getParameter(uint8_t domainId, uint8_t parameterId,
                             ParameterWrapper* parameterWrapper,
                             const ParameterWrapper* newValues,
                             uint16_t startAtIndex) override;

 private:
  // Drop stale bytes quickly (use with care to not drop fresh replies)
  ReturnValue_t drainRxNow();
  // Pull bytes into the ring buffer (non-blocking, thread-safe)
  ReturnValue_t drainRxIntoRing();
  // Report protocol issues (CRC/invalid IDs) as member to access triggerEvent()
  void reportProtocolIssuesInWindow(const uint8_t* buf, size_t n);

  // TX buffer for all commands (STATUS/SET/STOP are 6 bytes incl. CRC16)
  uint8_t txBuf[RwProtocol::CMD_LEN] = {};

  // Periodic STATUS poll divider (incremented each PST cycle)
  uint32_t statusPollCnt{0};
  uint32_t statusPollDivider{100}; // runtime-configurable via parameter

  // STATUS timeout tracking (in PST cycles) after a poll was sent
  static constexpr uint8_t STATUS_TIMEOUT_CYCLES = 6;
  int8_t statusAwaitCnt{-1}; // -1 = not awaiting; >=0 = cycles since last poll

  // TC-driven STATUS data reply routing
  bool             pendingTcStatusTm{false};
  MessageQueueId_t pendingTcStatusReportedTo{MessageQueueIF::NO_QUEUE};

  // Datasets
  RwReplySet replySet{this};
  RwHkSet    hkSet{this};

  // Runtime parameters
  int16_t  p_maxRpm{4000};   // clamp for SET_SPEED
  uint16_t p_maxSlewRpmS{0}; // 0 = disabled

  // Simple FDIR thresholds & debounce counters
  static constexpr int16_t  STUCK_RPM_THRESH        = 50;
  static constexpr uint8_t  STUCK_DEBOUNCE_FRAMES   = 3;
  static constexpr int16_t  TORQUE_HIGH_MNM_THRESH  = 600;
  static constexpr uint8_t  TORQUE_DEBOUNCE_FRAMES  = 5;
  uint8_t stuckCnt{0};
  uint8_t torqueHighCnt{0};

  // RX ring buffer (thread-safe; external storage avoids dynamic allocation)
  static constexpr size_t RX_RING_SIZE = 256;
  uint8_t rxStorage[RX_RING_SIZE] = {};
  SharedRingBuffer rxRing{/*objectId*/ 0xDEADB011, rxStorage, RX_RING_SIZE,
                          /*overwriteOld*/ false,
                          /*maxExcessBytes*/ RwProtocol::STATUS_LEN - 1};

  // Parameter helper to process get/set requests
  ParameterHelper parameterHelper{this};
};

// Debug/trace compile-time switch
#ifndef RW_VERBOSE
#define RW_VERBOSE 0
#endif
