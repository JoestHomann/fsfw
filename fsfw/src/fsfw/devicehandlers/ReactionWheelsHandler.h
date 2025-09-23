#pragma once

#include "RwProtocol.h"
#include "RwConfig.h"

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

/**
 * ReactionWheelsHandler
 * - Supports torque and speed commands (mNm, RPM).
 * - Periodically polls STATUS frames and publishes HK into local data pool.
 * - Provides TC-driven immediate STATUS polling.
 */
class ReactionWheelsHandler : public DeviceHandlerBase {
 public:
  // Internal command IDs (must match wire protocol)
  enum DeviceCmd : DeviceCommandId_t {
    CMD_SET_SPEED   = 0x01,
    CMD_STOP        = 0x02,
    CMD_STATUS      = 0x03,
    CMD_STATUS_POLL = 0x1003, // internal "poll now" command
    CMD_SET_TORQUE  = 0x04    // torque in mNm (int16 payload)
  };

  // Unified internal reply ID for polled STATUS
  enum ReplyId : DeviceCommandId_t { REPLY_STATUS_POLL = 0x2002 };

  // Parameters (Domain = 0x42)
  static constexpr uint8_t PARAM_DOMAIN = 0x42;
  enum class ParamId : uint8_t {
    MAX_RPM        = 1,  // int16
    MAX_SLEW_RPM_S = 2,  // uint16
    POLL_DIVIDER   = 3   // uint32
  };

  // Local pool IDs
  enum class PoolIds : lp_id_t {
    RAW_REPLY        = 1,
    HK_SPEED_RPM     = 2, // int16
    HK_TORQUE_mNm    = 3, // int16 (measured)
    HK_RUNNING       = 4, // uint8
    HK_FLAGS         = 5, // uint16
    HK_ERROR         = 6, // uint16
    HK_CRC_ERR_CNT   = 7, // uint32
    HK_MALFORMED_CNT = 8, // uint32
    HK_TIMESTAMP_MS  = 9  // uint32
  };

  // Dataset IDs
  static constexpr uint32_t DATASET_ID_RAW = 0xCA;
  static constexpr uint32_t DATASET_ID_HK  = 0xCB;

  // RAW reply dataset (keeps last STATUS frame for debugging)
  struct RwReplySet : public LocalDataSet {
    LocalPoolVector<uint8_t, RwProtocol::STATUS_LEN> raw;
    explicit RwReplySet(HasLocalDataPoolIF* owner)
        : LocalDataSet(owner, DATASET_ID_RAW, 1),
          raw(owner->getObjectId(), static_cast<lp_id_t>(PoolIds::RAW_REPLY), this) {}
  };

  // HK dataset snapshot
  struct RwHkSet : public LocalDataSet {
    LocalPoolVariable<int16_t>  speedRpm;
    LocalPoolVariable<int16_t>  torque_mNm;
    LocalPoolVariable<uint8_t>  running;
    LocalPoolVariable<uint16_t> flags;
    LocalPoolVariable<uint16_t> error;
    LocalPoolVariable<uint32_t> crcErrCnt;
    LocalPoolVariable<uint32_t> malformedCnt;
    LocalPoolVariable<uint32_t> timestampMs;

    static constexpr uint16_t HK_VAR_COUNT = 8;

    explicit RwHkSet(HasLocalDataPoolIF* owner)
        : LocalDataSet(owner, DATASET_ID_HK, HK_VAR_COUNT),
          speedRpm(owner->getObjectId(), static_cast<lp_id_t>(PoolIds::HK_SPEED_RPM), this),
          torque_mNm(owner->getObjectId(), static_cast<lp_id_t>(PoolIds::HK_TORQUE_mNm), this),
          running(owner->getObjectId(), static_cast<lp_id_t>(PoolIds::HK_RUNNING), this),
          flags(owner->getObjectId(), static_cast<lp_id_t>(PoolIds::HK_FLAGS), this),
          error(owner->getObjectId(), static_cast<lp_id_t>(PoolIds::HK_ERROR), this),
          crcErrCnt(owner->getObjectId(), static_cast<lp_id_t>(PoolIds::HK_CRC_ERR_CNT), this),
          malformedCnt(owner->getObjectId(), static_cast<lp_id_t>(PoolIds::HK_MALFORMED_CNT), this),
          timestampMs(owner->getObjectId(), static_cast<lp_id_t>(PoolIds::HK_TIMESTAMP_MS), this) {}
  };

  // Transition delays
  static constexpr uint32_t RW_DELAY_OFF_TO_ON_MS    = RwConfig::DELAY_OFF_TO_ON_MS;
  static constexpr uint32_t RW_DELAY_ON_TO_NORMAL_MS = RwConfig::DELAY_ON_TO_NORMAL_MS;

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

  // Subscribe periodic HK after task creation
  ReturnValue_t initializeAfterTaskCreation() override;

  // Map SID -> dataset for HK service
  LocalPoolDataSetBase* getDataSetHandle(sid_t sid) override;

  // Forward "STATUS now" TC to next cycle (Action interface)
  ReturnValue_t executeAction(ActionId_t actionId, MessageQueueId_t commandedBy,
                              const uint8_t* data, size_t size) override;

  // Runtime parameters (parameter service)
  ReturnValue_t getParameter(uint8_t domainId, uint8_t parameterId,
                             ParameterWrapper* parameterWrapper,
                             const ParameterWrapper* newValues,
                             uint16_t startAtIndex) override;

 private:
  // RX helpers
  ReturnValue_t drainRxNow();
  ReturnValue_t drainRxIntoRing();
  void reportProtocolIssuesInWindow(const uint8_t* buf, size_t n);

  // Error bookkeeping
  void onCrcError();
  void onMalformed();

  // TX buffer (frame to be sent)
  uint8_t txBuf[RwProtocol::CMD_LEN] = {};

  // Polling / timeout supervision
  uint32_t statusPollCnt{0};
  uint32_t statusPollDivider{RwConfig::STATUS_POLL_DIVIDER_DEFAULT};
  static constexpr uint8_t STATUS_TIMEOUT_CYCLES = RwConfig::STATUS_TIMEOUT_CYCLES;
  int8_t statusAwaitCnt{-1};

  // TC-driven STATUS reply routing
  bool             pendingTcStatusTm{false};
  MessageQueueId_t pendingTcStatusReportedTo{MessageQueueIF::NO_QUEUE};

  // Datasets
  RwReplySet replySet{this};
  RwHkSet    hkSet{this};

  // Runtime parameters
  int16_t  p_maxRpm{RwConfig::MAX_RPM_DEFAULT};
  uint16_t p_maxSlewRpmS{RwConfig::MAX_SLEW_RPM_S_DEFAULT};

  // Simple FDIR thresholds / debounce (from config)
  static constexpr int16_t STUCK_RPM_THRESH       = RwConfig::STUCK_RPM_THRESH;
  static constexpr uint8_t STUCK_DEBOUNCE_FRAMES  = RwConfig::STUCK_DEBOUNCE_FRAMES;
  static constexpr int16_t TORQUE_HIGH_MNM_THRESH = RwConfig::TORQUE_HIGH_MNM_THRESH;
  static constexpr uint8_t TORQUE_DEBOUNCE_FRAMES = RwConfig::TORQUE_DEBOUNCE_FRAMES;

  uint8_t stuckCnt{0};
  uint8_t torqueHighCnt{0};

  // Error counters (+ thresholds from Config)
  uint32_t crcErrCnt{0};
  uint32_t malformedCnt{0};
  static constexpr uint32_t CRC_ERR_EVENT_THRESH   = RwConfig::CRC_ERR_EVENT_THRESH;
  static constexpr uint32_t MALFORMED_EVENT_THRESH = RwConfig::MALFORMED_EVENT_THRESH;

  // RX ring buffer (collects stream; we search for latest valid STATUS)
  static constexpr std::size_t RX_RING_SIZE = RwConfig::RX_RING_SIZE;
  uint8_t rxStorage[RX_RING_SIZE] = {};
  SharedRingBuffer rxRing{/*objectId*/ RwConfig::RX_RING_OBJ_ID, rxStorage, RX_RING_SIZE,
                          /*overwriteOld*/ false,
                          /*maxExcessBytes*/ RwProtocol::STATUS_LEN - 1};

  // Parameter helper
  ParameterHelper parameterHelper{this};
};

// Debug switch
#ifndef RW_VERBOSE
#define RW_VERBOSE 0
#endif
