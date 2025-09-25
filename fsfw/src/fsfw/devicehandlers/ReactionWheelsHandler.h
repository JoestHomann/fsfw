#pragma once

#include "RwConfig.h"
#include "RwProtocol.h"
#include "fsfw/container/SharedRingBuffer.h"
#include "fsfw/datapoollocal/LocalDataPoolManager.h"
#include "fsfw/datapoollocal/LocalDataSet.h"
#include "fsfw/datapoollocal/LocalPoolVariable.h"
#include "fsfw/datapoollocal/LocalPoolVector.h"
#include "fsfw/devicehandlers/DeviceHandlerBase.h"
#include "fsfw/ipc/MessageQueueIF.h"
#include "fsfw/parameters/ParameterHelper.h"
#include "fsfw/parameters/ParameterWrapper.h"
#include "fsfw/returnvalues/returnvalue.h"

// ReactionWheelsHandler
// - Supports torque and speed commands (mNm, RPM).
// - Periodically polls STATUS frames and publishes HK into local data pool.
// - Provides TC-driven immediate STATUS polling.

class ReactionWheelsHandler : public DeviceHandlerBase {
 public:
  // ---------------- Command / Reply IDs -------------------------------------
  enum DeviceCmd : DeviceCommandId_t {
    CMD_SET_SPEED = 0x01,
    CMD_STOP = 0x02,
    CMD_STATUS = 0x03,
    CMD_STATUS_POLL = 0x1003,
    CMD_SET_TORQUE = 0x04
  };

  // Internal reply ID for STATUS replies
  enum ReplyId : DeviceCommandId_t { REPLY_STATUS_POLL = 0x2002 };

  // ---------------- Parameter service ---------------------------------------
  static constexpr uint8_t PARAM_DOMAIN = 0x42;
  enum class ParamId : uint8_t { MAX_RPM = 1, POLL_DIVIDER = 2 };

  // ---------------- Local pool layout ---------------------------------------
  enum class PoolIds : lp_id_t {
    RAW_REPLY = 1,
    HK_SPEED_RPM = 2,
    HK_TORQUE_mNm = 3,
    HK_RUNNING = 4,
    HK_FLAGS = 5,
    HK_CRC_ERR_CNT = 6,
    HK_MALFORMED_CNT = 7,
    HK_TIMESTAMP_MS = 8
  };

  // --------------- HK flag bits --------------------------------
  static constexpr uint16_t FLAG_STUCK = 0x0001;        // running==0 and |speed| above threshold
  static constexpr uint16_t FLAG_TORQUE_HIGH = 0x0002;  // |torque| above threshold

  // Dataset IDs
  static constexpr uint32_t DATASET_ID_RAW = 0xCA;
  static constexpr uint32_t DATASET_ID_HK = 0xCB;

  // RAW reply dataset (keeps last STATUS frame for debugging)
  struct RwReplySet : public LocalDataSet {
    LocalPoolVector<uint8_t, RwProtocol::STATUS_LEN> raw;  // last raw STATUS frame
    explicit RwReplySet(HasLocalDataPoolIF* owner)
        : LocalDataSet(owner, DATASET_ID_RAW, 1),
          raw(owner->getObjectId(), static_cast<lp_id_t>(PoolIds::RAW_REPLY), this) {}
  };

  // HK dataset snapshot
  struct RwHkSet : public LocalDataSet {
    LocalPoolVariable<int16_t> speedRpm;
    LocalPoolVariable<int16_t> torque_mNm;
    LocalPoolVariable<uint8_t> running;
    LocalPoolVariable<uint16_t> flags;
    LocalPoolVariable<uint32_t> crcErrCnt;
    LocalPoolVariable<uint32_t> malformedCnt;
    LocalPoolVariable<uint32_t> timestampMs;

    static constexpr uint16_t HK_VAR_COUNT = 7;

    explicit RwHkSet(HasLocalDataPoolIF* owner)
        : LocalDataSet(owner, DATASET_ID_HK, HK_VAR_COUNT),
          speedRpm(owner->getObjectId(), static_cast<lp_id_t>(PoolIds::HK_SPEED_RPM), this),
          torque_mNm(owner->getObjectId(), static_cast<lp_id_t>(PoolIds::HK_TORQUE_mNm), this),
          running(owner->getObjectId(), static_cast<lp_id_t>(PoolIds::HK_RUNNING), this),
          flags(owner->getObjectId(), static_cast<lp_id_t>(PoolIds::HK_FLAGS), this),
          crcErrCnt(owner->getObjectId(), static_cast<lp_id_t>(PoolIds::HK_CRC_ERR_CNT), this),
          malformedCnt(owner->getObjectId(), static_cast<lp_id_t>(PoolIds::HK_MALFORMED_CNT), this),
          timestampMs(owner->getObjectId(), static_cast<lp_id_t>(PoolIds::HK_TIMESTAMP_MS), this) {}
  };

  // ---------------- Modes ------------------------------------------------
  ReactionWheelsHandler(object_id_t objectId, object_id_t comIF, CookieIF* cookie);

  void doStartUp() override;
  void doShutDown() override;
  void modeChanged() override;

  // ---------------- DeviceHandlerBase hooks ---------------------------------
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

  ReturnValue_t initializeAfterTaskCreation() override;

  LocalPoolDataSetBase* getDataSetHandle(sid_t sid) override;

  ReturnValue_t executeAction(ActionId_t actionId, MessageQueueId_t commandedBy,
                              const uint8_t* data, size_t size) override;

  ReturnValue_t getParameter(uint8_t domainId, uint8_t parameterId,
                             ParameterWrapper* parameterWrapper, const ParameterWrapper* newValues,
                             uint16_t startAtIndex) override;

 private:
  // ---------------- Mode-Transition Delays ------------------------------------
  static constexpr uint32_t RW_DELAY_OFF_TO_ON_MS = RwConfig::DELAY_OFF_TO_ON_MS;
  static constexpr uint32_t RW_DELAY_ON_TO_NORMAL_MS = RwConfig::DELAY_ON_TO_NORMAL_MS;
  static constexpr uint32_t STOP_DELAY_MS = RwConfig::STOP_DELAY_MS;

  // ---------------- Mode-Transition retries -----------------------------------
  static constexpr uint8_t STOP_RETRIES = RwConfig::STOP_RETRIES;
  static constexpr uint32_t STOP_RETRY_MS = RwConfig::STOP_RETRY_MS;
  static constexpr uint8_t START_RETRIES = RwConfig::START_RETRIES;
  static constexpr uint32_t START_RETRY_MS = RwConfig::START_RETRY_MS;
  static constexpr uint32_t POLL_BLOCK_MS = RwConfig::POLL_BLOCK_MS;

  // ---------------- Mode machine states -------------------------------------

  // Shutdown
  bool shuttingDown_{false};
  bool stopSent_{false};
  uint32_t shutdownT0_{0};
  uint32_t lastStopTxMs_{0};
  uint8_t stopRetriesDone_{0};

  // Startup
  bool startingUp_{false};
  bool startupSent_{false};
  uint8_t startupRetriesDone_{0};
  uint32_t startupT0_{0};
  uint32_t startupLastTxMs_{0};

  // ---------------- RX helpers ----------------------------------------------
  ReturnValue_t drainRxNow();
  ReturnValue_t drainRxIntoRing();
  void reportProtocolIssuesInWindow(const uint8_t* buf, size_t n);

  // ---------------- Error handling ------------------------------------------
  void handleCrcError();
  void handleMalformed();

  // ---------------- Error counters ------------------------------------------
  uint32_t crcErrCnt{0};
  uint32_t malformedCnt{0};
  static constexpr uint32_t CRC_ERR_EVENT_THRESH = RwConfig::CRC_ERR_EVENT_THRESH;
  static constexpr uint32_t MALFORMED_EVENT_THRESH = RwConfig::MALFORMED_EVENT_THRESH;

  // ---------------- TX buffer ------------------------------------------------
  uint8_t txBuf[RwProtocol::CMD_LEN] = {};

  // ---------------- Polling / timeout supervision ---------------------------
  uint32_t statusPollCnt{0};
  uint32_t statusPollDivider{RwConfig::STATUS_POLL_DIVIDER_DEFAULT};
  static constexpr uint8_t STATUS_TIMEOUT_CYCLES = RwConfig::STATUS_TIMEOUT_CYCLES;
  int8_t statusAwaitCnt{-1};

  // ---------------- Poll block after external command -----------------------
  uint32_t lastExtCmdMs{0};

  // ---------------- TC-driven STATUS routing --------------------------------
  bool pendingTcStatusTm{false};
  MessageQueueId_t pendingTcStatusReportedTo{MessageQueueIF::NO_QUEUE};

  // ---------------- Local datasets ------------------------------------------
  RwReplySet replySet{this};
  RwHkSet hkSet{this};

  // ---------------- Runtime parameters --------------------------------------
  int16_t p_maxRpm{RwConfig::MAX_RPM_DEFAULT};

  // ---------------- FDIR thresholds / counts --------------------------------
  static constexpr int16_t STUCK_RPM_THRESH = RwConfig::STUCK_RPM_THRESH;
  static constexpr uint8_t STUCK_RPM_COUNT = RwConfig::STUCK_RPM_COUNT;
  static constexpr int16_t HIGH_TORQUE_THRESH = RwConfig::HIGH_TORQUE_THRESH;
  static constexpr uint8_t HIGH_TORQUE_COUNT = RwConfig::HIGH_TORQUE_COUNT;

  uint8_t stuckRpmCnt{0};
  uint8_t highTorqueCnt{0};

  // ---------------- RX ring buffer ------------------------------------------
  static constexpr std::size_t RX_RING_SIZE = RwConfig::RX_RING_SIZE;
  uint8_t rxStorage[RX_RING_SIZE] = {};
  SharedRingBuffer rxRing{/*objectId*/ RwConfig::RX_RING_OBJ_ID, rxStorage, RX_RING_SIZE,
                          /*overwriteOld*/ false,
                          /*maxExcessBytes*/ RwProtocol::STATUS_LEN - 1};

  // ---------------- Parameter helper ----------------------------------------
  ParameterHelper parameterHelper{this};
};

// Debug on/off switch (set to 1 to enable verbose logging)
#ifndef RW_VERBOSE
#define RW_VERBOSE 0
#endif
