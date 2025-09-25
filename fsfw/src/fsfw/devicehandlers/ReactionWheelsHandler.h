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
  // Wire-protocol aligned command IDs
  enum DeviceCmd : DeviceCommandId_t {
    CMD_SET_SPEED = 0x01,
    CMD_STOP = 0x02,
    CMD_STATUS = 0x03,
    CMD_STATUS_POLL = 0x1003,  // internal "poll now" command
    CMD_SET_TORQUE = 0x04      // torque in mNm
  };

  // Unified internal reply ID for polled STATUS
  enum ReplyId : DeviceCommandId_t { REPLY_STATUS_POLL = 0x2002 };

  // ---------------- Parameter service ---------------------------------------
  static constexpr uint8_t PARAM_DOMAIN = 0x42;
  enum class ParamId : uint8_t { MAX_RPM = 1, MAX_SLEW_RPM_S = 2, POLL_DIVIDER = 3 };

  // ---------------- Local pool layout ---------------------------------------
  enum class PoolIds : lp_id_t {
    RAW_REPLY = 1,
    HK_SPEED_RPM = 2,
    HK_TORQUE_mNm = 3,
    HK_RUNNING = 4,
    HK_FLAGS = 5,
    // 6 reserved (HK_ERROR removed)
    HK_CRC_ERR_CNT = 7,
    HK_MALFORMED_CNT = 8,
    HK_TIMESTAMP_MS = 9
  };

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
    LocalPoolVariable<int16_t> speedRpm;       // RW wheel speed
    LocalPoolVariable<int16_t> torque_mNm;     // RW torque
    LocalPoolVariable<uint8_t> running;        // 1 if wheel reports running
    LocalPoolVariable<uint16_t> flags;         // per-frame flags
    LocalPoolVariable<uint32_t> crcErrCnt;     // CRC errors seen
    LocalPoolVariable<uint32_t> malformedCnt;  // malformed frames seen
    LocalPoolVariable<uint32_t> timestampMs;   // reception timestamp

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

  // ---------------- Transition delays ---------------------------------------
  static constexpr uint32_t RW_DELAY_OFF_TO_ON_MS = RwConfig::DELAY_OFF_TO_ON_MS;
  static constexpr uint32_t RW_DELAY_ON_TO_NORMAL_MS = RwConfig::DELAY_ON_TO_NORMAL_MS;

  // ---------------- Lifecycle ------------------------------------------------
  ReactionWheelsHandler(object_id_t objectId, object_id_t comIF, CookieIF* cookie);

  void doStartUp() override;    // set MODE_NORMAL
  void doShutDown() override;   // set MODE_OFF
  void modeChanged() override;  // log mode change

  // ---------------- DeviceHandlerBase hooks ---------------------------------
  ReturnValue_t buildNormalDeviceCommand(DeviceCommandId_t* id) override;      // periodic/TC polls
  ReturnValue_t buildTransitionDeviceCommand(DeviceCommandId_t* id) override;  // none
  ReturnValue_t buildCommandFromCommand(DeviceCommandId_t deviceCommand, const uint8_t* data,
                                        size_t len) override;  // build wire cmd
  ReturnValue_t scanForReply(const uint8_t* start, size_t len, DeviceCommandId_t* foundId,
                             size_t* foundLen) override;  // parse/ring
  ReturnValue_t interpretDeviceReply(DeviceCommandId_t id,
                                     const uint8_t* packet) override;  // update HK/FDIR
  uint32_t getTransitionDelayMs(Mode_t from, Mode_t to) override;      // delays from config

  ReturnValue_t initializeLocalDataPool(localpool::DataPool& localDataPoolMap,
                                        LocalDataPoolManager& poolManager) override;  // pool layout
  void fillCommandAndReplyMap() override;                                             // DH maps

  // Subscribe periodic HK after task creation
  ReturnValue_t initializeAfterTaskCreation() override;

  // Map SID -> dataset for HK service
  LocalPoolDataSetBase* getDataSetHandle(sid_t sid) override;

  // Forward "STATUS now" TC to next cycle (Action interface)
  ReturnValue_t executeAction(ActionId_t actionId, MessageQueueId_t commandedBy,
                              const uint8_t* data, size_t size) override;

  // Runtime parameters (parameter service)
  ReturnValue_t getParameter(uint8_t domainId, uint8_t parameterId,
                             ParameterWrapper* parameterWrapper, const ParameterWrapper* newValues,
                             uint16_t startAtIndex) override;

 private:
  // ---------------- RX helpers ----------------------------------------------
  ReturnValue_t drainRxNow();       // drop a few stale frames from COM IF
  ReturnValue_t drainRxIntoRing();  // pull available bytes into ring
  void reportProtocolIssuesInWindow(const uint8_t* buf, size_t n);  // count CRC/malformed

  // ---------------- Error bookkeeping ---------------------------------------
  void handleCrcError();   // Handle CRC error: Count and trigger event
  void handleMalformed();  // Handle malformed: Count and trigger event

  // ---------------- TX buffer ------------------------------------------------
  uint8_t txBuf[RwProtocol::CMD_LEN] = {};  // last command frame to send

  // ---------------- Polling / timeout supervision ---------------------------
  uint32_t statusPollCnt{0};                                          // poll divider counter
  uint32_t statusPollDivider{RwConfig::STATUS_POLL_DIVIDER_DEFAULT};  // polls every N cycles
  static constexpr uint8_t STATUS_TIMEOUT_CYCLES = RwConfig::STATUS_TIMEOUT_CYCLES;  // wait cycles
  int8_t statusAwaitCnt{-1};  // >=0 means waiting for STATUS reply

  // ---------------- Poll block after external command ---------------------
  uint32_t lastExtCmdMs{0};                        // timestamp of last TC sent
  static constexpr uint32_t POLL_BLOCK_MS = 50;  // block polls for 50 ms after TC

  // ---------------- TC-driven STATUS routing --------------------------------
  bool pendingTcStatusTm{false};  // set when TC requested STATUS
  MessageQueueId_t pendingTcStatusReportedTo{MessageQueueIF::NO_QUEUE};  // who to echo data to

  // ---------------- Local datasets ------------------------------------------
  RwReplySet replySet{this};  // last raw STATUS frame
  RwHkSet hkSet{this};        // housekeeping snapshot

  // ---------------- Runtime parameters --------------------------------------
  int16_t p_maxRpm{RwConfig::MAX_RPM_DEFAULT};  // limit for SET_SPEED

  // ---------------- Simple FDIR thresholds / debounce -----------------------
  static constexpr int16_t STUCK_RPM_THRESH =
      RwConfig::STUCK_RPM_THRESH;  // speed threshold for stuck check
  static constexpr uint8_t STUCK_RPM_COUNT = RwConfig::STUCK_RPM_COUNT;  // count for stuck check
  static constexpr int16_t HIGH_TORQUE_THRESH =
      RwConfig::HIGH_TORQUE_THRESH;  // torque safety threshold
  static constexpr uint8_t HIGH_TORQUE_COUNT =
      RwConfig::HIGH_TORQUE_COUNT;  // count for torque safety

  uint8_t stuckRpmCnt{0};    // Counter for stuck detection
  uint8_t highTorqueCnt{0};  // Counter for high torque

  // ---------------- Error counters ------------------------------------------
  uint32_t crcErrCnt{0};     // CRC error counter
  uint32_t malformedCnt{0};  // malformed frame counter
  static constexpr uint32_t CRC_ERR_EVENT_THRESH = RwConfig::CRC_ERR_EVENT_THRESH;
  static constexpr uint32_t MALFORMED_EVENT_THRESH = RwConfig::MALFORMED_EVENT_THRESH;

  // ---------------- RX ring buffer ------------------------------------------
  static constexpr std::size_t RX_RING_SIZE = RwConfig::RX_RING_SIZE;  // bytes in ring
  uint8_t rxStorage[RX_RING_SIZE] = {};                                // backing storage
  SharedRingBuffer rxRing{// stream buffer for searching frames
                          /*objectId*/ RwConfig::RX_RING_OBJ_ID, rxStorage, RX_RING_SIZE,
                          /*overwriteOld*/ false,
                          /*maxExcessBytes*/ RwProtocol::STATUS_LEN - 1};

  // ---------------- Parameter helper ----------------------------------------
  ParameterHelper parameterHelper{this};  // FSFW parameter interface helper
};

// Debug on/off switch (set to 1 to enable verbose logging)
#ifndef RW_VERBOSE
#define RW_VERBOSE 0
#endif
