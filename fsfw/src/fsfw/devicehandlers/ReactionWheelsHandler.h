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

// Device handler for reaction wheels
// - English ASCII-only comments
// - All wire I/O via RwProtocol (no raw offsets here)

class ReactionWheelsHandler : public DeviceHandlerBase {
 public:
  // ---------------- Command / Reply IDs -------------------------------------
  // Wire-aligned command IDs (plus one internal poll command)
  enum DeviceCmd : DeviceCommandId_t {
    CMD_SET_SPEED   = 0x01,     // payload: int16 rpm
    CMD_STOP        = 0x02,     // no payload
    CMD_STATUS      = 0x03,     // no payload (TC-triggered status)
    CMD_SET_TORQUE  = 0x04,     // payload: int16 mNm
    CMD_STATUS_POLL = 0x1003    // internal periodic/forced poll
  };

  // Unified internal reply ID for polled STATUS
  enum ReplyId : DeviceCommandId_t { REPLY_STATUS_POLL = 0x2002 };

  // ---------------- Parameter service ---------------------------------------
  static constexpr uint8_t PARAM_DOMAIN = 0x42;
  enum class ParamId : uint8_t {
    MAX_RPM        = 1,   // int16 rpm
    MAX_SLEW_RPM_S = 2,   // int16 rpm/s
    POLL_DIVIDER   = 3    // uint16 cycles
  };

  // ---------------- Local pool layout ---------------------------------------
  enum class PoolIds : lp_id_t {
    RAW_REPLY        = 1,
    HK_SPEED_RPM     = 2,
    HK_TORQUE_mNm    = 3,
    HK_RUNNING       = 4,
    HK_FLAGS         = 5,
    HK_ERROR         = 6,
    HK_CRC_ERR_CNT   = 7,
    HK_MALFORMED_CNT = 8,
    HK_TIMESTAMP_MS  = 9
  };

  // Dataset IDs
  static constexpr uint32_t DATASET_ID_RAW = 0xCA;
  static constexpr uint32_t DATASET_ID_HK  = 0xCB;

  // RAW reply dataset (keeps last STATUS frame for debugging)
  struct RwReplySet : public LocalDataSet {
    LocalPoolVector<uint8_t, RwProtocol::STATUS_LEN> raw; // last raw STATUS frame
    explicit RwReplySet(HasLocalDataPoolIF* owner)
        : LocalDataSet(owner, DATASET_ID_RAW, 1),
          raw(owner->getObjectId(), static_cast<lp_id_t>(PoolIds::RAW_REPLY), this) {}
  };

  // HK dataset snapshot
  struct RwHkSet : public LocalDataSet {
    LocalPoolVariable<int16_t>  speedRpm;      // unit: rpm (signed)
    LocalPoolVariable<int16_t>  torque_mNm;    // unit: mNm (signed)
    LocalPoolVariable<uint8_t>  running;       // unit: 0/1
    LocalPoolVariable<uint16_t> flags;         // FDIR flags bitfield
    LocalPoolVariable<uint16_t> error;         // last error code (0=OK)
    LocalPoolVariable<uint32_t> crcErrCnt;     // CRC errors seen
    LocalPoolVariable<uint32_t> malformedCnt;  // malformed frames seen
    LocalPoolVariable<uint32_t> timestampMs;   // reception timestamp (uptime ms)

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

  // ---------------- Transition delays ---------------------------------------
  static constexpr uint32_t RW_DELAY_OFF_TO_ON_MS    = RwConfig::DELAY_OFF_TO_ON_MS;
  static constexpr uint32_t RW_DELAY_ON_TO_NORMAL_MS = RwConfig::DELAY_ON_TO_NORMAL_MS;

  // ---------------- Lifecycle ------------------------------------------------
  ReactionWheelsHandler(object_id_t objectId, object_id_t comIF, CookieIF* cookie);

  void doStartUp() override;   // set MODE_NORMAL
  void doShutDown() override;  // set MODE_OFF
  void modeChanged() override; // log mode change

  // ---------------- DeviceHandlerBase hooks ---------------------------------
  ReturnValue_t buildNormalDeviceCommand(DeviceCommandId_t* id) override;       // periodic/TC polls
  ReturnValue_t buildTransitionDeviceCommand(DeviceCommandId_t* id) override;   // none
  ReturnValue_t buildCommandFromCommand(DeviceCommandId_t deviceCommand,
                                        const uint8_t* data, size_t len) override; // build wire cmd
  ReturnValue_t scanForReply(const uint8_t* start, size_t len,
                             DeviceCommandId_t* foundId, size_t* foundLen) override;
  ReturnValue_t interpretDeviceReply(DeviceCommandId_t id, const uint8_t* packet) override;
  void fillCommandAndReplyMap() override;
  ReturnValue_t performOperation(uint8_t opCode) override;

  // Parameter interface
  ReturnValue_t getParameter(uint8_t domainId, uint8_t parameterId,
                             ParameterWrapper* parameterWrapper,
                             const ParameterWrapper* newValues,
                             uint16_t startAtIndex) override;

 private:
  // ---------------- RX helpers ----------------------------------------------
  ReturnValue_t drainRxNow();          // drop a few stale frames from COM IF
  ReturnValue_t drainRxIntoRing();     // pull available bytes into ring
  void reportProtocolIssuesInWindow(const uint8_t* buf, size_t n); // count CRC/malformed

  // ---------------- Error bookkeeping ---------------------------------------
  void onCrcError();     // bump CRC error count and maybe event
  void onMalformed();    // bump malformed count and maybe event

  // ---------------- TX buffer ------------------------------------------------
  uint8_t txBuf[RwProtocol::CMD_LEN] = {};  // last command frame to send

  // ---------------- Polling / timeout supervision ---------------------------
  uint32_t statusPollCnt{0};                          // poll divider counter
  uint32_t statusPollDivider{RwConfig::STATUS_POLL_DIVIDER_DEFAULT}; // polls every N cycles
  static constexpr uint8_t  STATUS_TIMEOUT_CYCLES = RwConfig::STATUS_TIMEOUT_CYCLES;
  int8_t   statusAwaitCnt{-1};                        // >=0 means waiting for STATUS reply

  // ---------------- Poll inhibit after external command ---------------------
  uint32_t lastExtCmdMs{0};                           // timestamp of last TC sent
  static constexpr uint32_t POLL_INHIBIT_MS = RwConfig::POLL_INHIBIT_MS; // inhibit window

  // ---------------- TC-driven STATUS routing --------------------------------
  bool             pendingTcStatusTm{false};          // true if a TC requested STATUS TM
  MessageQueueId_t pendingTcStatusReportedTo{MessageQueueIF::NO_QUEUE}; // who to echo data to

  // ---------------- RX ring buffer ------------------------------------------
  static constexpr std::size_t RX_RING_SIZE = RwConfig::RX_RING_SIZE; // bytes in ring
  uint8_t rxStorage[RX_RING_SIZE] = {};               // backing storage
  SharedRingBuffer rxRing{
      /*objectId*/ RwConfig::RX_RING_OBJ_ID,
      rxStorage,
      RX_RING_SIZE,
      /*overwriteOld*/ false,
      /*maxExcessBytes*/ RwProtocol::STATUS_LEN - 1};

  // ---------------- FDIR thresholds / events --------------------------------
  static constexpr int16_t STUCK_RPM_THRESH       = RwConfig::STUCK_RPM_THRESH;
  static constexpr uint8_t STUCK_DEBOUNCE_FRAMES  = RwConfig::STUCK_DEBOUNCE_FRAMES;
  static constexpr int16_t TORQUE_HIGH_MNM_THRESH = RwConfig::TORQUE_HIGH_MNM_THRESH;
  static constexpr uint8_t TORQUE_DEBOUNCE_FRAMES = RwConfig::TORQUE_DEBOUNCE_FRAMES;
  static constexpr uint32_t CRC_ERR_EVENT_THRESH  = RwConfig::CRC_ERR_EVENT_THRESH;
  static constexpr uint32_t MALFORMED_EVENT_THRESH= RwConfig::MALFORMED_EVENT_THRESH;

  // ---------------- FDIR / diagnostics counters -----------------------------
  uint16_t fdirFlags{0};       // bitfield: bit0=timeout, bit1=crc_err, bit2=malformed, etc.
  uint16_t lastErrorCode{0};   // last protocol/device error code (0=OK)
  uint32_t crcErrCnt{0};       // rolling counter
  uint32_t malformedCnt{0};    // rolling counter
  uint32_t rxDropCnt{0};     // RX ring overflow count

  // ---------------- Local datasets ------------------------------------------
  RwReplySet replySet{this};   // last raw STATUS frame
  RwHkSet    hkSet{this};      // housekeeping snapshot

  // ---------------- Runtime parameters --------------------------------------
  int16_t  p_maxRpm{RwConfig::MAX_RPM_DEFAULT};           // clamp for SET_SPEED
  uint16_t p_maxSlewRpmS{RwConfig::MAX_SLEW_RPM_S_DEFAULT}; // optional slew limit

  // ---------------- Parameter helper ----------------------------------------
  ParameterHelper parameterHelper{this};  // FSFW parameter interface helper
};

// Debug on/off switch (set to 1 to enable verbose logging)
#ifndef RW_VERBOSE
#define RW_VERBOSE 0
#endif
