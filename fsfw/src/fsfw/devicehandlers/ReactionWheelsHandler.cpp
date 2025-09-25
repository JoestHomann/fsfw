#include "ReactionWheelsHandler.h"

#include <cmath>
#include <cstring>
#include <vector>

#include "RwEvents.h"
#include "RwProtocol.h"
#include "fsfw/action/ActionHelper.h"
#include "fsfw/devicehandlers/DeviceHandlerIF.h"
#include "fsfw/ipc/MessageQueueIF.h"
#include "fsfw/ipc/MutexIF.h"
#include "fsfw/parameters/ParameterMessage.h"
#include "fsfw/returnvalues/returnvalue.h"
#include "fsfw/serviceinterface/ServiceInterfaceStream.h"
#include "fsfw/timemanager/Clock.h"
#include "fsfw/datapoollocal/ProvidesDataPoolSubscriptionIF.h"

// -------- Local constants (keep small, device-friendly) ----------------------
// Wait time after STOP before power-off (ms)
static constexpr uint32_t COAST_WAIT_MS = 80;

// -------- Construction -------------------------------------------------------
ReactionWheelsHandler::ReactionWheelsHandler(object_id_t objectId, object_id_t comIF,
                                             CookieIF* cookie)
    : DeviceHandlerBase(objectId, comIF, cookie) {
  rxRing.setToUseReceiveSizeFIFO(/*fifoDepth*/ 8);
  (void)rxRing.initialize();
}

// -------- Mode Handling: STARTUP (OFF -> ON) --------------------------------
// Startup handshake: drain RX, send one STATUS, wait short window for reply.
// If no reply arrives within OFF->ON delay, continue to ON but log via counters.
void ReactionWheelsHandler::doStartUp() {
  // Static FSM local to avoid header changes
  enum class S { IDLE, SEND_STATUS, WAIT_STATUS } ;
  static S s = S::IDLE;
  static uint32_t t0 = 0;
  static bool sentOnce = false;

  switch (s) {
    case S::IDLE: {
      (void)drainRxNow();  // drop stale bytes after power-up
      s = S::SEND_STATUS;
      [[fallthrough]];
    }
    case S::SEND_STATUS: {
      const size_t total = RwProtocol::buildStatusReq(txBuf, sizeof(txBuf));
      if (total != 0) {
        rawPacket = txBuf;
        rawPacketLen = total;
        statusAwaitCnt = 0;  // open reply window
        Clock::getUptime(&t0);
        sentOnce = true;
        s = S::WAIT_STATUS;
        return;  // stay in transition
      }
      // If we cannot build a frame, do not block startup
      s = S::WAIT_STATUS;
      Clock::getUptime(&t0);
      return;
    }
    case S::WAIT_STATUS: {
      // Consider startup successful if reply was seen (statusAwaitCnt reset by interpret)
      if (statusAwaitCnt < 0) {
        s = S::IDLE;
        sentOnce = false;
        setMode(MODE_ON);
        return;
      }
      // Allow graceful timeout using framework delay OFF->ON
      uint32_t now = 0;
      Clock::getUptime(&now);
      const uint32_t waitMs = RwConfig::DELAY_OFF_TO_ON_MS;
      if (sentOnce && (now - t0) >= waitMs) {
        // No reply, continue to ON but keep counters; FDIR will catch if device stays silent
        s = S::IDLE;
        sentOnce = false;
        setMode(MODE_ON);
        return;
      }
      return;  // keep waiting
    }
  }
}

// -------- Mode Handling: SHUTDOWN (ANY -> OFF) -------------------------------
// Coast-down before power-off: send STOP once, wait a short time, then OFF.
void ReactionWheelsHandler::doShutDown() {
  enum class S { SEND_COAST, WAIT_COAST } ;
  static S s = S::SEND_COAST;
  static uint32_t t0 = 0;
  switch (s) {
    case S::SEND_COAST: {
      const size_t total = RwProtocol::buildStop(txBuf, sizeof(txBuf));
      if (total != 0) {
        rawPacket = txBuf;
        rawPacketLen = total;
      }
      Clock::getUptime(&t0);
      s = S::WAIT_COAST;
      return;  // stay in transition
    }
    case S::WAIT_COAST: {
      uint32_t now = 0;
      Clock::getUptime(&now);
      if ((now - t0) >= COAST_WAIT_MS) {
        s = S::SEND_COAST;  // reset
        setMode(MODE_OFF);
        return;
      }
      return;  // keep waiting
    }
  }
}

// -------- Mode change hook ---------------------------------------------------
// Force immediate STATUS after entering NORMAL to sync state quickly.
void ReactionWheelsHandler::modeChanged() {
  Mode_t m{}; Submode_t s{};
  this->getMode(&m, &s);
  sif::info << "ReactionWheelsHandler: modeChanged -> " << static_cast<int>(m)
            << " (sub=" << static_cast<int>(s) << ")" << std::endl;

  if (m == MODE_NORMAL) {
    (void)drainRxNow();               // clear any stale bytes
    statusPollCnt = statusPollDivider;  // force next buildNormal() to poll immediately
  }
}

// -------- HK subscription ----------------------------------------------------
ReturnValue_t ReactionWheelsHandler::initializeAfterTaskCreation() {
  ReturnValue_t rv = DeviceHandlerBase::initializeAfterTaskCreation();
  if (rv != returnvalue::OK) {
    return rv;
  }
  const sid_t hkSid{getObjectId(), DATASET_ID_HK};
  subdp::RegularHkPeriodicParams params(hkSid, /*enable*/ true, /*period s*/ RwConfig::HK_PERIOD_S);
  auto* hkMgr = getHkManagerHandle();
  if (hkMgr == nullptr) {
    return returnvalue::FAILED;
  }
  return hkMgr->subscribeForRegularPeriodicPacket(params);
}

// -------- RX helpers ---------------------------------------------------------
ReturnValue_t ReactionWheelsHandler::drainRxNow() {
  if (communicationInterface == nullptr || comCookie == nullptr) {
    return returnvalue::OK;
  }
  for (int i = 0; i < 3; ++i) {
    ReturnValue_t rvReq =
        communicationInterface->requestReceiveMessage(comCookie, RwProtocol::STATUS_LEN);
    if (rvReq != returnvalue::OK) break;
    uint8_t* buf = nullptr;
    size_t sz = 0;
    ReturnValue_t rvRead = communicationInterface->readReceivedMessage(comCookie, &buf, &sz);
    if (rvRead != returnvalue::OK || buf == nullptr || sz == 0) break;
  }
  return returnvalue::OK;
}

ReturnValue_t ReactionWheelsHandler::drainRxIntoRing() {
  if (communicationInterface == nullptr || comCookie == nullptr) {
    return returnvalue::OK;
  }
  while (true) {
    (void)rxRing.lockRingBufferMutex(MutexIF::TimeoutType::WAITING, /*timeout ms*/ 2);
    const size_t freeSpace = rxRing.availableWriteSpace();
    if (freeSpace == 0) {
      (void)rxRing.unlockRingBufferMutex();
      break;
    }
    const size_t chunk = (freeSpace < 128) ? freeSpace : size_t(128);
    ReturnValue_t req = communicationInterface->requestReceiveMessage(comCookie, chunk);
    if (req != returnvalue::OK) {
      (void)rxRing.unlockRingBufferMutex();
      break;
    }
    uint8_t* buf = nullptr; size_t sz = 0;
    ReturnValue_t rd = communicationInterface->readReceivedMessage(comCookie, &buf, &sz);
    if (rd != returnvalue::OK || buf == nullptr || sz == 0) {
      (void)rxRing.unlockRingBufferMutex();
      break;
    }
    const size_t wrote = rxRing.writeData(buf, sz);
    if (wrote != sz) {
      sif::warning << "RW rxRing overflow: dropped " << (sz - wrote) << " bytes" << std::endl;
    }
    (void)rxRing.unlockRingBufferMutex();
  }
  return returnvalue::OK;
}

// -------- NORMAL mode: periodic STATUS polls --------------------------------
ReturnValue_t ReactionWheelsHandler::buildNormalDeviceCommand(DeviceCommandId_t* id) {
  // Avoid duplicate polls while waiting for a reply
  if (statusAwaitCnt >= 0) {
    if (++statusAwaitCnt > STATUS_TIMEOUT_CYCLES) {
      triggerEvent(RwEvents::TIMEOUT, 0, 0);
      statusAwaitCnt = -1;  // close poll window on timeout
    } else {
      *id = DeviceHandlerIF::NO_COMMAND_ID;
      return NOTHING_TO_SEND;
    }
  }

  // Block polls briefly after TC-driven actuator commands
  uint32_t nowMs = 0;
  Clock::getUptime(&nowMs);
  if (lastExtCmdMs != 0 && (nowMs - lastExtCmdMs) < POLL_BLOCK_MS) {
    *id = DeviceHandlerIF::NO_COMMAND_ID;
    return NOTHING_TO_SEND;
  }

  // TC-triggered immediate poll
  if (pendingTcStatusTm) {
    (void)drainRxNow();
    const size_t total = RwProtocol::buildStatusReq(txBuf, sizeof(txBuf));
    if (total == 0) {
      *id = DeviceHandlerIF::NO_COMMAND_ID;
      return NOTHING_TO_SEND;
    }
    rawPacket    = txBuf;
    rawPacketLen = total;
    *id          = CMD_STATUS_POLL;
    statusAwaitCnt = 0;
    return returnvalue::OK;
  }

  // Periodic STATUS poll
  if (++statusPollCnt >= statusPollDivider) {
    statusPollCnt = 0;
    const size_t total = RwProtocol::buildStatusReq(txBuf, sizeof(txBuf));
    if (total == 0) {
      *id = DeviceHandlerIF::NO_COMMAND_ID;
      return NOTHING_TO_SEND;
    }
    rawPacket    = txBuf;
    rawPacketLen = total;
    *id          = CMD_STATUS_POLL;
    statusAwaitCnt = 0;
    return returnvalue::OK;
  }

  *id = DeviceHandlerIF::NO_COMMAND_ID;
  return NOTHING_TO_SEND;
}

// -------- Transition commands (ON <-> NORMAL) --------------------------------
// No device-specific mode commands in this ICD; we keep transitions silent.
// STOP before OFF is handled in doShutDown().
ReturnValue_t ReactionWheelsHandler::buildTransitionDeviceCommand(DeviceCommandId_t* id) {
  *id = DeviceHandlerIF::NO_COMMAND_ID;
  return NOTHING_TO_SEND;
}

// -------- TC -> wire command builder -----------------------------------------
ReturnValue_t ReactionWheelsHandler::buildCommandFromCommand(DeviceCommandId_t deviceCommand,
                                                             const uint8_t* data, size_t len) {
  switch (deviceCommand) {
    case CMD_SET_SPEED: {
      if (len < sizeof(int16_t)) {
        return returnvalue::FAILED;
      }
      int16_t rpm = 0;
      std::memcpy(&rpm, data, sizeof(rpm));

      int16_t lim = p_maxRpm;
      if (lim <= 0) {
        lim = RwConfig::MAX_RPM_DEFAULT;
      }
      if (rpm >  lim) rpm =  lim;
      if (rpm < -lim) rpm = -lim;

      const size_t total = RwProtocol::buildSetSpeed(txBuf, sizeof(txBuf), rpm);
      if (total == 0) {
        return returnvalue::FAILED;
      }
      rawPacket = txBuf;
      rawPacketLen = total;
      Clock::getUptime(&lastExtCmdMs);   // short poll block
      return returnvalue::OK;
    }

    case CMD_SET_TORQUE: {
      if (len < sizeof(int16_t)) {
        return returnvalue::FAILED;
      }
      int16_t tq = 0;
      std::memcpy(&tq, data, sizeof(tq));

      const int16_t lim = RwConfig::MAX_TORQUE_MNM_DEFAULT;
      if (lim > 0) {
        if (tq >  lim) tq =  lim;
        if (tq < -lim) tq = -lim;
      }

      const size_t total = RwProtocol::buildSetTorque(txBuf, sizeof(txBuf), tq);
      if (total == 0) {
        return returnvalue::FAILED;
      }
      rawPacket = txBuf;
      rawPacketLen = total;
      Clock::getUptime(&lastExtCmdMs);   // short poll block
      return returnvalue::OK;
    }

    case CMD_STOP: {
      const size_t total = RwProtocol::buildStop(txBuf, sizeof(txBuf));
      if (total == 0) {
        return returnvalue::FAILED;
      }
      rawPacket = txBuf;
      rawPacketLen = total;
      Clock::getUptime(&lastExtCmdMs);   // short poll block
      return returnvalue::OK;
    }

    case CMD_STATUS: {
      const size_t total = RwProtocol::buildStatusReq(txBuf, sizeof(txBuf));
      if (total == 0) {
        return returnvalue::FAILED;
      }
      rawPacket = txBuf;
      rawPacketLen = total;
      return returnvalue::OK;
    }

    default:
      return returnvalue::FAILED;
  }
}

// -------- Command/reply map --------------------------------------------------
void ReactionWheelsHandler::fillCommandAndReplyMap() {
  insertInCommandMap(CMD_SET_SPEED,   false, 0);
  insertInCommandMap(CMD_SET_TORQUE,  false, 0);
  insertInCommandMap(CMD_STOP,        false, 0);

  insertInCommandAndReplyMap(CMD_STATUS_POLL, 5, &replySet, RwProtocol::STATUS_LEN,
                             /*periodic*/ false, /*hasDifferentReplyId*/ true, REPLY_STATUS_POLL,
                             /*countdown*/ nullptr);

  insertInCommandMap(CMD_STATUS, false, 0);
}

// -------- FDIR counters ------------------------------------------------------
void ReactionWheelsHandler::handleCrcError() {
  ++crcErrCnt;
  if (CRC_ERR_EVENT_THRESH != 0 && (crcErrCnt % CRC_ERR_EVENT_THRESH) == 0) {
    triggerEvent(RwEvents::CRC_ERROR, crcErrCnt, 0);
  }
}

void ReactionWheelsHandler::handleMalformed() {
  ++malformedCnt;
  if (MALFORMED_EVENT_THRESH != 0 && (malformedCnt % MALFORMED_EVENT_THRESH) == 0) {
    triggerEvent(RwEvents::INVALID_REPLY, malformedCnt, 0);
  }
}

// -------- RX analysis helpers ------------------------------------------------
void ReactionWheelsHandler::reportProtocolIssuesInWindow(const uint8_t* buf, size_t n) {
  if (buf == nullptr || n < 2) return;
  for (size_t i = 0; i + 1 < n; ++i) {
    if (buf[i] != RwProtocol::START_REPLY) continue;
    uint8_t id = buf[i + 1];

    if (id != static_cast<uint8_t>(RwProtocol::RespId::STATUS)) {
      handleMalformed();
      continue;
    }
    if (i + RwProtocol::STATUS_LEN > n) {
      handleMalformed();
      continue;
    }
    if (!RwProtocol::verifyCrc16(&buf[i], RwProtocol::STATUS_LEN)) {
      handleCrcError();
    }
  }
}

static inline bool findLatestValidStatus(const uint8_t* buf, size_t n, long& posOut) {
  if (buf == nullptr || n < RwProtocol::STATUS_LEN) return false;
  for (long i = static_cast<long>(n) - static_cast<long>(RwProtocol::STATUS_LEN); i >= 0; --i) {
    const uint8_t* p = &buf[static_cast<size_t>(i)];
    if (p[0] == RwProtocol::START_REPLY &&
        p[1] == static_cast<uint8_t>(RwProtocol::RespId::STATUS) &&
        RwProtocol::verifyCrc16(p, RwProtocol::STATUS_LEN)) {
      posOut = i;
      return true;
    }
  }
  return false;
}

// -------- Reply scanning -----------------------------------------------------
ReturnValue_t ReactionWheelsHandler::scanForReply(const uint8_t* start, size_t len,
                                                  DeviceCommandId_t* foundId, size_t* foundLen) {
  reportProtocolIssuesInWindow(start, len);

  long pos = -1;
  if (findLatestValidStatus(start, len, pos)) {
    const uint8_t* src = start + static_cast<size_t>(pos);
    std::memcpy(replySet.raw.value, src, RwProtocol::STATUS_LEN);
    replySet.raw.setValid(true);
    replySet.setValidity(true, true);
    *foundId = REPLY_STATUS_POLL;
    *foundLen = RwProtocol::STATUS_LEN;
    return returnvalue::OK;
  }

  (void)drainRxIntoRing();

  (void)rxRing.lockRingBufferMutex(MutexIF::TimeoutType::WAITING, /*timeout ms*/ 2);
  size_t avail = rxRing.getAvailableReadData();
  if (avail < RwProtocol::STATUS_LEN) {
    (void)rxRing.unlockRingBufferMutex();
    return returnvalue::FAILED;
  }

  std::vector<uint8_t> snapshot(avail);
  size_t copied = 0;
  if (rxRing.readData(snapshot.data(), avail, /*incrementReadPtr=*/false,
                      /*readRemaining=*/true, &copied) != returnvalue::OK) {
    (void)rxRing.unlockRingBufferMutex();
    return returnvalue::FAILED;
  }
  snapshot.resize(copied);

  reportProtocolIssuesInWindow(snapshot.data(), snapshot.size());

  if (!findLatestValidStatus(snapshot.data(), snapshot.size(), pos)) {
    (void)rxRing.unlockRingBufferMutex();
    return returnvalue::FAILED;
  }

  std::memcpy(replySet.raw.value, snapshot.data() + static_cast<size_t>(pos),
              RwProtocol::STATUS_LEN);
  replySet.raw.setValid(true);
  replySet.setValidity(true, true);

  (void)rxRing.deleteData(static_cast<size_t>(pos) + RwProtocol::STATUS_LEN,
                          /*deleteRemaining=*/false);
  (void)rxRing.unlockRingBufferMutex();

  *foundId = REPLY_STATUS_POLL;
  *foundLen = RwProtocol::STATUS_LEN;
  return returnvalue::OK;
}

// -------- Reply interpretation ----------------------------------------------
ReturnValue_t ReactionWheelsHandler::interpretDeviceReply(DeviceCommandId_t id,
                                                          const uint8_t* packet) {
  if (id != REPLY_STATUS_POLL) {
    return returnvalue::FAILED;
  }
  statusAwaitCnt = -1;

  const uint8_t* pkt = replySet.raw.isValid() ? replySet.raw.value : packet;

  // Decode according to ICD
  const int16_t speed   = static_cast<int16_t>((pkt[2] << 8) | pkt[3]);
  const int16_t torque  = static_cast<int16_t>((pkt[4] << 8) | pkt[5]);
  const uint8_t running = pkt[6];

  // Throttled status log
  static uint32_t logCtr = 0;
#if !defined(RW_STATUS_LOG_EVERY)
#define RW_STATUS_LOG_EVERY RwConfig::STATUS_LOG_EVERY
#endif
  if ((++logCtr % RW_STATUS_LOG_EVERY) == 0) {
    sif::info << "RW STATUS: speed=" << speed << " RPM, torque=" << torque
              << " mNm, running=" << int(running) << std::endl;
  }

  // Update HK dataset
  hkSet.read();
  hkSet.setValidity(false, true);

  hkSet.speedRpm.value     = speed;
  hkSet.torque_mNm.value   = torque;
  hkSet.running.value      = running;
  hkSet.flags.value        = 0;
  hkSet.crcErrCnt.value    = crcErrCnt;
  hkSet.malformedCnt.value = malformedCnt;

  uint32_t ms = 0;
  Clock::getUptime(&ms);
  hkSet.timestampMs.value = ms;

  // Simple FDIR flags with debouncing
  const bool stuckCond = (running == 0) && (std::abs(static_cast<int>(speed)) > STUCK_RPM_THRESH);
  if (stuckCond) {
    if (++stuckRpmCnt >= STUCK_RPM_COUNT) {
      triggerEvent(RwEvents::STUCK, static_cast<uint32_t>(speed),
                   static_cast<uint32_t>(running));
      stuckRpmCnt = 0;
      hkSet.flags.value |= 0x0001;
    }
  } else {
    stuckRpmCnt = 0;
  }

  const bool torqueHigh = (std::abs(static_cast<int>(torque)) > HIGH_TORQUE_THRESH);
  if (torqueHigh) {
    if (++highTorqueCnt >= HIGH_TORQUE_COUNT) {
      triggerEvent(RwEvents::TORQUE_HIGH, static_cast<uint32_t>(torque), 0);
      highTorqueCnt = 0;
      hkSet.flags.value |= 0x0002;
    }
  } else {
    highTorqueCnt = 0;
  }

  // Validate and commit
  hkSet.speedRpm.setValid(true);
  hkSet.torque_mNm.setValid(true);
  hkSet.running.setValid(true);
  hkSet.flags.setValid(true);
  hkSet.crcErrCnt.setValid(true);
  hkSet.malformedCnt.setValid(true);
  hkSet.timestampMs.setValid(true);
  hkSet.setValidity(true, true);
  hkSet.commit();

  // Echo STATUS to TC caller if TC-driven
  if (pendingTcStatusTm) {
    if (pendingTcStatusReportedTo != MessageQueueIF::NO_QUEUE) {
      (void)actionHelper.reportData(pendingTcStatusReportedTo, CMD_STATUS, pkt,
                                    RwProtocol::STATUS_LEN, /*append*/ false);
    }
    pendingTcStatusTm = false;
    pendingTcStatusReportedTo = MessageQueueIF::NO_QUEUE;
  }

  return returnvalue::OK;
}

// -------- Transition delays (framework hint) ---------------------------------
uint32_t ReactionWheelsHandler::getTransitionDelayMs(Mode_t from, Mode_t to) {
  if (from == MODE_OFF && to == MODE_ON) return RwConfig::DELAY_OFF_TO_ON_MS;
  if (from == MODE_ON && to == MODE_NORMAL) return RwConfig::DELAY_ON_TO_NORMAL_MS;
  return 0;
}

// -------- Local data pool registration ---------------------------------------
ReturnValue_t ReactionWheelsHandler::initializeLocalDataPool(localpool::DataPool& localDataPoolMap,
                                                             LocalDataPoolManager&) {
  localDataPoolMap.emplace(static_cast<lp_id_t>(PoolIds::RAW_REPLY),
                           new PoolEntry<uint8_t>(RwProtocol::STATUS_LEN));
  localDataPoolMap.emplace(static_cast<lp_id_t>(PoolIds::HK_SPEED_RPM),     new PoolEntry<int16_t>(1));
  localDataPoolMap.emplace(static_cast<lp_id_t>(PoolIds::HK_TORQUE_mNm),    new PoolEntry<int16_t>(1));
  localDataPoolMap.emplace(static_cast<lp_id_t>(PoolIds::HK_RUNNING),       new PoolEntry<uint8_t>(1));
  localDataPoolMap.emplace(static_cast<lp_id_t>(PoolIds::HK_FLAGS),         new PoolEntry<uint16_t>(1));
  localDataPoolMap.emplace(static_cast<lp_id_t>(PoolIds::HK_CRC_ERR_CNT),   new PoolEntry<uint32_t>(1));
  localDataPoolMap.emplace(static_cast<lp_id_t>(PoolIds::HK_MALFORMED_CNT), new PoolEntry<uint32_t>(1));
  localDataPoolMap.emplace(static_cast<lp_id_t>(PoolIds::HK_TIMESTAMP_MS),  new PoolEntry<uint32_t>(1));
  return returnvalue::OK;
}

// -------- Action entry (remember TC-driven STATUS) ---------------------------
ReturnValue_t ReactionWheelsHandler::executeAction(ActionId_t actionId,
                                                   MessageQueueId_t commandedBy,
                                                   const uint8_t* data, size_t size) {
  const auto cmd = static_cast<DeviceCommandId_t>(actionId);
  if (cmd == CMD_STATUS) {
    pendingTcStatusTm = true;
    pendingTcStatusReportedTo = commandedBy;
  }
  return DeviceHandlerBase::executeAction(actionId, commandedBy, data, size);
}

// -------- Parameter IF -------------------------------------------------------
ReturnValue_t ReactionWheelsHandler::getParameter(uint8_t domainId, uint8_t parameterId,
                                                  ParameterWrapper* parameterWrapper,
                                                  const ParameterWrapper* newValues,
                                                  uint16_t /*startAtIndex*/) {
  if (domainId != PARAM_DOMAIN) {
    return returnvalue::FAILED;
  }
  switch (static_cast<ParamId>(parameterId)) {
    case ParamId::MAX_RPM:
      if (newValues != nullptr) {
        int16_t v = 0;
        if (newValues->getElement(&v, 0) != returnvalue::OK) {
          return returnvalue::FAILED;
        }
        p_maxRpm = v;
      }
      parameterWrapper->set(p_maxRpm);
      return returnvalue::OK;

    case ParamId::POLL_DIVIDER:
      if (newValues != nullptr) {
        uint32_t v = 0;
        if (newValues->getElement(&v, 0) != returnvalue::OK) {
          return returnvalue::FAILED;
        }
        statusPollDivider = v;
      }
      parameterWrapper->set(statusPollDivider);
      return returnvalue::OK;

    default:
      return returnvalue::FAILED;
  }
}

// -------- Dataset routing ----------------------------------------------------
LocalPoolDataSetBase* ReactionWheelsHandler::getDataSetHandle(sid_t sid) {
  if (sid.ownerSetId == DATASET_ID_HK) {
    return &hkSet;
  }
  if (sid.ownerSetId == DATASET_ID_RAW) {
    return &replySet;
  }
  return DeviceHandlerBase::getDataSetHandle(sid);
}
