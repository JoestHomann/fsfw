// ReactionWheelsHandler.cpp

#include "ReactionWheelsHandler.h"

#include <cstring>
#include <iomanip>
#include <vector>

#include "RwEvents.h"
#include "RwProtocol.h"

#include "fsfw/devicehandlers/DeviceHandlerIF.h"
#include "fsfw/ipc/MessageQueueIF.h"
#include "fsfw/objectmanager/SystemObject.h"
#include "fsfw/serviceinterface/ServiceInterface.h"
#include "fsfw/timemanager/Clock.h"

// --- small helper for debug hex dumps ---
#if RW_VERBOSE
static void dumpHexWarn(const char* tag, const uint8_t* p, size_t n) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
  sif::warning << tag << " (" << n << "): ";
  for (size_t i = 0; i < n; ++i) {
    sif::warning << std::hex << std::setw(2) << std::setfill('0') << int(p[i]) << " ";
  }
  sif::warning << std::dec << std::endl;
#else
  (void)tag;
  (void)p;
  (void)n;
#endif
}
#endif

// -----------------------------------------------------------------------------
// ctor
ReactionWheelsHandler::ReactionWheelsHandler(object_id_t objectId, object_id_t comIF,
                                             CookieIF* cookie)
    : DeviceHandlerBase(objectId, comIF, cookie) {
  // optional receive-sizes FIFO for shared ringbuffer (diagnostic use)
  rxRing.setToUseReceiveSizeFIFO(/*fifoDepth*/ 8);
  (void)rxRing.initialize();
}

// -----------------------------------------------------------------------------
// mode hooks
void ReactionWheelsHandler::doStartUp() {
  sif::info << "ReactionWheelsHandler: doStartUp()" << std::endl;
  setMode(MODE_NORMAL);
}

void ReactionWheelsHandler::doShutDown() {
  sif::info << "ReactionWheelsHandler: doShutDown()" << std::endl;
  setMode(MODE_OFF);
}

void ReactionWheelsHandler::modeChanged() {
  Mode_t m{};
  Submode_t s{};
  getMode(&m, &s);
  sif::info << "ReactionWheelsHandler: modeChanged -> " << int(m) << "/" << int(s) << std::endl;
}

// -----------------------------------------------------------------------------
// periodic polling builder
ReturnValue_t ReactionWheelsHandler::buildNormalDeviceCommand(DeviceCommandId_t* id) {
  // 1) If a poll is outstanding, check for timeout
  if (statusAwaitCnt >= 0) {
    if (++statusAwaitCnt > STATUS_TIMEOUT_CYCLES) {
      triggerEvent(RwEvents::TIMEOUT, /*p1*/ 0, /*p2*/ 0);
      statusAwaitCnt = -1;
      // continue to normal flow without immediate re-poll to avoid spam
    } else {
      *id = DeviceHandlerIF::NO_COMMAND_ID;
      return NOTHING_TO_SEND;
    }
  }

  // 2) Inhibit polls right after external command
  uint32_t nowMs = 0;
  Clock::getUptime(&nowMs);
  if (lastExtCmdMs != 0 && (nowMs - lastExtCmdMs) < POLL_INHIBIT_MS) {
    *id = DeviceHandlerIF::NO_COMMAND_ID;
    return NOTHING_TO_SEND;
  }

  // 3) TC-driven immediate status poll
  if (pendingTcStatusTm) {
#if RW_VERBOSE
    sif::warning << "buildNormalDeviceCommand: TC-driven STATUS poll" << std::endl;
#endif
    (void)drainRxNow();
    if (!RwProtocol::buildStatusReq(txBuf, sizeof(txBuf))) {
      *id = DeviceHandlerIF::NO_COMMAND_ID;
      return NOTHING_TO_SEND;
    }
    rawPacket = txBuf;
    *id = CMD_STATUS_POLL;
    statusAwaitCnt = 0;
#if RW_VERBOSE
    dumpHexWarn("DH TX (TC STATUS)", txBuf, RwProtocol::CMD_LEN);
#endif
    return returnvalue::OK;
  }

  // 4) Periodic status poll
  if (++statusPollCnt >= statusPollDivider) {
    statusPollCnt = 0;
    if (!RwProtocol::buildStatusReq(txBuf, sizeof(txBuf))) {
      *id = DeviceHandlerIF::NO_COMMAND_ID;
      return NOTHING_TO_SEND;
    }
    rawPacket = txBuf;
    *id = CMD_STATUS_POLL;
    statusAwaitCnt = 0;
#if RW_VERBOSE
    dumpHexWarn("DH TX (periodic STATUS)", txBuf, RwProtocol::CMD_LEN);
#endif
    return returnvalue::OK;
  }

  *id = DeviceHandlerIF::NO_COMMAND_ID;
  return NOTHING_TO_SEND;
}

// no transition commands
ReturnValue_t ReactionWheelsHandler::buildTransitionDeviceCommand(DeviceCommandId_t* id) {
  *id = DeviceHandlerIF::NO_COMMAND_ID;
  return NOTHING_TO_SEND;
}

// build outgoing wire command for an external device command
ReturnValue_t ReactionWheelsHandler::buildCommandFromCommand(DeviceCommandId_t deviceCommand,
                                                             const uint8_t* data, size_t len) {
  switch (deviceCommand) {
    case CMD_SET_SPEED: {
      if (len < sizeof(int16_t)) {
        return returnvalue::FAILED;
      }
      int16_t rpm = 0;
      std::memcpy(&rpm, data, sizeof(rpm));
      // clamp against configured max RPM
      if (p_maxRpm <= 0) {
        p_maxRpm = RwConfig::MAX_RPM_DEFAULT;
      }
      if (rpm > p_maxRpm) rpm = p_maxRpm;
      if (rpm < -p_maxRpm) rpm = -p_maxRpm;

      if (!RwProtocol::buildSetSpeed(txBuf, sizeof(txBuf), rpm)) {
        return returnvalue::FAILED;
      }
      rawPacket = txBuf;
#if RW_VERBOSE
      dumpHexWarn("DH TX (SET_SPEED)", txBuf, RwProtocol::CMD_LEN);
#endif
      Clock::getUptime(&lastExtCmdMs);  // enable short poll inhibit
      return returnvalue::OK;
    }
    case CMD_SET_TORQUE: {
      if (len < sizeof(int16_t)) {
        return returnvalue::FAILED;
      }
      int16_t tq = 0;
      std::memcpy(&tq, data, sizeof(tq));
      if (!RwProtocol::buildSetTorque(txBuf, sizeof(txBuf), tq)) {
        return returnvalue::FAILED;
      }
      rawPacket = txBuf;
#if RW_VERBOSE
      dumpHexWarn("DH TX (SET_TORQUE)", txBuf, RwProtocol::CMD_LEN);
#endif
      Clock::getUptime(&lastExtCmdMs);
      return returnvalue::OK;
    }
    case CMD_STOP: {
      if (!RwProtocol::buildStop(txBuf, sizeof(txBuf))) {
        return returnvalue::FAILED;
      }
      rawPacket = txBuf;
#if RW_VERBOSE
      dumpHexWarn("DH TX (STOP)", txBuf, RwProtocol::CMD_LEN);
#endif
      Clock::getUptime(&lastExtCmdMs);
      return returnvalue::OK;
    }
    case CMD_STATUS: {
      if (!RwProtocol::buildStatusReq(txBuf, sizeof(txBuf))) {
        return returnvalue::FAILED;
      }
      rawPacket = txBuf;
#if RW_VERBOSE
      dumpHexWarn("DH TX (STATUS)", txBuf, RwProtocol::CMD_LEN);
#endif
      return returnvalue::OK;
    }
    default:
#if RW_VERBOSE
      sif::warning << "ReactionWheelsHandler: unknown deviceCommand 0x" << std::hex << deviceCommand
                   << std::dec << std::endl;
#endif
      return returnvalue::FAILED;
  }
}

// map commands and replies
void ReactionWheelsHandler::fillCommandAndReplyMap() {
  insertInCommandMap(CMD_SET_SPEED, false, 0);
  insertInCommandMap(CMD_SET_TORQUE, false, 0);
  insertInCommandMap(CMD_STOP, false, 0);

  insertInCommandAndReplyMap(CMD_STATUS_POLL, /*maxDelayMs*/ 5, &replySet,
                             RwProtocol::STATUS_LEN, /*periodic*/ false,
                             /*hasDifferentReplyId*/ true, REPLY_STATUS_POLL, nullptr);

  insertInCommandMap(CMD_STATUS, false, 0);
}

// -----------------------------------------------------------------------------
// low-level error counters
void ReactionWheelsHandler::onCrcError() {
  ++crcErrCnt;
  if (CRC_ERR_EVENT_THRESH != 0 && (crcErrCnt % CRC_ERR_EVENT_THRESH) == 0) {
    triggerEvent(RwEvents::CRC_ERROR, crcErrCnt, 0);
  }
}

void ReactionWheelsHandler::onMalformed() {
  ++malformedCnt;
  if (MALFORMED_EVENT_THRESH != 0 && (malformedCnt % MALFORMED_EVENT_THRESH) == 0) {
    triggerEvent(RwEvents::INVALID_REPLY, malformedCnt, 0);
  }
}

// -----------------------------------------------------------------------------
// window scanner used by scanForReply
void ReactionWheelsHandler::reportProtocolIssuesInWindow(const uint8_t* buf, size_t n) {
  if (buf == nullptr || n < 2) {
    return;
  }
  for (size_t i = 0; i + 1 < n; ++i) {
    if (buf[i] != RwProtocol::START_REPLY) {
      continue;
    }
    const size_t remain = n - i;
    if (remain < RwProtocol::STATUS_LEN) {
      onMalformed();
      continue;
    }
    RwProtocol::Status tmp{};
    const auto res = RwProtocol::parseStatus(&buf[i], RwProtocol::STATUS_LEN, tmp);
    if (res == RwProtocol::ParseResult::CRC_ERROR) {
      onCrcError();
    } else if (res != RwProtocol::ParseResult::OK) {
      onMalformed();
    }
  }
}

// -----------------------------------------------------------------------------
// scanForReply: try to find a valid STATUS frame in current chunk or ring
ReturnValue_t ReactionWheelsHandler::scanForReply(const uint8_t* start, size_t len,
                                                  DeviceCommandId_t* foundId, size_t* foundLen) {
#if RW_VERBOSE
  sif::warning << "scanForReply: called with len=" << len << std::endl;
#endif
  if (foundId == nullptr || foundLen == nullptr) {
    return returnvalue::FAILED;
  }

  // 1) scan current chunk
  reportProtocolIssuesInWindow(start, len);
  if (start != nullptr && len >= RwProtocol::STATUS_LEN) {
    for (long i = static_cast<long>(len) - static_cast<long>(RwProtocol::STATUS_LEN); i >= 0;
         --i) {
      const uint8_t* p = &start[static_cast<size_t>(i)];
      if (p[0] != RwProtocol::START_REPLY) {
        continue;
      }
      RwProtocol::Status s{};
      const auto pr = RwProtocol::parseStatus(p, RwProtocol::STATUS_LEN, s);
      if (pr == RwProtocol::ParseResult::OK) {
        std::memcpy(replySet.raw.value, p, RwProtocol::STATUS_LEN);
        replySet.raw.setValid(true);
        replySet.setValidity(true, true);
        *foundId = REPLY_STATUS_POLL;
        *foundLen = RwProtocol::STATUS_LEN;
        return returnvalue::OK;
      }
    }
  }

  // 2) ring snapshot without consuming
  const size_t avail = rxRing.getAvailableReadData();
  if (avail > 0) {
    std::vector<uint8_t> snap(avail);
    size_t trueAmount = 0;
    (void)rxRing.readData(snap.data(), avail, /*incrementReadPtr*/ false,
                          /*readRemaining*/ true, &trueAmount);
#if RW_VERBOSE
    dumpHexWarn("scanForReply: ring-snap", snap.data(), (trueAmount > 64 ? 64 : trueAmount));
#endif
    if (trueAmount >= RwProtocol::STATUS_LEN) {
      for (long i = static_cast<long>(trueAmount) - static_cast<long>(RwProtocol::STATUS_LEN);
           i >= 0; --i) {
        const uint8_t* p = &snap[static_cast<size_t>(i)];
        if (p[0] != RwProtocol::START_REPLY) {
          continue;
        }
        RwProtocol::Status s{};
        const auto pr = RwProtocol::parseStatus(p, RwProtocol::STATUS_LEN, s);
        if (pr == RwProtocol::ParseResult::OK) {
          std::memcpy(replySet.raw.value, p, RwProtocol::STATUS_LEN);
          replySet.raw.setValid(true);
          replySet.setValidity(true, true);
          *foundId = REPLY_STATUS_POLL;
          *foundLen = RwProtocol::STATUS_LEN;
          return returnvalue::OK;
        }
      }
    }
  }

  *foundId = REPLY_STATUS_POLL;
  *foundLen = RwProtocol::STATUS_LEN;
  return returnvalue::OK;
}

// -----------------------------------------------------------------------------
// interpret STATUS frame and update HK
ReturnValue_t ReactionWheelsHandler::interpretDeviceReply(DeviceCommandId_t id,
                                                          const uint8_t* packet) {
  if (id != REPLY_STATUS_POLL) {
    return returnvalue::FAILED;
  }
  statusAwaitCnt = -1;

  // choose source frame: prefer dataset if scanForReply put it there
  const uint8_t* pkt = (replySet.raw.isValid()) ? replySet.raw.value : packet;
  if (pkt == nullptr) {
    return returnvalue::FAILED;
  }

#if RW_VERBOSE
  dumpHexWarn("interpretDeviceReply: frame", pkt, RwProtocol::STATUS_LEN);
#endif

  RwProtocol::Status st{};
  const auto pr = RwProtocol::parseStatus(pkt, RwProtocol::STATUS_LEN, st);
  if (pr != RwProtocol::ParseResult::OK) {
    if (pr == RwProtocol::ParseResult::CRC_ERROR) {
      onCrcError();
    } else {
      onMalformed();
    }
    return returnvalue::FAILED;
  }

  // throttled log
  if (++statusPollCnt % RwConfig::STATUS_LOG_EVERY == 0) {
#if RW_VERBOSE
    sif::warning << "RW RX: rpm=" << st.speedRpm << " tq_mNm=" << st.torque_mNm
                 << " running=" << int(st.running) << std::endl;
#endif
  }

  // update HK
  hkSet.read();
  hkSet.speedRpm.value = st.speedRpm;
  hkSet.torque_mNm.value = st.torque_mNm;
  hkSet.running.value = st.running;
  hkSet.flags.value = fdirFlags;
  hkSet.error.value = lastErrorCode;
  hkSet.crcErrCnt.value = crcErrCnt;
  hkSet.malformedCnt.value = malformedCnt;

  uint32_t nowMs = 0;
  Clock::getUptime(&nowMs);
  hkSet.timestampMs.value = nowMs;

  hkSet.speedRpm.setValid(true);
  hkSet.torque_mNm.setValid(true);
  hkSet.running.setValid(true);
  hkSet.flags.setValid(true);
  hkSet.error.setValid(true);
  hkSet.crcErrCnt.setValid(true);
  hkSet.malformedCnt.setValid(true);
  hkSet.timestampMs.setValid(true);
  hkSet.setValidity(true, true);
  hkSet.commit();

  // clear TC-driven flag if set
  if (pendingTcStatusTm) {
    pendingTcStatusTm = false;
  }
  return returnvalue::OK;
}

// -----------------------------------------------------------------------------
// pull available bytes from COM IF into ring
ReturnValue_t ReactionWheelsHandler::performOperation(uint8_t opCode) {
  (void)opCode;
  uint8_t* buf = nullptr;
  size_t sz = 0;

  const ReturnValue_t rvReq =
      communicationInterface->requestReceiveMessage(comCookie, RwProtocol::STATUS_LEN);
  if (rvReq != returnvalue::OK) {
    return returnvalue::OK;
  }
  const ReturnValue_t rvRead = communicationInterface->readReceivedMessage(comCookie, &buf, &sz);
  if (rvRead != returnvalue::OK || buf == nullptr || sz == 0) {
    return returnvalue::OK;
  }

#if RW_VERBOSE
  dumpHexWarn("performOperation: RX", buf, (sz > 64 ? 64 : sz));
#endif

  const ReturnValue_t rvWr = rxRing.writeData(buf, sz);
  if (rvWr != returnvalue::OK) {
    ++rxDropCnt;
#if RW_VERBOSE
    sif::warning << "performOperation: rx ring overflow, dropCnt=" << rxDropCnt << std::endl;
#endif
  }
  return returnvalue::OK;
}

// -----------------------------------------------------------------------------
// drop some stale frames from COM IF (used before forced polls)
ReturnValue_t ReactionWheelsHandler::drainRxNow() {
  for (int i = 0; i < 8; ++i) {
    ReturnValue_t rvReq =
        communicationInterface->requestReceiveMessage(comCookie, RwProtocol::STATUS_LEN);
    if (rvReq != returnvalue::OK) break;
    uint8_t* buf = nullptr;
    size_t sz = 0;
    ReturnValue_t rvRead = communicationInterface->readReceivedMessage(comCookie, &buf, &sz);
    if (rvRead != returnvalue::OK || buf == nullptr || sz == 0) break;
#if RW_VERBOSE
    dumpHexWarn("drainRxNow: drop", buf, (sz > 32 ? 32 : sz));
#endif
  }
  return returnvalue::OK;
}

// -----------------------------------------------------------------------------
// parameter interface
ReturnValue_t ReactionWheelsHandler::getParameter(uint8_t domainId, uint8_t parameterId,
                                                  ParameterWrapper* parameterWrapper,
                                                  const ParameterWrapper* /*newValues*/,
                                                  uint16_t /*startAtIndex*/) {
  if (domainId != PARAM_DOMAIN) {
    return returnvalue::FAILED;
  }
  switch (static_cast<ParamId>(parameterId)) {
    case ParamId::MAX_RPM:
      parameterWrapper->set<int16_t>(p_maxRpm);
      return returnvalue::OK;
    case ParamId::MAX_SLEW_RPM_S:
      parameterWrapper->set<uint16_t>(p_maxSlewRpmS);
      return returnvalue::OK;
    case ParamId::POLL_DIVIDER:
      parameterWrapper->set<uint32_t>(statusPollDivider);
      return returnvalue::OK;
    default:
      return returnvalue::FAILED;
  }
}
