#include "ReactionWheelsHandler.h"

#include <cstring>
#include <iomanip>
#include <vector>
#include <cmath>

#include "RwProtocol.h"
#include "fsfw/action/ActionHelper.h"
#include "fsfw/devicehandlers/DeviceHandlerIF.h"
#include "fsfw/ipc/MessageQueueIF.h"
#include "fsfw/ipc/MutexIF.h"
#include "fsfw/parameters/ParameterMessage.h"
#include "fsfw/returnvalues/returnvalue.h"
#include "fsfw/serviceinterface/ServiceInterfaceStream.h"
#include "RwEvents.h"

#if RW_VERBOSE
static void dumpHexWarn(const char* tag, const uint8_t* p, size_t n) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
  sif::warning << tag << " (" << n << "): ";
  for (size_t i = 0; i < n; ++i) {
    sif::warning << std::hex << std::setw(2) << std::setfill('0') << int(p[i]) << " ";
  }
  sif::warning << std::dec << std::endl;
#else
  (void)tag; (void)p; (void)n;
#endif
}
#else
static inline void dumpHexWarn(const char*, const uint8_t*, size_t) {}
#endif

ReactionWheelsHandler::ReactionWheelsHandler(object_id_t objectId, object_id_t comIF,
                                             CookieIF* cookie)
    : DeviceHandlerBase(objectId, comIF, cookie) {
  // Initialize optional RX-size FIFO and shared ring buffer
  rxRing.setToUseReceiveSizeFIFO(/*fifoDepth*/ 8);
  (void)rxRing.initialize();
}

void ReactionWheelsHandler::doStartUp() {
  // In this sim we can go to NORMAL right away
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
  this->getMode(&m, &s);
  sif::info << "ReactionWheelsHandler: modeChanged -> " << static_cast<int>(m)
            << " (sub=" << static_cast<int>(s) << ")" << std::endl;
}

// Quickly drop stale bytes. Use with care to not drop fresh replies.
ReturnValue_t ReactionWheelsHandler::drainRxNow() {
  if (communicationInterface == nullptr || comCookie == nullptr) {
    return returnvalue::OK;
  }
  for (int i = 0; i < 3; ++i) {
    ReturnValue_t rvReq =
        communicationInterface->requestReceiveMessage(comCookie, RwProtocol::STATUS_LEN);
    if (rvReq != returnvalue::OK) {
      break;
    }
    uint8_t* buf = nullptr;
    size_t sz = 0;
    ReturnValue_t rvRead = communicationInterface->readReceivedMessage(comCookie, &buf, &sz);
    if (rvRead != returnvalue::OK || buf == nullptr || sz == 0) {
      break;
    }
#if RW_VERBOSE
    sif::warning << "drainRxNow: dropped " << sz << " stale bytes" << std::endl;
    dumpHexWarn("drainRxNow: bytes", buf, (sz > 32 ? 32 : sz));
#endif
  }
  return returnvalue::OK;
}

// Pull available bytes into the shared ring buffer (now: "drain until empty")
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
    // Use moderately sized chunks to speed up draining while avoiding backend warnings.
    const size_t chunk = (freeSpace < 128) ? freeSpace : size_t(128);
    ReturnValue_t req = communicationInterface->requestReceiveMessage(comCookie, chunk);
    if (req != returnvalue::OK) {
      (void)rxRing.unlockRingBufferMutex();
      break;
    }
    uint8_t* buf = nullptr;
    size_t sz = 0;
    ReturnValue_t rd = communicationInterface->readReceivedMessage(comCookie, &buf, &sz);
    if (rd != returnvalue::OK || buf == nullptr || sz == 0) {
      (void)rxRing.unlockRingBufferMutex();
      break;
    }
    const size_t wrote = rxRing.writeData(buf, sz);
    if (wrote != sz) {
      sif::warning << "RW rxRing overflow: dropped " << (sz - wrote) << " bytes" << std::endl;
    }
#if RW_VERBOSE
    sif::warning << "drainRxIntoRing: wrote " << wrote << " bytes" << std::endl;
#endif
    (void)rxRing.unlockRingBufferMutex();
  }
  return returnvalue::OK;
}

ReturnValue_t ReactionWheelsHandler::buildNormalDeviceCommand(DeviceCommandId_t* id) {
  // STATUS timeout supervision (per PST cycle)
  if (statusAwaitCnt >= 0) {
    if (++statusAwaitCnt > STATUS_TIMEOUT_CYCLES) {
      triggerEvent(RwEvents::TIMEOUT, 0, 0);
      statusAwaitCnt = -1;  // reset after reporting
    }
  }

  // TC-driven STATUS request: send immediately
  if (pendingTcStatusTm) {
#if RW_VERBOSE
    sif::warning << "buildNormalDeviceCommand: tcStatusPending=true -> immediate STATUS poll"
                 << std::endl;
#endif
    // Flush any stale frames so the next one is guaranteed to belong to this poll.
    (void)drainRxNow();

    const size_t total = RwProtocol::buildStatusReq(txBuf, sizeof(txBuf));
    if (total == 0) {
      *id = DeviceHandlerIF::NO_COMMAND_ID;
      return NOTHING_TO_SEND;
    }
    rawPacket    = txBuf;
    rawPacketLen = total;
    *id          = CMD_STATUS_POLL;
    statusAwaitCnt = 0;  // start waiting for reply
    dumpHexWarn("DH TX (TC-driven STATUS poll)", txBuf, total);
    return returnvalue::OK;
  }

  // Periodic STATUS poll (optional)
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
    statusAwaitCnt = 0;  // start waiting for reply
    dumpHexWarn("DH TX (periodic STATUS poll)", txBuf, total);
    return returnvalue::OK;
  }

  *id = DeviceHandlerIF::NO_COMMAND_ID;
  return NOTHING_TO_SEND;
}

ReturnValue_t ReactionWheelsHandler::buildTransitionDeviceCommand(DeviceCommandId_t* id) {
  *id = DeviceHandlerIF::NO_COMMAND_ID;
  return NOTHING_TO_SEND;
}

ReturnValue_t ReactionWheelsHandler::buildCommandFromCommand(DeviceCommandId_t deviceCommand,
                                                             const uint8_t* data, size_t len) {
  switch (deviceCommand) {
    case CMD_SET_SPEED: {
      if (len < sizeof(int16_t)) {
        return returnvalue::FAILED;
      }
      int16_t rpm = 0;
      std::memcpy(&rpm, data, sizeof(rpm));
      // Apply runtime limit
      if (rpm >  p_maxRpm) rpm =  p_maxRpm;
      if (rpm < -p_maxRpm) rpm = -p_maxRpm;

      const size_t total = RwProtocol::buildSetSpeed(txBuf, sizeof(txBuf), rpm);
      if (total == 0) {
        return returnvalue::FAILED;
      }
      rawPacket    = txBuf;
      rawPacketLen = total;
      dumpHexWarn("DH TX (TC frame: SET_SPEED)", txBuf, total);
      return returnvalue::OK;
    }

    case CMD_STOP: {
      const size_t total = RwProtocol::buildStop(txBuf, sizeof(txBuf));
      if (total == 0) {
        return returnvalue::FAILED;
      }
      rawPacket    = txBuf;
      rawPacketLen = total;
      dumpHexWarn("DH TX (TC frame: STOP)", txBuf, total);
      return returnvalue::OK;
    }

    case CMD_STATUS: {
      const size_t total = RwProtocol::buildStatusReq(txBuf, sizeof(txBuf));
      if (total == 0) {
        return returnvalue::FAILED;
      }
      rawPacket    = txBuf;
      rawPacketLen = total;
      dumpHexWarn("DH TX (TC frame: STATUS)", txBuf, total);
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

void ReactionWheelsHandler::fillCommandAndReplyMap() {
  // Direct commands
  insertInCommandMap(CMD_SET_SPEED, false, 0);
  insertInCommandMap(CMD_STOP, false, 0);

  // Unified reply classification for STATUS polls (periodic or TC-driven)
  insertInCommandAndReplyMap(
      /*deviceCommand*/ CMD_STATUS_POLL,
      /*maxDelayCycles*/ 5,
      /*replyDataSet*/ &replySet,
      /*replyLen*/ RwProtocol::STATUS_LEN,
      /*periodic*/ false,
      /*hasDifferentReplyId*/ true,
      /*replyId*/ REPLY_STATUS_POLL,
      /*countdown*/ nullptr);

  // No direct reply for CMD_STATUS (action); data goes out via ActionHelper::reportData(...)
  insertInCommandMap(CMD_STATUS, false, 0);
}

// Report protocol issues (CRC, invalid reply IDs)
void ReactionWheelsHandler::reportProtocolIssuesInWindow(const uint8_t* buf, size_t n) {
  if (buf == nullptr || n < 2) return;
  for (size_t i = 0; i + 1 < n; ++i) {
    if (buf[i] != RwProtocol::START_REPLY) continue;
    uint8_t id = buf[i + 1];
    // Unexpected response ID
    if (id != static_cast<uint8_t>(RwProtocol::RespId::STATUS)) {
      triggerEvent(RwEvents::INVALID_REPLY, static_cast<uint32_t>(id), 0);
      continue;
    }
    // Incomplete frame -> ignore (may complete in next cycle)
    if (i + RwProtocol::STATUS_LEN > n) {
      continue;
    }
    // CRC check
    if (!RwProtocol::verifyCrc16(&buf[i], RwProtocol::STATUS_LEN)) {
      triggerEvent(RwEvents::CRC_ERROR, 0, 0);
    }
  }
}

// Find newest valid STATUS frame (with correct CRC), searching backwards
static inline bool findLatestValidStatus(const uint8_t* buf, size_t n, long& posOut) {
  if (buf == nullptr || n < RwProtocol::STATUS_LEN) { return false; }
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

ReturnValue_t ReactionWheelsHandler::scanForReply(const uint8_t* start, size_t len,
                                                  DeviceCommandId_t* foundId, size_t* foundLen) {
#if RW_VERBOSE
  sif::warning << "scanForReply: called with len=" << len << std::endl;
#endif
  // 1) Report protocol problems in the provided window (CRC/invalid IDs)
  reportProtocolIssuesInWindow(start, len);

  // 2) Primarily search provided window for a valid frame
  long pos = -1;
  if (findLatestValidStatus(start, len, pos)) {
#if RW_VERBOSE
    sif::warning << "scanForReply: match in provided buffer at off=" << pos << std::endl;
#endif
    const uint8_t* src = start + static_cast<size_t>(pos);
    std::memcpy(replySet.raw.value, src, RwProtocol::STATUS_LEN);
    replySet.raw.setValid(true);
    replySet.setValidity(true, true);
    *foundId  = REPLY_STATUS_POLL;
    *foundLen = RwProtocol::STATUS_LEN;
    return returnvalue::OK;
  }

  // 3) Fallback: search in the ring buffer
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

  // Report protocol problems in the ring snapshot
  reportProtocolIssuesInWindow(snapshot.data(), snapshot.size());

  if (!findLatestValidStatus(snapshot.data(), snapshot.size(), pos)) {
    (void)rxRing.unlockRingBufferMutex();
    return returnvalue::FAILED;
  }

  std::memcpy(replySet.raw.value, snapshot.data() + static_cast<size_t>(pos),
              RwProtocol::STATUS_LEN);
  replySet.raw.setValid(true);
  replySet.setValidity(true, true);

#if RW_VERBOSE
  sif::warning << "scanForReply: matched STATUS in ring at off=" << pos
               << ", deleting " << (pos + static_cast<long>(RwProtocol::STATUS_LEN))
               << " bytes from ring" << std::endl;
#endif
  (void)rxRing.deleteData(static_cast<size_t>(pos) + RwProtocol::STATUS_LEN,
                          /*deleteRemaining=*/false);
  (void)rxRing.unlockRingBufferMutex();

  *foundId  = REPLY_STATUS_POLL;
  *foundLen = RwProtocol::STATUS_LEN;
  return returnvalue::OK;
}

ReturnValue_t ReactionWheelsHandler::interpretDeviceReply(DeviceCommandId_t id,
                                                          const uint8_t* packet) {
  if (id != REPLY_STATUS_POLL) {
    return returnvalue::FAILED;
  }
  // We received a reply -> clear timeout tracking
  statusAwaitCnt = -1;

  // Prefer the dataset frame if available; otherwise use the incoming pointer
  const uint8_t* pkt = replySet.raw.isValid() ? replySet.raw.value : packet;
  dumpHexWarn("interpretDeviceReply: frame", pkt, RwProtocol::STATUS_LEN);

  // Parse big-endian fields
  const int16_t speed   = static_cast<int16_t>((pkt[2] << 8) | pkt[3]);
  const int16_t torque  = static_cast<int16_t>((pkt[4] << 8) | pkt[5]);
  const uint8_t running = pkt[6];

  // Console output (nice to have while developing)
  sif::info << "RW STATUS: speed=" << speed << " RPM, torque=" << torque
            << " mNm, running=" << int(running) << std::endl;

  // Update HK dataset
  hkSet.setValidity(false, true);
  hkSet.speedRpm.value   = speed;
  hkSet.torque_mNm.value = torque;
  hkSet.running.value    = running;
  hkSet.flags.value      = 0;  // set real flags if available
  hkSet.error.value      = 0;  // set real error if available
  hkSet.speedRpm.setValid(true);
  hkSet.torque_mNm.setValid(true);
  hkSet.running.setValid(true);
  hkSet.flags.setValid(true);
  hkSet.error.setValid(true);
  hkSet.setValidity(true, true);

  // Simple FDIR examples (debounced)
  const bool stuckCond =
      (hkSet.running.value == 0) && (std::abs(static_cast<int>(hkSet.speedRpm.value)) > STUCK_RPM_THRESH);
  if (stuckCond) {
    if (++stuckCnt >= STUCK_DEBOUNCE_FRAMES) {
      triggerEvent(RwEvents::STUCK,
                   static_cast<uint32_t>(hkSet.speedRpm.value),
                   static_cast<uint32_t>(hkSet.running.value));
      stuckCnt = 0;
      hkSet.flags.value |= 0x0001;
      hkSet.flags.setValid(true);
    }
  } else {
    stuckCnt = 0;
  }

  const bool torqueHigh =
      (std::abs(static_cast<int>(hkSet.torque_mNm.value)) > TORQUE_HIGH_MNM_THRESH);
  if (torqueHigh) {
    if (++torqueHighCnt >= TORQUE_DEBOUNCE_FRAMES) {
      triggerEvent(RwEvents::TORQUE_HIGH,
                   static_cast<uint32_t>(hkSet.torque_mNm.value),
                   0);
      torqueHighCnt = 0;
      hkSet.flags.value |= 0x0002;
      hkSet.flags.setValid(true);
    }
  } else {
    torqueHighCnt = 0;
  }

  if (hkSet.error.value != 0) {
    triggerEvent(RwEvents::ERROR_CODE, static_cast<uint32_t>(hkSet.error.value), 0);
  }

  // Ensure RAW dataset contains the frame
  if (!replySet.raw.isValid()) {
    std::memcpy(replySet.raw.value, pkt, RwProtocol::STATUS_LEN);
    replySet.raw.setValid(true);
    replySet.setValidity(true, true);
  }

  // If a TC requested a data reply, route it back via ActionHelper
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

uint32_t ReactionWheelsHandler::getTransitionDelayMs(Mode_t from, Mode_t to) {
  if (from == MODE_OFF && to == MODE_ON)     return RW_DELAY_OFF_TO_ON_MS;
  if (from == MODE_ON  && to == MODE_NORMAL) return RW_DELAY_ON_TO_NORMAL_MS;
  return 0;
}

ReturnValue_t ReactionWheelsHandler::initializeLocalDataPool(
    localpool::DataPool& localDataPoolMap, LocalDataPoolManager&) {
  // RAW (9B)
  localDataPoolMap.emplace(static_cast<lp_id_t>(PoolIds::RAW_REPLY),
                           new PoolEntry<uint8_t>(RwProtocol::STATUS_LEN));
  // HK variables
  localDataPoolMap.emplace(static_cast<lp_id_t>(PoolIds::HK_SPEED_RPM),
                           new PoolEntry<int16_t>(1));
  localDataPoolMap.emplace(static_cast<lp_id_t>(PoolIds::HK_TORQUE_mNm),
                           new PoolEntry<int16_t>(1));
  localDataPoolMap.emplace(static_cast<lp_id_t>(PoolIds::HK_RUNNING),
                           new PoolEntry<uint8_t>(1));
  localDataPoolMap.emplace(static_cast<lp_id_t>(PoolIds::HK_FLAGS),
                           new PoolEntry<uint16_t>(1));
  localDataPoolMap.emplace(static_cast<lp_id_t>(PoolIds::HK_ERROR),
                           new PoolEntry<uint16_t>(1));
  return returnvalue::OK;
}

ReturnValue_t ReactionWheelsHandler::executeAction(ActionId_t actionId,
                                                   MessageQueueId_t commandedBy,
                                                   const uint8_t* data, size_t size) {
#if RW_VERBOSE
  sif::warning << "executeAction: actionId=0x" << std::hex << int(actionId) << " size=" << std::dec
               << size << " commandedBy=0x" << std::hex << commandedBy << std::dec << std::endl;
#endif
  const auto cmd = static_cast<DeviceCommandId_t>(actionId);
  if (cmd == CMD_STATUS) {
    // Remember requester and trigger next poll immediately
    pendingTcStatusTm = true;
    pendingTcStatusReportedTo = commandedBy;
  }
  return DeviceHandlerBase::executeAction(actionId, commandedBy, data, size);
}

// Parameters: get/set via ParameterHelper-compatible signature
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

    case ParamId::MAX_SLEW_RPM_S:
      if (newValues != nullptr) {
        uint16_t v = 0;
        if (newValues->getElement(&v, 0) != returnvalue::OK) {
          return returnvalue::FAILED;
        }
        p_maxSlewRpmS = v;
      }
      parameterWrapper->set(p_maxSlewRpmS);
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
