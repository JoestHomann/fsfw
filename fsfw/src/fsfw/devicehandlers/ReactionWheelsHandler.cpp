#include "ReactionWheelsHandler.h"

#include <cstring>   // memcpy
#include <iomanip>   // debug formatting
#include <vector>

#include "RwProtocol.h"
#include "fsfw/action/ActionHelper.h"    // reportData(...)
#include "fsfw/devicehandlers/DeviceHandlerIF.h"
#include "fsfw/ipc/MessageQueueIF.h"     // MessageQueueIF::NO_QUEUE
#include "fsfw/ipc/MutexIF.h"
#include "fsfw/returnvalues/returnvalue.h"
#include "fsfw/serviceinterface/ServiceInterfaceStream.h"

// Debug On/Off switch (set to 1 to enable debug output)
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
  // Configure optional receive-size FIFO and initialize the shared ring buffer.
  rxRing.setToUseReceiveSizeFIFO(/*fifoDepth*/ 8);
  (void)rxRing.initialize();
}

void ReactionWheelsHandler::doStartUp() {
  // Give the device a brief warm-up period before entering NORMAL
  if (warmupCnt < warmupCycles) {
    ++warmupCnt;
    return;
  }
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

// Helper: Drain UART RX until no more bytes are immediately available (drop).
// Hinweis: Diese Funktion wird *nicht* direkt vor TC-getriebenem STATUS-Poll aufgerufen,
// damit wir keine frischen Replies verwerfen.
ReturnValue_t ReactionWheelsHandler::drainRxNow() {
  if (communicationInterface == nullptr || comCookie == nullptr) {
    return returnvalue::OK;
  }

  // Drain at most a few frames; each frame on the wire is STATUS_LEN bytes.
  for (int i = 0; i < 3; ++i) {
    // Request exactly one frame to avoid UartComIF "Only read X of Y" warnings.
    ReturnValue_t rvReq =
        communicationInterface->requestReceiveMessage(comCookie, RwProtocol::STATUS_LEN);
    if (rvReq != returnvalue::OK) {
      // No pending data or not ready; stop draining
      break;
    }

    uint8_t* buf = nullptr;
    size_t sz = 0;
    ReturnValue_t rvRead = communicationInterface->readReceivedMessage(comCookie, &buf, &sz);
    if (rvRead != returnvalue::OK || buf == nullptr || sz == 0) {
      break;  // nothing to drop
    }
#if RW_VERBOSE
    sif::warning << "drainRxNow: dropped " << sz << " stale bytes" << std::endl;
    dumpHexWarn("drainRxNow: bytes", buf, (sz > 32 ? 32 : sz));
#endif
    // Note: buffer ownership is managed by the ComIF, do not free here.
  }

  return returnvalue::OK;
}

// Helper: Drain UART RX into the shared ring buffer (non-blocking, thread-safe)
ReturnValue_t ReactionWheelsHandler::drainRxIntoRing() {
  if (communicationInterface == nullptr || comCookie == nullptr) {
    return returnvalue::OK;
  }

  for (int i = 0; i < 4; ++i) {
    (void)rxRing.lockRingBufferMutex(MutexIF::TimeoutType::WAITING, /*timeout ms*/ 2);

    const size_t freeSpace = rxRing.availableWriteSpace();  // FSFW naming
    if (freeSpace == 0) {
      (void)rxRing.unlockRingBufferMutex();
      break;
    }

    const size_t chunk = (freeSpace < 64) ? freeSpace : size_t(64);
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
  // 1) If a STATUS TM is pending due to a TC request, service it immediately.
  if (pendingTcStatusTm) {
#if RW_VERBOSE
    sif::warning << "buildNormalDeviceCommand: tcStatusPending=true -> immediate STATUS poll"
                 << std::endl;
#endif
    // WICHTIG: Hier nichts droppen! In scanForReply() wählen wir die neueste gültige Antwort.

    const size_t total = RwProtocol::buildStatusReq(txBuf, sizeof(txBuf));
    if (total == 0) {
      *id = DeviceHandlerIF::NO_COMMAND_ID;
      return NOTHING_TO_SEND;
    }
    rawPacket    = txBuf;
    rawPacketLen = total;
    *id          = CMD_STATUS_POLL;

    dumpHexWarn("DH TX (TC-driven STATUS poll)", txBuf, total);
    return returnvalue::OK;
  }

  // 2) Snooze only if no TC-driven STATUS is pending.
  if (pollSnooze > 0) {
    --pollSnooze;
#if RW_VERBOSE
    sif::warning << "buildNormalDeviceCommand: pollSnooze>0 -> NO_COMMAND (snooze left="
                 << pollSnooze << ")" << std::endl;
#endif
    *id = DeviceHandlerIF::NO_COMMAND_ID;
    return NOTHING_TO_SEND;
  }

  // 3) Periodic STATUS poll
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

    dumpHexWarn("DH TX (periodic STATUS poll)", txBuf, total);
    return returnvalue::OK;
  }

  // 4) Nothing to send this cycle
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
      lastTargetRpm = rpm;

      const size_t total = RwProtocol::buildSetSpeed(txBuf, sizeof(txBuf), rpm);
      if (total == 0) {
        return returnvalue::FAILED;
      }

      rawPacket    = txBuf;
      rawPacketLen = total;

      dumpHexWarn("DH TX (TC frame: SET_SPEED)", txBuf, total);
      pollSnooze = POLL_SNOOZE_CYCLES;  // short backoff after TC
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
      pollSnooze = POLL_SNOOZE_CYCLES;
      return returnvalue::OK;
    }

    case CMD_STATUS: {
      // Action-driven STATUS request (send same wire frame, reply handled as POLL).
      const size_t total = RwProtocol::buildStatusReq(txBuf, sizeof(txBuf));
      if (total == 0) {
        return returnvalue::FAILED;
      }

      rawPacket    = txBuf;
      rawPacketLen = total;

      dumpHexWarn("DH TX (TC frame: STATUS)", txBuf, total);
      pollSnooze = POLL_SNOOZE_CYCLES;
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

  // Poll mapping (single, unified reply classification)
  insertInCommandAndReplyMap(
      /*deviceCommand*/ CMD_STATUS_POLL,
      /*maxDelayCycles*/ 5,
      /*replyDataSet*/ &replySet,
      /*replyLen*/ RwProtocol::STATUS_LEN,  // reply is 9 bytes (CRC-16)
      /*periodic*/ false,
      /*hasDifferentReplyId*/ true,
      /*replyId*/ REPLY_STATUS_POLL,
      /*countdown*/ nullptr);

  // No direct reply for CMD_STATUS (TC); data goes via ActionHelper::reportData(...)
  insertInCommandMap(CMD_STATUS, false, 0);
}

// Helper: suche neueste gültige STATUS-Antwort in einem Bytefenster
static inline bool findLatestStatusInWindow(const uint8_t* buf, size_t n, long& posOut) {
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

  // 1) Primär: Das von FSFW/ComIF bereitgestellte Fenster untersuchen.
  long pos = -1;
  if (findLatestStatusInWindow(start, len, pos)) {
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

  // 2) Fallback: Asynchron eingetroffene Bytes aus dem UART in den Ring ziehen und dort suchen.
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

  if (!findLatestStatusInWindow(snapshot.data(), snapshot.size(), pos)) {
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

  // Prefer ring-buffered/dataset frame if available; otherwise fall back to the incoming pointer.
  const uint8_t* pkt = replySet.raw.isValid() ? replySet.raw.value : packet;
  dumpHexWarn("interpretDeviceReply: frame", pkt, RwProtocol::STATUS_LEN);

  // Parse big-endian fields (simple and explicit)
  const int16_t speed   = static_cast<int16_t>((pkt[2] << 8) | pkt[3]);
  const int16_t torque  = static_cast<int16_t>((pkt[4] << 8) | pkt[5]);
  const uint8_t running = pkt[6];

  // Console output
  sif::info << "RW STATUS: speed=" << speed << " RPM, torque=" << torque
            << " mNm, running=" << int(running) << std::endl;

  // Ensure dataset contains the frame (already set in scanForReply; keep it valid here too).
  if (!replySet.raw.isValid()) {
    std::memcpy(replySet.raw.value, pkt, RwProtocol::STATUS_LEN);
    replySet.raw.setValid(true);
    replySet.setValidity(true, true);
  }

  // If a TC is waiting for a DATA reply, route it back to the PUS service
  if (pendingTcStatusTm) {
    if (pendingTcStatusReportedTo == MessageQueueIF::NO_QUEUE) {
#if RW_VERBOSE
      sif::warning << "interpretDeviceReply: reportTo queue unknown; cannot send DATA_REPLY"
                   << std::endl;
#endif
    } else {
      (void)actionHelper.reportData(pendingTcStatusReportedTo, CMD_STATUS, pkt,
                                    RwProtocol::STATUS_LEN, /*append*/ false);
    }
    // Clear TC-wait flags after delivering the data
    pendingTcStatusTm = false;
    pendingTcStatusReportedTo = MessageQueueIF::NO_QUEUE;
  }

  return returnvalue::OK;
}

uint32_t ReactionWheelsHandler::getTransitionDelayMs(Mode_t /*from*/, Mode_t /*to*/) { //DELETE???
  return 6000;
}

ReturnValue_t ReactionWheelsHandler::initializeLocalDataPool(localpool::DataPool& localDataPoolMap,
                                                             LocalDataPoolManager&) {
  // Backing entry for LocalPoolVector<uint8_t, STATUS_LEN> in dataset
  localDataPoolMap.emplace(static_cast<lp_id_t>(PoolIds::RAW_REPLY),
                           new PoolEntry<uint8_t>(RwProtocol::STATUS_LEN));
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
    // Remember requester, snooze polls briefly, and trigger next poll immediately
    pendingTcStatusTm = true;
    pendingTcStatusReportedTo = commandedBy;
    pollSnooze = POLL_SNOOZE_CYCLES;
  }
  return DeviceHandlerBase::executeAction(actionId, commandedBy, data, size);
}
