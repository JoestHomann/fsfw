#include "fsfw/devicehandlers/ReactionWheelsHandler.h"

#include <cstring>   // memcpy
#include <iomanip>   // debug formatting

#include "fsfw/devicehandlers/RwProtocol.h"
#include "fsfw/action/ActionHelper.h"  // reportData(...)
#include "fsfw/devicehandlers/DeviceHandlerIF.h"
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
    : DeviceHandlerBase(objectId, comIF, cookie) {}

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

// Helper: Drain UART RX until no more bytes are immediately available
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

ReturnValue_t ReactionWheelsHandler::buildNormalDeviceCommand(DeviceCommandId_t* id) {
  // 1) If a STATUS TM is pending due to a TC request, service it immediately.
  if (pendingTcStatusTm) {
#if RW_VERBOSE
    sif::warning << "buildNormalDeviceCommand: tcStatusPending=true -> immediate STATUS poll"
                 << std::endl;
#endif
    (void)drainRxNow();  // drop stale frames before sending

    const size_t total = RwProtocol::buildStatusReq(txBuf, sizeof(txBuf));
    if (total == 0) {
      *id = DeviceHandlerIF::NO_COMMAND_ID;
      return NOTHING_TO_SEND;
    }
    rawPacket   = txBuf;
    rawPacketLen= total;
    *id = CMD_STATUS_POLL;

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
    rawPacket   = txBuf;
    rawPacketLen= total;
    *id = CMD_STATUS_POLL;

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
      /*replyLen*/ RwProtocol::STATUS_LEN,  // reply is 9 bytes now (CRC-16)
      /*periodic*/ false,
      /*hasDifferentReplyId*/ true,
      /*replyId*/ REPLY_STATUS_POLL,
      /*countdown*/ nullptr);

  // No direct reply for CMD_STATUS (TC); data goes via ActionHelper::reportData(...)
  insertInCommandMap(CMD_STATUS, false, 0);
}

ReturnValue_t ReactionWheelsHandler::scanForReply(const uint8_t* start, size_t len,
                                                  DeviceCommandId_t* foundId, size_t* foundLen) {
#if RW_VERBOSE
  dumpHexWarn("scanForReply: head bytes", start, (len > 16 ? 16 : len));
#endif

  // Expect STATUS_LEN-byte frame with CRC-16:
  // [AB, 10, spdH, spdL, torH, torL, running, crcH, crcL]
  if (len < RwProtocol::STATUS_LEN) {
    return returnvalue::FAILED;
  }
  if (start[0] != RwProtocol::START_REPLY ||
      start[1] != static_cast<uint8_t>(RwProtocol::RespId::STATUS)) {
    return returnvalue::FAILED;
  }
  if (!RwProtocol::verifyCrc16(start, RwProtocol::STATUS_LEN)) {
    return returnvalue::FAILED;
  }

  *foundId  = REPLY_STATUS_POLL;  // unified classification to match the map
  *foundLen = RwProtocol::STATUS_LEN;

#if RW_VERBOSE
  sif::warning << "scanForReply: MATCH -> foundId=0x" << std::hex << int(REPLY_STATUS_POLL)
               << std::dec << " len=" << *foundLen << std::endl;
  dumpHexWarn("scanForReply: matched frame", start, RwProtocol::STATUS_LEN);
#endif
  return returnvalue::OK;
}

ReturnValue_t ReactionWheelsHandler::interpretDeviceReply(DeviceCommandId_t id,
                                                          const uint8_t* packet) {
  if (id != REPLY_STATUS_POLL) {
    return returnvalue::FAILED;
  }

  dumpHexWarn("interpretDeviceReply: frame", packet, RwProtocol::STATUS_LEN);

  // Parse big-endian fields (simple and explicit)
  const int16_t speed   = static_cast<int16_t>((packet[2] << 8) | packet[3]);
  const int16_t torque  = static_cast<int16_t>((packet[4] << 8) | packet[5]);
  const uint8_t running = packet[6];

  // Console output (keep user-visible)
  sif::info << "RW STATUS: speed=" << speed << " RPM, torque=" << torque
            << " mNm, running=" << int(running) << std::endl;

  // Update local dataset (store full wire frame)
  std::memcpy(replySet.raw.value, packet, RwProtocol::STATUS_LEN);
  replySet.raw.setValid(true);
  replySet.setValidity(true, true);

  // If a TC is waiting for a DATA reply, route it back to the PUS service
  if (pendingTcStatusTm) {
    if (pendingTcStatusReportedTo == MessageQueueIF::NO_QUEUE) {
#if RW_VERBOSE
      sif::warning << "interpretDeviceReply: reportTo queue unknown; cannot send DATA_REPLY"
                   << std::endl;
#endif
    } else {
      (void)actionHelper.reportData(pendingTcStatusReportedTo, CMD_STATUS, packet,
                                    RwProtocol::STATUS_LEN, /*append*/ false);
    }
    // Clear TC-wait flags after delivering the data
    pendingTcStatusTm = false;
    pendingTcStatusReportedTo = MessageQueueIF::NO_QUEUE;
  }

  return returnvalue::OK;
}

uint32_t ReactionWheelsHandler::getTransitionDelayMs(Mode_t /*from*/, Mode_t /*to*/) {
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
