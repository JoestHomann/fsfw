#include "RwCommanderHandler.h"

#include <cstring>
#include <iomanip>      // Debug formatting
#include <algorithm>    // std::min
#include "fsfw/returnvalues/returnvalue.h"
#include "fsfw/serviceinterface/ServiceInterfaceStream.h"
#include "fsfw/devicehandlers/DeviceHandlerIF.h"
#include "fsfw/action/HasActionsIF.h"
#include "fsfw/action/ActionHelper.h"  // reportData(...)

// Debug On/Off switch (set to 1 to enable debug output):
#define RW_VERBOSE 0

#if RW_VERBOSE
static void dumpHexWarn(const char* tag, const uint8_t* p, size_t n) {
# if FSFW_CPP_OSTREAM_ENABLED == 1
  sif::warning << tag << " (" << n << "): ";
  for (size_t i = 0; i < n; ++i) {
    sif::warning << std::hex << std::setw(2) << std::setfill('0') << int(p[i]) << " ";
  }
  sif::warning << std::dec << std::endl;
# else
  (void)tag; (void)p; (void)n;
# endif
}
#else
static inline void dumpHexWarn(const char*, const uint8_t*, size_t) {}
#endif

RwCommanderHandler::RwCommanderHandler(object_id_t objectId, object_id_t comIF, CookieIF* cookie)
    : DeviceHandlerBase(objectId, comIF, cookie) {}

void RwCommanderHandler::doStartUp() {
  if (warmupCnt < warmupCycles) {
    ++warmupCnt;
    return;
  }
  sif::info << "RwCommanderHandler: doStartUp()" << std::endl;
  setMode(MODE_NORMAL);
}

void RwCommanderHandler::doShutDown() {
  sif::info << "RwCommanderHandler: doShutDown()" << std::endl;
  setMode(MODE_OFF);
}

ReturnValue_t RwCommanderHandler::performOperation(uint8_t opCode) {
  return DeviceHandlerBase::performOperation(opCode);
}

void RwCommanderHandler::modeChanged() {
  Mode_t m{}; Submode_t s{};
  this->getMode(&m, &s);
#if RW_VERBOSE
  sif::info << "RwCommanderHandler: modeChanged -> " << static_cast<int>(m)
            << " (sub=" << static_cast<int>(s) << ")" << std::endl;
#endif
}

ReturnValue_t RwCommanderHandler::buildNormalDeviceCommand(DeviceCommandId_t* id) {
  if (pollSnooze > 0) {
    --pollSnooze;
#if RW_VERBOSE
    sif::warning << "buildNormalDeviceCommand: pollSnooze>0 -> NO_COMMAND (snooze left="
                 << pollSnooze << ")" << std::endl;
#endif
    *id = DeviceHandlerIF::NO_COMMAND_ID;
    return NOTHING_TO_SEND;
  }

  // Wenn ein TC-STATUS outstanding ist, poll sofort (damit der späte Frame zügig kommt)
  if (pendingTcStatusTm) {
#if RW_VERBOSE
    sif::warning << "buildNormalDeviceCommand: tcStatusPending=true" << std::endl;
#endif

    txBuf[0] = START_BYTE_CMD;
    txBuf[1] = static_cast<uint8_t>(CMD_STATUS);
    txBuf[2] = 0x00;
    txBuf[3] = 0x00;
    txBuf[4] = crc8(txBuf, 4);

    rawPacket    = txBuf;
    rawPacketLen = 5;
    *id          = CMD_STATUS_POLL;

    lastSentWasPoll = true;

    dumpHexWarn("DH TX (poll frame)", txBuf, 5);
#if RW_VERBOSE
    sif::info << "RwCommanderHandler: periodic STATUS" << std::endl;
#endif
    // Zähler bewusst nicht hochzählen, um TC-Flow nicht zu blockieren
    return returnvalue::OK;
  }

  if (++statusPollCnt >= statusPollDivider) {
    statusPollCnt = 0;

    txBuf[0] = START_BYTE_CMD;
    txBuf[1] = static_cast<uint8_t>(CMD_STATUS);
    txBuf[2] = 0x00;
    txBuf[3] = 0x00;
    txBuf[4] = crc8(txBuf, 4);

    rawPacket    = txBuf;
    rawPacketLen = 5;
    *id          = CMD_STATUS_POLL;

    lastSentWasPoll = true;

    dumpHexWarn("DH TX (poll frame)", txBuf, 5);
#if RW_VERBOSE
    sif::info << "RwCommanderHandler: periodic STATUS" << std::endl;
#endif
    return returnvalue::OK;
  }

  *id = DeviceHandlerIF::NO_COMMAND_ID;
  return NOTHING_TO_SEND;
}

ReturnValue_t RwCommanderHandler::buildTransitionDeviceCommand(DeviceCommandId_t* id) {
  *id = DeviceHandlerIF::NO_COMMAND_ID;
  return NOTHING_TO_SEND;
}

ReturnValue_t RwCommanderHandler::buildCommandFromCommand(DeviceCommandId_t deviceCommand,
                                                          const uint8_t* data, size_t len) {
  switch (deviceCommand) {
    case CMD_SET_SPEED: {
      if (len < sizeof(int16_t)) {
        return returnvalue::FAILED;
      }
      int16_t rpm = 0;
      std::memcpy(&rpm, data, sizeof(rpm));
      lastTargetRpm = rpm;
      const uint16_t u = static_cast<uint16_t>(rpm);

      txBuf[0] = START_BYTE_CMD;
      txBuf[1] = static_cast<uint8_t>(CMD_SET_SPEED);
      txBuf[2] = static_cast<uint8_t>((u >> 8) & 0xFF);
      txBuf[3] = static_cast<uint8_t>( u       & 0xFF);
      txBuf[4] = crc8(txBuf, 4);

      rawPacket    = txBuf;
      rawPacketLen = 5;

      lastSentWasPoll = false;
      pollSnooze      = POLL_SNOOZE_CYCLES;

      dumpHexWarn("DH TX (TC frame: SET_SPEED)", txBuf, 5);
#if RW_VERBOSE
      sif::info << "RwCommanderHandler: SET_SPEED " << rpm << " RPM" << std::endl;
#endif
      return returnvalue::OK;
    }

    case CMD_STOP: {
      txBuf[0] = START_BYTE_CMD;
      txBuf[1] = static_cast<uint8_t>(CMD_STOP);
      txBuf[2] = 0x00;
      txBuf[3] = 0x00;
      txBuf[4] = crc8(txBuf, 4);

      rawPacket    = txBuf;
      rawPacketLen = 5;

      lastSentWasPoll = false;
      pollSnooze      = POLL_SNOOZE_CYCLES;

      dumpHexWarn("DH TX (TC frame: STOP)", txBuf, 5);
#if RW_VERBOSE
      sif::info << "RwCommanderHandler: STOP" << std::endl;
#endif
      return returnvalue::OK;
    }

    case CMD_STATUS: {
      txBuf[0] = START_BYTE_CMD;
      txBuf[1] = static_cast<uint8_t>(CMD_STATUS);
      txBuf[2] = 0x00;
      txBuf[3] = 0x00;
      txBuf[4] = crc8(txBuf, 4);

      rawPacket    = txBuf;
      rawPacketLen = 5;

      lastSentWasPoll = false;
      pollSnooze      = POLL_SNOOZE_CYCLES;

      dumpHexWarn("DH TX (TC frame: STATUS)", txBuf, 5);
#if RW_VERBOSE
      sif::info << "RwCommanderHandler: STATUS (TC)" << std::endl;
#endif
      return returnvalue::OK;
    }

    default:
#if RW_VERBOSE
      sif::warning << "RwCommanderHandler: unknown deviceCommand 0x"
                   << std::hex << deviceCommand << std::dec << std::endl;
#endif
      return returnvalue::FAILED;
  }
}

void RwCommanderHandler::fillCommandAndReplyMap() {
  // Commands without different reply id
  insertInCommandMap(CMD_SET_SPEED, false, 0);
  insertInCommandMap(CMD_STOP,      false, 0);

  // Poll mapping (no forwarding)
  insertInCommandAndReplyMap(
      /*deviceCommand*/       CMD_STATUS_POLL,
      /*maxDelayCycles*/      5,
      /*replyDataSet*/        &replySet,
      /*replyLen*/            8,
      /*periodic*/            false,
      /*hasDifferentReplyId*/ true,
      /*replyId*/             REPLY_STATUS_POLL,
      /*countdown*/           nullptr);

  // Kein direktes Reply für CMD_STATUS (TC); Daten gehen via ActionHelper::reportData(...)
  insertInCommandMap(CMD_STATUS, false, 0);

#if RW_VERBOSE
  sif::info << "RwCommanderHandler: command/reply map set up." << std::endl;
#endif
}

ReturnValue_t RwCommanderHandler::scanForReply(const uint8_t* start, size_t len,
                                               DeviceCommandId_t* foundId, size_t* foundLen) {
#if RW_VERBOSE
  sif::warning << "scanForReply: len=" << len << std::endl;
  if (len > 0) {
    dumpHexWarn("scanForReply: head bytes", start, std::min(len, size_t(16)));
  }
#endif

  // Expect 8-byte frame: AB 10 <spdH spdL> <torH torL> <running> <crc>
  if (len < 8) {
    return returnvalue::FAILED;
  }
  if (start[0] != START_BYTE_REPLY) {
    return returnvalue::FAILED;
  }
  if (start[1] != REPLY_STATUS_WIRE) {
    return returnvalue::FAILED;
  }
  if (crc8(start, 7) != start[7]) {
    return returnvalue::FAILED;
  }

  *foundId  = (lastSentWasPoll ? REPLY_STATUS_POLL : REPLY_STATUS_TC);
  *foundLen = 8;

#if RW_VERBOSE
  sif::warning << "scanForReply: MATCH wireId=0x" << std::hex << int(start[1])
               << " lastSentWasPoll=" << std::dec << int(lastSentWasPoll)
               << " -> foundId=0x" << std::hex
               << int(lastSentWasPoll ? REPLY_STATUS_POLL : REPLY_STATUS_TC)
               << std::dec << " foundLen=" << *foundLen << std::endl;
  dumpHexWarn("scanForReply: matched frame", start, 8);

  sif::warning << "scanForReply: classify="
               << (lastSentWasPoll ? "POLL" : "TC")
               << " | flags{tcPending=" << pendingTcStatusTm
               << ", lastSentWasPoll=" << lastSentWasPoll << "}"
               << std::endl;
#endif

  return returnvalue::OK;
}

ReturnValue_t RwCommanderHandler::interpretDeviceReply(DeviceCommandId_t id,
                                                       const uint8_t* packet) {
  if (id == REPLY_STATUS_TC || id == REPLY_STATUS_POLL) {
    dumpHexWarn("interpretDeviceReply: frame", packet, 8);
#if RW_VERBOSE
    sif::warning << "interpretDeviceReply: id=0x" << std::hex << int(id)
                 << " (TC? " << std::dec << (id == REPLY_STATUS_TC) << ")" << std::endl;
    sif::warning << "interpretDeviceReply: flags{tcPending=" << pendingTcStatusTm
                 << ", lastSentWasPoll=" << lastSentWasPoll << "}" << std::endl;
#endif

    const int16_t speed   = static_cast<int16_t>((packet[2] << 8) | packet[3]);
    const int16_t torque  = static_cast<int16_t>((packet[4] << 8) | packet[5]);
    const uint8_t running = packet[6];

    // Console output 
    sif::info << "RW STATUS: speed=" << speed
              << " RPM, torque=" << torque
              << " mNm, running=" << int(running)
              << (id == REPLY_STATUS_POLL ? " [poll]" : " [tc]") << std::endl;


    // Update local dataset
    for (size_t i = 0; i < 8; ++i) {
      replySet.raw[i] = packet[i];
    }
    replySet.raw.setValid(true);
    replySet.setValidity(true, true);
#if RW_VERBOSE
    sif::warning << "interpretDeviceReply: replySet.raw valid=" << replySet.raw.isValid()
                 << " dataset validity set (true)" << std::endl;
#endif

    // Wenn ein TC auf Antwort wartet, route DATA_REPLY zurück zum PUS-Dienst
    if (pendingTcStatusTm) {
      if (pendingTcStatusReportedTo == MessageQueueIF::NO_QUEUE) {
#if RW_VERBOSE
        sif::warning << "interpretDeviceReply: reportTo queue unknown; cannot send DATA_REPLY"
                     << std::endl;
#endif
      } else {
        ReturnValue_t rv = actionHelper.reportData(
            pendingTcStatusReportedTo, CMD_STATUS, packet, 8, /*append*/ false);
#if RW_VERBOSE
        if (rv != returnvalue::OK) {
          sif::warning << "interpretDeviceReply: reportData(CMD_STATUS) failed rv=" << rv
                       << std::endl;
        } else {
          sif::warning << "interpretDeviceReply: reportData(CMD_STATUS) sent to q=0x"
                       << std::hex << pendingTcStatusReportedTo << std::dec << std::endl;
        }
#endif
      }
      // Nach Versand: Flag löschen, Queue zurücksetzen
      pendingTcStatusTm         = false;
      pendingTcStatusReportedTo = MessageQueueIF::NO_QUEUE;
    }

    return returnvalue::OK;
  }

  return returnvalue::FAILED;
}

uint32_t RwCommanderHandler::getTransitionDelayMs(Mode_t /*from*/, Mode_t /*to*/) {
  return 6000;
}

ReturnValue_t RwCommanderHandler::initializeLocalDataPool(localpool::DataPool& localDataPoolMap,
                                                          LocalDataPoolManager&) {
  // Backing entry für LocalPoolVector<uint8_t,8> in dataset
  localDataPoolMap.emplace(static_cast<lp_id_t>(PoolIds::RAW_REPLY), new PoolEntry<uint8_t>(8));
  return returnvalue::OK;
}

ReturnValue_t RwCommanderHandler::executeAction(ActionId_t actionId,
                                                MessageQueueId_t commandedBy,
                                                const uint8_t* data,
                                                size_t size) {
#if RW_VERBOSE
  sif::warning << "executeAction: actionId=0x" << std::hex << int(actionId)
               << " size=" << std::dec << size
               << " commandedBy=0x" << std::hex << commandedBy << std::dec << std::endl;
#endif

  const auto cmd = static_cast<DeviceCommandId_t>(actionId);

  if (cmd == CMD_STATUS) {
    // Requester merken, Poll kurz snoozen und nächsten Poll forcieren
    pendingTcStatusTm         = true;
    pendingTcStatusReportedTo = commandedBy;
    lastSentWasPoll           = false;
    pollSnooze                = POLL_SNOOZE_CYCLES;

#if RW_VERBOSE
    sif::warning << "executeAction: CMD_STATUS -> lastSentWasPoll=false, pollSnooze="
                 << POLL_SNOOZE_CYCLES << " reportTo=0x"
                 << std::hex << pendingTcStatusReportedTo << std::dec
                 << " forceNextPoll=true (cnt=" << statusPollCnt << "/" << statusPollDivider
                 << ")" << std::endl;
#endif
  }

  return DeviceHandlerBase::executeAction(actionId, commandedBy, data, size);
}

uint8_t RwCommanderHandler::crc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int j = 0; j < 8; ++j) {
      if (crc & 0x80) crc = static_cast<uint8_t>((crc << 1) ^ 0x07);
      else            crc = static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}
