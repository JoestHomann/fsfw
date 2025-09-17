#include "RwCommanderHandler.h"

#include <cstring>
#include "fsfw/returnvalues/returnvalue.h"
#include "fsfw/serviceinterface/ServiceInterfaceStream.h"
#include "fsfw/devicehandlers/DeviceHandlerIF.h"
#include "fsfw/action/HasActionsIF.h"


RwCommanderHandler::RwCommanderHandler(object_id_t objectId, object_id_t comIF, CookieIF* cookie)
    : DeviceHandlerBase(objectId, comIF, cookie) {}

void RwCommanderHandler::doStartUp() {
  // Give device a brief warm-up if you want
  if (warmupCnt < warmupCycles) {
    ++warmupCnt;
    return; // stay in transition
  }
  // Enter NORMAL so buildNormalDeviceCommand() is called periodically
  setMode(MODE_NORMAL);
}

void RwCommanderHandler::doShutDown() {
  sif::info << "RwCommanderHandler: doShutDown()" << std::endl;
  setMode(MODE_OFF);
}

ReturnValue_t RwCommanderHandler::performOperation(uint8_t opCode) {
  // Let base do its state machine
  return DeviceHandlerBase::performOperation(opCode);
}

void RwCommanderHandler::modeChanged() {
  Mode_t m{}; Submode_t s{};
  this->getMode(&m, &s);  // FSFW: pointer-based getter
  sif::info << "RwCommanderHandler: modeChanged -> " << static_cast<int>(m)
            << " (sub=" << static_cast<int>(s) << ")" << std::endl;
}

ReturnValue_t RwCommanderHandler::buildNormalDeviceCommand(DeviceCommandId_t* id) {
  // Skip polls for a short window after external TCs so their replies are not interleaved
  if (pollSnooze > 0) {
    --pollSnooze;
    *id = DeviceHandlerIF::NO_COMMAND_ID;
    return NOTHING_TO_SEND;
  }

  // Periodic poll
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

#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::info << "RwCommanderHandler: periodic STATUS" << std::endl;
#endif
    return returnvalue::OK;
  }

  // Nothing to send this cycle
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
      // RPM arrives as native int16_t from IPC store
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
      pollSnooze = POLL_SNOOZE_CYCLES;

      sif::info << "RwCommanderHandler: SET_SPEED " << rpm << " RPM" << std::endl;
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
      pollSnooze = POLL_SNOOZE_CYCLES;

      sif::info << "RwCommanderHandler: STOP" << std::endl;
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
      pollSnooze = POLL_SNOOZE_CYCLES;

      sif::info << "RwCommanderHandler: STATUS (TC)" << std::endl;
      return returnvalue::OK;
    }

    default:
      sif::warning << "RwCommanderHandler: unknown deviceCommand 0x"
                   << std::hex << deviceCommand << std::dec << std::endl;
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
      /*replyDataSet*/        &replySet,   // ok; will not be forwarded
      /*replyLen*/            8,
      /*periodic*/            false,
      /*hasDifferentReplyId*/ true,
      /*replyId*/             REPLY_STATUS_POLL,
      /*countdown*/           nullptr);

  // TC STATUS mapping (will be forwarded as DATA_REPLY)
  insertInCommandAndReplyMap(
      /*deviceCommand*/       CMD_STATUS,
      /*maxDelayCycles*/      5,
      /*replyDataSet*/        &replySet,   // required for REPLY_DIRECT_COMMAND_DATA (DATA_REPLY)
      /*replyLen*/            0,           // length comes from dataset serialization
      /*periodic*/            false,
      /*hasDifferentReplyId*/ true,
      /*replyId*/             REPLY_STATUS_TC,
      /*countdown*/           nullptr);

  sif::info << "RwCommanderHandler: command/reply map set up." << std::endl;
}


ReturnValue_t RwCommanderHandler::scanForReply(const uint8_t* start, size_t len,
                                               DeviceCommandId_t* foundId, size_t* foundLen) {
  // Expect 8-byte frame: AB 10 <spdH spdL> <torH torL> <running> <crc>
  if (len < 8) {
    return returnvalue::FAILED;
  }
  if (start[0] != START_BYTE_REPLY) {
    return returnvalue::FAILED;
  }
  // 0x10 is the on-wire reply discriminator
  if (start[1] != REPLY_STATUS_WIRE) {
    return returnvalue::FAILED;
  }
  // Verify simple CRC8 over first 7 bytes
  if (crc8(start, 7) != start[7]) {
    return returnvalue::FAILED;
  }

  // Demultiplex: decide whether this belongs to a poll or to a TC
  *foundId  = (lastSentWasPoll ? REPLY_STATUS_POLL : REPLY_STATUS_TC);
  *foundLen = 8;
  return returnvalue::OK;
}

ReturnValue_t RwCommanderHandler::interpretDeviceReply(DeviceCommandId_t id,
                                                       const uint8_t* packet) {
  if (id == REPLY_STATUS_TC || id == REPLY_STATUS_POLL) {
    // Decode for logging
    const int16_t speed   = static_cast<int16_t>((packet[2] << 8) | packet[3]);
    const int16_t torque  = static_cast<int16_t>((packet[4] << 8) | packet[5]);
    const uint8_t running = packet[6];

    sif::info << "RW STATUS: speed=" << speed
              << " RPM, torque=" << torque
              << " mNm, running=" << int(running)
              << (id == REPLY_STATUS_POLL ? " [poll]" : " [tc]") << std::endl;

    // ---- Fill dataset so the base can serialize it into the store ----
    // Write the raw 8-byte device frame into the LocalPoolVector.
    for (size_t i = 0; i < 8; ++i) {
      replySet.raw[i] = packet[i];
    }
    // Mark the variable valid and the dataset valid
    replySet.raw.setValid(true);
    replySet.setValidity(true, true);  // <== important: mark dataset valid/changed

    if (id == REPLY_STATUS_TC) {
      // Preferred: FSFW-official direct reply via dataset
      handleDeviceTm(replySet, REPLY_STATUS_TC);

      // ---- Optional fallback: also push a raw direct reply with the same payload. ----
      // This guarantees we see something on the PUS side even if dataset serialization is empty.
      // You can keep this while debugging and remove later if not needed.
      handleDeviceTm(packet, 8, REPLY_STATUS_TC);
    }

    return returnvalue::OK;
  }

  return returnvalue::FAILED;
}


// Give the state machine time between mode transitions so timeouts don't fire immediately.
uint32_t RwCommanderHandler::getTransitionDelayMs(Mode_t /*from*/, Mode_t /*to*/) {
  return 6000;  // 6000 ms; tune as needed for your setup
}

ReturnValue_t RwCommanderHandler::initializeLocalDataPool(localpool::DataPool& localDataPoolMap,
                                                          LocalDataPoolManager&) {
  // Backing entry for LocalPoolVector<uint8_t,8> in the dataset
  localDataPoolMap.emplace(static_cast<lp_id_t>(PoolIds::RAW_REPLY), new PoolEntry<uint8_t>(8));
  return returnvalue::OK;
}

// Route actions: For STATUS only, mark flags; then delegate to base so the command lifecycle runs.
// This avoids bypassing the base (which would prevent proper sending & completion).
ReturnValue_t RwCommanderHandler::executeAction(ActionId_t actionId,
                                                MessageQueueId_t commandedBy,
                                                const uint8_t* data,
                                                size_t size) {
  const auto cmd = static_cast<DeviceCommandId_t>(actionId);

  if (cmd == CMD_STATUS) {
    // We owe one TM for this TC; also keep polls from racing the reply
    pendingTcStatusTm = true;
    lastSentWasPoll   = false;
    pollSnooze        = POLL_SNOOZE_CYCLES;
  }

  // Let the base enqueue and drive the command (it will call buildCommandFromCommand)
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
