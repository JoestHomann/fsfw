#include "RwCommanderHandler.h"

#include <cstring>

#include "fsfw/returnvalues/returnvalue.h"
#include "fsfw/serviceinterface/ServiceInterfaceStream.h"
#include "fsfw/devicehandlers/DeviceHandlerIF.h"
#include "fsfw/storagemanager/StorageManagerIF.h"

RwCommanderHandler::RwCommanderHandler(object_id_t objectId, object_id_t comIF, CookieIF* cookie)
    : DeviceHandlerBase(objectId, comIF, cookie) {}

void RwCommanderHandler::doStartUp() {
  // Give device a brief warm-up if you want
  if (warmupCnt < warmupCycles) {
    ++warmupCnt;
    return;  // stay in transition
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

// FSFW base calls parameterless modeChanged(); query current mode if needed.
void RwCommanderHandler::modeChanged() {
  Mode_t mode{};
  Submode_t sub{};
  getMode(&mode, &sub);  // Provided by DeviceHandlerBase

  // Logging the new state
  sif::info << "RwCommanderHandler: modeChanged to mode=" << int(mode)
            << " submode=" << int(sub) << std::endl;
}

ReturnValue_t RwCommanderHandler::buildTransitionDeviceCommand(DeviceCommandId_t* /*id*/) {
  // No special transitional commands in this minimal example
  return NOTHING_TO_SEND;
}

ReturnValue_t RwCommanderHandler::buildNormalDeviceCommand(DeviceCommandId_t* id) {
  // Skip polling while a TC reply is expected (snooze window)
  if (pollSnooze > 0) {
    --pollSnooze;
    return NOTHING_TO_SEND;
  }

  // Example: poll STATUS periodically when in normal mode
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

#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::info << "RwCommanderHandler: poll STATUS" << std::endl;
#endif
    return returnvalue::OK;
  }

  // Otherwise nothing to send this cycle
  return NOTHING_TO_SEND;
}

ReturnValue_t RwCommanderHandler::buildCommandFromCommand(DeviceCommandId_t deviceCommand,
                                                          const uint8_t* commandData,
                                                          size_t commandDataLen) {
  // This function is called when external commands (e.g. from a PUS service)
  // request a device command. We build the corresponding on-wire frame.

  switch (deviceCommand) {
    case CMD_SET_SPEED: {
      // RPM arrives as native int16_t from IPC store (not BE high/low)
      if (commandDataLen < sizeof(int16_t)) {
        return returnvalue::FAILED;
      }
      int16_t rpm = 0;
      std::memcpy(&rpm, commandData, sizeof(rpm));
      lastTargetRpm = rpm;

      txBuf[0] = START_BYTE_CMD;
      txBuf[1] = static_cast<uint8_t>(CMD_SET_SPEED);
      txBuf[2] = static_cast<uint8_t>((rpm >> 8) & 0xFF);
      txBuf[3] = static_cast<uint8_t>( rpm       & 0xFF);
      txBuf[4] = crc8(txBuf, 4);

      rawPacket    = txBuf;
      rawPacketLen = 5;

      // Snooze polls so the direct reply of this TC is not overshadowed
      pollSnooze = POLL_SNOOZE_CYCLES;
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

      pollSnooze = POLL_SNOOZE_CYCLES;
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

      // Important: also snooze, otherwise a following poll might „steal“ the reply
      pollSnooze = POLL_SNOOZE_CYCLES;
      return returnvalue::OK;
    }

    default:
      return COMMAND_NOT_SUPPORTED;
  }
}

void RwCommanderHandler::fillCommandAndReplyMap() {
  // Commands without different reply id
  insertInCommandMap(CMD_SET_SPEED, false, 0);
  insertInCommandMap(CMD_STOP,      false, 0);

  // 1) Polling command: map as "reply-expected" so the base will read/parse replies.
  //    There is no commander for polls; the base will not forward DATA_REPLY if none is set,
  //    but interpretDeviceReply() will still be called (for logging / datasets).
  insertInCommandAndReplyMap(
      /*deviceCommand*/       CMD_STATUS_POLL,
      /*maxDelayCycles*/      5,
      /*replyDataSet*/        &replySet,
      /*replyLen*/            8,
      /*periodic*/            false,
      /*hasDifferentReplyId*/ true,
      /*replyId*/             REPLY_STATUS,
      /*countdown*/           nullptr);

  // 2) TC STATUS: also maps to REPLY_STATUS, but this one *has* a commander.
  //    Keep this AFTER the poll mapping and use the snooze in buildCommandFromCommand()
  //    so the TC reply is not interleaved with a poll frame.
  insertInCommandAndReplyMap(
      /*deviceCommand*/       CMD_STATUS,
      /*maxDelayCycles*/      5,
      /*replyDataSet*/        &replySet,
      /*replyLen*/            8,
      /*periodic*/            false,
      /*hasDifferentReplyId*/ true,
      /*replyId*/             REPLY_STATUS,
      /*countdown*/           nullptr);

  sif::info << "RwCommanderHandler: command/reply map set up." << std::endl;
}


ReturnValue_t RwCommanderHandler::scanForReply(const uint8_t* start, size_t len,
                                               DeviceCommandId_t* foundId, size_t* foundLen) {
  // Very small parser: look for the 8-byte reply starting with 0xAB 0x10
  if (len < 8) {
    return DeviceHandlerIF::LENGTH_MISSMATCH;
  }
  if (start[0] != START_BYTE_REPLY) {
    // Not our frame start: tell base to move ahead by one byte
    return returnvalue::FAILED;
  }
  if (start[1] != REPLY_STATUS) {
    // Unknown second byte – skip one byte
    return returnvalue::FAILED;
  }

  // Verify simple CRC8 over first 7 bytes
  if (crc8(start, 7) != start[7]) {
    // Corrupt frame; skip one byte to resync
    return returnvalue::FAILED;
  }

  *foundId  = REPLY_STATUS;
  *foundLen = 8;
  return returnvalue::OK;
}

ReturnValue_t RwCommanderHandler::interpretDeviceReply(DeviceCommandId_t id,
                                                       const uint8_t* packet) {
  if (id == REPLY_STATUS) {
    // Decode for logging only (dataset transports the raw frame unchanged)
    const int16_t speed   = static_cast<int16_t>((packet[2] << 8) | packet[3]);
    const int16_t torque  = static_cast<int16_t>((packet[4] << 8) | packet[5]);
    const uint8_t running = packet[6];

    sif::info << "RW STATUS: speed=" << speed
              << " rpm, torque=" << torque
              << " mNm, running=" << int(running) << std::endl;

    // --- FSFW-official direct-reply path:
    // 1) Fill the reply dataset (we forward the raw 8-byte device frame).
    for (size_t i = 0; i < 8; ++i) {
      replySet.raw[i] = packet[i];
    }
    replySet.raw.setValid(true);

    // 2) Ask the base to send it as DATA_REPLY to the command initiator.
    //    This becomes DeviceHandlerMessage::REPLY_DIRECT_COMMAND_DATA on the receiver side.
    handleDeviceTm(replySet, REPLY_STATUS);

    return returnvalue::OK;
  }
  return returnvalue::FAILED;
}

// Give the state machine time between mode transitions so timeouts don't fire immediately.
uint32_t RwCommanderHandler::getTransitionDelayMs(Mode_t /*from*/, Mode_t /*to*/) {
  return 200;  // 200 ms; tune as needed for your setup
}

ReturnValue_t RwCommanderHandler::initializeLocalDataPool(localpool::DataPool& localDataPoolMap,
                                                          LocalDataPoolManager&) {
  // We just need one entry to back the LocalPoolVector<uint8_t,8>.
  localDataPoolMap.emplace(static_cast<lp_id_t>(PoolIds::RAW_REPLY),
                           new PoolEntry<uint8_t>(8));
  return returnvalue::OK;
}

ReturnValue_t RwCommanderHandler::executeAction(ActionId_t, MessageQueueId_t,
                                                const uint8_t*, size_t) {
  // No custom actions here
  return returnvalue::FAILED;
}

uint8_t RwCommanderHandler::crc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int j = 0; j < 8; ++j) {
      if (crc & 0x80) {
        crc = static_cast<uint8_t>((crc << 1) ^ 0x07);
      } else {
        crc = static_cast<uint8_t>(crc << 1);
      }
    }
  }
  return crc;
}
