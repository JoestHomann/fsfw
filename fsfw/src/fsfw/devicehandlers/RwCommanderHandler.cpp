#include "RwCommanderHandler.h"

#include <cstring>
#include "fsfw/src/fsfw/returnvalues/returnvalue.h"
#include "fsfw/src/fsfw/serviceinterface/ServiceInterfaceStream.h"

RwCommanderHandler::RwCommanderHandler(object_id_t objectId, object_id_t comIF, CookieIF* cookie)
    : DeviceHandlerBase(objectId, comIF, cookie) {}

void RwCommanderHandler::doStartUp() {
  // Keep it simple: go to NORMAL so buildNormalDeviceCommand() runs.
  sif::info << "RwCommanderHandler: doStartUp()" << std::endl;
  setMode(MODE_NORMAL);
}

void RwCommanderHandler::doShutDown() {
  sif::info << "RwCommanderHandler: doShutDown()" << std::endl;
  setMode(MODE_OFF);
}

ReturnValue_t RwCommanderHandler::buildNormalDeviceCommand(DeviceCommandId_t* id) {
  // Periodically poll STATUS so we see telemetry without external TC
  if (++statusPollCnt >= statusPollDivider) {
    statusPollCnt = 0;

    txBuf[0] = START_BYTE_CMD;
    txBuf[1] = static_cast<uint8_t>(CMD_STATUS);
    txBuf[2] = 0x00;
    txBuf[3] = 0x00;
    txBuf[4] = crc8(txBuf, 4);

    rawPacket    = txBuf;
    rawPacketLen = 5;
    *id = CMD_STATUS;
// === DEBUG: dump the exact 5 bytes we send ===
#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::info << "RwCommanderHandler: sending raw ["
              << std::hex
              << int(rawPacket[0]) << " " << int(rawPacket[1]) << " "
              << int(rawPacket[2]) << " " << int(rawPacket[3]) << " "
              << int(rawPacket[4]) << std::dec << "]" << std::endl;
#else
    sif::printInfo("RwCommanderHandler: sending raw [%02X %02X %02X %02X %02X]\n",
                   rawPacket[0], rawPacket[1], rawPacket[2], rawPacket[3], rawPacket[4]);
#endif
// =============================================
    sif::info << "RwCommanderHandler: periodic STATUS" << std::endl;
    return returnvalue::OK;
  }

  // Nothing to send this cycle.
  return returnvalue::OK;
}

ReturnValue_t RwCommanderHandler::buildTransitionDeviceCommand(DeviceCommandId_t* id) {
  // No special transition traffic required for this simple handler
  return returnvalue::OK;
}

ReturnValue_t RwCommanderHandler::buildCommandFromCommand(DeviceCommandId_t deviceCommand,
                                                          const uint8_t* data, size_t len) {
  // Map framework commands to raw protocol frames
  switch (deviceCommand) {
    case CMD_SET_SPEED: {
      // Expect a 16-bit signed RPM or a float
      int16_t rpm = 0;
      if (len >= sizeof(int16_t)) {
        // default: first 2 bytes as big-endian signed (host sends in native? choose one)
        // Here we accept a host-provided int16_t in native endianness:
        std::memcpy(&rpm, data, sizeof(int16_t));
      } else if (len >= sizeof(float)) {
        float f;
        std::memcpy(&f, data, sizeof(float));
        rpm = static_cast<int16_t>(f);
      }

      lastTargetRpm = rpm;
      uint16_t u = static_cast<uint16_t>(rpm);

      txBuf[0] = START_BYTE_CMD;
      txBuf[1] = static_cast<uint8_t>(CMD_SET_SPEED);
      txBuf[2] = static_cast<uint8_t>((u >> 8) & 0xFF);
      txBuf[3] = static_cast<uint8_t>(u & 0xFF);
      txBuf[4] = crc8(txBuf, 4);

      rawPacket    = txBuf;
      rawPacketLen = 5;
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
      sif::info << "RwCommanderHandler: STATUS" << std::endl;
      return returnvalue::OK;
    }
    default:
      sif::warning << "RwCommanderHandler: unknown deviceCommand 0x" << std::hex
                   << deviceCommand << std::dec << std::endl;
      return returnvalue::FAILED;
  }
}

void RwCommanderHandler::fillCommandAndReplyMap() {
  // Commands we can send
  insertInCommandAndReplyMap(CMD_SET_SPEED, 5);
  insertInCommandAndReplyMap(CMD_STOP, 3);
  insertInCommandAndReplyMap(CMD_STATUS, 2);

  // Replies we expect
  insertInReplyMap(REPLY_STATUS, 2);

  sif::info << "RwCommanderHandler: command/reply map set up." << std::endl;
}

ReturnValue_t RwCommanderHandler::scanForReply(const uint8_t* start, size_t len,
                                               DeviceCommandId_t* foundId, size_t* foundLen) {
  // Look for a single 8-byte STATUS frame: AB 10 .. .. .. .. .. CRC
  if (len < 8) {
    return returnvalue::FAILED;
  }
  if (start[0] != START_BYTE_REPLY) {
    return returnvalue::FAILED;
  }
  if (crc8(start, 7) != start[7]) {
    return returnvalue::FAILED;
  }
  if (start[1] == REPLY_STATUS) {
    *foundId  = REPLY_STATUS;
    *foundLen = 8;
    return returnvalue::OK;
  }
  return returnvalue::FAILED;
}

ReturnValue_t RwCommanderHandler::interpretDeviceReply(DeviceCommandId_t id,
                                                       const uint8_t* packet) {
  if (id == REPLY_STATUS) {
    int16_t speed  = static_cast<int16_t>((packet[2] << 8) | packet[3]);
    int16_t torque = static_cast<int16_t>((packet[4] << 8) | packet[5]);
    bool running   = (packet[6] != 0);
    sif::info << "RW STATUS: speed=" << speed << " RPM, torque=" << torque
              << " mNm, running=" << running << std::endl;
    return returnvalue::OK;
  }
  return returnvalue::FAILED;
}

uint32_t RwCommanderHandler::getTransitionDelayMs(Mode_t, Mode_t) { return 0; }

ReturnValue_t RwCommanderHandler::initializeLocalDataPool(localpool::DataPool&,
                                                          LocalDataPoolManager&) {
  // No local datapool variables here (yet)
  return returnvalue::OK;
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
