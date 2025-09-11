#include "RwCommanderHandler.h"

#include <cstring>
#include <cstdint>
#include "fsfw/returnvalues/returnvalue.h"
#include "fsfw/serviceinterface/ServiceInterfaceStream.h"
#include "fsfw/devicehandlers/DeviceHandlerIF.h"


RwCommanderHandler::RwCommanderHandler(object_id_t objectId, object_id_t comIF, CookieIF* cookie)
    : DeviceHandlerBase(objectId, comIF, cookie) {}

void RwCommanderHandler::doStartUp() {
  if (warmupCnt < warmupCycles) { ++warmupCnt; return; }
  // Enter ON; only later switch to NORMAL when explicitly commanded
  setMode(MODE_ON);
}

void RwCommanderHandler::doShutDown() {
  sif::info << "RwCommanderHandler: doShutDown()" << std::endl;
  setMode(MODE_OFF);
}

ReturnValue_t RwCommanderHandler::performOperation(uint8_t opCode) {
  //sif::info << "RwCommanderHandler: performOperation opcode=" << int(opCode) << std::endl; // proves PST tick
  return DeviceHandlerBase::performOperation(opCode);
}

void RwCommanderHandler::modeChanged() {
  Mode_t m{};
  Submode_t s{};
  this->getMode(&m, &s);  // FSFW: pointer-based getter
  sif::info << "RwCommanderHandler: modeChanged -> " << static_cast<int>(m)
            << " (sub=" << static_cast<int>(s) << ")" << std::endl;
}

ReturnValue_t RwCommanderHandler::buildNormalDeviceCommand(DeviceCommandId_t* id) {
  // DEBUG: proves the task calls into our handler
  //sif::info << "RwCommanderHandler: tick" << std::endl;

  // Give the USB CDC device a tiny grace period after open/reset
  if (warmupCnt < warmupCycles) {
    ++warmupCnt;
    *id = DeviceHandlerIF::NO_COMMAND_ID;
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
    *id          = CMD_STATUS;

#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::info << "RwCommanderHandler: sending raw ["
              << std::hex
              << int(rawPacket[0]) << " " << int(rawPacket[1]) << " "
              << int(rawPacket[2]) << " " << int(rawPacket[3]) << " "
              << int(rawPacket[4]) << std::dec << "]" << std::endl;
#endif
    sif::info << "RwCommanderHandler: periodic STATUS" << std::endl;
    return returnvalue::OK;
  }

  // Nothing to send this cycle
  *id = DeviceHandlerIF::NO_COMMAND_ID;
  return returnvalue::OK;
}

ReturnValue_t RwCommanderHandler::buildTransitionDeviceCommand(DeviceCommandId_t* id) {
  *id = DeviceHandlerIF::NO_COMMAND_ID;
  return returnvalue::OK;
}


ReturnValue_t RwCommanderHandler::buildCommandFromCommand(DeviceCommandId_t deviceCommand,
                                                          const uint8_t* data, size_t len) {
  switch (deviceCommand) {
    case CMD_SET_SPEED: {
      // Expected PUS-8 parameter layouts:
      //  (A) 2 bytes: int16_t RPM, BIG-ENDIAN (preferred)
      //  (B) 4 bytes: float RPM, IEEE-754 (fallback)
      if (data == nullptr) {
        sif::warning << "RwCommanderHandler: SET_SPEED with null data" << std::endl;
        return DeviceHandlerIF::INVALID_NUMBER_OR_LENGTH_OF_PARAMETERS;
      }

      int16_t rpm = 0;
      if (len == 2) {
        // Parse big-endian signed 16-bit RPM
        rpm = static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) |
                                   static_cast<uint16_t>(data[1]));
      } else if (len == 4) {
        // Fallback: 4-byte float RPM (assumes IEEE-754 on both ends)
        float f = 0.0f;
        std::memcpy(&f, data, sizeof(float));
        // Clamp to int16_t range before cast to avoid UB on overflow
        if (f > static_cast<float>(INT16_MAX)) f = static_cast<float>(INT16_MAX);
        if (f < static_cast<float>(INT16_MIN)) f = static_cast<float>(INT16_MIN);
        rpm = static_cast<int16_t>(f);
      } else {
        sif::warning << "RwCommanderHandler: SET_SPEED invalid len=" << len
                     << " (expected 2 or 4 bytes)" << std::endl;
        return DeviceHandlerIF::INVALID_NUMBER_OR_LENGTH_OF_PARAMETERS;
      }

      lastTargetRpm = rpm;

      // Build device packet (two's complement is preserved by uint16_t cast)
      const uint16_t u = static_cast<uint16_t>(rpm);
      txBuf[0] = START_BYTE_CMD;
      txBuf[1] = static_cast<uint8_t>(CMD_SET_SPEED);
      txBuf[2] = static_cast<uint8_t>((u >> 8) & 0xFF);
      txBuf[3] = static_cast<uint8_t>(u & 0xFF);
      txBuf[4] = crc8(txBuf, 4);  // CRC over first 4 bytes

      rawPacket    = txBuf;
      rawPacketLen = 5;

      sif::info << "RwCommanderHandler: SET_SPEED " << rpm << " RPM" << std::endl;
      return returnvalue::OK;
    }

    case CMD_STOP: {
      // No parameters expected; ignore any stray bytes but log once if present.
      if (len != 0) {
        sif::debug << "RwCommanderHandler: STOP with nonzero len=" << len
                   << " (ignored)" << std::endl;
      }

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
      // No parameters expected; ignore any stray bytes but log once if present.
      if (len != 0) {
        sif::debug << "RwCommanderHandler: STATUS with nonzero len=" << len
                   << " (ignored)" << std::endl;
      }

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

    default: {
      sif::warning << "RwCommanderHandler: unknown deviceCommand 0x"
                   << std::hex << deviceCommand << std::dec << std::endl;
      return DeviceHandlerIF::INVALID_COMMAND_PARAMETER;
    }
  }
}


void RwCommanderHandler::fillCommandAndReplyMap() {
  // Map action IDs (equal to device command IDs) that can be called via PUS-8
  insertInCommandMap(CMD_SET_SPEED, false, 0); // immediate fire-and-forget
  insertInCommandMap(CMD_STOP,      false, 0);

  // If you want status as a polled reply with a specific reply ID:
  insertInCommandAndReplyMap(
      CMD_STATUS,           // deviceCommand / actionId
      5,                    // max delay cycles
      nullptr,              // no HK dataset
      8,                    // expected reply length
      false,                // not periodic
      true,                 // has different reply ID
      REPLY_STATUS,         // the reply id used in interpretDeviceReply()
      nullptr);
}

ReturnValue_t RwCommanderHandler::scanForReply(const uint8_t* start, size_t len,
                                               DeviceCommandId_t* foundId, size_t* foundLen) {
                                            
  
  // ------ DEBUG BEGIN -----
  sif::info << "RW: scanForReply len=" << len;
if (len >= 1) {
  sif::info << " first=[" << std::hex << int(start[0]);
  if (len >= 2) { sif::info << " " << int(start[1]); }
  if (len >= 3) { sif::info << " " << int(start[2]); }
  sif::info << "]" << std::dec << std::endl;
} else {
  sif::info << std::endl;
}
  // ------ DEBUG END -----

  if (len < 8) return returnvalue::FAILED;
  if (start[0] != START_BYTE_REPLY) return returnvalue::FAILED;
  if (crc8(start, 7) != start[7])   return returnvalue::FAILED;

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
    const uint16_t sp_u = static_cast<uint16_t>((uint16_t(packet[2]) << 8) | uint16_t(packet[3]));
    const uint16_t tq_u = static_cast<uint16_t>((uint16_t(packet[4]) << 8) | uint16_t(packet[5]));
    const int16_t  speed  = static_cast<int16_t>(sp_u);
    const int16_t  torque = static_cast<int16_t>(tq_u);
    const uint8_t running = packet[6];

    sif::info << "RW STATUS: speed=" << speed
              << " RPM, torque=" << torque
              << " mNm, running=" << int(running) << std::endl;

    
    return returnvalue::OK;
  }
  return returnvalue::FAILED;
}

// Give the state machine time between mode transitions so timeouts don't fire immediately.
uint32_t RwCommanderHandler::getTransitionDelayMs(Mode_t /*from*/, Mode_t /*to*/) {
  return 200;  // 200 ms; tune as needed for your setup
}

ReturnValue_t RwCommanderHandler::initializeLocalDataPool(localpool::DataPool&, LocalDataPoolManager&) {
  return returnvalue::OK;
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
