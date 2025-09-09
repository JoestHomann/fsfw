#include "RwCommanderHandler.h"

#include <cstring>
#include "fsfw/returnvalues/returnvalue.h"
#include "fsfw/serviceinterface/ServiceInterfaceStream.h"

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
  // Optional trace: proves the task calls into our handler
  sif::info << "RwCommanderHandler: tick" << std::endl;

  // Give the USB CDC device a tiny grace period after open/reset
  if (warmupCnt < warmupCycles) {
    ++warmupCnt;
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
  return returnvalue::OK;
}

ReturnValue_t RwCommanderHandler::buildTransitionDeviceCommand(DeviceCommandId_t*) {
  return returnvalue::OK;
}

ReturnValue_t RwCommanderHandler::buildCommandFromCommand(DeviceCommandId_t deviceCommand,
                                                          const uint8_t* data, size_t len) {
  switch (deviceCommand) {
    case CMD_SET_SPEED: {
      int16_t rpm = 0;
      if (len >= sizeof(int16_t)) {
        std::memcpy(&rpm, data, sizeof(int16_t));  // payload from IPC store
      } else if (len >= sizeof(float)) {
        float f;
        std::memcpy(&f, data, sizeof(float));
        rpm = static_cast<int16_t>(f);
      }
      lastTargetRpm = rpm;
      const uint16_t u = static_cast<uint16_t>(rpm);

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
      sif::warning << "RwCommanderHandler: unknown deviceCommand 0x"
                   << std::hex << deviceCommand << std::dec << std::endl;
      return returnvalue::FAILED;
  }
}

void RwCommanderHandler::fillCommandAndReplyMap() {
  // Commands we emit
  insertInCommandAndReplyMap(CMD_SET_SPEED, 5);
  insertInCommandAndReplyMap(CMD_STOP,      5);
  insertInCommandAndReplyMap(CMD_STATUS,    5);

  // Reply we expect (exactly 8 bytes)
  insertInReplyMap(REPLY_STATUS, 8);

  sif::info << "RwCommanderHandler: command/reply map set up." << std::endl;
}

ReturnValue_t RwCommanderHandler::scanForReply(const uint8_t* start, size_t len,
                                               DeviceCommandId_t* foundId, size_t* foundLen) {
                                            
  
  // ------ DEBUG BEGIN -----
  sif::info << "RW: scanForReply len=" << len;
  if (len >= 1) {
    sif::info << " first=[" << std::hex << int(start[0])
              << (len>=2 ? (' ' + std::to_string(int(start[1]))) : "")
              << (len>=3 ? (' ' + std::to_string(int(start[2]))) : "")
              << "]" << std::dec << std::endl;
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
    const int16_t speed  = static_cast<int16_t>((packet[2] << 8) | packet[3]);
    const int16_t torque = static_cast<int16_t>((packet[4] << 8) | packet[5]);
    const bool running   = (packet[6] != 0);
    sif::info << "RW STATUS: speed=" << speed << " RPM, torque=" << torque
              << " mNm, running=" << running << std::endl;
    return returnvalue::OK;
  }
  return returnvalue::FAILED;
}

// Give the state machine time between mode transitions so timeouts don't fire immediately.
uint32_t RwCommanderHandler::getTransitionDelayMs(Mode_t /*from*/, Mode_t /*to*/) {
  return 2000;  // 2 s; tune as needed for your setup
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
