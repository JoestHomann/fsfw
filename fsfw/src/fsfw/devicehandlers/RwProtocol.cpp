#include "fsfw/devicehandlers/RwProtocol.h"

#include <fsfw/globalfunctions/CRC.h>  // Use FSFW's CRC-16/CCITT implementation

namespace {
inline uint16_t be16(const uint8_t* p) {
  return (static_cast<uint16_t>(p[0]) << 8) | static_cast<uint16_t>(p[1]);
}
}  // namespace

size_t RwProtocol::buildSetSpeed(uint8_t* out, size_t cap, int16_t rpm) {
  if (out == nullptr || cap < CMD_LEN) {
    return 0;
  }
  // Frame: AA <cmd=01> <rpmH> <rpmL> <crcH> <crcL>
  out[0] = START_CMD;
  out[1] = static_cast<uint8_t>(CmdId::SET_SPEED);
  out[2] = static_cast<uint8_t>((rpm >> 8) & 0xFF);
  out[3] = static_cast<uint8_t>( rpm       & 0xFF);
  const uint16_t crc = CRC::crc16ccitt(out, 4);
  out[4] = static_cast<uint8_t>((crc >> 8) & 0xFF);
  out[5] = static_cast<uint8_t>( crc       & 0xFF);
  return CMD_LEN;
}

size_t RwProtocol::buildStop(uint8_t* out, size_t cap) {
  if (out == nullptr || cap < CMD_LEN) {
    return 0;
  }
  // Frame: AA <cmd=02> 00 00 <crcH> <crcL>
  out[0] = START_CMD;
  out[1] = static_cast<uint8_t>(CmdId::STOP);
  out[2] = 0x00;
  out[3] = 0x00;
  const uint16_t crc = CRC::crc16ccitt(out, 4);
  out[4] = static_cast<uint8_t>((crc >> 8) & 0xFF);
  out[5] = static_cast<uint8_t>( crc       & 0xFF);
  return CMD_LEN;
}

size_t RwProtocol::buildStatusReq(uint8_t* out, size_t cap) {
  if (out == nullptr || cap < CMD_LEN) {
    return 0;
  }
  // Frame: AA <cmd=03> 00 00 <crcH> <crcL>
  out[0] = START_CMD;
  out[1] = static_cast<uint8_t>(CmdId::STATUS_REQ);
  out[2] = 0x00;
  out[3] = 0x00;
  const uint16_t crc = CRC::crc16ccitt(out, 4);
  out[4] = static_cast<uint8_t>((crc >> 8) & 0xFF);
  out[5] = static_cast<uint8_t>( crc       & 0xFF);
  return CMD_LEN;
}

bool RwProtocol::verifyCrc16(const uint8_t* buf, size_t totalLen) {
  if (buf == nullptr || totalLen < 2) {
    return false;
  }
  const size_t dataLen = totalLen - 2;
  const uint16_t rx   = be16(&buf[dataLen]);
  const uint16_t calc = CRC::crc16ccitt(buf, dataLen);
  return rx == calc;
}

bool RwProtocol::parseStatus(const uint8_t* buf, size_t len, Status& out) {
  // Expected STATUS frame: 9 bytes total
  if (buf == nullptr || len < STATUS_LEN) {
    return false;
  }
  if (buf[0] != START_REPLY || buf[1] != static_cast<uint8_t>(RespId::STATUS)) {
    return false;
  }
  if (!verifyCrc16(buf, STATUS_LEN)) {
    return false;
  }

  out.speedRpm  = static_cast<int16_t>((buf[2] << 8) | buf[3]);
  out.torqueMnM = static_cast<int16_t>((buf[4] << 8) | buf[5]);
  out.running   = buf[6];
  return true;
}
