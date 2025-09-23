// fsfw/devicehandlers/RwProtocol.cpp
#include "RwProtocol.h"

namespace {

static uint16_t crc16ccitt_false(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t(data[i]) << 8);
    for (int b = 0; b < 8; ++b) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc = (crc << 1);
      }
    }
  }
  return crc;
}

static inline void be_store16(uint8_t* ph, uint8_t* pl, uint16_t v) {
  *ph = static_cast<uint8_t>((v >> 8) & 0xFF);
  *pl = static_cast<uint8_t>(v & 0xFF);
}

}  // namespace

namespace RwProtocol {

uint16_t calcCrc16(const uint8_t* buf, size_t len) { return crc16ccitt_false(buf, len); }

bool verifyCrc16(const uint8_t* buf, size_t len) {
  if (buf == nullptr || len < 2) return false;
  const uint16_t exp = (uint16_t(buf[len - 2]) << 8) | buf[len - 1];
  const uint16_t act = calcCrc16(buf, len - 2);
  return exp == act;
}

size_t buildSetSpeed(uint8_t* out, size_t cap, int16_t rpm) {
  if (out == nullptr || cap < CMD_LEN) return 0;
  out[0] = START_CMD;
  out[1] = static_cast<uint8_t>(CmdId::SET_SPEED);
  out[2] = static_cast<uint8_t>((rpm >> 8) & 0xFF);
  out[3] = static_cast<uint8_t>(rpm & 0xFF);
  const uint16_t crc = calcCrc16(out, CMD_LEN - 2);
  be_store16(&out[4], &out[5], crc);
  return CMD_LEN;
}

size_t buildSetTorque(uint8_t* out, size_t cap, int16_t torque_mNm) {
  if (out == nullptr || cap < CMD_LEN) return 0;
  out[0] = START_CMD;
  out[1] = static_cast<uint8_t>(CmdId::SET_TORQUE);
  out[2] = static_cast<uint8_t>((torque_mNm >> 8) & 0xFF);
  out[3] = static_cast<uint8_t>(torque_mNm & 0xFF);
  const uint16_t crc = calcCrc16(out, CMD_LEN - 2);
  be_store16(&out[4], &out[5], crc);
  return CMD_LEN;
}

size_t buildStop(uint8_t* out, size_t cap) {
  if (out == nullptr || cap < CMD_LEN) return 0;
  out[0] = START_CMD;
  out[1] = static_cast<uint8_t>(CmdId::STOP);
  out[2] = 0x00;
  out[3] = 0x00;
  const uint16_t crc = calcCrc16(out, CMD_LEN - 2);
  be_store16(&out[4], &out[5], crc);
  return CMD_LEN;
}

size_t buildStatusReq(uint8_t* out, size_t cap) {
  if (out == nullptr || cap < CMD_LEN) return 0;
  out[0] = START_CMD;
  out[1] = static_cast<uint8_t>(CmdId::STATUS_REQ);
  out[2] = 0x00;
  out[3] = 0x00;
  const uint16_t crc = calcCrc16(out, CMD_LEN - 2);
  be_store16(&out[4], &out[5], crc);
  return CMD_LEN;
}

bool parseStatus(const uint8_t* buf, size_t len, Status& out) {
  if (buf == nullptr || len < STATUS_LEN) return false;
  if (buf[0] != START_REPLY || buf[1] != static_cast<uint8_t>(RespId::STATUS)) return false;
  if (!verifyCrc16(buf, STATUS_LEN)) return false;
  out.speedRpm = static_cast<int16_t>((buf[2] << 8) | buf[3]);
  out.torqueMnM = static_cast<int16_t>((buf[4] << 8) | buf[5]);
  out.running = buf[6];
  return true;
}

}  // namespace RwProtocol
