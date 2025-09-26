#include "RwProtocol.h"
#include "fsfw/globalfunctions/CRC.h"

/*
 * RwProtocol.cpp - Implementation of Reaction Wheel serial protocol

 * Outsourced from ReactionWheelsHandler to allow reuse in other ReactionWheelHandlers
 * and in the RwPusService. Uses CRC implementation from fsfw/globalfunctions/CRC.h
 *
 *  - Joest Homann
 */

namespace {

// Store 16-bit value in big-endian into two byte pointers
static inline void be_store16(uint8_t* ph, uint8_t* pl, uint16_t v) {
  *ph = static_cast<uint8_t>((v >> 8) & 0xFF);
  *pl = static_cast<uint8_t>(v & 0xFF);
}

}  // namespace

namespace RwProtocol {

// Calculate CRC-16 for a given buffer (no trailing CRC in len) using project CRC
uint16_t calcCrc16(const uint8_t* buf, size_t len) {
  if (buf == nullptr) return 0;
  return CRC::crc16ccitt(buf, static_cast<uint32_t>(len), 0xFFFF);
}

// Verify buffer ending with 2-byte big-endian CRC
// Returns true if last two bytes match calcCrc16(buf, len-2)
bool verifyCrc16(const uint8_t* buf, size_t len) {
  if (buf == nullptr || len < 2) return false;
  const uint16_t exp = static_cast<uint16_t>(buf[len - 2] << 8) | buf[len - 1];
  const uint16_t act = calcCrc16(buf, len - 2);
  return exp == act;
}

// Build "SET_SPEED" command frame
// Layout (6 bytes): [START_CMD, id, rpmH, rpmL, crcH, crcL]
size_t buildSetSpeed(uint8_t* out, size_t cap, int16_t rpm) {
  if (out == nullptr || cap < CMD_LEN) return 0;
  out[0] = START_CMD;
  out[1] = static_cast<uint8_t>(CmdId::SET_SPEED);
  out[2] = static_cast<uint8_t>((rpm >> 8) & 0xFF);
  out[3] = static_cast<uint8_t>( rpm       & 0xFF);
  const uint16_t crc = calcCrc16(out, CMD_LEN - 2);
  be_store16(&out[4], &out[5], crc);
  return CMD_LEN;
}

// Build "SET_TORQUE" command frame 
// Layout (6 bytes): [START_CMD, id, tqH, tqL, crcH, crcL]
// torque_mNm is signed mNm (int16)
size_t buildSetTorque(uint8_t* out, size_t cap, int16_t torque_mNm) {
  if (out == nullptr || cap < CMD_LEN) return 0;
  out[0] = START_CMD;
  out[1] = static_cast<uint8_t>(CmdId::SET_TORQUE);
  out[2] = static_cast<uint8_t>((torque_mNm >> 8) & 0xFF);
  out[3] = static_cast<uint8_t>( torque_mNm       & 0xFF);
  const uint16_t crc = calcCrc16(out, CMD_LEN - 2);
  be_store16(&out[4], &out[5], crc);
  return CMD_LEN;
}

// Build "STOP" command frame (no payload)
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

// Build "STATUS" command frame (no payload)
size_t buildStatus(uint8_t* out, size_t cap) {
  if (out == nullptr || cap < CMD_LEN) return 0;
  out[0] = START_CMD;
  out[1] = static_cast<uint8_t>(CmdId::STATUS);
  out[2] = 0x00;
  out[3] = 0x00;
  const uint16_t crc = calcCrc16(out, CMD_LEN - 2);
  be_store16(&out[4], &out[5], crc);
  return CMD_LEN;
}

// Parse STATUS reply frame
// Expected layout (9 bytes): [START_REPLY, RespId::STATUS, spdH, spdL, torH, torL, running, crcH, crcL]
// Returns true on successful CRC and field extraction
bool parseStatus(const uint8_t* buf, size_t len, Status& out) {
  if (buf == nullptr || len < STATUS_LEN) return false;
  if (buf[0] != START_REPLY) return false;
  if (buf[1] != static_cast<uint8_t>(RespId::STATUS)) return false;
  if (!verifyCrc16(buf, STATUS_LEN)) return false;

  out.speedRpm  = static_cast<int16_t>((buf[2] << 8) | buf[3]);
  out.torqueMnM = static_cast<int16_t>((buf[4] << 8) | buf[5]);
  out.running   = buf[6];
  return true;
}

}  // namespace RwProtocol
