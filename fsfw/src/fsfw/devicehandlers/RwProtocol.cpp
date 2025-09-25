#include "RwProtocol.h"

namespace RwProtocol {

// Internal helpers
namespace {
inline void be16Store(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>((v >> 8) & 0xFF);
  p[1] = static_cast<uint8_t>(v & 0xFF);
}

inline uint16_t be16Load(const uint8_t* p) {
  return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no xorout, no reflection
uint16_t crc16_impl(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; ++b) {
      if (crc & 0x8000) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// Writes CMD frame with given cmd id and 16-bit value
bool buildCmd(uint8_t* out, size_t outLen, uint8_t cmdId, int16_t value) {
  if (out == nullptr || outLen < CMD_LEN) {
    return false;
  }
  out[0] = START_CMD;
  out[1] = cmdId;
  be16Store(&out[2], static_cast<uint16_t>(value));  // two's complement as-is

  const uint16_t crc = crc16_impl(out, 4);          // CRC over bytes 0..3
  be16Store(&out[4], crc);
  return true;
}
}  // namespace

// Public CRC function
uint16_t crc16CcittFalse(const uint8_t* data, size_t len) {
  if (data == nullptr) {
    return 0;
  }
  return crc16_impl(data, len);
}

// Builders
bool buildSetSpeed(uint8_t* out, size_t outLen, int16_t rpm) {
  return buildCmd(out, outLen, CMD_SET_SPEED, rpm);
}

bool buildSetTorque(uint8_t* out, size_t outLen, int16_t mNm) {
  return buildCmd(out, outLen, CMD_SET_TORQUE, mNm);
}

bool buildStop(uint8_t* out, size_t outLen) {
  return buildCmd(out, outLen, CMD_STOP, 0);
}

bool buildStatusReq(uint8_t* out, size_t outLen) {
  return buildCmd(out, outLen, CMD_STATUS_REQ, 0);
}

// Parser
ParseResult parseStatus(const uint8_t* in, size_t len, Status& out) {
  // Basic checks
  if (in == nullptr || len != STATUS_LEN) {
    return ParseResult::LEN_ERROR;
  }
  if (in[0] != START_REPLY) {
    return ParseResult::BAD_START;
  }
  if (in[1] != RESP_STATUS) {
    return ParseResult::BAD_ID;
  }

  // CRC over bytes 0..6, compare with [7..8]
  const uint16_t crcCalc = crc16_impl(in, 7);
  const uint16_t crcWire = be16Load(&in[7]);
  if (crcCalc != crcWire) {
    return ParseResult::CRC_ERROR;
  }

  // Extract fields
  out.speedRpm   = static_cast<int16_t>(be16Load(&in[2]));
  out.torque_mNm = static_cast<int16_t>(be16Load(&in[4]));
  out.running    = in[6];

  // Basic sanity (optional, keep minimal here)
  // running is 0/1 by convention, do not hard-fail for other values
  return ParseResult::OK;
}

}  // namespace RwProtocol
