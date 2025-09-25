#ifndef RW_PROTOCOL_H_
#define RW_PROTOCOL_H_

#include <cstdint>
#include <cstddef>

namespace RwProtocol {

// Wire constants
constexpr uint8_t START_CMD   = 0xAA;  // Host->RW
constexpr uint8_t START_REPLY = 0xAB;  // RW->Host

// Command IDs on wire
constexpr uint8_t CMD_SET_SPEED  = 0x01;  // payload: int16 rpm
constexpr uint8_t CMD_STOP       = 0x02;  // payload: none (value=0)
constexpr uint8_t CMD_STATUS_REQ = 0x03;  // payload: none (value=0)
constexpr uint8_t CMD_SET_TORQUE = 0x04;  // payload: int16 mNm

// Reply IDs on wire
constexpr uint8_t RESP_STATUS = 0x10;

// Fixed frame sizes
constexpr size_t CMD_LEN    = 6;  // [0]=0xAA [1]=cmd [2..3]=val [4..5]=crc16
constexpr size_t STATUS_LEN = 9;  // [0]=0xAB [1]=0x10 [2..3]=rpm [4..5]=mNm [6]=run [7..8]=crc16

static_assert(CMD_LEN == 6, "CMD frame size must be 6 bytes");
static_assert(STATUS_LEN == 9, "STATUS frame size must be 9 bytes");

// Parser result codes
enum class ParseResult : uint8_t {
  OK = 0,
  LEN_ERROR,
  BAD_START,
  BAD_ID,
  CRC_ERROR,
  MALFORMED
};

// Decoded status struct
struct Status {
  int16_t speedRpm = 0;   // signed rpm
  int16_t torque_mNm = 0; // signed mNm
  uint8_t running = 0;    // 0/1
};

// Build helpers (return true if out buffer filled with CMD_LEN bytes)
bool buildSetSpeed(uint8_t* out, size_t outLen, int16_t rpm);
bool buildSetTorque(uint8_t* out, size_t outLen, int16_t mNm);
bool buildStop(uint8_t* out, size_t outLen);
bool buildStatusReq(uint8_t* out, size_t outLen);

// Parse status reply (expects STATUS_LEN bytes)
ParseResult parseStatus(const uint8_t* in, size_t len, Status& out);

// CRC utility (exposed for tests/diagnostics)
uint16_t crc16CcittFalse(const uint8_t* data, size_t len);

}  // namespace RwProtocol

#endif  // RW_PROTOCOL_H_
