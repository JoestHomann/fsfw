// fsfw/devicehandlers/RwProtocol.h
#pragma once
#include <cstddef>
#include <cstdint>

/**
 * English comments!
 * Minimal byte-oriented protocol used by the ReactionWheelsHandler and the Python simulator.
 * - Commands are fixed-length frames with CRC16-CCITT (FALSE).
 * - Replies (STATUS) are fixed-length frames with CRC16-CCITT (FALSE).
 *
 * Command frame (6 bytes):
 *   [AA, cmdId, p0, p1, crcH, crcL]
 *
 * Status reply frame (9 bytes):
 *   [AB, 0x10, spdH, spdL, torH, torL, running, crcH, crcL]
 */
namespace RwProtocol {

static constexpr uint8_t START_CMD = 0xAA;
static constexpr uint8_t START_REPLY = 0xAB;

static constexpr size_t CMD_LEN = 6;
static constexpr size_t STATUS_LEN = 9;

enum class CmdId : uint8_t {
  SET_SPEED = 0x01,
  STOP = 0x02,
  STATUS_REQ = 0x03,
  SET_TORQUE = 0x04  // NEW: torque command in mNm
};

enum class RespId : uint8_t { STATUS = 0x10 };

struct Status {
  int16_t speedRpm{0};
  int16_t torqueMnM{0};
  uint8_t running{0};
};

// --- Builders (return total frame length or 0 on error) ---
size_t buildSetSpeed(uint8_t* out, size_t cap, int16_t rpm);
size_t buildSetTorque(uint8_t* out, size_t cap, int16_t torque_mNm);  // NEW
size_t buildStop(uint8_t* out, size_t cap);
size_t buildStatusReq(uint8_t* out, size_t cap);

// --- CRC helpers ---
uint16_t calcCrc16(const uint8_t* buf, size_t len);
bool verifyCrc16(const uint8_t* buf, size_t len);

// --- Parser ---
bool parseStatus(const uint8_t* buf, size_t len, Status& out);

}  // namespace RwProtocol
