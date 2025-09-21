#pragma once
#include <cstddef>
#include <cstdint>

namespace RwProtocol {

// --- Wire constants (device-facing) ---
inline constexpr uint8_t START_CMD   = 0xAA;
inline constexpr uint8_t START_REPLY = 0xAB;

enum class CmdId  : uint8_t { SET_SPEED = 0x01, STOP = 0x02, STATUS_REQ = 0x03 };
enum class RespId : uint8_t { STATUS    = 0x10 };

// --- Fixed frame sizes (keep protocol-level magic numbers in one place) ---
inline constexpr size_t CMD_LEN    = 6;  // [AA, ID, p2, p1, CRC_H, CRC_L]
inline constexpr size_t STATUS_LEN = 9;  // [AB, 0x10, spdH, spdL, torH, torL, run, CRC_H, CRC_L]

// --- Build command frames (Host -> Device) ---
// Big-endian fields; CRC-16/CCITT-FALSE appended big-endian.
// Return: total bytes (payload + 2 CRC) or 0 on error.
size_t buildSetSpeed(uint8_t* out, size_t cap, int16_t rpm);
size_t buildStop(uint8_t* out, size_t cap);
size_t buildStatusReq(uint8_t* out, size_t cap);

// --- Verify full-frame CRC (totalLen includes trailing 2 CRC bytes) ---
bool verifyCrc16(const uint8_t* buf, size_t totalLen);

// --- Optional STATUS parser ---
// Layout: [AB, 0x10, spdH, spdL, torH, torL, running, crcH, crcL]
struct Status {
  int16_t speedRpm{0};
  int16_t torqueMnM{0};
  uint8_t running{0};
};

// Returns true if layout + CRC are valid and fields were parsed.
bool parseStatus(const uint8_t* buf, size_t len, Status& out);

}  // namespace RwProtocol
