#pragma once

#include <cstddef>
#include <cstdint>

namespace RwProtocol {
  // Wire constants
  static constexpr uint8_t START_CMD   = 0xAA;
  static constexpr uint8_t START_REPLY = 0xAB;

  // Command / Response IDs
  enum class CmdId  : uint8_t { SET_SPEED = 0x01, STOP = 0x02, STATUS_REQ = 0x03 };
  enum class RespId : uint8_t { STATUS    = 0x10 };

  // Fixed frame sizes (bytes, including CRC-16)
  static constexpr size_t CMD_LEN    = 6;  // AA, ID, P1, P2, CRC_H, CRC_L
  static constexpr size_t STATUS_LEN = 9;  // AB, 10, spdH, spdL, torH, torL, run, crcH, crcL

  struct Status {
    int16_t  speedRpm{0};
    int16_t  torqueMnM{0};
    uint8_t  running{0};
  };

  [[nodiscard]] size_t buildSetSpeed(uint8_t* out, size_t cap, int16_t rpm);
  [[nodiscard]] size_t buildStop(uint8_t* out, size_t cap);
  [[nodiscard]] size_t buildStatusReq(uint8_t* out, size_t cap);

  [[nodiscard]] bool verifyCrc16(const uint8_t* frame, size_t totalLen);
  [[nodiscard]] bool parseStatus(const uint8_t* buf, size_t len, Status& out);
}
