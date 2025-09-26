#pragma once

#include <cstddef>
#include <cstdint>

/*
 * RwProtocol.h - Implementation of Reaction Wheel serial protocol
 *
 * Outsourced from ReactionWheelsHandler to allow reuse in other ReactionWheelHandlers
 * and in the RwPusService.
 * 
 * Implements the protocal as follows:
 * 
 * Command frame (6 bytes):
 *  [AA, cmdId, p0, p1, crcH, crcL]
 *
 * Status reply frame (9 bytes):
 *   [AB, 0x10, spdH, spdL, torH, torL, running, crcH, crcL]
 *
 *  - Joest Homann
 */

namespace RwProtocol {

  // Start bytes
  static constexpr uint8_t START_CMD   = 0xAA;
  static constexpr uint8_t START_REPLY = 0xAB;

  // Message lengths
  static constexpr size_t CMD_LEN    = 6;
  static constexpr size_t STATUS_LEN = 9;

  // Command IDs (second byte in command frame)
  enum class CmdId : uint8_t {
    SET_SPEED   = 0x01, // payload p0..p1: int16 rpm (big-endian)
    STOP        = 0x02, // payload p0..p1: 0x0000
    STATUS      = 0x03, // payload p0..p1: 0x0000
    SET_TORQUE  = 0x04  // payload p0..p1: int16 torque_mNm (big-endian)
  };

  // Reply IDs (second byte in reply frame)
  enum class RespId : uint8_t {
    STATUS = 0x10 // status reply frame
  };

  // Parsed STATUS fields (host endian)
  struct Status {
    int16_t speedRpm{0};  // signed RPM
    int16_t torqueMnM{0}; // signed torque in mNm
    uint8_t running{0};   // 1 if running, 0 otherwise
  };

  // --- Builders (return total frame length, or 0 on error) ------------------
  size_t buildSetSpeed(uint8_t* out, size_t cap, int16_t rpm);  // Build SET_SPEED command
  size_t buildSetTorque(uint8_t* out, size_t cap, int16_t torque_mNm);  // Build SET_TORQUE command
  size_t buildStop(uint8_t* out, size_t cap); // Build STOP command
  size_t buildStatus(uint8_t* out, size_t cap);  // Build STATUS request command

  // --- CRC helpers ----------------------------------------------------------
  uint16_t calcCrc16(const uint8_t* buf, size_t len); // Calculate CRC16-CCITT (FALSE)
  bool verifyCrc16(const uint8_t* buf, size_t len); // Verify CRC16-CITT (FALSE)

  // --- Parser ---------------------------------------------------------------
  bool parseStatus(const uint8_t* buf, size_t len, Status& out);  // Parse STATUS reply frame into Status struct.

} // namespace RwProtocol
