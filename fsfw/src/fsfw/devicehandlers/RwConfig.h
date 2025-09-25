#pragma once
#include <cstddef>
#include <cstdint>

// Central thresholds, limits, and timing for the RW stack.
// Keep values realistic and documented in units.

namespace RwConfig {

// --- Timeouts and polling ---
constexpr int      STATUS_TIMEOUT_CYCLES      = 5;     // cycles to wait for STATUS reply
constexpr uint32_t POLL_INHIBIT_MS            = 20;    // ms to inhibit polls after an external TC
constexpr uint16_t STATUS_POLL_DIVIDER_DEFAULT= 10;    // poll every N handler cycles
constexpr uint16_t STATUS_LOG_EVERY           = 20;    // log every N parsed STATUS frames

// --- Limits (units noted) ---
constexpr int16_t  MAX_RPM_DEFAULT            = 4000;  // rpm, signed clamp for SET_SPEED
constexpr uint16_t MAX_SLEW_RPM_S_DEFAULT     = 1000;  // rpm/s, optional slew limit

// --- FDIR thresholds ---
constexpr int16_t  STUCK_RPM_THRESH           = 5;     // rpm, abs error considered stuck
constexpr uint8_t  STUCK_DEBOUNCE_FRAMES      = 5;     // frames above threshold before event
constexpr int16_t  TORQUE_HIGH_MNM_THRESH     = 200;   // mNm, abs torque high threshold
constexpr uint8_t  TORQUE_DEBOUNCE_FRAMES     = 3;     // frames above threshold before event
constexpr uint32_t CRC_ERR_EVENT_THRESH       = 10;    // raise event every N CRC errors
constexpr uint32_t MALFORMED_EVENT_THRESH     = 10;    // raise event every N malformed frames

// --- Driver delays (if used by mode transitions) ---
constexpr uint32_t DELAY_OFF_TO_ON_MS         = 0;
constexpr uint32_t DELAY_ON_TO_NORMAL_MS      = 0;

// --- RX ring buffer ---
constexpr std::size_t RX_RING_SIZE            = 512;   // bytes in ring
// Object ID for ring buffer diagnostics (any free ID is fine in your object map)
constexpr uint32_t   RX_RING_OBJ_ID           = 0x7357A201;

} // namespace RwConfig
