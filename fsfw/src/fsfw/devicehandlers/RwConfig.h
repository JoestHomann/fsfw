#pragma once
#include <cstdint>

namespace RwConfig {
// ----- Subsystem ID ----------------------------------------------------------
constexpr uint8_t  RW_SUBSYSTEM_ID     = 0x44;     // for events (RwEvents)

// ----- Mode-Transition Delays ------------------------------------------------
constexpr uint32_t DELAY_OFF_TO_ON_MS    = 100;
constexpr uint32_t DELAY_ON_TO_NORMAL_MS = 100;

// ----- Polling / Timeouts ----------------------------------------------------
constexpr uint32_t STATUS_POLL_DIVIDER_DEFAULT = 100; // PST cycles between periodic STATUS polls
constexpr uint8_t  STATUS_TIMEOUT_CYCLES       = 6;   // cycles until timeout after a poll

// ----- RX Ring Buffer --------------------------------------------------------
constexpr std::size_t RX_RING_SIZE   = 256;
constexpr uint32_t    RX_RING_OBJ_ID = 0xDEADB011; // arbitrary, needs to be unique only

// ----- Reaction Wheel Limits -------------------------------------------------
constexpr int16_t  MAX_RPM_DEFAULT        = 4000; // clamp SET_SPEED
constexpr uint16_t MAX_SLEW_RPM_S_DEFAULT = 0;    // 0 = disabled

// ----- FDIR Thresholds -------------------------------------------------------
constexpr int16_t STUCK_RPM_THRESH       = 50;   // running == 0 but |rpm| > threshold
constexpr uint8_t STUCK_DEBOUNCE_FRAMES  = 3;
constexpr int16_t TORQUE_HIGH_MNM_THRESH = 600;
constexpr uint8_t TORQUE_DEBOUNCE_FRAMES = 5;

// ----- Event Backoff (emit event every N errors) -----------------------------
constexpr uint32_t CRC_ERR_EVENT_THRESH    = 10;
constexpr uint32_t MALFORMED_EVENT_THRESH  = 10;

// ----- Housekeeping Push (PUS Service 3) ------------------------------------
// Default period for periodic HK packets (in seconds).
constexpr float HK_PERIOD_S = 0.5f; // HK_interval = PST_Period * nonDiagIntervalFactor * HK_PERIOD_S

} // namespace RwConfig
