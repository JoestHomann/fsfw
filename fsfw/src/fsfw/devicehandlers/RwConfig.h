#pragma once
#include <cstdint>

namespace RwConfig {

// ----- Subsystem ID (for events) -------------------------------------------
constexpr uint8_t RW_SUBSYSTEM_ID = 0x44;

// ----- Mode-Transition Delays ----------------------------------------------
constexpr uint32_t DELAY_OFF_TO_ON_MS = 100;
constexpr uint32_t DELAY_ON_TO_NORMAL_MS = 100;

// ----- Polling / Timeouts ---------------------------------------------------
constexpr uint32_t STATUS_POLL_DIVIDER_DEFAULT = 2;  // PST cycles between periodic STATUS polls
constexpr uint8_t STATUS_TIMEOUT_CYCLES = 4;         // cycles until timeout after a poll
constexpr uint32_t STATUS_LOG_EVERY = 12;            // log every nth STATUS frame

// ----- RX Ring Buffer -------------------------------------------------------
constexpr std::size_t RX_RING_SIZE = 256;
constexpr uint32_t RX_RING_OBJ_ID = 0xDEADB011;  // arbitrary, only needs to be unique

// ----- Reaction Wheel Limits ------------------------------------------------
constexpr int16_t MAX_RPM_DEFAULT = 4000;       // clamp SET_SPEED (absolute limit)
constexpr uint16_t MAX_SLEW_RPM_S_DEFAULT = 0;  // 0 = disabled (no slew limiting)

// Clamp for torque command (absolute, in mNm)
constexpr int16_t MAX_TORQUE_MNM_DEFAULT = 5;  // adjust to your wheel capability

// ----- FDIR Thresholds ------------------------------------------------------
constexpr int16_t STUCK_RPM_THRESH = 50;  // running == 0 but |rpm| > threshold
constexpr uint8_t STUCK_DEBOUNCE_FRAMES = 3;
constexpr int16_t TORQUE_HIGH_MNM_THRESH = 600;
constexpr uint8_t TORQUE_DEBOUNCE_FRAMES = 5;

// ----- Event Backoff (emit event every N errors) ----------------------------
constexpr uint32_t CRC_ERR_EVENT_THRESH = 10;
constexpr uint32_t MALFORMED_EVENT_THRESH = 10;

// ----- Housekeeping (Service 3) ---------------------------------------------
// Period for periodic HK generation by the LocalDataPoolManager (seconds).
// Note: Actual TM emission rate also depends on global HK distributor settings.
constexpr float HK_PERIOD_S =
    1000.0f;  // HK_Period = PST_Period * nonDiagIntervalFactor * HK_PERIOD_S = 0.1 * 5 * N

// ----- Typed TM (PUS 220) versions -----------------------------------------
// Keep small version integers to allow forward-compatible parsing on ground.
constexpr uint8_t RW_TYPED_TM_VERSION = 1;   // for RW typed TM (e.g. subservice 131)
constexpr uint8_t ACS_TYPED_TM_VERSION = 1;  // for ACS diagnostics typed TM (e.g. subservice 132)

// ----- PUS 220 subservice numbers (shared constants) -----------------------
// These values are used by the PUS service implementation and ground tools.
namespace Pus220 {
// Commands
constexpr uint8_t SET_SPEED = 1;
constexpr uint8_t STOP = 2;
constexpr uint8_t STATUS = 3;
constexpr uint8_t SET_TORQUE = 4;
constexpr uint8_t SET_MODE = 10;
constexpr uint8_t ACS_SET_ENABLE = 140;  // Enable/disable ACS controller

// Telemetry
constexpr uint8_t TM_STATUS_RAW = 130;    // legacy/raw STATUS (if still used)
constexpr uint8_t TM_STATUS_TYPED = 131;  // typed RW status TM
constexpr uint8_t TM_ACS_DIAG = 132;      // typed ACS diagnostics TM
}  // namespace Pus220

}  // namespace RwConfig
