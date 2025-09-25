#pragma once
#include <cstdint>

// PUS-220 configuration for Reaction Wheel / ACS typed telemetry and commands.
// Only constants here; no logic.

namespace RwPusConfig {

// Service number
constexpr uint8_t SVC_RW = 220;

// Common subservice IDs (extend as needed)
enum class Subservice : uint8_t {
  RW_SET_SPEED      = 1,   // TC, payload: OID|int16 rpm
  RW_STATUS_REQUEST = 3,   // TC, payload: OID
  ACS_SET_ENABLE    = 140  // TC, payload: OID|uint8 enable (0/1)
};

// Typed telemetry subservice (TM)
constexpr uint8_t TM_RW_TYPED_HK  = 132; // TM 220/132 for RW typed HK
constexpr uint8_t TM_ACS_TYPED_HK = 132; // TM 220/132 for ACS typed HK (separate payload)

// Version fields for typed telemetry payloads
constexpr uint8_t RW_TYPED_TM_VERSION  = 1;
constexpr uint8_t ACS_TYPED_TM_VERSION = 1;

// Optional helpers (sizes in bytes, informative only)
constexpr uint8_t OID_SIZE     = 4; // FSFW object ID size (typical)
constexpr uint8_t U16_SIZE     = 2;
constexpr uint8_t U8_SIZE      = 1;

} // namespace RwPusConfig
