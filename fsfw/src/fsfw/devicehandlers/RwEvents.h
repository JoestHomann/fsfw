#pragma once
#include <fsfw/events/Event.h>
#include "RwConfig.h"

// Event IDs for the Reaction Wheel subsystem
class RwEvents {
 public:
  // FSFW macro MAKE_EVENT expects SUBSYSTEM_ID to be available here
  static constexpr uint8_t SUBSYSTEM_ID = RwConfig::RW_SUBSYSTEM_ID;

  static constexpr Event STUCK          = MAKE_EVENT(0, severity::MEDIUM);
  static constexpr Event TORQUE_HIGH    = MAKE_EVENT(1, severity::LOW);
  static constexpr Event ERROR_CODE     = MAKE_EVENT(2, severity::HIGH);
  static constexpr Event TIMEOUT        = MAKE_EVENT(3, severity::LOW);
  static constexpr Event INVALID_REPLY  = MAKE_EVENT(4, severity::LOW);
  static constexpr Event CRC_ERROR      = MAKE_EVENT(5, severity::LOW);
};
