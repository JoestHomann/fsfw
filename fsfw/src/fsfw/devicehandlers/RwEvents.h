#pragma once
#include "fsfw/events/Event.h"

// FSFW's MAKE_EVENT SUBSYSTEM_ID for ReactionWheelsHandler
#ifndef SUBSYSTEM_ID
#define SUBSYSTEM_ID 0x44
#endif

namespace RwEvents {
  // FDIR / telemetry-related events
  static constexpr Event STUCK         = MAKE_EVENT(0, severity::MEDIUM);
  static constexpr Event TORQUE_HIGH   = MAKE_EVENT(1, severity::LOW);
  static constexpr Event ERROR_CODE    = MAKE_EVENT(2, severity::HIGH);

  // Communication / protocol events
  static constexpr Event CRC_ERROR     = MAKE_EVENT(3, severity::LOW);
  static constexpr Event TIMEOUT       = MAKE_EVENT(4, severity::MEDIUM);
  static constexpr Event INVALID_REPLY = MAKE_EVENT(5, severity::LOW);
}
