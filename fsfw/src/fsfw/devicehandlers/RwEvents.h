#pragma once
#include <fsfw/events/Event.h>

#include "RwConfig.h"

/*
 * RwEvents.h - Event definitions for ReactionWheelsHandler
 *
 * Provides event IDs used by the RW device handler and services.
 * Outsourced from ReactionWheelsHandler to allow reuse in other RW handlers and services.
 * Catalogue of Events that the Event Manager can handle for the Reaction Wheel subsystem.
 * Based on Bätz Diss., p. 135–136
 * 
 * - Joest Homann
 */

class RwEvents {
 public:
  // FSFW SUBSYSTEM_ID of ReactionWheelsHandler
  static constexpr uint8_t SUBSYSTEM_ID = RwConfig::RW_SUBSYSTEM_ID;

  // Wheel reports not running but speed is above threshold
  static constexpr Event STUCK = MAKE_EVENT(0, severity::MEDIUM);

  // Measured torque magnitude is above safety threshold
  static constexpr Event TORQUE_HIGH = MAKE_EVENT(1, severity::LOW);

  // No reply within timeout window during STATUS polling
  static constexpr Event TIMEOUT = MAKE_EVENT(3, severity::LOW);

  // Reply frame has unexpected layout or length
  static constexpr Event INVALID_REPLY = MAKE_EVENT(4, severity::LOW);

  // CRC-16 check failed for a received frame
  static constexpr Event CRC_ERROR = MAKE_EVENT(5, severity::LOW);
};
