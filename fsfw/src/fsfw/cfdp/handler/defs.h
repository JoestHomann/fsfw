#ifndef FSFW_CFDP_HANDLER_DEFS_H
#define FSFW_CFDP_HANDLER_DEFS_H

namespace cfdp {

enum class CfdpStates { IDLE, BUSY_CLASS_1_NACKED, BUSY_CLASS_2_ACKED, SUSPENDED };

static constexpr uint8_t SSID = SUBSYSTEM_ID::CFDP;

namespace events {

static constexpr Event STORE_ERROR = event::makeEvent(SSID, 0, severity::LOW);
static constexpr Event MSG_QUEUE_ERROR = event::makeEvent(SSID, 1, severity::LOW);
static constexpr Event SERIALIZATION_ERROR = event::makeEvent(SSID, 2, severity::LOW);

}  // namespace events

}  // namespace cfdp
#endif  // FSFW_CFDP_HANDLER_DEFS_H
