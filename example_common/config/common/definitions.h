#pragma once

#include <cstdint>

/**
 * Enumerations for used PUS service IDs.
 */
namespace pus {
enum ServiceIds : uint8_t {
  PUS_SERVICE_1 = 1,
  PUS_SERVICE_2 = 2,
  PUS_SERVICE_3 = 3,
  PUS_SERVICE_5 = 5,
  PUS_SERVICE_8 = 8,
  PUS_SERVICE_9 = 9,
  PUS_SERVICE_11 = 11,
  PUS_SERVICE_17 = 17,
  PUS_SERVICE_20 = 20,
  PUS_SERVICE_200 = 200
};
}

namespace common {

/**
 *  The APID is a 14 bit identifier which can be used to distinguish processes and applications
 * on a spacecraft. For more details, see the related ECSS/CCSDS standards.
 * For this example, we are going to use a constant APID
 */
static constexpr uint16_t COMMON_PUS_APID = 0xEF;
static constexpr uint16_t COMMON_CFDP_APID = 0xF0;
static constexpr uint16_t COMMON_CFDP_CLIENT_ENTITY_ID = 0x01;

}  // namespace common
