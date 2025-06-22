#ifndef COMMON_CONFIG_COMMONSUBSYSTEMIDS_H_
#define COMMON_CONFIG_COMMONSUBSYSTEMIDS_H_

#include "fsfw/events/fwSubsystemIdRanges.h"

/**
 * The subsystem IDs will be part of the event IDs used throughout the FSFW.
 */
namespace SUBSYSTEM_ID {
enum commonSubsystemId : uint8_t {
  COMMON_SUBSYSTEM_ID_START = FW_SUBSYSTEM_ID_RANGE,
  TEST_TASK_ID = 105,
  COMMON_SUBSYSTEM_ID_END
};
}

#endif /* COMMON_CONFIG_COMMONSUBSYSTEMIDS_H_ */
