#ifndef FSFW_CONTAINER_DEFINITIONS_H_
#define FSFW_CONTAINER_DEFINITIONS_H_

#include "fsfw/retval.h"

namespace containers {
static const ReturnValue_t KEY_ALREADY_EXISTS = returnvalue::makeCode(CLASS_ID::FIXED_MAP, 0x01);
static const ReturnValue_t MAP_FULL = returnvalue::makeCode(CLASS_ID::FIXED_MAP, 0x02);
static const ReturnValue_t KEY_DOES_NOT_EXIST = returnvalue::makeCode(CLASS_ID::FIXED_MAP, 0x03);

static const ReturnValue_t LIST_FULL = returnvalue::makeCode(CLASS_ID::ARRAY_LIST, 0x01);
}  // namespace containers

#endif /* FSFW_CONTAINER_DEFINITIONS_H_ */
