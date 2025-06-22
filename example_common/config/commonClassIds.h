#ifndef COMMON_CONFIG_COMMONCLASSIDS_H_
#define COMMON_CONFIG_COMMONCLASSIDS_H_

#include "fsfw/returnvalues/FwClassIds.h"

namespace CLASS_ID {
enum commonClassIds : uint8_t {
  COMMON_CLASS_ID_START = FW_CLASS_ID_COUNT,
  DUMMY_HANDLER,       // DDH
  COMMON_CLASS_ID_END  // [EXPORT] : [END]
};
}

#endif /* COMMON_CONFIG_COMMONCLASSIDS_H_ */
