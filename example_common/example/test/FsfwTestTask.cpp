#include "FsfwTestTask.h"

#include <commonConfig.h>

#if FSFW_ADD_FMT_TESTS == 1
#include "testFmt.h"
#endif

FsfwTestTask::FsfwTestTask(object_id_t objectId, bool periodicEvent)
    : TestTask(objectId), periodicEvent(periodicEvent) {
#if FSFW_ADD_FMT_TESTS == 1
  fmtTests();
#endif
}

ReturnValue_t FsfwTestTask::performPeriodicAction() {
  if (periodicEvent) {
    triggerEvent(TEST_EVENT, 0x1234, 0x4321);
  }
  return returnvalue::OK;
}
