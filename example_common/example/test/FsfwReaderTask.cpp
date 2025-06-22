#include "FsfwReaderTask.h"

#include <OBSWConfig.h>
#include <fsfw/datapool/PoolReadGuard.h>
#include <fsfw/serviceinterface/ServiceInterface.h>
#include <fsfw/tasks/TaskFactory.h>
#include <fsfw/timemanager/Stopwatch.h>

FsfwReaderTask::FsfwReaderTask(object_id_t objectId, bool enablePrintout)
    : SystemObject(objectId),
      printoutEnabled(enablePrintout),
      opDivider(10),
      readSet(this->getObjectId(), gp_id_t(objects::TEST_DUMMY_1, FsfwDemoSet::PoolIds::VARIABLE),
              gp_id_t(objects::TEST_DUMMY_2, FsfwDemoSet::PoolIds::VARIABLE),
              gp_id_t(objects::TEST_DUMMY_3, FsfwDemoSet::PoolIds::VARIABLE)) {
  /* Special protection for set reading because each variable is read from a
   * different pool */
  readSet.setReadCommitProtectionBehaviour(true);
}

FsfwReaderTask::~FsfwReaderTask() = default;

ReturnValue_t FsfwReaderTask::initializeAfterTaskCreation() {
  /* Give other task some time to set up local data pools. */
  TaskFactory::delayTask(20);
  return returnvalue::OK;
}

ReturnValue_t FsfwReaderTask::performOperation(uint8_t operationCode) {
  PoolReadGuard readHelper(&readSet);

  uint32_t variable1 = readSet.variable1.value;
  uint32_t variable2 = readSet.variable2.value;
  uint32_t variable3 = readSet.variable3.value;

#if OBSW_VERBOSE_LEVEL >= 1
  if (opDivider.checkAndIncrement() and printoutEnabled) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::info << "FsfwPeriodicTask::performOperation: Reading variables." << std::endl;
    sif::info << "Variable read from demo object 1: " << variable1 << std::endl;
    sif::info << "Variable read from demo object 2: " << variable2 << std::endl;
    sif::info << "Variable read from demo object 3: " << variable3 << std::endl;
#else
    sif::printInfo("FsfwPeriodicTask::performOperation: Reading variables.\n\r");
    sif::printInfo("Variable read from demo object 1: %d\n\r", variable1);
    sif::printInfo("Variable read from demo object 2: %d\n\r", variable2);
    sif::printInfo("Variable read from demo object 3: %d\n\r", variable3);
#endif
  }
#else
  if (variable1 and variable2 and variable3) {
  };
#endif
  return returnvalue::OK;
}
