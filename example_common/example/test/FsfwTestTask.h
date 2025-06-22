#ifndef EXAMPLE_COMMON_EXAMPLE_TEST_FSFWTESTTASK_H_
#define EXAMPLE_COMMON_EXAMPLE_TEST_FSFWTESTTASK_H_

#include "events/subsystemIdRanges.h"
#include "fsfw/events/Event.h"
#include "fsfw_tests/integration/task/TestTask.h"

class FsfwTestTask : public TestTask {
 public:
  FsfwTestTask(object_id_t objectId, bool periodicEvent);

  ReturnValue_t performPeriodicAction() override;

 private:
  bool periodicEvent = false;
  static constexpr uint8_t subsystemId = SUBSYSTEM_ID::TEST_TASK_ID;
  static constexpr Event TEST_EVENT = event::makeEvent(subsystemId, 0, severity::INFO);
};

#endif /* EXAMPLE_COMMON_EXAMPLE_TEST_FSFWTESTTASK_H_ */
