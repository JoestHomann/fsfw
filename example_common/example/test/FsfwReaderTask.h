#ifndef MISSION_DEMO_FSFWPERIODICTASK_H_
#define MISSION_DEMO_FSFWPERIODICTASK_H_

#include <fsfw/globalfunctions/PeriodicOperationDivider.h>
#include <fsfw/objectmanager/SystemObject.h>
#include <fsfw/tasks/ExecutableObjectIF.h>

#include "testdefinitions/demoDefinitions.h"

class FsfwReaderTask : public ExecutableObjectIF, public SystemObject {
 public:
  FsfwReaderTask(object_id_t objectId, bool enablePrintout);
  ~FsfwReaderTask() override;

  ReturnValue_t initializeAfterTaskCreation() override;
  ReturnValue_t performOperation(uint8_t operationCode = 0) override;

 private:
  bool printoutEnabled = false;
  PeriodicOperationDivider opDivider;
  CompleteDemoReadSet readSet;
};

#endif /* MISSION_DEMO_FSFWPERIODICTASK_H_ */
