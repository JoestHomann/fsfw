#ifndef EXAMPLE_COMMON_EXAMPLE_CONTROLLER_FSFWTESTCONTROLLER_H_
#define EXAMPLE_COMMON_EXAMPLE_CONTROLLER_FSFWTESTCONTROLLER_H_

#include "fsfw/controller/ExtendedControllerBase.h"
#include "fsfw_tests/integration/controller/TestController.h"

class FsfwTestController : public TestController {
 public:
  FsfwTestController(object_id_t objectId, object_id_t device0, object_id_t device1,
                     uint8_t verboseLevel = 0);
  virtual ~FsfwTestController();
  ReturnValue_t handleCommandMessage(CommandMessage *message) override;

  /**
   * Periodic helper from ControllerBase, implemented by child class.
   */
  void performControlOperation() override;

 private:
  object_id_t device0Id;
  object_id_t device1Id;
  testdevice::TestDataSet deviceDataset0;
  testdevice::TestDataSet deviceDataset1;

  uint8_t verboseLevel = 0;
  bool traceVariable = false;
  uint8_t traceCycles = 5;
  uint8_t traceCounter = traceCycles;

  enum TraceTypes { NONE, TRACE_DEV_0_UINT8, TRACE_DEV_0_VECTOR };
  TraceTypes currentTraceType = TraceTypes::NONE;

  ReturnValue_t initializeAfterTaskCreation() override;
  void handleChangedDataset(sid_t sid, store_address_t storeId, bool *clearMessage) override;
  void handleChangedPoolVariable(gp_id_t globPoolId, store_address_t storeId,
                                 bool *clearMessage) override;
  ReturnValue_t initializeLocalDataPool(localpool::DataPool &localDataPoolMap,
                                        LocalDataPoolManager &poolManager) override;
  LocalPoolDataSetBase *getDataSetHandle(sid_t sid) override;
  ReturnValue_t checkModeCommand(Mode_t mode, Submode_t submode,
                                 uint32_t *msToReachTheMode) override;
};

#endif /* EXAMPLE_COMMON_EXAMPLE_CONTROLLER_FSFWTESTCONTROLLER_H_ */
