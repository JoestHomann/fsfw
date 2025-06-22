#ifndef MISSION_DEMO_FSFWDEMOTASK_H_
#define MISSION_DEMO_FSFWDEMOTASK_H_

#include <fsfw/datapoollocal/LocalPoolVariable.h>
#include <fsfw/datapoollocal/StaticLocalDataSet.h>
#include <fsfw/ipc/MessageQueueIF.h>
#include <fsfw/monitoring/AbsLimitMonitor.h>
#include <fsfw/objectmanager/SystemObject.h>
#include <fsfw/tasks/ExecutableObjectIF.h>

#include "testdefinitions/demoDefinitions.h"

class PeriodicTaskIF;

/**
 * @brief 	This demo set shows the local data pool functionality and fixed
 * 			timeslot capabilities of the FSFW.
 *
 * @details
 * There will be multiple demo objects. Each demo object will generate a random
 * number and send that number via message queues to the next demo object
 * (e.g. DUMMY_1 sends the number to DUMMY_2 etc.). The receiving object
 * will check the received value against the sent value by extracting the sent
 * value directly from the sender via the local data pool interface.
 * If the timing is set up correctly, the values will always be the same.
 */
class FsfwExampleTask : public ExecutableObjectIF, public SystemObject, public HasLocalDataPoolIF {
 public:
  enum OpCodes { SEND_RAND_NUM, RECEIVE_RAND_NUM, DELAY_SHORT };

  static constexpr uint8_t MONITOR_ID = 2;

  /**
   * @brief 	Simple constructor, only expects object ID.
   * @param objectId
   */
  FsfwExampleTask(object_id_t objectId);

  virtual ~FsfwExampleTask();

  /**
   * @brief	The performOperation method is executed in a task.
   * @details	There are no restrictions for calls within this method, so any
   * 			other member of the class can be used.
   * @return	Currently, the return value is ignored.
   */
  virtual ReturnValue_t performOperation(uint8_t operationCode = 0) override;

  /**
   * @brief 	This function will be called by the global object manager
   * @details
   * This function will always be called before any tasks are started.
   * It can also be used to return error codes in the software initialization
   * process cleanly.
   * @return
   */
  virtual ReturnValue_t initialize() override;

  /**
   * @brief 	This function will be called by the OSAL task handlers
   * @details
   * This function will be called before the first #performOperation
   * call after the tasks have been started. It can be used if some
   * initialization process requires task specific properties like
   * periodic intervals (by using the PeriodicTaskIF* handle).
   * @return
   */
  virtual ReturnValue_t initializeAfterTaskCreation() override;

  /**
   * This function will be called by the OSAL task handler. The
   * task interface handle can be cached to access task specific properties.
   * @param task
   */
  void setTaskIF(PeriodicTaskIF *task) override;

  object_id_t getObjectId() const override;

  MessageQueueId_t getMessageQueueId();

 private:
  LocalDataPoolManager poolManager;
  FsfwDemoSet *senderSet = nullptr;
  FsfwDemoSet demoSet;
  AbsLimitMonitor<int32_t> monitor;
  PeriodicTaskIF *task = nullptr;
  MessageQueueIF *commandQueue = nullptr;

  /* HasLocalDatapoolIF overrides */
  MessageQueueId_t getCommandQueue() const override;
  LocalPoolDataSetBase *getDataSetHandle(sid_t sid) override;
  uint32_t getPeriodicOperationFrequency() const override;
  ReturnValue_t initializeLocalDataPool(localpool::DataPool &localDataPoolMap,
                                        LocalDataPoolManager &poolManager) override;
  LocalDataPoolManager *getHkManagerHandle() override;

  object_id_t getNextRecipient();
  object_id_t getSender();

  ReturnValue_t performMonitoringDemo();
  ReturnValue_t performSendOperation();
  ReturnValue_t performReceiveOperation();
};

#endif /* MISSION_DEMO_FSFWDEMOTASK_H_ */
