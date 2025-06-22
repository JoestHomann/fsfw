#include "FsfwExampleTask.h"

#include <fsfw/ipc/CommandMessage.h>
#include <fsfw/ipc/QueueFactory.h>
#include <fsfw/objectmanager/ObjectManager.h>
#include <fsfw/serviceinterface/ServiceInterface.h>
#include <fsfw/tasks/TaskFactory.h>

#include "OBSWConfig.h"
#include "commonObjects.h"
#include "objects/systemObjectList.h"

FsfwExampleTask::FsfwExampleTask(object_id_t objectId)
    : SystemObject(objectId),
      poolManager(this, nullptr),
      demoSet(this),
      monitor(objectId, MONITOR_ID, gp_id_t(objectId, FsfwDemoSet::VARIABLE_LIMIT), 30, 10) {
  commandQueue = QueueFactory::instance()->createMessageQueue(10, CommandMessage::MAX_MESSAGE_SIZE);
}

FsfwExampleTask::~FsfwExampleTask() {}

ReturnValue_t FsfwExampleTask::performOperation(uint8_t operationCode) {
  if (operationCode == OpCodes::DELAY_SHORT) {
    TaskFactory::delayTask(5);
  }

  // TODO: Move this to new test controller?
  ReturnValue_t result = performMonitoringDemo();
  if (result != returnvalue::OK) {
    return result;
  }

  if (operationCode == OpCodes::SEND_RAND_NUM) {
    result = performSendOperation();
    if (result != returnvalue::OK) {
      return result;
    }
  }

  if (operationCode == OpCodes::RECEIVE_RAND_NUM) {
    result = performReceiveOperation();
  }

  return 0;
}

object_id_t FsfwExampleTask::getNextRecipient() {
  switch (this->getObjectId()) {
    case (objects::TEST_DUMMY_1): {
      return objects::TEST_DUMMY_2;
    }
    case (objects::TEST_DUMMY_2): {
      return objects::TEST_DUMMY_3;
    }
    case (objects::TEST_DUMMY_3): {
      return objects::TEST_DUMMY_1;
    }
    default:
      return objects::TEST_DUMMY_1;
  }
}

object_id_t FsfwExampleTask::getSender() {
  switch (this->getObjectId()) {
    case (objects::TEST_DUMMY_1): {
      return objects::TEST_DUMMY_3;
    }
    case (objects::TEST_DUMMY_2): {
      return objects::TEST_DUMMY_1;
    }
    case (objects::TEST_DUMMY_3): {
      return objects::TEST_DUMMY_2;
    }
    default:
      return objects::TEST_DUMMY_1;
  }
}

ReturnValue_t FsfwExampleTask::initialize() {
  // Get the dataset of the sender. Will be cached for later checks.
  object_id_t sender = getSender();
  auto *senderIF = ObjectManager::instance()->get<HasLocalDataPoolIF>(sender);
  if (senderIF == nullptr) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::error << "FsfwDemoTask::initialize: Sender object invalid!" << std::endl;
#else
    sif::printError("FsfwDemoTask::initialize: Sender object invalid!\n");
#endif
    return returnvalue::FAILED;
  }

  // we need a private copy of the previous dataset.. or we use the shared
  // dataset.
  senderSet = new FsfwDemoSet(senderIF);
  if (senderSet == nullptr) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::error << "FsfwDemoTask::initialize: Sender dataset invalid!" << std::endl;
#else
    sif::printError("FsfwDemoTask::initialize: Sender dataset invalid!\n");
#endif
    return returnvalue::FAILED;
  }
  return poolManager.initialize(commandQueue);
}

ReturnValue_t FsfwExampleTask::initializeAfterTaskCreation() {
  return poolManager.initializeAfterTaskCreation();
}

object_id_t FsfwExampleTask::getObjectId() const { return SystemObject::getObjectId(); }

MessageQueueId_t FsfwExampleTask::getMessageQueueId() { return commandQueue->getId(); }

void FsfwExampleTask::setTaskIF(PeriodicTaskIF *task) { this->task = task; }

LocalPoolDataSetBase *FsfwExampleTask::getDataSetHandle(sid_t sid) { return &demoSet; }

uint32_t FsfwExampleTask::getPeriodicOperationFrequency() const { return task->getPeriodMs(); }

ReturnValue_t FsfwExampleTask::initializeLocalDataPool(localpool::DataPool &localDataPoolMap,
                                                       LocalDataPoolManager &poolManager) {
  localDataPoolMap.emplace(FsfwDemoSet::PoolIds::VARIABLE, new PoolEntry<uint32_t>({0}));
  localDataPoolMap.emplace(FsfwDemoSet::PoolIds::VARIABLE_LIMIT, new PoolEntry<uint16_t>({0}));
  return returnvalue::OK;
}

ReturnValue_t FsfwExampleTask::performMonitoringDemo() {
  ReturnValue_t result = demoSet.variableLimit.read(MutexIF::TimeoutType::WAITING, 20);
  if (result != returnvalue::OK) {
    /* Configuration error */
#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::error << "DummyObject::performOperation: Could not read variableLimit!" << std::endl;
#else
    sif::printError("DummyObject::performOperation: Could not read variableLimit!\n");
#endif
    return result;
  }
  if (this->getObjectId() == objects::TEST_DUMMY_5) {
    if (demoSet.variableLimit.value > 20) {
      demoSet.variableLimit.value = 0;
    }
    demoSet.variableLimit.value++;
    demoSet.variableLimit.commit(20);
    monitor.check();
  }
  return returnvalue::OK;
}

ReturnValue_t FsfwExampleTask::performSendOperation() {
  object_id_t nextRecipient = getNextRecipient();
  auto *target = ObjectManager::instance()->get<FsfwExampleTask>(nextRecipient);
  if (target == nullptr) {
    /* Configuration error */
#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::error << "DummyObject::performOperation: Next recipient does not exist!" << std::endl;
#else
    sif::printError("DummyObject::performOperation: Next recipient does not exist!\n");
#endif
    return returnvalue::FAILED;
  }

  uint32_t randomNumber = rand() % 100;
  CommandMessage message;
  message.setParameter(randomNumber);
  message.setParameter2(this->getMessageQueueId());

  /* Send message using own message queue */
  ReturnValue_t result = commandQueue->sendMessage(target->getMessageQueueId(), &message);
  if (result != returnvalue::OK && result != MessageQueueIF::FULL) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::error << "FsfwDemoTask::performSendOperation: Send failed with " << result << std::endl;
#else
    sif::printError("FsfwDemoTask::performSendOperation: Send failed with %hu\n", result);
#endif
  }

  /* Send message without via MessageQueueSenderIF */
  result = MessageQueueSenderIF::sendMessage(target->getMessageQueueId(), &message,
                                             commandQueue->getId());
  if (result != returnvalue::OK && result != MessageQueueIF::FULL) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::error << "FsfwDemoTask::performSendOperation: Send failed with " << result << std::endl;
#else
    sif::printError("FsfwDemoTask::performSendOperation: Send failed with %hu\n", result);
#endif
  }

  demoSet.variableWrite.value = randomNumber;
  result = demoSet.variableWrite.commit(20);

  return result;
}

ReturnValue_t FsfwExampleTask::performReceiveOperation() {
  ReturnValue_t result = returnvalue::OK;
  while (result != MessageQueueIF::EMPTY) {
    CommandMessage receivedMessage;
    result = commandQueue->receiveMessage(&receivedMessage);
    if (result != returnvalue::OK && result != MessageQueueIF::EMPTY) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
      sif::debug << "Receive failed with " << result << std::endl;
#endif
      break;
    }
    if (result != MessageQueueIF::EMPTY) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
#if OBSW_VERBOSE_LEVEL >= 2
      sif::debug << "Message Received by " << getObjectId() << " from Queue "
                 << receivedMessage.getSender() << " ObjectId " << receivedMessage.getParameter()
                 << " Queue " << receivedMessage.getParameter2() << std::endl;
#endif
#endif

      if (senderSet == nullptr) {
        return returnvalue::FAILED;
      }

      result = senderSet->variableRead.read(MutexIF::TimeoutType::WAITING, 20);
      if (result != returnvalue::OK) {
        return result;
      }
      if (senderSet->variableRead.value != receivedMessage.getParameter()) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
        sif::error << "FsfwDemoTask::performReceiveOperation: Variable " << std::hex << "0x"
                   << senderSet->variableRead.getDataPoolId() << std::dec << " has wrong value."
                   << std::endl;
        sif::error << "Value: " << demoSet.variableRead.value
                   << ", expected: " << receivedMessage.getParameter() << std::endl;
#endif
      }
    }
  }
  return result;
}

MessageQueueId_t FsfwExampleTask::getCommandQueue() const { return commandQueue->getId(); }

LocalDataPoolManager *FsfwExampleTask::getHkManagerHandle() { return &poolManager; }
