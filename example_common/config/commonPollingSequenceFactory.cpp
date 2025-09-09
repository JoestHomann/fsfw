#include <fsfw/devicehandlers/DeviceHandlerIF.h>
#include <fsfw/tasks/FixedTimeslotTaskIF.h>

#include "example/test/FsfwExampleTask.h"
#include "objects/systemObjectList.h"
#include "pollingsequence/pollingSequenceFactory.h"
#include "commonObjects.h"

ReturnValue_t pst::pollingSequenceExamples(FixedTimeslotTaskIF *thisSequence) {
  uint32_t length = thisSequence->getPeriodMs();

  thisSequence->addSlot(objects::TEST_DUMMY_1, length * 0, FsfwExampleTask::OpCodes::SEND_RAND_NUM);
  thisSequence->addSlot(objects::TEST_DUMMY_2, length * 0, FsfwExampleTask::OpCodes::SEND_RAND_NUM);
  thisSequence->addSlot(objects::TEST_DUMMY_3, length * 0, FsfwExampleTask::OpCodes::SEND_RAND_NUM);

  thisSequence->addSlot(objects::TEST_DUMMY_1, length * 0.2,
                        FsfwExampleTask::OpCodes::RECEIVE_RAND_NUM);
  thisSequence->addSlot(objects::TEST_DUMMY_2, length * 0.2,
                        FsfwExampleTask::OpCodes::RECEIVE_RAND_NUM);
  thisSequence->addSlot(objects::TEST_DUMMY_3, length * 0.2,
                        FsfwExampleTask::OpCodes::RECEIVE_RAND_NUM);

  thisSequence->addSlot(objects::TEST_DUMMY_1, length * 0.5, FsfwExampleTask::OpCodes::DELAY_SHORT);
  thisSequence->addSlot(objects::TEST_DUMMY_2, length * 0.5, FsfwExampleTask::OpCodes::DELAY_SHORT);
  thisSequence->addSlot(objects::TEST_DUMMY_3, length * 0.5, FsfwExampleTask::OpCodes::DELAY_SHORT);

  if (thisSequence->checkSequence() == returnvalue::OK) {
    return returnvalue::OK;
  } else {
#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::error << "pst::pollingSequenceInitFunction: Initialization errors!" << std::endl;
#else
    sif::printError("pst::pollingSequenceInitFunction: Initialization errors!\n");
#endif
    return returnvalue::OK;
  }
}

ReturnValue_t pst::pollingSequenceDevices(FixedTimeslotTaskIF *thisSequence) {
  uint32_t length = thisSequence->getPeriodMs();

  thisSequence->addSlot(objects::TEST_DEVICE_HANDLER_0, 0, DeviceHandlerIF::PERFORM_OPERATION);
  thisSequence->addSlot(objects::TEST_DEVICE_HANDLER_1, 0, DeviceHandlerIF::PERFORM_OPERATION);

  thisSequence->addSlot(objects::TEST_DEVICE_HANDLER_0, 0.3 * length, DeviceHandlerIF::SEND_WRITE);
  thisSequence->addSlot(objects::TEST_DEVICE_HANDLER_1, 0.3 * length, DeviceHandlerIF::SEND_WRITE);

  thisSequence->addSlot(objects::TEST_DEVICE_HANDLER_0, 0.45 * length, DeviceHandlerIF::GET_WRITE);
  thisSequence->addSlot(objects::TEST_DEVICE_HANDLER_1, 0.45 * length, DeviceHandlerIF::GET_WRITE);

  thisSequence->addSlot(objects::TEST_DEVICE_HANDLER_0, 0.6 * length, DeviceHandlerIF::SEND_READ);
  thisSequence->addSlot(objects::TEST_DEVICE_HANDLER_1, 0.6 * length, DeviceHandlerIF::SEND_READ);

  thisSequence->addSlot(objects::TEST_DEVICE_HANDLER_0, 0.8 * length, DeviceHandlerIF::GET_READ);
  thisSequence->addSlot(objects::TEST_DEVICE_HANDLER_1, 0.8 * length, DeviceHandlerIF::GET_READ);

  // ======================================================================
  // === RW COMMANDER HANDLER (ADDED) =====================================
  // Schedules the RwCommanderHandler so that buildNormalDeviceCommand()
  // is called periodically in MODE_NORMAL. Make sure that:
  //  1) objects::RW_COMMANDER_HANDLER exists in systemObjectList.h,
  //  2) the handler is created in the Object Factory, and
  //  3) doStartUp() switches to MODE_NORMAL (otherwise no normal commands).
  // The phases below spread the DeviceHandler operations over the cycle:
  //   - PERFORM_OPERATION: state machine + build{Normal,Transition}DeviceCommand
  //   - SEND_WRITE / GET_WRITE: transmit command bytes
  //   - SEND_READ  / GET_READ : request & collect reply bytes
  // ======================================================================
  thisSequence->addSlot(objects::RW_CMD_HANDLER, 0.00 * length, DeviceHandlerIF::PERFORM_OPERATION);
  thisSequence->addSlot(objects::RW_CMD_HANDLER, 0.30 * length, DeviceHandlerIF::SEND_WRITE);
  thisSequence->addSlot(objects::RW_CMD_HANDLER, 0.45 * length, DeviceHandlerIF::GET_WRITE);
  thisSequence->addSlot(objects::RW_CMD_HANDLER, 0.60 * length, DeviceHandlerIF::SEND_READ);
  thisSequence->addSlot(objects::RW_CMD_HANDLER, 0.80 * length, DeviceHandlerIF::GET_READ);

  // Add this just before the final checkSequence() block:
#if FSFW_CPP_OSTREAM_ENABLED == 1
  sif::info << "PST devices: scheduled RW_CMD_HANDLER with 5 slots; period(ms)="
            << length << std::endl; // Debug: proves this PST is compiled & called
#endif

  // === END RW COMMANDER HANDLER =========================================
  // ======================================================================


  if (thisSequence->checkSequence() == returnvalue::OK) {
    return returnvalue::OK;
  } else {
#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::error << "pst::pollingSequenceTestFunction: Initialization errors!" << std::endl;
#else
    sif::printError("pst::pollingSequenceTestFunction: Initialization errors!\n");
#endif
    return returnvalue::OK;
  }
}
