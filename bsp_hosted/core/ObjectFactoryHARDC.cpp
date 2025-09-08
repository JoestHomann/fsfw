#include "ObjectFactory.h"

#include "OBSWConfig.h"
#include "bsp_hosted/fsfwconfig/objects/systemObjectList.h"
#include "commonConfig.h"
#include "example/core/GenericFactory.h"
#include "example/test/FsfwTestTask.h"
#include "example/utility/TmFunnel.h"
#include "fsfw/storagemanager/PoolManager.h"
#include "fsfw/tcdistribution/CcsdsDistributor.h"
#include "fsfw/tmtcservices/CommandingServiceBase.h"
#include "fsfw_hal/host/HostFilesystem.h"


// JH
#include "fsfw/src/fsfw/devicehandlers/ReactionWheelsHandler.h"
#include "fsfw/src/fsfw_hal/linux/serial/SerialComIF.h"
#include "fsfw/src/fsfw_hal/linux/serial/SerialCookie.h"
#include "fsfw/tasks/TaskFactory.h"
// --- JH HARDCODE ---
#include "fsfw/modes/HasModesIF.h"          // mode enum
#include "fsfw/modes/ModeMessage.h"         // ModeMessage helper
#include "fsfw/ipc/QueueFactory.h"          // to create a temporary sender queue
#include "fsfw/ipc/MessageQueueIF.h"
#include "fsfw/ipc/CommandMessage.h"
// --- JH HARDCODE ---
// JH

#if OBSW_USE_TCP_SERVER == 0
#include <fsfw/osal/common/UdpTcPollingTask.h>
#include <fsfw/osal/common/UdpTmTcBridge.h>
#else
#include "fsfw/osal/common/TcpTmTcBridge.h"
#include "fsfw/osal/common/TcpTmTcServer.h"
#endif

#if OBSW_ADD_CFDP_COMPONENTS == 1
// These CFDP containers are user supplied because their size might differ depending on
// which system the example is run on
namespace cfdp {
PacketInfoList<128> PACKET_INFO;
PacketInfoListBase* PACKET_LIST_PTR = &PACKET_INFO;
LostSegmentsList<128> LOST_SEGMENTS;
LostSegmentsListBase* LOST_SEGMENTS_PTR = &LOST_SEGMENTS;
}  // namespace cfdp
#endif

void ObjectFactory::produce(void* args) {
  Factory::setStaticFrameworkObjectIds();
  StorageManagerIF* tcStore;
  StorageManagerIF* tmStore;
#if OBSW_ADD_CORE_COMPONENTS == 1
  {
    LocalPool::LocalPoolConfig poolCfg = {{100, 16}, {50, 32},   {40, 64},
                                          {30, 128}, {20, 1024}, {10, 2048}};
    tcStore = new PoolManager(objects::TC_STORE, poolCfg);
  }

  {
    LocalPool::LocalPoolConfig poolCfg = {{100, 16}, {50, 32},   {40, 64},
                                          {30, 128}, {20, 1024}, {10, 2048}};
    tmStore = new PoolManager(objects::TM_STORE, poolCfg);
  }

  {
    LocalPool::LocalPoolConfig poolCfg = {{100, 16}, {50, 32},   {40, 64},
                                          {30, 128}, {20, 1024}, {10, 2048}};
    new PoolManager(objects::IPC_STORE, poolCfg);
  }
  PusTmFunnel* funnel;
  CcsdsDistributor* ccsdsDistrib;
  // TMTC Reception via TCP/IP socket
#if OBSW_USE_TCP_SERVER == 0
  auto tmtcBridge = new UdpTmTcBridge(objects::TCPIP_TMTC_BRIDGE, objects::CCSDS_DISTRIBUTOR);
  tmtcBridge->setMaxNumberOfPacketsStored(50);
  sif::info << "Opening UDP TMTC server on port " << tmtcBridge->getUdpPort() << std::endl;
  new UdpTcPollingTask(objects::TCPIP_TMTC_POLLING_TASK, objects::TCPIP_TMTC_BRIDGE);
#else
  auto tmtcBridge = new TcpTmTcBridge(objects::TCPIP_TMTC_BRIDGE, objects::CCSDS_DISTRIBUTOR);
  tmtcBridge->setMaxNumberOfPacketsStored(50);
  auto tmtcServer = new TcpTmTcServer(objects::TCPIP_TMTC_POLLING_TASK, objects::TCPIP_TMTC_BRIDGE);
  sif::info << "Opening TCP TMTC server on port " << tmtcServer->getTcpPort() << std::endl;
  // TODO: Set the set of valid space packet IDs. Otherwise, parsing might fail
#endif
  ObjectFactory::produceGenericObjects(&funnel, *tmtcBridge, &ccsdsDistrib, *tcStore, *tmStore);

#endif /* OBSW_ADD_CORE_COMPONENTS == 1 */

  bool periodicEvent = false;
#if OBSW_TASK_PERIODIC_EVENT == 1
  periodicEvent = true;
#endif
  new FsfwTestTask(objects::TEST_TASK, periodicEvent);

// JH
// Create SerialComIF and SerialCookie for Reaction Wheel

// 1) SerialComIF needs an object_id_t now
auto* serialComIf = new SerialComIF(objects::RW_SERIAL_COM_IF);
// NOTE: SerialComIF is a SystemObject and auto-registers itself.

// 2) SerialCookie expects 5 args: (handlerId, deviceFile, UartBaudRate, readBufLen, UartModes)
//    - First arg must be the device handler's object ID (the consumer of the cookie)
auto* rwSerialCookie = new SerialCookie(
    objects::RW_HANDLER,           // object_id_t of the handler using this cookie
    "/dev/ttyACM0",                // device file (adjust to your system)
    UartBaudRate::RATE_9600,       // adapt if your Arduino uses 115200 etc.
    1024,                          // read buffer size in bytes (tune as needed)
    UartModes::NON_CANONICAL       // raw mode for binary protocols
);

// 3) Create the ReactionWheelsHandler (assumes ctor: (handlerId, comIfId, CookieIF*))
auto* rwHandler =
    new ReactionWheelsHandler(objects::RW_HANDLER, objects::RW_SERIAL_COM_IF, rwSerialCookie);

// 4) Periodic task to run the handler
PeriodicTaskIF* rwTask = TaskFactory::instance()->createPeriodicTask(
    "RW_HANDLER_TASK",  // task name
    40,                 // priority (adjust to your system)
    4096,               // stack size
    0.2,                // period [s] (5 Hz)
    nullptr             // deadline missed callback
);

// 5) Add the handler to the task
rwTask->addComponent(rwHandler);

// --- JH HARDCODE ---
// Request initial mode via a ModeMessage (setMode is protected).
// We send MODE_ON; with the JH hardcoded test active, doStartUp() will switch to MODE_NORMAL.
{
  CommandMessage modeCmd;
  ModeMessage::setModeMessage(&modeCmd, ModeMessage::CMD_MODE_COMMAND,
                              HasModesIF::MODE_ON, 0 /*submode*/);

  // Create a tiny temporary sender queue and push the message to the handler's command queue.
  MessageQueueIF* tmpSender =
      QueueFactory::instance()->createMessageQueue(1, CommandMessage::MAX_MESSAGE_SIZE);
  tmpSender->sendMessage(rwHandler->getCommandQueue(), &modeCmd);

#if FSFW_CPP_OSTREAM_ENABLED == 1
  sif::info << "Requested initial RW handler mode: MODE_ON (via ModeMessage)" << std::endl;
#else
  sif::printInfo("Requested initial RW handler mode: MODE_ON (via ModeMessage)\n");
#endif
}

// Start the dedicated RW handler task so the hardcoded sequence can run.
rwTask->startTask();
#if FSFW_CPP_OSTREAM_ENABLED == 1
  sif::info << "RW_HANDLER_TASK started." << std::endl;
#else
  sif::printInfo("RW_HANDLER_TASK started.\n");
#endif
// --- JH HARDCODE ---
// JH
}
