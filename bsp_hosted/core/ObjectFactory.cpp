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
#include "fsfw/tasks/TaskFactory.h"

// JH
//#include "fsfw/src/fsfw/devicehandlers/ReactionWheelsHandler.h"
#include "fsfw/src/fsfw_hal/linux/serial/SerialComIF.h"
#include "fsfw/src/fsfw_hal/linux/serial/SerialCookie.h"
// JH

// --- JH: RwCommanderHandler & PUS service ---
#include "fsfw/src/fsfw/devicehandlers/RwCommanderHandler.h"
#include "fsfw/src/fsfw/tmtcservices/RwPusService.h"
#include "fsfw/devicehandlers/DeviceHandlerIF.h"  // for DeviceHandlerIF::MODE_RAW
#include "fsfw/modes/ModeMessage.h"
#include "fsfw/modes/HasModesIF.h"
#include "fsfw/ipc/QueueFactory.h"
#include "fsfw/ipc/CommandMessage.h"
// --- JH ---

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

  // TMTC reception via UDP/TCP socket
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
  // TODO: Configure valid space packet IDs if required
#endif

  ObjectFactory::produceGenericObjects(&funnel, *tmtcBridge, &ccsdsDistrib, *tcStore, *tmStore);
#endif /* OBSW_ADD_CORE_COMPONENTS == 1 */

  bool periodicEvent = false;
#if OBSW_TASK_PERIODIC_EVENT == 1
  periodicEvent = true;
#endif
  new FsfwTestTask(objects::TEST_TASK, periodicEvent);

  /*
  // JH (optional): Original ReactionWheelsHandler.
  // Keep this commented if you use the same serial port for the new commander,
  // otherwise /dev/ttyACM0 will be opened twice.

  // SerialComIF and SerialCookie for Reaction Wheel
  auto* serialComIf = new SerialComIF(objects::RW_SERIAL_COM_IF);

  auto* rwSerialCookie = new SerialCookie(
      objects::RW_HANDLER,          // owner (the device handler's object id)
      "/dev/ttyACM0",               // device file
      UartBaudRate::RATE_9600,      // enum from fsfw_hal/linux/serial/helper.h
      256,                          // max expected reply length [bytes]
      UartModes::NON_CANONICAL      // binary protocol (no line delimiters)
  );

  // Create the ReactionWheelsHandler
  auto* rwHandler = new ReactionWheelsHandler(
      objects::RW_HANDLER,          // handler object id
      objects::RW_SERIAL_COM_IF,    // comIF object id
      rwSerialCookie
  );

  // Create a periodic task for the ReactionWheelsHandler
  PeriodicTaskIF* rwTask = TaskFactory::instance()->createPeriodicTask(
      "RW_HANDLER_TASK",  // task name
      40,                 // priority
      4096,               // stack size
      0.2,                // period [s]
      nullptr             // deadline missed callback (optional)
  );

  // Add the handler so it gets executed periodically
  rwTask->addComponent(rwHandler);
  */

  // ---------------- rwCommanderHandler ------------------------
  // --- JH: RwCommanderHandler (minimal serial commander)
  // WARNING: Do not open the same /dev/tty* in multiple handlers at once.
  auto* rwCmdComIf = new SerialComIF(objects::RW_CMD_SERIAL_COM_IF);

  auto* rwCmdCookie = new SerialCookie(
      objects::RW_CMD_HANDLER,      // owner id (handler)
      "/dev/ttyACM0",               // adjust to your setup
      UartBaudRate::RATE_9600,      // must match Arduino sketch
      1024,                         // read buffer size
      UartModes::NON_CANONICAL      // raw binary mode
  );

  // Create the commander handler
  auto* rwCmdHandler =
      new RwCommanderHandler(objects::RW_CMD_HANDLER, objects::RW_CMD_SERIAL_COM_IF, rwCmdCookie);

  // Put handler into its own periodic task (e.g., 5 Hz)
  PeriodicTaskIF* rwCmdTask =
      TaskFactory::instance()->createPeriodicTask("RW_CMD_TASK", 40, 4096, 0.2, nullptr);
  rwCmdTask->addComponent(rwCmdHandler);
  rwCmdTask->startTask();

  // Put the handler into RAW (required for RAW forwarding via PUS service)
  {
    CommandMessage mm;
    ModeMessage::setModeMessage(&mm, ModeMessage::CMD_MODE_COMMAND,
                                DeviceHandlerIF::MODE_RAW, 0);  // NOTE: DeviceHandlerIF, not HasModesIF
    auto* tmp = QueueFactory::instance()->createMessageQueue(1, CommandMessage::MAX_MESSAGE_SIZE);
    tmp->sendMessage(rwCmdHandler->getCommandQueue(), &mm);
  }
  // ---------------- rwCommanderHandler ------------------------

  // ---------------- RwPusService (PUS-220) --------------------
  // If you already have a common TM/TC services task, simply addComponent(rwPus) there.
  constexpr uint16_t RW_PUS_APID = 0x00EF;  // pick an APID suitable for your setup
  auto* rwPus = new RwPusService(objects::RW_PUS_SERVICE, RW_PUS_APID);

  PeriodicTaskIF* pusTask =
      TaskFactory::instance()->createPeriodicTask("PUS_RW_SERVICE", 35, 4096, 0.2, nullptr);
  pusTask->addComponent(rwPus);
  pusTask->startTask();
  // ---------------- RwPusService (PUS-220) --------------------
}
