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
#include "fsfw/devicehandlers/DeviceHandlerFailureIsolation.h"
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

  // ---------------- rwCommanderHandler ------------------------
  // --- JH: RwCommanderHandler (minimal serial commander)
(void) new SerialComIF(objects::RW_CMD_SERIAL_COM_IF);  // SystemObject registriert sich selbst

auto* rwCmdCookie = new SerialCookie(
    objects::RW_CMD_HANDLER,
    "/dev/ttyACM0",                // ggf. anpassen
    UartBaudRate::RATE_9600,
    1024,
    UartModes::NON_CANONICAL);

rwCmdCookie->setReadCycles(5);      // allow up to 5 read() attempts per GET_READ phase
rwCmdCookie->setToFlushInput(true); // optional: flush stale bytes after opening port

auto* rwCmdHandler =
    new RwCommanderHandler(objects::RW_CMD_HANDLER,
                           objects::RW_CMD_SERIAL_COM_IF,
                           rwCmdCookie);

  // ---------------- RwPusService (PUS-220) --------------------
  // If you already have a common TM/TC services task, simply addComponent(rwPus) there.
  constexpr uint16_t RW_PUS_APID = 0x00EF;  // pick an APID suitable for your setup
  auto* rwPus = new RwPusService(objects::RW_PUS_SERVICE,
                               RW_PUS_APID,
                               /*serviceId*/ 220,
                               /*numParallelCommands*/ 4,
                               /*timeoutSeconds*/ 5);


  PeriodicTaskIF* pusTask =
      TaskFactory::instance()->createPeriodicTask("PUS_RW_SERVICE", 35, 4096, 0.2, nullptr);
  pusTask->addComponent(rwPus);
  pusTask->startTask();
  // ---------------- RwPusService (PUS-220) --------------------
}
