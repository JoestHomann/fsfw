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
#include "fsfw/tasks/TaskFactory.h"
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
  // 1. Create the ReactionWheelsHandler object
  auto* rwHandler =
      new ReactionWheelsHandler(objects::RW_HANDLER,  // Verwende die Object-ID, passe ggf. an!
                               0,       // comIF (dummy/0, falls keine echte Kommunikation)
                               nullptr  // comCookie (nullptr für Simulationsstart)
      );

  // 2. Create a periodic task for the ReactionWheelHandler
  PeriodicTaskIF* rwTask = TaskFactory::instance()->createPeriodicTask(
      "RW_HANDLER_TASK",  // Task name
      40,                 // Priority (z.B. 40, passe an dein System an)
      4096,               // Stack size (z.B. 4096)
      0.2,                // Period in seconds (z.B. 0.2 = 200ms = 5 Hz)
      nullptr             // Deadline missed callback (optional)
  );

  // 3. Add the handler to the task so it gets executed periodically
  rwTask->addComponent(rwHandler);
  // JH
}
