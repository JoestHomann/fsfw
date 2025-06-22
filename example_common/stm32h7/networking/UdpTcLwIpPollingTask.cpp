#include "UdpTcLwIpPollingTask.h"

#include "TmTcLwIpUdpBridge.h"
#include "app_dhcp.h"
#include "app_ethernet.h"
#include "ethernetif.h"
#include "fsfw/ipc/MutexGuard.h"
#include "fsfw/objectmanager/ObjectManager.h"
#include "fsfw/serviceinterface/ServiceInterface.h"
#include "lwip/timeouts.h"
#include "networking.h"

UdpTcLwIpPollingTask::UdpTcLwIpPollingTask(object_id_t objectId, object_id_t bridgeId,
                                           struct netif *gnetif)
    : SystemObject(objectId), periodicHandleCounter(0), bridgeId(bridgeId), gnetif(gnetif) {}

UdpTcLwIpPollingTask::~UdpTcLwIpPollingTask() = default;

ReturnValue_t UdpTcLwIpPollingTask::initialize() {
  udpBridge = ObjectManager::instance()->get<TmTcLwIpUdpBridge>(bridgeId);
  if (udpBridge == nullptr) {
    return ObjectManagerIF::CHILD_INIT_FAILED;
  }
  if (netif_is_link_up(gnetif)) {
    networking::setEthCableConnected(true);
  }
  return RETURN_OK;
}

/* Poll the EMAC Interface and pass content to the network interface (lwIP) */
ReturnValue_t UdpTcLwIpPollingTask::performOperation(uint8_t operationCode) {
  /* Read a received packet from the Ethernet buffers and send it
      to the lwIP for handling */
  ethernetif_input(gnetif);

  /* Handle timeouts */
  sys_check_timeouts();

#if LWIP_NETIF_LINK_CALLBACK == 1
  networking::ethernetLinkPeriodicHandle(gnetif);
#endif

  if (udpBridge != nullptr) {
    MutexGuard lg(udpBridge->bridgeLock);
    /* In case ethernet cable is disconnected */
    if (not networking::getEthCableConnected() and udpBridge->comLinkUp()) {
      udpBridge->physicalConnectStatusChange(false);
    } else if (networking::getEthCableConnected() and not udpBridge->comLinkUp()) {
      udpBridge->physicalConnectStatusChange(true);
    }
  }

#if LWIP_DHCP == 1
  DHCP_Periodic_Handle(gnetif);
#endif

  return RETURN_OK;
}
