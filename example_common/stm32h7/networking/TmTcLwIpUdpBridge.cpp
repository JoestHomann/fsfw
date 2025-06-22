#include "TmTcLwIpUdpBridge.h"

#include <fsfw/ipc/MutexGuard.h>
#include <fsfw/serialize/EndianConverter.h>
#include <fsfw/serviceinterface/ServiceInterface.h>

#include "app_ethernet.h"
#include "udp_config.h"

TmTcLwIpUdpBridge::TmTcLwIpUdpBridge(object_id_t objectId, object_id_t ccsdsPacketDistributor,
                                     object_id_t tmStoreId, object_id_t tcStoreId)
    : TmTcBridge(objectId, ccsdsPacketDistributor, tmStoreId, tcStoreId) {
  TmTcLwIpUdpBridge::lastAdd.addr = IPADDR_TYPE_ANY;
}

TmTcLwIpUdpBridge::~TmTcLwIpUdpBridge() = default;

ReturnValue_t TmTcLwIpUdpBridge::initialize() {
  TmTcBridge::initialize();
  bridgeLock = MutexFactory::instance()->createMutex();
  if (bridgeLock == nullptr) {
    return ObjectManagerIF::CHILD_INIT_FAILED;
  }
  ReturnValue_t result = udp_server_init();
  return result;
}

ReturnValue_t TmTcLwIpUdpBridge::udp_server_init() {
  err_t err;
  /* Create a new UDP control block  */
  TmTcLwIpUdpBridge::upcb = udp_new();
  if (TmTcLwIpUdpBridge::upcb) {
    sif::printInfo("Opening UDP server on port %d\n", UDP_SERVER_PORT);
    /* Bind the upcb to the UDP_PORT port */
    /* Using IP_ADDR_ANY allow the upcb to be used by any local interface */
    err = udp_bind(TmTcLwIpUdpBridge::upcb, IP_ADDR_ANY, UDP_SERVER_PORT);

    if (err == ERR_OK) {
      /* Set a receive callback for the upcb */
      udp_recv(TmTcLwIpUdpBridge::upcb, &udp_server_receive_callback, (void *)this);
      return RETURN_OK;
    } else {
      udp_remove(TmTcLwIpUdpBridge::upcb);
      return RETURN_FAILED;
    }
  } else {
    return RETURN_FAILED;
  }
}

ReturnValue_t TmTcLwIpUdpBridge::performOperation(uint8_t operationCode) {
  TmTcBridge::performOperation();

#if OBSW_TCPIP_UDP_WIRETAPPING == 1
  if (connectFlag) {
    uint32_t ipAddress = ((ip4_addr *)&lastAdd)->addr;
    int ipAddress1 = (ipAddress & 0xFF000000) >> 24;
    int ipAddress2 = (ipAddress & 0xFF0000) >> 16;
    int ipAddress3 = (ipAddress & 0xFF00) >> 8;
    int ipAddress4 = ipAddress & 0xFF;
#if OBSW_VERBOSE_LEVEL == 1
#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::info << "TmTcLwIpUdpBridge: Client IP Address " << std::dec << ipAddress4 << "."
              << ipAddress3 << "." << ipAddress2 << "." << ipAddress1 << std::endl;
    uint16_t portSwapped = EndianConverter::convertBigEndian(lastPort);
    sif::info << "TmTcLwIpUdpBridge: Client IP Port " << (int)portSwapped << std::endl;
#else
    sif::printInfo("TmTcLwIpUdpBridge: Client IP Address %d.%d.%d.%d\n", ipAddress4, ipAddress3,
                   ipAddress2, ipAddress1);
    uint16_t portSwapped = EndianConverter::convertBigEndian(lastPort);
    sif::printInfo("TmTcLwIpUdpBridge: Client IP Port: %d\n", portSwapped);
#endif
#endif
    connectFlag = false;
  }
#endif

  return RETURN_OK;
}

ReturnValue_t TmTcLwIpUdpBridge::sendTm(const uint8_t *data, size_t dataLen) {
  struct pbuf *p_tx = pbuf_alloc(PBUF_TRANSPORT, dataLen, PBUF_RAM);
  if ((p_tx != nullptr) && (lastAdd.addr != IPADDR_TYPE_ANY) && (upcb != nullptr)) {
    /* copy data to pbuf */
    err_t err = pbuf_take(p_tx, (const char *)data, dataLen);
    if (err != ERR_OK) {
      pbuf_free(p_tx);
      return err;
    }
    /* Connect to the remote client */
    err = udp_connect(TmTcLwIpUdpBridge::upcb, &lastAdd, lastPort);
    if (err != ERR_OK) {
      pbuf_free(p_tx);
      return err;
    }
    /* Tell the client that we have accepted it */
    err = udp_send(TmTcLwIpUdpBridge::upcb, p_tx);
    pbuf_free(p_tx);
    if (err != ERR_OK) {
      return err;
    }

    /* free the UDP connection, so we can accept new clients */
    udp_disconnect(TmTcLwIpUdpBridge::upcb);
  } else {
    return RETURN_FAILED;
  }
  return RETURN_OK;
}

void TmTcLwIpUdpBridge::udp_server_receive_callback(void *arg, struct udp_pcb *upcb_,
                                                    struct pbuf *p, const ip_addr_t *addr,
                                                    u16_t port) {
  auto udpBridge = reinterpret_cast<TmTcLwIpUdpBridge *>(arg);
  if (udpBridge == nullptr) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::warning << "TmTcLwIpUdpBridge::udp_server_receive_callback: Invalid UDP bridge!"
                 << std::endl;
#else
    sif::printWarning(
        "TmTcLwIpUdpBridge::udp_server_receive_callback: Invalid "
        "UDP bridge!\n");
#endif
  }
  /* allocate pbuf from RAM*/
  struct pbuf *p_tx = pbuf_alloc(PBUF_TRANSPORT, p->len, PBUF_RAM);

  if (p_tx != nullptr) {
    if (udpBridge != nullptr) {
      MutexGuard lg(udpBridge->bridgeLock);
      udpBridge->upcb = upcb_;
      udpBridge->lastAdd = *addr;
      udpBridge->lastPort = port;
      if (not udpBridge->comLinkUp()) {
        udpBridge->registerCommConnect();
#if OBSW_TCPIP_UDP_WIRETAPPING == 1
        udpBridge->connectFlag = true;
#endif
        /* This should have already been done, but we will still do it */
        udpBridge->physicalConnectStatusChange(true);
      }
    }
    pbuf_take(p_tx, (char *)p->payload, p->len);
    /* send the received data to the uart port */
    char *data = reinterpret_cast<char *>(p_tx->payload);
    *(data + p_tx->len) = '\0';

#if OBSW_TCPIP_UDP_WIRETAPPING == 1
    udpBridge->printData(reinterpret_cast<uint8_t *>(p->payload), p->len);
#endif

    store_address_t storeId;
    ReturnValue_t returnValue =
        udpBridge->tcStore->addData(&storeId, reinterpret_cast<uint8_t *>(p->payload), p->len);
    if (returnValue != RETURN_OK) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
      sif::warning << "UDP Server: Data storage failed" << std::endl;
#endif
      pbuf_free(p_tx);
      return;
    }
    TmTcMessage message(storeId);
    if (udpBridge->tmTcReceptionQueue->sendToDefault(&message) != RETURN_OK) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
      sif::warning << "TmTcLwIpUdpBridgw::udp_server_receive_callback:"
                   << " Sending message to queue failed" << std::endl;
#endif
      udpBridge->tcStore->deleteData(storeId);
    }
  }
  /* Free the p_tx buffer */
  pbuf_free(p_tx);
}

/* Caller must ensure thread-safety */
bool TmTcLwIpUdpBridge::comLinkUp() const { return communicationLinkUp; }

/* Caller must ensure thread-safety */
void TmTcLwIpUdpBridge::physicalConnectStatusChange(bool connect) {
  if (connect) {
    /* Physical connection does not mean there is a recipient to send packets
    too. This will be done by the receive callback! */
    physicalConnection = true;
  } else {
    physicalConnection = false;
    /* If there is no physical connection, we can't send anything back */
    registerCommDisconnect();
  }
}
