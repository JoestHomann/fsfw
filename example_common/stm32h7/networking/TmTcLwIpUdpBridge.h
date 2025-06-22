#ifndef BSP_STM32_RTEMS_NETWORKING_TMTCUDPBRIDGE_H_
#define BSP_STM32_RTEMS_NETWORKING_TMTCUDPBRIDGE_H_

#include <lwip/ip_addr.h>
#include <lwip/udp.h>

#include "commonConfig.h"
#include "fsfw/tmtcservices/TmTcBridge.h"

/**
 * This bridge is used to forward TMTC packets received via LwIP UDP to the
 * internal software bus.
 */
class TmTcLwIpUdpBridge : public TmTcBridge {
  friend class UdpTcLwIpPollingTask;

 public:
  TmTcLwIpUdpBridge(object_id_t objectId, object_id_t ccsdsPacketDistributor, object_id_t tmStoreId,
                    object_id_t tcStoreId);
  ~TmTcLwIpUdpBridge() override;

  ReturnValue_t initialize() override;
  ReturnValue_t udp_server_init();

  /**
   * In addition to default implementation, ethernet link status is checked.
   * @param operationCode
   * @return
   */
  ReturnValue_t performOperation(uint8_t operationCode) override;

  /** TM Send implementation uses udp_send function from lwIP stack
   * @param data
   * @param dataLen
   * @return
   */
  ReturnValue_t sendTm(const uint8_t *data, size_t dataLen) override;

  /**
   * @brief This function is called when an UDP datagram has been
   * received on the port UDP_PORT.
   * @param arg
   * @param upcb_
   * @param p
   * @param addr Source address which will be bound to TmTcUdpBridge::lastAdd
   * @param port
   */
  static void udp_server_receive_callback(void *arg, struct udp_pcb *upcb_, struct pbuf *p,
                                          const ip_addr_t *addr, u16_t port);

  /**
   * Check whether the communication link is up.
   * Caller must ensure thread-safety by using the bridge lock.
   * @return
   */
  [[nodiscard]] bool comLinkUp() const;

 private:
  struct udp_pcb *upcb = nullptr;
  ip_addr_t lastAdd{};
  u16_t lastPort = 0;
  bool physicalConnection = false;
  MutexIF *bridgeLock = nullptr;

#if OBSW_TCPIP_UDP_WIRETAPPING == 1
  bool connectFlag = false;
#endif

  /**
   * Used to notify bridge about change in the physical ethernet connection.
   * Connection does not mean that replies are possible (recipient not set yet),
   * but disconnect means that we can't send anything. Caller must ensure
   * thread-safety by using the bridge lock.
   */
  void physicalConnectStatusChange(bool connect);
};

#endif /* BSP_STM32_RTEMS_NETWORKING_TMTCUDPBRIDGE_H_ */
