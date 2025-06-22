#pragma once

#include <fsfw/objectmanager/SystemObject.h>
#include <fsfw/returnvalues/HasReturnvaluesIF.h>
#include <fsfw/tasks/ExecutableObjectIF.h>
#include <lwip/netif.h>

class TmTcLwIpUdpBridge;

/**
 * @brief Separate task to poll EMAC interface.
 * 		  Polled data is passed to the netif (lwIP)
 */
class UdpTcLwIpPollingTask : public SystemObject,
                             public ExecutableObjectIF,
                             public HasReturnvaluesIF {
 public:
  UdpTcLwIpPollingTask(object_id_t objectId, object_id_t bridgeId, struct netif *gnetif);
  ~UdpTcLwIpPollingTask() override;

  ReturnValue_t initialize() override;

  /**
   * Executed periodically.
   * @param operationCode
   * @return
   */
  ReturnValue_t performOperation(uint8_t operationCode) override;

 private:
  static const uint8_t PERIODIC_HANDLE_TRIGGER = 5;
  uint8_t periodicHandleCounter;
  object_id_t bridgeId = 0;
  TmTcLwIpUdpBridge *udpBridge = nullptr;
  struct netif *gnetif = nullptr;
};
