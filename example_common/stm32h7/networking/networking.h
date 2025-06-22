#ifndef BSP_STM32H7_RTEMS_NETWORKING_NETWORKING_H_
#define BSP_STM32H7_RTEMS_NETWORKING_NETWORKING_H_

#include <lwip/netif.h>

namespace networking {

void setEthCableConnected(bool status);
bool getEthCableConnected();
void setLwipAddresses(ip_addr_t *ipaddr, ip_addr_t *netmask, ip_addr_t *gw);

}  // namespace networking

#endif /* BSP_STM32H7_RTEMS_NETWORKING_NETWORKING_H_ */
