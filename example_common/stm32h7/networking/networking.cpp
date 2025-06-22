#include "networking.h"

#include "udp_config.h"

bool ethernetCableConnected = false;

void networking::setEthCableConnected(bool status) { ethernetCableConnected = status; }

bool networking::getEthCableConnected() { return ethernetCableConnected; }

void networking::setLwipAddresses(ip_addr_t *ipaddr, ip_addr_t *netmask, ip_addr_t *gw) {
  IP4_ADDR(ipaddr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
  IP4_ADDR(netmask, NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);
  IP4_ADDR(gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
}
