/* Includes ------------------------------------------------------------------*/
#include "app_ethernet.h"

#include "ethernetif.h"
#include "networking.h"
#include "udp_config.h"

#if LWIP_DHCP
#include "app_dhcp.h"
#endif

#include <OBSWConfig.h>
#include <lwip/netif.h>
#include <lwipopts.h>
#include <stm32h7xx_nucleo.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t ethernetLinkTimer = 0;

/* Private function prototypes -----------------------------------------------*/
void handle_status_change(struct netif *netif, bool link_up);

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Notify the User about the network interface config status
 * @param  netif: the network interface
 * @retval None
 */
void networking::ethernetLinkStatusUpdated(struct netif *netif) {
  if (netif_is_link_up(netif)) {
    networking::setEthCableConnected(true);
    handle_status_change(netif, true);
  } else {
    networking::setEthCableConnected(false);
    handle_status_change(netif, false);
  }
}

void handle_status_change(struct netif *netif, bool link_up) {
  if (link_up) {
#if LWIP_DHCP
    /* Update DHCP state machine */
    set_dhcp_state(DHCP_START);
#else
    uint8_t iptxt[20];
    sprintf((char *)iptxt, "%s", ip4addr_ntoa(netif_ip4_addr(netif)));
    printf("\rNetwork cable connected. Static IP address: %s | Port: %d\n\r", iptxt,
           UDP_SERVER_PORT);
#if OBSW_ETHERNET_USE_LED1_LED2 == 1
    BSP_LED_On(LED1);
    BSP_LED_Off(LED2);
#endif
#endif /* LWIP_DHCP */
  } else {
    printf("Network cable disconnected\n\r");
#if LWIP_DHCP
    /* Update DHCP state machine */
    set_dhcp_state(DHCP_LINK_DOWN);
#else
#if OBSW_ETHERNET_USE_LED1_LED2 == 1
    BSP_LED_Off(LED1);
    BSP_LED_On(LED2);
#endif
#endif /* LWIP_DHCP */
  }
}

#if LWIP_NETIF_LINK_CALLBACK

/**
 * @brief  Ethernet Link periodic check
 * @param  netif
 * @retval None
 */
void networking::ethernetLinkPeriodicHandle(struct netif *netif) {
  /* Ethernet Link every 100ms */
  if (HAL_GetTick() - ethernetLinkTimer >= 100) {
    ethernetLinkTimer = HAL_GetTick();
    ethernet_link_check_state(netif);
  }
}

#endif /* LWIP_NETIF_LINK_CALLBACK */
