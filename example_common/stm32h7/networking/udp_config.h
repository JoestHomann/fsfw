#ifndef COMMON_STM32_NUCLEO_NETWORKING_UDP_CONFIG_H_
#define COMMON_STM32_NUCLEO_NETWORKING_UDP_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

/* UDP local connection port. Client needs to bind to this port */
#define UDP_SERVER_PORT 7

/*Static DEST IP ADDRESS:
 * DEST_IP_ADDR0.DEST_IP_ADDR1.DEST_IP_ADDR2.DEST_IP_ADDR3 */
#define DEST_IP_ADDR0 ((uint8_t)169U)
#define DEST_IP_ADDR1 ((uint8_t)254U)
#define DEST_IP_ADDR2 ((uint8_t)39U)
#define DEST_IP_ADDR3 ((uint8_t)2U)

/*Static IP ADDRESS*/
#define IP_ADDR0 169
#define IP_ADDR1 254
#define IP_ADDR2 1
#define IP_ADDR3 38

/*NETMASK*/
#define NETMASK_ADDR0 255
#define NETMASK_ADDR1 255
#define NETMASK_ADDR2 0
#define NETMASK_ADDR3 0

/*Gateway Address*/
#define GW_ADDR0 192
#define GW_ADDR1 168
#define GW_ADDR2 178
#define GW_ADDR3 1

#ifdef __cplusplus
}
#endif

#endif /* COMMON_STM32_NUCLEO_NETWORKING_UDP_CONFIG_H_ */
