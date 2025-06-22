# These are part of the RTEMS BSP for RTEMS
if(FSFW_OSAL MATCHES freertos)
  target_sources(${TARGET_NAME} PRIVATE ethernetif.c)
endif()

target_sources(
  ${TARGET_NAME} PRIVATE UdpTcLwIpPollingTask.cpp TmTcLwIpUdpBridge.cpp
                         networking.cpp app_dhcp.cpp app_ethernet.cpp)
