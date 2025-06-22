#include "STM32TestTask.h"

#include "OBSWConfig.h"
#include "stm32h7xx_nucleo.h"

STM32TestTask::STM32TestTask(object_id_t objectId, bool enablePrintout, bool blinkyLed)
    : TestTask(objectId), blinkyLed(blinkyLed) {
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED2);
  BSP_LED_Init(LED3);
}

ReturnValue_t STM32TestTask::initialize() {
  if (testSpi) {
    spiComIF = new SpiComIF(objects::SPI_COM_IF);
    spiTest = new SpiTest(*spiComIF);
  }
  return TestTask::initialize();
}

ReturnValue_t STM32TestTask::performPeriodicAction() {
  if (blinkyLed) {
#if OBSW_ETHERNET_USE_LED1_LED2 == 0
    BSP_LED_Toggle(LED1);
    BSP_LED_Toggle(LED2);
#endif
    BSP_LED_Toggle(LED3);
  }
  if (testSpi) {
    spiTest->performOperation();
  }
  return TestTask::performPeriodicAction();
}
