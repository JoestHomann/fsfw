#ifndef BSP_STM32_BOARDTEST_STM32TESTTASK_H_
#define BSP_STM32_BOARDTEST_STM32TESTTASK_H_

#include "bsp_stm32h7_freertos/boardtest/SpiTest.h"
#include "fsfw_tests/integration/task/TestTask.h"

class STM32TestTask : public TestTask {
 public:
  STM32TestTask(object_id_t objectId, bool enablePrintout, bool blinkyLed = true);

  ReturnValue_t initialize() override;
  ReturnValue_t performPeriodicAction() override;

 private:
  SpiComIF *spiComIF = nullptr;
  SpiTest *spiTest = nullptr;

  bool blinkyLed = false;
  bool testSpi = false;
};

#endif /* BSP_STM32_BOARDTEST_STM32TESTTASK_H_ */
