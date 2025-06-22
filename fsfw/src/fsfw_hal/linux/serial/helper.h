#ifndef FSFW_HAL_LINUX_UART_HELPER_H_
#define FSFW_HAL_LINUX_UART_HELPER_H_

#include <linux/serial.h>
#include <termios.h>

enum class Parity { NONE, EVEN, ODD };

enum class StopBits { ONE_STOP_BIT, TWO_STOP_BITS };

enum class UartModes { CANONICAL, NON_CANONICAL };

enum class BitsPerWord { BITS_5, BITS_6, BITS_7, BITS_8 };

enum class UartBaudRate {
  RATE_50,
  RATE_75,
  RATE_110,
  RATE_134,
  RATE_150,
  RATE_200,
  RATE_300,
  RATE_600,
  RATE_1200,
  RATE_1800,
  RATE_2400,
  RATE_4800,
  RATE_9600,
  RATE_19200,
  RATE_38400,
  RATE_57600,
  RATE_115200,
  RATE_230400,
  RATE_460800,
  RATE_500000,
  RATE_576000,
  RATE_921600,
  RATE_1000000,
  RATE_1152000,
  RATE_1500000,
  RATE_2000000,
  RATE_2500000,
  RATE_3000000,
  RATE_3500000,
  RATE_4000000
};

namespace uart {

void setMode(struct termios& options, UartModes mode);
/**
 * @brief   This functions adds the  baudrate specified in the uartCookie to the termios options
 *          struct.
 */
void setBaudrate(struct termios& options, UartBaudRate baud);

void setStopbits(struct termios& options, StopBits bits);

void setBitsPerWord(struct termios& options, BitsPerWord bits);

void enableRead(struct termios& options);

void setParity(struct termios& options, Parity parity);

void ignoreCtrlLines(struct termios& options);

int readCountersAndErrors(int serialPort, serial_icounter_struct& icounter);

}  // namespace uart

#endif /* FSFW_HAL_LINUX_UART_HELPER_H_ */
