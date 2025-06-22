#include <fsfw_hal/linux/serial/helper.h>
#include <sys/ioctl.h>

#include "fsfw/serviceinterface.h"

void uart::setMode(struct termios& options, UartModes mode) {
  if (mode == UartModes::NON_CANONICAL) {
    /* Disable canonical mode */
    options.c_lflag &= ~ICANON;
  } else if (mode == UartModes::CANONICAL) {
    options.c_lflag |= ICANON;
  }
}

void uart::setBaudrate(struct termios& options, UartBaudRate baud) {
  switch (baud) {
    case UartBaudRate::RATE_50:
      cfsetspeed(&options, B50);
      break;
    case UartBaudRate::RATE_75:
      cfsetspeed(&options, B75);
      break;
    case UartBaudRate::RATE_110:
      cfsetspeed(&options, B110);
      break;
    case UartBaudRate::RATE_134:
      cfsetspeed(&options, B134);
      break;
    case UartBaudRate::RATE_150:
      cfsetspeed(&options, B150);
      break;
    case UartBaudRate::RATE_200:
      cfsetspeed(&options, B200);
      break;
    case UartBaudRate::RATE_300:
      cfsetspeed(&options, B300);
      break;
    case UartBaudRate::RATE_600:
      cfsetspeed(&options, B600);
      break;
    case UartBaudRate::RATE_1200:
      cfsetspeed(&options, B1200);
      break;
    case UartBaudRate::RATE_1800:
      cfsetspeed(&options, B1800);
      break;
    case UartBaudRate::RATE_2400:
      cfsetspeed(&options, B2400);
      break;
    case UartBaudRate::RATE_4800:
      cfsetspeed(&options, B4800);
      break;
    case UartBaudRate::RATE_9600:
      cfsetspeed(&options, B9600);
      break;
    case UartBaudRate::RATE_19200:
      cfsetspeed(&options, B19200);
      break;
    case UartBaudRate::RATE_38400:
      cfsetspeed(&options, B38400);
      break;
    case UartBaudRate::RATE_57600:
      cfsetspeed(&options, B57600);
      break;
    case UartBaudRate::RATE_115200:
      cfsetspeed(&options, B115200);
      break;
    case UartBaudRate::RATE_230400:
      cfsetspeed(&options, B230400);
      break;
#ifndef __APPLE__
    case UartBaudRate::RATE_460800:
      cfsetspeed(&options, B460800);
      break;
    case UartBaudRate::RATE_500000:
      cfsetspeed(&options, B500000);
      break;
    case UartBaudRate::RATE_576000:
      cfsetspeed(&options, B576000);
      break;
    case UartBaudRate::RATE_921600:
      cfsetspeed(&options, B921600);
      break;
    case UartBaudRate::RATE_1000000:
      cfsetspeed(&options, B1000000);
      break;
    case UartBaudRate::RATE_1152000:
      cfsetspeed(&options, B1152000);
      break;
    case UartBaudRate::RATE_1500000:
      cfsetspeed(&options, B1500000);
      break;
    case UartBaudRate::RATE_2000000:
      cfsetspeed(&options, B2000000);
      break;
    case UartBaudRate::RATE_2500000:
      cfsetspeed(&options, B2500000);
      break;
    case UartBaudRate::RATE_3000000:
      cfsetspeed(&options, B3000000);
      break;
    case UartBaudRate::RATE_3500000:
      cfsetspeed(&options, B3500000);
      break;
    case UartBaudRate::RATE_4000000:
      cfsetspeed(&options, B4000000);
      break;
#endif  // ! __APPLE__
    default:
#if FSFW_CPP_OSTREAM_ENABLED == 1
      sif::warning << "UartComIF::configureBaudrate: Baudrate not supported" << std::endl;
#endif
      break;
  }
}

void uart::setBitsPerWord(struct termios& options, BitsPerWord bits) {
  options.c_cflag &= ~CSIZE;  // Clear all the size bits
  if (bits == BitsPerWord::BITS_5) {
    options.c_cflag |= CS5;
  } else if (bits == BitsPerWord::BITS_6) {
    options.c_cflag |= CS6;
  } else if (bits == BitsPerWord::BITS_7) {
    options.c_cflag |= CS7;
  } else if (bits == BitsPerWord::BITS_8) {
    options.c_cflag |= CS8;
  }
}

void uart::enableRead(struct termios& options) { options.c_cflag |= CREAD; }

void uart::ignoreCtrlLines(struct termios& options) { options.c_cflag |= CLOCAL; }

void uart::setParity(struct termios& options, Parity parity) {
  /* Clear parity bit */
  options.c_cflag &= ~PARENB;
  switch (parity) {
    case Parity::EVEN:
      options.c_cflag |= PARENB;
      options.c_cflag &= ~PARODD;
      break;
    case Parity::ODD:
      options.c_cflag |= PARENB;
      options.c_cflag |= PARODD;
      break;
    default:
      break;
  }
}

int uart::readCountersAndErrors(int serialPort, serial_icounter_struct& icounter) {
  return ioctl(serialPort, TIOCGICOUNT, &icounter);
}

void uart::setStopbits(struct termios& options, StopBits bits) {
  if (bits == StopBits::TWO_STOP_BITS) {
    // Use two stop bits
    options.c_cflag |= CSTOPB;
  } else {
    // Clear stop field, only one stop bit used in communication
    options.c_cflag &= ~CSTOPB;
  }
}
