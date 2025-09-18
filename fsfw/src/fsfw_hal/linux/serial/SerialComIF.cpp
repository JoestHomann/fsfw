#include "SerialComIF.h"

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cstring>

#include "fsfw/FSFW.h"
#include "fsfw/serviceinterface.h"
#include "fsfw_hal/linux/utility.h"

// --- JH START ---
#include <sys/ioctl.h>    // Needed to assert DTR/RTS: TIOCMGET, TIOCMSET, TIOCM_DTR, TIOCM_RTS
// --- JH END ---

// Debug On/Off switch for RW (set to 1 to enable debug output):
#define RW_VERBOSE 0

SerialComIF::SerialComIF(object_id_t objectId) : SystemObject(objectId) {}

SerialComIF::~SerialComIF() {}

ReturnValue_t SerialComIF::initializeInterface(CookieIF* cookie) {
  std::string deviceFile;

  if (cookie == nullptr) {
    return NULLPOINTER;
  }

  SerialCookie* uartCookie = dynamic_cast<SerialCookie*>(cookie);
  if (uartCookie == nullptr) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::error << "UartComIF::initializeInterface: Invalid UART Cookie!" << std::endl;
#endif
    return NULLPOINTER;
  }

  deviceFile = uartCookie->getDeviceFile();

  auto uartDeviceMapIter = uartDeviceMap.find(deviceFile);
  if (uartDeviceMapIter == uartDeviceMap.end()) {
    int fileDescriptor = configureUartPort(uartCookie);
    if (fileDescriptor < 0) {
      return returnvalue::FAILED;
    }
    size_t maxReplyLen = uartCookie->getMaxReplyLen();
    UartElements uartElements = {fileDescriptor, std::vector<uint8_t>(maxReplyLen), 0};
    auto status = uartDeviceMap.emplace(deviceFile, uartElements);
    if (status.second == false) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
      sif::warning << "UartComIF::initializeInterface: Failed to insert device " << deviceFile
                   << "to UART device map" << std::endl;
#endif
      return returnvalue::FAILED;
    }
  } else {
#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::warning << "UartComIF::initializeInterface: UART device " << deviceFile
                 << " already in use" << std::endl;
#endif
    return returnvalue::FAILED;
  }

  return returnvalue::OK;
}

int SerialComIF::configureUartPort(SerialCookie* uartCookie) {
  struct termios options = {};

  std::string deviceFile = uartCookie->getDeviceFile();
  int flags = O_RDWR;
  if (uartCookie->getUartMode() == UartModes::CANONICAL) {
    // In non-canonical mode, don't specify O_NONBLOCK because these properties will be
    // controlled by the VTIME and VMIN parameters and O_NONBLOCK would override this
    flags |= O_NONBLOCK;
  }
  int fd = open(deviceFile.c_str(), flags);

  if (fd < 0) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::warning << "UartComIF::configureUartPort: Failed to open uart " << deviceFile
                 << "with error code " << errno << strerror(errno) << std::endl;
#endif
    return fd;
  }

  /* Read in existing settings */
  if (tcgetattr(fd, &options) != 0) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::warning << "UartComIF::configureUartPort: Error " << errno
                 << "from tcgetattr: " << strerror(errno) << std::endl;
#endif
    return fd;
  }

  uart::setParity(options, uartCookie->getParity());
  setStopBitOptions(&options, uartCookie);
  setDatasizeOptions(&options, uartCookie);
  setFixedOptions(&options);
  uart::setMode(options, uartCookie->getUartMode());
  if (uartCookie->getInputShouldBeFlushed()) {
    tcflush(fd, TCIFLUSH);
  }

  /* Sets uart to non-blocking mode. Read returns immediately when there are no data available */
  options.c_cc[VTIME] = 0;
  options.c_cc[VMIN] = 0;

  uart::setBaudrate(options, uartCookie->getBaudrate());

  /* Save option settings */
  if (tcsetattr(fd, TCSANOW, &options) != 0) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::warning << "UartComIF::configureUartPort: Failed to set options with error " << errno
                 << ": " << strerror(errno);
#endif
    return fd;
  }
  
  // --- JH START ---
  // : Assert DTR/RTS so Arduino leaves while(!Serial) like pyserial does ---
  int mstat = 0;
  if (ioctl(fd, TIOCMGET, &mstat) == 0) {
    mstat |= TIOCM_DTR | TIOCM_RTS;         // raise DTR & RTS
    (void)ioctl(fd, TIOCMSET, &mstat);
  }
  // If your board resets on open, a short delay can help:
  usleep(500 *1000); // 500 ms
  // --- JH END ---

  return fd;
  
}


void SerialComIF::setStopBitOptions(struct termios* options, SerialCookie* uartCookie) {
  /* Clear stop field. Sets stop bit to one bit */
  options->c_cflag &= ~CSTOPB;
  switch (uartCookie->getStopBits()) {
    case StopBits::TWO_STOP_BITS:
      options->c_cflag |= CSTOPB;
      break;
    default:
      break;
  }
}

void SerialComIF::setDatasizeOptions(struct termios* options, SerialCookie* uartCookie) {
  /* Clear size bits */
  options->c_cflag &= ~CSIZE;
  switch (uartCookie->getBitsPerWord()) {
    case BitsPerWord::BITS_5:
      options->c_cflag |= CS5;
      break;
    case BitsPerWord::BITS_6:
      options->c_cflag |= CS6;
      break;
    case BitsPerWord::BITS_7:
      options->c_cflag |= CS7;
      break;
    case BitsPerWord::BITS_8:
      options->c_cflag |= CS8;
      break;
    default:
#if FSFW_CPP_OSTREAM_ENABLED == 1
      sif::warning << "UartComIF::setDatasizeOptions: Invalid size specified" << std::endl;
#endif
      break;
  }
}

void SerialComIF::setFixedOptions(struct termios* options) {
  /* Disable RTS/CTS hardware flow control */
  options->c_cflag &= ~CRTSCTS;
  /* Turn on READ & ignore ctrl lines (CLOCAL = 1) */
  options->c_cflag |= CREAD | CLOCAL;
  /* Disable echo */
  options->c_lflag &= ~ECHO;
  /* Disable erasure */
  options->c_lflag &= ~ECHOE;
  /* Disable new-line echo */
  options->c_lflag &= ~ECHONL;
  /* Disable interpretation of INTR, QUIT and SUSP */
  options->c_lflag &= ~ISIG;
  /* Turn off s/w flow ctrl */
  options->c_iflag &= ~(IXON | IXOFF | IXANY);
  /* Disable any special handling of received bytes */
  options->c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  /* Prevent special interpretation of output bytes (e.g. newline chars) */
  options->c_oflag &= ~OPOST;
  /* Prevent conversion of newline to carriage return/line feed */
  options->c_oflag &= ~ONLCR;
}

ReturnValue_t SerialComIF::sendMessage(CookieIF* cookie, const uint8_t* sendData, size_t sendLen) {
  int fd = 0;
  std::string deviceFile;

  if (sendLen == 0) {
    return returnvalue::OK;
  }

  if (sendData == nullptr) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::warning << "UartComIF::sendMessage: Send data is nullptr" << std::endl;
#endif
    return returnvalue::FAILED;
  }

  SerialCookie* uartCookie = dynamic_cast<SerialCookie*>(cookie);
  if (uartCookie == nullptr) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::warning << "UartComIF::sendMessasge: Invalid UART Cookie!" << std::endl;
#endif
    return NULLPOINTER;
  }

  deviceFile = uartCookie->getDeviceFile();
  auto uartDeviceMapIter = uartDeviceMap.find(deviceFile);
  if (uartDeviceMapIter == uartDeviceMap.end()) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::debug << "UartComIF::sendMessage: Device file " << deviceFile << "not in UART map"
               << std::endl;
#endif
    return returnvalue::FAILED;
  }

  fd = uartDeviceMapIter->second.fileDescriptor;

  if (write(fd, sendData, sendLen) != static_cast<int>(sendLen)) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::error << "UartComIF::sendMessage: Failed to send data with error code " << errno
               << ": Error description: " << strerror(errno) << std::endl;
#endif
    return returnvalue::FAILED;
  }

  // ---------- JH START DEBUG ------------
  #if RW_VERBOSE
  sif::info << "SerialComIF: write " << sendLen << " bytes" << std::endl;
  for(size_t i=0;i<sendLen;i++) { sif::info << std::hex << int(sendData[i]) << " "; }
  sif::info << std::dec << std::endl;
  #endif
  // ----------- JH END DEBUG -------------
  
  return returnvalue::OK;
}

ReturnValue_t SerialComIF::getSendSuccess(CookieIF* cookie) { return returnvalue::OK; }

ReturnValue_t SerialComIF::requestReceiveMessage(CookieIF* cookie, size_t requestLen) {
  std::string deviceFile;

  SerialCookie* uartCookie = dynamic_cast<SerialCookie*>(cookie);
  if (uartCookie == nullptr) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::debug << "UartComIF::requestReceiveMessage: Invalid Uart Cookie!" << std::endl;
#endif
    return NULLPOINTER;
  }

  UartModes uartMode = uartCookie->getUartMode();
  deviceFile = uartCookie->getDeviceFile();
  auto uartDeviceMapIter = uartDeviceMap.find(deviceFile);

  if (uartMode == UartModes::NON_CANONICAL and requestLen == 0) {
    return returnvalue::OK;
  }

  if (uartDeviceMapIter == uartDeviceMap.end()) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::debug << "UartComIF::requestReceiveMessage: Device file " << deviceFile
               << " not in uart map" << std::endl;
#endif
    return returnvalue::FAILED;
  }

  if (uartMode == UartModes::CANONICAL) {
    return handleCanonicalRead(*uartCookie, uartDeviceMapIter, requestLen);
  } else if (uartMode == UartModes::NON_CANONICAL) {
    return handleNoncanonicalRead(*uartCookie, uartDeviceMapIter, requestLen);
  } else {
    return returnvalue::FAILED;
  }
}

ReturnValue_t SerialComIF::handleCanonicalRead(SerialCookie& uartCookie,
                                               UartDeviceMap::iterator& iter, size_t requestLen) {
  ReturnValue_t result = returnvalue::OK;
  uint8_t maxReadCycles = uartCookie.getReadCycles();
  uint8_t currentReadCycles = 0;
  int bytesRead = 0;
  size_t currentBytesRead = 0;
  size_t maxReplySize = uartCookie.getMaxReplyLen();
  int fd = iter->second.fileDescriptor;
  auto bufferPtr = iter->second.replyBuffer.data();
  iter->second.replyLen = 0;
  do {
    size_t allowedReadSize = 0;
    if (currentBytesRead >= maxReplySize) {
      // Overflow risk. Emit warning, trigger event and break. If this happens,
      // the reception buffer is not large enough or data is not polled often enough.
#if FSFW_VERBOSE_LEVEL >= 1
#if FSFW_CPP_OSTREAM_ENABLED == 1
      sif::warning << "UartComIF::requestReceiveMessage: Next read would cause overflow!"
                   << std::endl;
#else
      sif::printWarning(
          "UartComIF::requestReceiveMessage: "
          "Next read would cause overflow!");
#endif
#endif
      result = UART_RX_BUFFER_TOO_SMALL;
      break;
    } else {
      allowedReadSize = maxReplySize - currentBytesRead;
    }

    bytesRead = read(fd, bufferPtr, allowedReadSize);
    if (bytesRead < 0) {
      // EAGAIN: No data available in non-blocking mode
      if (errno != EAGAIN) {
#if FSFW_VERBOSE_LEVEL >= 1
#if FSFW_CPP_OSTREAM_ENABLED == 1
        sif::warning << "UartComIF::handleCanonicalRead: read failed with code" << errno << ": "
                     << strerror(errno) << std::endl;
#else
        sif::printWarning("UartComIF::handleCanonicalRead: read failed with code %d: %s\n", errno,
                          strerror(errno));
#endif
#endif
        return returnvalue::FAILED;
      }

    } else if (bytesRead > 0) {
      iter->second.replyLen += bytesRead;
      bufferPtr += bytesRead;
      currentBytesRead += bytesRead;
    }
    currentReadCycles++;
  } while (bytesRead > 0 and currentReadCycles < maxReadCycles);
  return result;
}

ReturnValue_t SerialComIF::handleNoncanonicalRead(SerialCookie& uartCookie,
                                                  UartDeviceMap::iterator& iter,
                                                  size_t requestLen) {
  int fd = iter->second.fileDescriptor;
  auto bufferPtr = iter->second.replyBuffer.data();
  // Size check to prevent buffer overflow
  if (requestLen > uartCookie.getMaxReplyLen()) {
#if FSFW_VERBOSE_LEVEL >= 1
#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::warning << "UartComIF::requestReceiveMessage: Next read would cause overflow!"
                 << std::endl;
#else
    sif::printWarning(
        "UartComIF::requestReceiveMessage: "
        "Next read would cause overflow!");
#endif
#endif
    return UART_RX_BUFFER_TOO_SMALL;
  }
  int bytesRead = read(fd, bufferPtr, requestLen);
  if (bytesRead < 0) {
    return returnvalue::FAILED;
  } else if (bytesRead != static_cast<int>(requestLen)) {
    if (uartCookie.isReplySizeFixed()) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
      sif::warning << "UartComIF::requestReceiveMessage: Only read " << bytesRead << " of "
                   << requestLen << " bytes" << std::endl;
#endif
      return returnvalue::FAILED;
    }
  }
  iter->second.replyLen = bytesRead;
  return returnvalue::OK;
}

ReturnValue_t SerialComIF::readReceivedMessage(CookieIF* cookie, uint8_t** buffer, size_t* size) {
  std::string deviceFile;

  SerialCookie* uartCookie = dynamic_cast<SerialCookie*>(cookie);
  if (uartCookie == nullptr) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::debug << "UartComIF::readReceivedMessage: Invalid uart cookie!" << std::endl;
#endif
    return NULLPOINTER;
  }

  deviceFile = uartCookie->getDeviceFile();
  auto uartDeviceMapIter = uartDeviceMap.find(deviceFile);
  if (uartDeviceMapIter == uartDeviceMap.end()) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::debug << "UartComIF::readReceivedMessage: Device file " << deviceFile << " not in uart map"
               << std::endl;
#endif
    return returnvalue::FAILED;
  }

  *buffer = uartDeviceMapIter->second.replyBuffer.data();
  *size = uartDeviceMapIter->second.replyLen;

  /* Length is reset to 0 to prevent reading the same data twice */
  uartDeviceMapIter->second.replyLen = 0;

  // ---------- JH START DEBUG ------------
  #if RW_VERBOSE
  if (*size > 0) {
    sif::info << "SerialComIF: read " << *size << " bytes" << std::endl;
    for(size_t i=0;i<*size;i++) { sif::info << std::hex << int((*buffer)[i]) << " "; }
    sif::info << std::dec << std::endl;
  }
#endif
// ----------- JH END DEBUG -------------
  return returnvalue::OK;
}

ReturnValue_t SerialComIF::flushUartRxBuffer(CookieIF* cookie) {
  std::string deviceFile;
  SerialCookie* uartCookie = dynamic_cast<SerialCookie*>(cookie);
  if (uartCookie == nullptr) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::warning << "UartComIF::flushUartRxBuffer: Invalid uart cookie!" << std::endl;
#endif
    return NULLPOINTER;
  }
  deviceFile = uartCookie->getDeviceFile();
  auto uartDeviceMapIter = uartDeviceMap.find(deviceFile);
  if (uartDeviceMapIter != uartDeviceMap.end()) {
    int fd = uartDeviceMapIter->second.fileDescriptor;
    tcflush(fd, TCIFLUSH);
    return returnvalue::OK;
  }
  return returnvalue::FAILED;
}

ReturnValue_t SerialComIF::flushUartTxBuffer(CookieIF* cookie) {
  std::string deviceFile;
  SerialCookie* uartCookie = dynamic_cast<SerialCookie*>(cookie);
  if (uartCookie == nullptr) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::warning << "UartComIF::flushUartTxBuffer: Invalid uart cookie!" << std::endl;
#endif
    return NULLPOINTER;
  }
  deviceFile = uartCookie->getDeviceFile();
  auto uartDeviceMapIter = uartDeviceMap.find(deviceFile);
  if (uartDeviceMapIter != uartDeviceMap.end()) {
    int fd = uartDeviceMapIter->second.fileDescriptor;
    tcflush(fd, TCOFLUSH);
    return returnvalue::OK;
  }
  return returnvalue::FAILED;
}

ReturnValue_t SerialComIF::flushUartTxAndRxBuf(CookieIF* cookie) {
  std::string deviceFile;
  SerialCookie* uartCookie = dynamic_cast<SerialCookie*>(cookie);
  if (uartCookie == nullptr) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::warning << "UartComIF::flushUartTxAndRxBuf: Invalid uart cookie!" << std::endl;
#endif
    return NULLPOINTER;
  }
  deviceFile = uartCookie->getDeviceFile();
  auto uartDeviceMapIter = uartDeviceMap.find(deviceFile);
  if (uartDeviceMapIter != uartDeviceMap.end()) {
    int fd = uartDeviceMapIter->second.fileDescriptor;
    tcflush(fd, TCIOFLUSH);
    return returnvalue::OK;
  }
  return returnvalue::FAILED;
}
