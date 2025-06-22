#include "SerialCookie.h"

#include <fsfw/serviceinterface.h>

SerialCookie::SerialCookie(object_id_t handlerId, std::string deviceFile, UartBaudRate baudrate,
                           size_t maxReplyLen, UartModes uartMode)
    : handlerId(handlerId),
      deviceFile(deviceFile),
      uartMode(uartMode),
      baudrate(baudrate),
      maxReplyLen(maxReplyLen) {}

SerialCookie::~SerialCookie() {}

UartBaudRate SerialCookie::getBaudrate() const { return baudrate; }

size_t SerialCookie::getMaxReplyLen() const { return maxReplyLen; }

std::string SerialCookie::getDeviceFile() const { return deviceFile; }

void SerialCookie::setParityOdd() { parity = Parity::ODD; }

void SerialCookie::setParityEven() { parity = Parity::EVEN; }

Parity SerialCookie::getParity() const { return parity; }

void SerialCookie::setBitsPerWord(BitsPerWord bitsPerWord_) { bitsPerWord = bitsPerWord_; }

BitsPerWord SerialCookie::getBitsPerWord() const { return bitsPerWord; }

StopBits SerialCookie::getStopBits() const { return stopBits; }

void SerialCookie::setTwoStopBits() { stopBits = StopBits::TWO_STOP_BITS; }

void SerialCookie::setOneStopBit() { stopBits = StopBits::ONE_STOP_BIT; }

UartModes SerialCookie::getUartMode() const { return uartMode; }

void SerialCookie::setReadCycles(uint8_t readCycles) { this->readCycles = readCycles; }

void SerialCookie::setToFlushInput(bool enable) { this->flushInput = enable; }

uint8_t SerialCookie::getReadCycles() const { return readCycles; }

bool SerialCookie::getInputShouldBeFlushed() { return this->flushInput; }

object_id_t SerialCookie::getHandlerId() const { return this->handlerId; }

void SerialCookie::setNoFixedSizeReply() { replySizeFixed = false; }

bool SerialCookie::isReplySizeFixed() { return replySizeFixed; }
