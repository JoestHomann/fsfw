#ifndef SAM9G20_COMIF_COOKIES_UART_COOKIE_H_
#define SAM9G20_COMIF_COOKIES_UART_COOKIE_H_

#include <fsfw/devicehandlers/CookieIF.h>
#include <fsfw/objectmanager/SystemObjectIF.h>
#include <fsfw_hal/linux/serial/helper.h>

#include <string>

/**
 * @brief   Cookie for the UartComIF. There are many options available to configure the UART driver.
 *          The constructor only requests for common options like the baudrate. Other options can
 *          be set by member functions.
 *
 * @author 	J. Meier
 */
class SerialCookie : public CookieIF {
 public:
  /**
   * @brief	Constructor for the uart cookie.
   * @param deviceFile    The device file specifying the uart to use, e.g. "/dev/ttyPS1"
   * @param uartMode      Specify the UART mode. The canonical mode should be used if the
   *                      messages are separated by a delimited character like '\n'. See the
   *                      termios documentation for more information
   * @param baudrate      The baudrate to use for input and output.
   * @param maxReplyLen   The maximum size an object using this cookie expects
   * @details
   * Default configuration: No parity
   *                        8 databits (number of bits transfered with one uart frame)
   *                        One stop bit
   */
  SerialCookie(object_id_t handlerId, std::string deviceFile, UartBaudRate baudrate,
               size_t maxReplyLen, UartModes uartMode = UartModes::NON_CANONICAL);

  virtual ~SerialCookie();

  UartBaudRate getBaudrate() const;
  size_t getMaxReplyLen() const;
  std::string getDeviceFile() const;
  Parity getParity() const;
  BitsPerWord getBitsPerWord() const;
  StopBits getStopBits() const;
  UartModes getUartMode() const;
  object_id_t getHandlerId() const;

  /**
   * The UART ComIF will only perform a specified number of read cycles for the canonical mode.
   * The user can specify how many of those read cycles are performed for one device handler
   * communication cycle. An example use-case would be to read all available GPS NMEA strings
   * at once.
   * @param readCycles
   */
  void setReadCycles(uint8_t readCycles);
  uint8_t getReadCycles() const;

  /**
   * Allows to flush the data which was received but has not been read yet. This is useful
   * to discard obsolete data at software startup.
   */
  void setToFlushInput(bool enable);
  bool getInputShouldBeFlushed();

  /**
   * Functions two enable parity checking.
   */
  void setParityOdd();
  void setParityEven();

  /**
   * Function two set number of bits per UART frame.
   */
  void setBitsPerWord(BitsPerWord bitsPerWord_);

  /**
   * Function to specify the number of stopbits.
   */
  void setTwoStopBits();
  void setOneStopBit();

  /**
   * Calling this function prevents the UartComIF to return failed if not all requested bytes
   * could be read. This is required by a device handler when the size of a reply is not known.
   */
  void setNoFixedSizeReply();

  bool isReplySizeFixed();

 private:
  const object_id_t handlerId;
  std::string deviceFile;
  const UartModes uartMode;
  bool flushInput = false;
  UartBaudRate baudrate;
  size_t maxReplyLen = 0;
  Parity parity = Parity::NONE;
  BitsPerWord bitsPerWord = BitsPerWord::BITS_8;
  uint8_t readCycles = 1;
  StopBits stopBits = StopBits::ONE_STOP_BIT;
  bool replySizeFixed = true;
};

#endif
