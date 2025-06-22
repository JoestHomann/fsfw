#pragma once

#include <fsfw/container/SimpleRingBuffer.h>
#include <fsfw/globalfunctions/DleEncoder.h>
#include <fsfw/returnvalues/returnvalue.h>

#include <cstddef>
#include <utility>

/**
 * @brief This base helper class can be used to extract DLE encoded packets from a data stream
 * @details
 * The core API of the parser takes received packets which can contains DLE packets. The parser
 * can deal with DLE packets split across multiple packets. It does so by using a dedicated
 * decoding ring buffer. The user can process received packets and detect errors by
 * overriding two provided virtual methods. This also allows detecting multiple DLE packets
 * inside one passed packet.
 */
class DleParser {
 public:
  static constexpr ReturnValue_t NO_PACKET_FOUND = returnvalue::makeCode(1, 1);
  static constexpr ReturnValue_t POSSIBLE_PACKET_LOSS = returnvalue::makeCode(1, 2);
  using BufPair = std::pair<uint8_t*, size_t>;

  enum class ContextType { NONE, PACKET_FOUND, ERROR };

  enum class ErrorTypes {
    NONE,
    ENCODED_BUF_TOO_SMALL,
    DECODING_BUF_TOO_SMALL,
    DECODE_ERROR,
    RING_BUF_ERROR,
    CONSECUTIVE_STX_CHARS,
    CONSECUTIVE_ETX_CHARS
  };

  union ErrorInfo {
    size_t len;
    ReturnValue_t res;
  };

  using ErrorPair = std::pair<ErrorTypes, ErrorInfo>;

  struct Context {
   public:
    Context() { setType(ContextType::PACKET_FOUND); }

    void setType(ContextType type) {
      this->type = type;
      if (type == ContextType::PACKET_FOUND) {
        error.first = ErrorTypes::NONE;
        error.second.len = 0;
      } else {
        decodedPacket.first = nullptr;
        decodedPacket.second = 0;
      }
    }

    ContextType getType() const { return type; }

    BufPair decodedPacket = {};
    ErrorPair error;

   private:
    ContextType type;
  };

  /**
   * Base class constructor
   * @param decodeRingBuf Ring buffer used to store multiple packets to allow detecting DLE packets
   * split across multiple packets
   * @param decoder Decoder instance
   * @param encodedBuf Buffer used to store encoded packets. It has to be large enough to hold
   * the largest expected encoded DLE packet size
   * @param decodedBuf Buffer used to store decoded packets. It has to be large enough to hold the
   * largest expected decoded DLE packet size
   * @param handler Function which will be called on a found packet
   * @param args Arbitrary user argument
   */
  DleParser(SimpleRingBuffer& decodeRingBuf, DleEncoder& decoder, BufPair encodedBuf,
            BufPair decodedBuf);

  /**
   * This function allows to pass new data into the parser. It then scans for DLE packets
   * automatically and inserts (part of) the packet into a ring buffer if necessary.
   * @param data
   * @param len
   * @return
   */
  ReturnValue_t passData(const uint8_t* data, size_t len);

  ReturnValue_t parseRingBuf(size_t& bytesRead);

  ReturnValue_t confirmBytesRead(size_t bytesRead);

  const Context& getContext();
  /**
   * Example found packet handler
   * function call
   * @param packet Decoded packet
   * @param len Length of detected packet
   */
  void defaultFoundPacketHandler(uint8_t* packet, size_t len, void* args);
  /**
   * Will be called if an error occured in the #passData call
   * @param err
   * @param ctx Context information depending on the error type
   *  - For buffer length errors, will be set to the detected packet length which is too large
   *  - For decode or ring buffer errors, will be set to the result returned from the failed call
   */
  void defaultErrorHandler();

  static void errorPrinter(const char* str, const char* opt = nullptr);

  void setErrorContext(ErrorTypes err, ErrorInfo ctx);
  /**
   * Resets the parser by resetting the internal states and clearing the decoding ring buffer
   */
  void reset();

 private:
  SimpleRingBuffer& decodeRingBuf;
  DleEncoder& decoder;
  BufPair encodedBuf;
  BufPair decodedBuf;
  Context ctx;
};
