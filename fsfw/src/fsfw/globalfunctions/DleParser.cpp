#include "DleParser.h"

#include <fsfw/serviceinterface/ServiceInterface.h>

#include <cstdio>
#include <fstream>
#include <iostream>

DleParser::DleParser(SimpleRingBuffer& decodeRingBuf, DleEncoder& decoder, BufPair encodedBuf,
                     BufPair decodedBuf)
    : decodeRingBuf(decodeRingBuf),
      decoder(decoder),
      encodedBuf(encodedBuf),
      decodedBuf(decodedBuf) {}

ReturnValue_t DleParser::passData(const uint8_t* data, size_t len) {
  if (data == nullptr or len == 0) {
    return returnvalue::FAILED;
  }
  return decodeRingBuf.writeData(data, len);
}

ReturnValue_t DleParser::parseRingBuf(size_t& readSize) {
  ctx.setType(DleParser::ContextType::NONE);
  size_t availableData = decodeRingBuf.getAvailableReadData();
  if (availableData == 0) {
    return NO_PACKET_FOUND;
  }
  if (availableData > encodedBuf.second) {
    ErrorInfo info;
    info.len = decodeRingBuf.getAvailableReadData();
    setErrorContext(ErrorTypes::DECODING_BUF_TOO_SMALL, info);
    return returnvalue::FAILED;
  }
  ReturnValue_t result = decodeRingBuf.readData(encodedBuf.first, availableData);
  if (result != returnvalue::OK) {
    ErrorInfo info;
    info.res = result;
    setErrorContext(ErrorTypes::RING_BUF_ERROR, info);
    return result;
  }
  bool stxFound = false;
  size_t stxIdx = 0;
  for (size_t vectorIdx = 0; vectorIdx < availableData; vectorIdx++) {
    // handle STX char
    if (encodedBuf.first[vectorIdx] == DleEncoder::STX_CHAR) {
      if (not stxFound) {
        stxFound = true;
        stxIdx = vectorIdx;
      } else {
        // might be lost packet, so we should advance the read pointer
        // without skipping the STX
        readSize = vectorIdx;
        ErrorInfo info;
        setErrorContext(ErrorTypes::CONSECUTIVE_STX_CHARS, info);
        return POSSIBLE_PACKET_LOSS;
      }
    }
    // handle ETX char
    if (encodedBuf.first[vectorIdx] == DleEncoder::ETX_CHAR) {
      if (stxFound) {
        // This is propably a packet, so we decode it.
        size_t decodedLen = 0;
        size_t dummy = 0;

        ReturnValue_t result =
            decoder.decode(&encodedBuf.first[stxIdx], availableData - stxIdx, &dummy,
                           decodedBuf.first, decodedBuf.second, &decodedLen);
        if (result == returnvalue::OK) {
          ctx.setType(ContextType::PACKET_FOUND);
          ctx.decodedPacket.first = decodedBuf.first;
          ctx.decodedPacket.second = decodedLen;
          readSize = ++vectorIdx;
          return returnvalue::OK;
        } else {
          // invalid packet, skip.
          readSize = ++vectorIdx;
          ErrorInfo info;
          info.res = result;
          setErrorContext(ErrorTypes::DECODE_ERROR, info);
          return POSSIBLE_PACKET_LOSS;
        }
      } else {
        // might be lost packet, so we should advance the read pointer
        readSize = ++vectorIdx;
        ErrorInfo info;
        info.len = 0;
        setErrorContext(ErrorTypes::CONSECUTIVE_ETX_CHARS, info);
        return POSSIBLE_PACKET_LOSS;
      }
    }
  }
  return NO_PACKET_FOUND;
}

void DleParser::defaultFoundPacketHandler(uint8_t* packet, size_t len, void* args) {
#if FSFW_VERBOSE_LEVEL >= 1
#if FSFW_CPP_OSTREAM_ENABLED == 1
  sif::info << "DleParserBase::handleFoundPacket: Detected DLE packet with " << len << " bytes"
            << std::endl;
#else
  sif::printInfo("DleParserBase::handleFoundPacket: Detected DLE packet with %d bytes\n", len);
#endif
#endif
}

void DleParser::defaultErrorHandler() {
  if (ctx.getType() != DleParser::ContextType::ERROR) {
    errorPrinter("No error");
    return;
  }
  switch (ctx.error.first) {
    case (ErrorTypes::NONE): {
      errorPrinter("No error");
      break;
    }
    case (ErrorTypes::DECODE_ERROR): {
      errorPrinter("Decode Error");
      break;
    }
    case (ErrorTypes::RING_BUF_ERROR): {
      errorPrinter("Ring Buffer Error");
      break;
    }
    case (ErrorTypes::ENCODED_BUF_TOO_SMALL):
    case (ErrorTypes::DECODING_BUF_TOO_SMALL): {
      char opt[64];
      snprintf(opt, sizeof(opt), ": Too small for packet with length %zu",
               ctx.decodedPacket.second);
      if (ctx.error.first == ErrorTypes::ENCODED_BUF_TOO_SMALL) {
        errorPrinter("Encoded buf too small", opt);
      } else {
        errorPrinter("Decoding buf too small", opt);
      }
      break;
    }
    case (ErrorTypes::CONSECUTIVE_STX_CHARS): {
      errorPrinter("Consecutive STX chars detected");
      break;
    }
    case (ErrorTypes::CONSECUTIVE_ETX_CHARS): {
      errorPrinter("Consecutive ETX chars detected");
      break;
    }
  }
}

void DleParser::errorPrinter(const char* str, const char* opt) {
  if (opt == nullptr) {
    opt = "";
  }
#if FSFW_VERBOSE_LEVEL >= 1
#if FSFW_CPP_OSTREAM_ENABLED == 1
  sif::info << "DleParserBase::handleParseError: " << str << opt << std::endl;
#else
  sif::printInfo("DleParserBase::handleParseError: %s%s\n", str, opt);
#endif
#endif
}

void DleParser::setErrorContext(ErrorTypes err, ErrorInfo info) {
  ctx.setType(ContextType::ERROR);
  ctx.error.first = err;
  ctx.error.second = info;
}

ReturnValue_t DleParser::confirmBytesRead(size_t bytesRead) {
  return decodeRingBuf.deleteData(bytesRead);
}

const DleParser::Context& DleParser::getContext() { return ctx; }

void DleParser::reset() { decodeRingBuf.clear(); }
