#pragma once

#include <cstdint>
#include <cstddef>
#include "fsfw/devicehandlers/DeviceHandlerBase.h"

/**
 * Minimal device handler that implements the rw_commander protocol:
 *   - Commands: SET_SPEED (0x01), STOP (0x02), STATUS (0x03)
 *   - Reply:    STATUS (0x10) with 8-byte frame: AB 10 [spdH spdL] [torH torL] [running] CRC
 * It sends raw frames over the configured SerialComIF via the Cookie.
 */
class RwCommanderHandler : public DeviceHandlerBase {
 public:
  RwCommanderHandler(object_id_t objectId, object_id_t comIF, CookieIF* comCookie);
  ~RwCommanderHandler() override = default;

  // Protocol CRC8 helper (poly 0x07, init 0x00)
  static uint8_t crc8(const uint8_t* data, size_t len);

 protected:
  // DeviceHandlerBase hooks
  void doStartUp() override;
  void doShutDown() override;

  ReturnValue_t buildNormalDeviceCommand(DeviceCommandId_t* id) override;
  ReturnValue_t buildTransitionDeviceCommand(DeviceCommandId_t* id) override;
  ReturnValue_t buildCommandFromCommand(DeviceCommandId_t deviceCommand, const uint8_t* data,
                                        size_t len) override;

  void fillCommandAndReplyMap() override;

  ReturnValue_t scanForReply(const uint8_t* start, size_t len, DeviceCommandId_t* foundId,
                             size_t* foundLen) override;
  ReturnValue_t interpretDeviceReply(DeviceCommandId_t id, const uint8_t* packet) override;

  uint32_t getTransitionDelayMs(Mode_t modeFrom, Mode_t modeTo) override;
  ReturnValue_t initializeLocalDataPool(localpool::DataPool& localDataPoolMap,
                                        LocalDataPoolManager& poolManager) override;

 private:
  // Protocol constants
  static constexpr uint8_t START_BYTE_CMD   = 0xAA;
  static constexpr uint8_t START_BYTE_REPLY = 0xAB;

  // DeviceCommand IDs (also used as protocol command IDs)
  static constexpr DeviceCommandId_t CMD_SET_SPEED = 0x01;
  static constexpr DeviceCommandId_t CMD_STOP      = 0x02;
  static constexpr DeviceCommandId_t CMD_STATUS    = 0x03;

  // Reply IDs
  static constexpr DeviceCommandId_t REPLY_STATUS  = 0x10;

  // Small TX buffer
  uint8_t txBuf[8] = {0};

  // Optional: poll STATUS periodically every N normal cycles
  uint8_t statusPollDivider = 5;     // send STATUS every 5 normal calls
  uint8_t statusPollCnt     = 0;

  // Last commanded target speed (for visibility/logging)
  int16_t lastTargetRpm = 0;
};
