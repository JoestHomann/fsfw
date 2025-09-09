#pragma once

#include <cstddef>
#include <cstdint>
#include "fsfw/devicehandlers/DeviceHandlerBase.h"

/**
 * Minimal device handler for a reaction wheel simulator.
 * Protocol over UART (binary):
 *   TX commands (5 bytes): AA <id> <hi> <lo> <crc8>
 *   RX reply   (8 bytes):  AB 0x10 <spdH spdL> <torH torL> <running> <crc8>
 */
class RwCommanderHandler : public DeviceHandlerBase {
 public:
  RwCommanderHandler(object_id_t objectId, object_id_t comIF, CookieIF* comCookie);
  ~RwCommanderHandler() override = default;

  static uint8_t crc8(const uint8_t* data, size_t len);

  ReturnValue_t performOperation(uint8_t opCode) override;
  

 protected:
  void doStartUp() override;
  void doShutDown() override;

  
  void modeChanged() override;

  ReturnValue_t buildNormalDeviceCommand(DeviceCommandId_t* id) override;
  ReturnValue_t buildTransitionDeviceCommand(DeviceCommandId_t* id) override;
  ReturnValue_t buildCommandFromCommand(DeviceCommandId_t deviceCommand, const uint8_t* data,
                                        size_t len) override;

  void fillCommandAndReplyMap() override;

  ReturnValue_t scanForReply(const uint8_t* start, size_t len, DeviceCommandId_t* foundId,
                             size_t* foundLen) override;
  ReturnValue_t interpretDeviceReply(DeviceCommandId_t id, const uint8_t* packet) override;

  uint32_t getTransitionDelayMs(Mode_t modeFrom, Mode_t modeTo) override;
  ReturnValue_t initializeLocalDataPool(localpool::DataPool& map,
                                        LocalDataPoolManager& mgr) override;

 private:
  // Protocol constants / IDs
  static constexpr uint8_t START_BYTE_CMD   = 0xAA;
  static constexpr uint8_t START_BYTE_REPLY = 0xAB;

  static constexpr DeviceCommandId_t CMD_SET_SPEED = 0x01;
  static constexpr DeviceCommandId_t CMD_STOP      = 0x02;
  static constexpr DeviceCommandId_t CMD_STATUS    = 0x03;

  static constexpr DeviceCommandId_t REPLY_STATUS  = 0x10;

  // Small TX buffer reused for every frame
  uint8_t txBuf[8] = {0};

  // Periodic STATUS polling
  uint8_t  statusPollDivider = 1;   // send STATUS about every 10 s at 5 Hz task
  uint8_t  statusPollCnt     = 0;
  uint32_t warmupCycles      = 1;   // skip first cycles after port open
  uint32_t warmupCnt         = 0;

  int16_t lastTargetRpm = 0;
};
