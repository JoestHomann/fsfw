#pragma once

#include <cstdint>
#include <cstddef>

#include "fsfw/devicehandlers/DeviceHandlerBase.h"

class ReactionWheelsHandler : public DeviceHandlerBase {
 public:
  ReactionWheelsHandler(object_id_t objectId, object_id_t comIF, CookieIF* comCookie);
  virtual ~ReactionWheelsHandler();

  // CRC8 calculation function
  static uint8_t CRC8(const uint8_t* data, size_t len);

  // Setters and getters for target and current speed
  void setTargetSpeed(float speed);
  float getTargetSpeed() const;
  void setCurrentSpeed(float speed);
  float getCurrentSpeed() const;

 protected:
  // required methods inherited by DeviceHandlerBase
  void doStartUp() override;
  void doShutDown() override;

  ReturnValue_t buildNormalDeviceCommand(DeviceCommandId_t* id) override;
  ReturnValue_t buildTransitionDeviceCommand(DeviceCommandId_t* id) override;
  ReturnValue_t buildCommandFromCommand(DeviceCommandId_t deviceCommand, const uint8_t* commandData,
                                        size_t commandDataLen) override;

  void fillCommandAndReplyMap() override;

  ReturnValue_t scanForReply(const uint8_t* start, size_t len, DeviceCommandId_t* foundId,
                             size_t* foundLen) override;
  ReturnValue_t interpretDeviceReply(DeviceCommandId_t id, const uint8_t* packet) override;
  uint32_t getTransitionDelayMs(Mode_t modeFrom, Mode_t modeTo) override;
  ReturnValue_t initializeLocalDataPool(localpool::DataPool& localDataPoolMap,
                                        LocalDataPoolManager& poolManager) override;

  // Custom state enum
  enum class InternalState { IDLE, RUNNING, ERROR };
  InternalState internalState = InternalState::IDLE;

 private:
  // RW parameters
  float targetSpeed = 0.0f;   // Target speed in RPM
  float currentSpeed = 0.0f;  // Current speed in RPM
  float maxSpeed = 1000.0f;   // Max allowed speed in RPM

  // PI controller
  float speedIntegral = 0.0f;
  float kp = 0.8f;
  float ki = 0.1f;
  float dt = 0.2f;         // Task period [s]
  float maxAccel = 50.0f;  // [RPM/s]

  uint8_t cmdBuffer[8] = {0}; // Command buffer

  // Command IDs
  static constexpr DeviceCommandId_t CMD_SET_SPEED = 0x01;
  static constexpr DeviceCommandId_t CMD_STOP      = 0x02;
  static constexpr DeviceCommandId_t CMD_STATUS    = 0x03;

  // Reply IDs
  static constexpr DeviceCommandId_t REPLY_STATUS  = 0x10;

  // Error flag
  bool errorFlag = false;

  // Error check
  void checkForErrors();
};
