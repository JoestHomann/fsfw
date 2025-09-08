#pragma once

#include <cstdint>

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
  // This can be used to track the internal state of the reaction wheel handler.
  enum class InternalState { IDLE, RUNNING, ERROR };
  InternalState internalState = InternalState::IDLE;

 private:
  // General variables for the reaction wheel handler
  float targetSpeed = 0.0f;   // Target speed for the reaction wheel [e.g., in RPM]                  // Anm.: Ggfs. als struct übergeben
  float currentSpeed = 0.0f;  // Current (simulated or measured) speed
  float maxSpeed = 1000.0f;   // Maximum allowed speed

  // PI-controller variables
  float speedIntegral = 0.0f;
  float kp = 0.8f;         // Proportional gain
  float ki = 0.1f;         // Integral gain
  float dt = 0.2f;         // Task period [s], anpassen je nach Scheduler!
  float maxAccel = 50.0f;  // [RPM/s], optional Begrenzung für realistischeren Ramp-Up

  uint8_t cmdBuffer[8] = {0}; // Buffer for command data

  // DeviceCommand IDs für ReactionWheel-Kommandos
  static constexpr DeviceCommandId_t CMD_SET_SPEED = 0x01; // Dummy values                  // Anm.: Ggfs. als public static constexpr definieren
  static constexpr DeviceCommandId_t CMD_RESET = 0x02;
  static constexpr DeviceCommandId_t CMD_STOP = 0x03;

  // (Optional für Antworten)
  static constexpr DeviceCommandId_t REPLY_ACK = 0x10; // Dummy values
  static constexpr DeviceCommandId_t REPLY_STATUS = 0x11;

  // Error flag for error handling
  bool errorFlag = false;

  // Method to check for errors and handle them
  void checkForErrors();
};