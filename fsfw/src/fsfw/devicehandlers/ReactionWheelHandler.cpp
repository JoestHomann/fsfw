#include "ReactionWheelHandler.h"

#include "fsfw/returnvalues/returnvalue.h"
#include "fsfw/serviceinterface/ServiceInterfaceStream.h"  // For sif::info

ReactionWheelHandler::ReactionWheelHandler(object_id_t objectId, object_id_t comIF,
                                           CookieIF* comCookie)
    : DeviceHandlerBase(objectId, comIF, comCookie) {}

ReactionWheelHandler::~ReactionWheelHandler() {}

void ReactionWheelHandler::doStartUp() {
  // Called during the transition to ON mode.
  // You could send initialization or power-up commands here.

  sif::info << "ReactionWheelHandler: Entering doStartUp(), switching to MODE_ON"
            << std::endl;  // Output current mode for debugging
  setMode(MODE_ON);
}

void ReactionWheelHandler::doShutDown() {
  sif::info << "RWHandler: doShutDown(), switching to MODE_OFF" << std::endl;
  currentSpeed = 0.0f;
  speedIntegral = 0.0f;
  errorFlag = false;
  setMode(MODE_OFF);
}

ReturnValue_t ReactionWheelHandler::buildNormalDeviceCommand(DeviceCommandId_t* id) {
  // Call error check first
  checkForErrors();
  if (mode == MODE_NORMAL && !errorFlag) {
    // PI-Controller Logic
    float error = targetSpeed - currentSpeed;
    speedIntegral += error * dt;

    // Integral Windup Protection
    float integralLimit = 0.5f * maxSpeed;
    if (speedIntegral > integralLimit) speedIntegral = integralLimit;
    if (speedIntegral < -integralLimit) speedIntegral = -integralLimit;

    // PI Control Signal Calculation
    float controlSignal = kp * error + ki * speedIntegral;

    // Limit the control signal to prevent too rapid acceleration
    if (controlSignal > maxAccel * dt) controlSignal = maxAccel * dt;
    if (controlSignal < -maxAccel * dt) controlSignal = -maxAccel * dt;

    currentSpeed += controlSignal;

    // Clamp the current speed to the maximum allowed speed
    if (currentSpeed > maxSpeed) currentSpeed = maxSpeed;
    if (currentSpeed < 0) currentSpeed = 0;

    sif::info << "RWHandler: PI-Regelung, target=" << targetSpeed << " current=" << currentSpeed
              << " control=" << controlSignal << std::endl;

    // Prepare the command buffer to send the target speed to the device
    uint8_t* buffer = cmdBuffer;  
    buffer[0] = 0xAA;             // Startbyte
    buffer[1] = 0x01;             // CMD_SET_SPEED
    uint16_t rpm = static_cast<uint16_t>(targetSpeed);
    buffer[2] = (rpm >> 8) & 0xFF;
    buffer[3] = rpm & 0xFF;
    buffer[4] = CRC8(buffer, 4);  // CRC-Function

    rawPacket = buffer;
    rawPacketLen = 5;
    *id = CMD_SET_SPEED;  // Command-Id setzen, falls gebraucht

    sif::info << "RWHandler: Sende Paket an Arduino, target=" << targetSpeed << std::endl;
    return returnvalue::OK;
  }
  return returnvalue::OK;
}

ReturnValue_t ReactionWheelHandler::buildTransitionDeviceCommand(DeviceCommandId_t* id) {
  sif::info << "RWHandler: buildTransitionDeviceCommand called." << std::endl;
  // Handle transitions between modes.
  if (mode == MODE_ON) {
    currentSpeed = 0.0f;  // Reset speed when transitioning to ON mode
    speedIntegral = 0.0f;
    errorFlag = false;
  }
  // Handle transition to OFF mode
  else if (mode == MODE_OFF) {
    currentSpeed = 0.0f;
    speedIntegral = 0.0f;
    errorFlag = false;
  }

  return returnvalue::OK;
}

ReturnValue_t ReactionWheelHandler::buildCommandFromCommand(DeviceCommandId_t deviceCommand,
                                                            const uint8_t* commandData,
                                                            size_t commandDataLen) {
  switch (deviceCommand) {
    case CMD_SET_SPEED:
      if (commandDataLen >= sizeof(float)) {
        float newSpeed;
        memcpy(&newSpeed, commandData, sizeof(float));
        setTargetSpeed(newSpeed);
        sif::info << "RWHandler: SET_SPEED received, new target=" << newSpeed << std::endl;
      } else {
        sif::warning << "RWHandler: SET_SPEED command - not enough data!" << std::endl;
      }
      break;
    case CMD_RESET:
      // Reset the handler to initial state
      currentSpeed = 0.0f;
      speedIntegral = 0.0f;
      errorFlag = false;
      setMode(MODE_ON);
      sif::info << "RWHandler: RESET command received - handler reset." << std::endl;
      break;
    case CMD_STOP:
      targetSpeed = 0.0f;
      sif::info << "RWHandler: STOP command received - target speed set to zero." << std::endl;
      break;
    default:
      sif::warning << "RWHandler: Unknown command received: " << deviceCommand << std::endl;
      return returnvalue::FAILED;
  }
  return returnvalue::OK;
}

void ReactionWheelHandler::fillCommandAndReplyMap() {
  // Register the SET_SPEED command
  insertInCommandAndReplyMap(
      CMD_SET_SPEED,  // Command ID
      5               // maxDelayCycles: Wartezyklen auf Antwort (anpassen)
      // Optional: replyDataSet, replyLen, periodic, hasDifferentReplyId, replyId, Countdown
  );
  // Register the RESET command
  insertInCommandAndReplyMap(CMD_RESET,
                             3  // z.B. nur 3 Zyklen Antwortzeit nötig
  );
  // Register the STOP command
  insertInCommandAndReplyMap(CMD_STOP, 3);
  // Optional: Auch Replies mappen (für Status, ACKs etc.)
  insertInReplyMap(REPLY_ACK, 2);
  insertInReplyMap(REPLY_STATUS, 2);

  sif::info << "RWHandler: fillCommandAndReplyMap - registered SET_SPEED, RESET, STOP."
            << std::endl;
}

ReturnValue_t ReactionWheelHandler::scanForReply(const uint8_t* start, size_t len, DeviceCommandId_t* foundId, size_t* foundLen) {
    // Minimum: Startbyte + Typ + Daten + CRC = 8
    if (len < 8) return returnvalue::FAILED;
    if (start[0] != 0xAB) return returnvalue::FAILED;
    if (CRC8(start, 7) != start[7]) return returnvalue::FAILED;
    if (start[1] == 0x10) { // STATUS REPLY
        *foundId = REPLY_STATUS;
        *foundLen = 8;
        return returnvalue::OK;
    }
    return returnvalue::FAILED;
}

ReturnValue_t ReactionWheelHandler::interpretDeviceReply(DeviceCommandId_t id, const uint8_t* packet) {
    if (id == REPLY_STATUS) {
        int16_t speed_rpm = (packet[2] << 8) | packet[3];
        int16_t torque = (packet[4] << 8) | packet[5];
        bool running = packet[6];
        this->currentSpeed = speed_rpm;
        
        sif::info << "RWHandler: " << speed_rpm << "rpm." << std::endl;
        return returnvalue::OK;
    }
    return returnvalue::FAILED;
}

uint32_t ReactionWheelHandler::getTransitionDelayMs(Mode_t modeFrom, Mode_t modeTo) {
  // Set the delay for mode transitions.
  if (modeFrom == MODE_OFF && modeTo == MODE_ON) {
    return 500;  // 500 ms
  }
  return 0;
}

ReturnValue_t ReactionWheelHandler::initializeLocalDataPool(localpool::DataPool& localDataPoolMap,
                                                            LocalDataPoolManager& poolManager) {
  sif::info << "RWHandler: initializeLocalDataPool called (no datapool variables yet)."
            << std::endl;
  return returnvalue::OK;
}

// Setters and getters for target and current speed
void ReactionWheelHandler::setTargetSpeed(float speed) { targetSpeed = speed; }
float ReactionWheelHandler::getTargetSpeed() const { return targetSpeed; }
void ReactionWheelHandler::setCurrentSpeed(float speed) { currentSpeed = speed; }
float ReactionWheelHandler::getCurrentSpeed() const { return currentSpeed; }

// Error checking function
void ReactionWheelHandler::checkForErrors() {
  if (currentSpeed > maxSpeed + 1e-3f) {  // small tolerance to avoid floating point inaccuracies
    errorFlag = true;
    currentSpeed = maxSpeed;
    sif::warning << "RWHandler: Current speed exceeds max speed! Switching to maxSpeed."
                 << std::endl;
    setMode(MODE_ERROR_ON);  // Switch to error mode if speed exceeds max limit
  } else if (currentSpeed < 0.0f) {
    errorFlag = true;
    currentSpeed = 0.0f;
    sif::warning << "RWHandler: Negative speed detected! Clamping to zero." << std::endl;
    setMode(MODE_ERROR_ON);
  } else {
    errorFlag = false;
  }
}


uint8_t ReactionWheelHandler::CRC8(const uint8_t* data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }
    return crc;
}