#include "ReactionWheelsHandler.h"

#include "fsfw/returnvalues/returnvalue.h"
#include "fsfw/serviceinterface/ServiceInterfaceStream.h"  // For sif::info

ReactionWheelsHandler::ReactionWheelsHandler(object_id_t objectId, object_id_t comIF,
                                             CookieIF* comCookie)
    : DeviceHandlerBase(objectId, comIF, comCookie) {}

ReactionWheelsHandler::~ReactionWheelsHandler() {}

void ReactionWheelsHandler::doStartUp() {
  // Called during the transition to ON mode.
  // You could send initialization or power-up commands here.

  sif::info << "ReactionWheelsHandler: Entering doStartUp(), switching to MODE_ON"
            << std::endl;  // Output current mode for debugging
  setMode(MODE_ON);
}

void ReactionWheelsHandler::doShutDown() {
  sif::info << "RWHandler: doShutDown(), switching to MODE_OFF" << std::endl;
  currentSpeed = 0.0f;
  speedIntegral = 0.0f;
  errorFlag = false;
  setMode(MODE_OFF);
}

ReturnValue_t ReactionWheelsHandler::buildNormalDeviceCommand(DeviceCommandId_t* id) {
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
  }
  return returnvalue::OK;
}

ReturnValue_t ReactionWheelsHandler::buildTransitionDeviceCommand(DeviceCommandId_t* id) {
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

ReturnValue_t ReactionWheelsHandler::buildCommandFromCommand(DeviceCommandId_t deviceCommand,
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

void ReactionWheelsHandler::fillCommandAndReplyMap() {
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

ReturnValue_t ReactionWheelsHandler::scanForReply(const uint8_t* start, size_t len,
                                                  DeviceCommandId_t* foundId, size_t* foundLen) {
  // Simulate scanning for a reply.
  // In a real implementation, you would parse the incoming data to find a valid reply.
  sif::info << "RWHandler: scanForReply called (no real data in dummy version)." << std::endl;
  *foundId = 0;   // Dummy-ID
  *foundLen = 0;  // Dummy-Length
  return returnvalue::OK;
}

ReturnValue_t ReactionWheelsHandler::interpretDeviceReply(DeviceCommandId_t id,
                                                          const uint8_t* packet) {
  // Simulate interpreting a device reply.
  // In a real implementation, you would parse the packet and update internal state.
  sif::info << "RWHandler: interpretDeviceReply called for id=" << id << std::endl;

  return returnvalue::OK;
}

uint32_t ReactionWheelsHandler::getTransitionDelayMs(Mode_t modeFrom, Mode_t modeTo) {
  // Set the delay for mode transitions.
  if (modeFrom == MODE_OFF && modeTo == MODE_ON) {
    return 500;  // 500 ms
  }
  return 0;
}

ReturnValue_t ReactionWheelsHandler::initializeLocalDataPool(localpool::DataPool& localDataPoolMap,
                                                             LocalDataPoolManager& poolManager) {
  sif::info << "RWHandler: initializeLocalDataPool called (no datapool variables yet)."
            << std::endl;
  return returnvalue::OK;
}

// Setters and getters for target and current speed
void ReactionWheelsHandler::setTargetSpeed(float speed) { targetSpeed = speed; }
float ReactionWheelsHandler::getTargetSpeed() const { return targetSpeed; }
void ReactionWheelsHandler::setCurrentSpeed(float speed) { currentSpeed = speed; }
float ReactionWheelsHandler::getCurrentSpeed() const { return currentSpeed; }


// Error checking function
void ReactionWheelsHandler::checkForErrors() {
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