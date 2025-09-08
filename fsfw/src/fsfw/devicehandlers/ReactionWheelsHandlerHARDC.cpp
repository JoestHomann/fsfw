#include "ReactionWheelsHandler.h"

#include <cstring>  // for memcpy
#include "fsfw/returnvalues/returnvalue.h"
#include "fsfw/serviceinterface/ServiceInterfaceStream.h"  // For sif::info

// --- JH HARDCODE ---
// Compile-time switch to enable/disable the hardcoded link test block easily
#ifndef JH_HC_TEST
#define JH_HC_TEST 1
#endif

#if JH_HC_TEST
namespace {
// One-shot state machine to emit two commands across two handler build cycles.
enum class HcState { IDLE, SEND_SET, SEND_STATUS, DONE };
// Start with sending SET_SPEED after startup
HcState hcState = HcState::SEND_SET;

// Wait a few cycles after opening the tty so Arduino can reboot (typical behavior)
int hcBootWaitCycles = 10; // 10 * 0.2 s = ~2 s at 5 Hz
}
#endif
// --- JH HARDCODE ---

ReactionWheelsHandler::ReactionWheelsHandler(object_id_t objectId, object_id_t comIF,
                                             CookieIF* comCookie)
    : DeviceHandlerBase(objectId, comIF, comCookie) {}

ReactionWheelsHandler::~ReactionWheelsHandler() {}

void ReactionWheelsHandler::doStartUp() {
  sif::info << "ReactionWheelsHandler: Entering doStartUp()" << std::endl;  // debug

#if JH_HC_TEST
  // For the hardcoded test, go straight to NORMAL so buildNormalDeviceCommand() runs.
  setMode(MODE_NORMAL);
#else
  setMode(MODE_ON);
#endif

#if JH_HC_TEST
  // Reset one-shot sequence and boot wait on every startup.
  hcState = HcState::SEND_SET;
  hcBootWaitCycles = 10;
#endif

  sif::info << "RWHandler: doStartUp() done, current mode=" << static_cast<int>(this->mode)
            << std::endl;
}

void ReactionWheelsHandler::doShutDown() {
  sif::info << "RWHandler: doShutDown(), switching to MODE_OFF" << std::endl;
  currentSpeed = 0.0f;
  speedIntegral = 0.0f;
  errorFlag = false;

  // --- JH HARDCODE ---
#if JH_HC_TEST
  hcState = HcState::SEND_SET; // allow the two-step sequence again after shutdown
#endif
  // --- JH HARDCODE ---

  setMode(MODE_OFF);
}

ReturnValue_t ReactionWheelsHandler::buildNormalDeviceCommand(DeviceCommandId_t* id) {

  // --- JH HARDCODE ---
#if JH_HC_TEST
  // Give the Arduino time to reboot after opening the serial port
  if (hcBootWaitCycles > 0) {
    --hcBootWaitCycles;
    return returnvalue::OK;
  }

  // Send SET_SPEED 1000 once, then a STATUS request once, then fall back to normal logic.
  if (hcState == HcState::SEND_SET) {
    // Build "SET_SPEED 1000" (= 0xAA 0x01 0x03 0xE8 CRC)
    uint8_t* buffer = cmdBuffer;
    const uint16_t rpm = 1000;
    buffer[0] = 0xAA;               // start byte
    buffer[1] = 0x01;               // CMD_SET_SPEED
    buffer[2] = static_cast<uint8_t>((rpm >> 8) & 0xFF);  // high byte
    buffer[3] = static_cast<uint8_t>(rpm & 0xFF);         // low byte
    buffer[4] = CRC8(buffer, 4);    // CRC over first 4 bytes

    rawPacket    = buffer;
    rawPacketLen = 5;
    *id = CMD_SET_SPEED;

    // Keep internal target aligned for the later PI loop
    this->targetSpeed = static_cast<float>(rpm);

    sif::info << "RWHandler[JH]: HARDCODED sending SET_SPEED 1000 RPM" << std::endl;

#if FSFW_CPP_OSTREAM_ENABLED == 1
    // Dump TX bytes
    sif::info << "RWHandler[JH]: TX =";
    for (size_t i = 0; i < rawPacketLen; ++i) {
      sif::info << " 0x" << std::hex << static_cast<int>(rawPacket[i]) << std::dec;
    }
    sif::info << std::endl;
#endif

    hcState = HcState::SEND_STATUS;
    return returnvalue::OK; // one packet per build call
  }

  if (hcState == HcState::SEND_STATUS) {
    // Build "STATUS" (= 0xAA 0x03 0x00 0x00 CRC)
    uint8_t* buffer = cmdBuffer;
    buffer[0] = 0xAA;            // start byte
    buffer[1] = 0x03;            // CMD_STATUS
    buffer[2] = 0x00;            // value high
    buffer[3] = 0x00;            // value low
    buffer[4] = CRC8(buffer, 4); // CRC over first 4 bytes

    rawPacket    = buffer;
    rawPacketLen = 5;
    *id = CMD_STATUS;

    sif::info << "RWHandler[JH]: HARDCODED sending STATUS request" << std::endl;

#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::info << "RWHandler[JH]: TX =";
    for (size_t i = 0; i < rawPacketLen; ++i) {
      sif::info << " 0x" << std::hex << static_cast<int>(rawPacket[i]) << std::dec;
    }
    sif::info << std::endl;
#endif

    hcState = HcState::DONE;
    return returnvalue::OK;
  }
#endif
  // --- JH HARDCODE ---

  // Normal PI control and periodic SET_SPEED command
  checkForErrors();
  if (mode == MODE_NORMAL && !errorFlag) {
    // PI-Controller Logic
    float error = targetSpeed - currentSpeed;
    speedIntegral += error * dt;

    // Integral windup protection
    float integralLimit = 0.5f * maxSpeed;
    if (speedIntegral > integralLimit) speedIntegral = integralLimit;
    if (speedIntegral < -integralLimit) speedIntegral = -integralLimit;

    // PI control signal
    float controlSignal = kp * error + ki * speedIntegral;

    // Acceleration limit
    if (controlSignal > maxAccel * dt) controlSignal = maxAccel * dt;
    if (controlSignal < -maxAccel * dt) controlSignal = -maxAccel * dt;

    currentSpeed += controlSignal;

    // Clamp speed
    if (currentSpeed > maxSpeed) currentSpeed = maxSpeed;
    if (currentSpeed < 0) currentSpeed = 0;

    sif::info << "RWHandler: PI loop, target=" << targetSpeed
              << " current=" << currentSpeed
              << " control=" << controlSignal << std::endl;

    // Build periodic SET_SPEED to device (based on targetSpeed)
    uint8_t* buffer = cmdBuffer;
    uint16_t rpm = static_cast<uint16_t>(targetSpeed);
    buffer[0] = 0xAA;             // start byte
    buffer[1] = 0x01;             // CMD_SET_SPEED
    buffer[2] = static_cast<uint8_t>((rpm >> 8) & 0xFF);
    buffer[3] = static_cast<uint8_t>(rpm & 0xFF);
    buffer[4] = CRC8(buffer, 4);  // CRC

    rawPacket    = buffer;
    rawPacketLen = 5;
    *id = CMD_SET_SPEED;

#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::info << "RWHandler: TX (PI) =";
    for (size_t i = 0; i < rawPacketLen; ++i) {
      sif::info << " 0x" << std::hex << static_cast<int>(rawPacket[i]) << std::dec;
    }
    sif::info << std::endl;
#endif

    return returnvalue::OK;
  }
  return returnvalue::OK;
}

ReturnValue_t ReactionWheelsHandler::buildTransitionDeviceCommand(DeviceCommandId_t* id) {
  sif::info << "RWHandler: buildTransitionDeviceCommand called." << std::endl;

  // Default transition handling
  if (mode == MODE_ON) {
    currentSpeed = 0.0f;
    speedIntegral = 0.0f;
    errorFlag = false;
  } else if (mode == MODE_OFF) {
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
        std::memcpy(&newSpeed, commandData, sizeof(float));
        setTargetSpeed(newSpeed);
        sif::info << "RWHandler: SET_SPEED received, new target=" << newSpeed << std::endl;
      } else {
        sif::warning << "RWHandler: SET_SPEED command - not enough data!" << std::endl;
      }
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
  // --- JH HARDCODE ---
  // Make SET_SPEED periodic so buildNormalDeviceCommand() is called each cycle.
  insertInCommandAndReplyMap(
      CMD_SET_SPEED,
      static_cast<uint8_t>(5),      // maxDelayCycles
      nullptr,                      // reply dataset (none)
      static_cast<size_t>(0),       // expected reply length (none)
      true                          // periodic
  );
  // STATUS one-shot (requested by hardcoded block only)
  insertInCommandAndReplyMap(CMD_STATUS, 5);
  // STOP one-shot
  insertInCommandAndReplyMap(CMD_STOP, 3);
  // Expected replies
  insertInReplyMap(REPLY_STATUS, 2);

  sif::info << "RWHandler: fillCommandAndReplyMap - registered SET_SPEED, STATUS, STOP."
            << std::endl;
  // --- JH HARDCODE ---
}

ReturnValue_t ReactionWheelsHandler::scanForReply(const uint8_t* start, size_t len,
                                                  DeviceCommandId_t* foundId, size_t* foundLen) {
  // Minimum: start + type + data + CRC = 8
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

ReturnValue_t ReactionWheelsHandler::interpretDeviceReply(DeviceCommandId_t id,
                                                          const uint8_t* packet) {
  if (id == REPLY_STATUS) {
    int16_t speed_rpm = static_cast<int16_t>((packet[2] << 8) | packet[3]);
    int16_t torque    = static_cast<int16_t>((packet[4] << 8) | packet[5]);
    bool running      = (packet[6] != 0);
    this->currentSpeed = static_cast<float>(speed_rpm);

    sif::info << "RWHandler: STATUS: speed=" << speed_rpm
              << " rpm, torque=" << torque
              << " mNm, running=" << running << std::endl;
    return returnvalue::OK;
  }
  return returnvalue::FAILED;
}

uint32_t ReactionWheelsHandler::getTransitionDelayMs(Mode_t modeFrom, Mode_t modeTo) {
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
  if (currentSpeed > maxSpeed + 1e-3f) {
    errorFlag = true;
    currentSpeed = maxSpeed;
    sif::warning << "RWHandler: Current speed exceeds max speed! Switching to maxSpeed."
                 << std::endl;
    setMode(MODE_ERROR_ON);
  } else if (currentSpeed < 0.0f) {
    errorFlag = true;
    currentSpeed = 0.0f;
    sif::warning << "RWHandler: Negative speed detected! Clamping to zero." << std::endl;
    setMode(MODE_ERROR_ON);
  } else {
    errorFlag = false;
  }
}

uint8_t ReactionWheelsHandler::CRC8(const uint8_t* data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80)
        crc = static_cast<uint8_t>((crc << 1) ^ 0x07);
      else
        crc = static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}
