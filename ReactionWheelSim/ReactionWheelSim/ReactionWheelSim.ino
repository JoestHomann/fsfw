/**
  ******************************************************************************
  * @file           : ReactionWheelSim.ino
  * @brief          : Reaction Wheel Simulator with Physical Dynamics and Noise
  ******************************************************************************
  * @details
  *
  * This program simulates the physical behavior of a reaction wheel on an 
  * embedded microcontroller (Arduino Nano 33 BLE Sense Rev2), primarily for 
  * hardware-in-the-loop (HIL) testing with the FSFW (Flight Software Framework).
  *
  * Simulation features:
  *   - Realistic reaction wheel dynamics:
  *       * Wheel inertia
  *       * Acceleration limits
  *       * Commanded speed tracking
  *   - Additive stochastic noise:
  *       * Simulated using band-limited white noise (BLWN)
  *       * Models torque ripple or actuator noise
  *   - Serial command interface (binary, FSFW-compatible):
  *       * SET speed [RPM]
  *       * STOP wheel
  *       * STATUS request (returns current speed, torque, and running flag)
  *   - CRC8 packet integrity verification
  *   - LED indicators:
  *       * LED_BUILTIN blinks at wheel frequency
  *       * LED_PWR (P1.09) indicates protocol or CRC errors
  *
  ******************************************************************************
  * @attention
  *
  * Created in 2025 for academic and experimental use.
  * Provided as-is, without warranty of any kind.
  *
  ******************************************************************************
  */

// Commands declaration
const byte START_BYTE_CMD   = 0xAA;
const byte START_BYTE_REPLY = 0xAB;

const byte CMD_SET_SPEED    = 0x01;
const byte CMD_STOP         = 0x02;
const byte CMD_STATUS       = 0x03;

const byte RESP_STATUS      = 0x10;

const int LED_PIN = LED_BUILTIN;

// Physical parameters (constant)
const float J_rw = 0.001f;         // [kg m^2] MoI of reaction wheel
const float J_sc = 0.5f;           // [kg m^2] MoI of S/C
const float alpha_max = 5.0f;      // [rad/s^2] maximum rw acceleration
const float dt = 0.01f;            // [s] simulation time step (default: 10ms)

// Runtime parameters initialisation
float omega_rw = 0.0f;             // [rad/s] current angular rate
float omega_rw_target = 0.0f;      // [rad/s] target velocity
float alpha_rw = 0.0f;             // [rad/s^2] current acceleration
bool errorFlag = false;            // Error flag
unsigned long lastUpdate = 0;

// Setup with initialisation
void setup() {
  Serial.begin(9600);
  while (!Serial);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_PWR, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(LED_PWR, LOW);
}

// Loop with function calls
void loop() {
  handleSerialPackets();
  updateWheelDynamics();
  visualizeSpeed();
  updateErrorLED();
}

// ------------------ Function declarations ---------------------

// handleSerialPackets
void handleSerialPackets() {
  if (Serial.available() >= 5) {
    byte packet[5];
    Serial.readBytes(packet, 5);

    if (packet[0] != START_BYTE_CMD) {
      errorFlag = true;
      return;
    }
    if (crc8(packet, 4) != packet[4]) {
      errorFlag = true;
      return;
    }

    errorFlag = false; // package correct, reset error flag
    byte cmd = packet[1];
    int16_t value = (int16_t)((int16_t)packet[2] << 8 | packet[3]);

    switch (cmd) {
      case CMD_SET_SPEED:
        omega_rw_target = (float)value * 2.0f * PI / 60.0f; // [U/min] conversion to [rad/s]
        break;
      case CMD_STOP:
        omega_rw_target = 0.0f;
        break;
      case CMD_STATUS:
        sendStatusReply();
        break;
      default:
        errorFlag = true; // unknown command
        break;
    }
  }
}

// updateWheelDynamics
void updateWheelDynamics() {
  unsigned long now = millis();
  if (now - lastUpdate < 10) return;
  lastUpdate = now;

  float error = omega_rw_target - omega_rw;
  float desired_alpha = constrain(error / dt, -alpha_max, alpha_max);

  alpha_rw = desired_alpha;
  float tau_n = generateBLWN();
  omega_rw += (alpha_rw + tau_n / J_rw) * dt;
}

// generateBLWN
float generateBLWN() {
  static float wn = 0.0f;
  const float noiseGain = 0.002f;     // [Nm] maximum Noise
  const float alpha = 0.1f;           // 0 < alpha < 1 (band-limitation)
  float white = ((float)random(-1000, 1000) / 1000.0f); // random number between -1 and 1
  wn = (1 - alpha) * wn + alpha * white;
  return wn * noiseGain;
}

// visualizeSpeed
void visualizeSpeed() {
  float freq = abs(omega_rw) / (2 * PI); // Hz
  float delay_ms = (freq > 0.5f) ? 500.0f / freq : 1000.0f;

  static unsigned long lastBlink = 0;
  static bool state = false;

  if (millis() - lastBlink >= delay_ms) {
    state = !state;
    digitalWrite(LED_PIN, state);
    lastBlink = millis();
  }
}

// updateErrorLED
void updateErrorLED() {
  if (errorFlag) {
    digitalWrite(LED_PWR, HIGH); // Error: LED ON
  } else {
    digitalWrite(LED_PWR, LOW);  // No Error: LED OFF
  }
}

// sendStatusReply
void sendStatusReply() {
  byte reply[8];
  reply[0] = START_BYTE_REPLY;
  reply[1] = RESP_STATUS;
  int16_t speed_rpm = omega_rw * 60.0f / (2 * PI); // [rad/s] conversion to [U/min]
  int16_t torque_mNm = J_rw * alpha_rw * 1000.0f;   // [Nm] conversion to [mNm]
  reply[2] = (speed_rpm >> 8) & 0xFF;
  reply[3] = speed_rpm & 0xFF;
  reply[4] = (torque_mNm >> 8) & 0xFF;
  reply[5] = torque_mNm & 0xFF;
  reply[6] = (omega_rw_target != 0.0f) ? 1 : 0;
  reply[7] = crc8(reply, 7);

  Serial.write(reply, 8);
}

// crc8
byte crc8(const byte* data, byte len) {
  byte crc = 0x00;
  for (byte i = 0; i < len; i++) {
    crc ^= data[i];
    for (byte j = 0; j < 8; j++) {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x07;
      else
        crc <<= 1;
    }
  }
  return crc;
}
