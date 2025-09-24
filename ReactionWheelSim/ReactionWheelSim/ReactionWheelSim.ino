/**
  ******************************************************************************
  * @file           : ReactionWheelSim.ino
  * @brief          : Reaction Wheel Simulator (CRC-16/CCITT, 6B CMD / 9B STATUS)
  ******************************************************************************
  * Wire protocol (big-endian):
  *   CMD (Host->Sim):    [0]=0xAA, [1]=CMD_ID, [2]=valH, [3]=valL, [4]=crcH, [5]=crcL
  *   STATUS (Sim->Host): [0]=0xAB, [1]=0x10,
  *                       [2..3]=speed_rpm(i16), [4..5]=torque_mNm(i16), [6]=running(u8),
  *                       [7..8]=CRC16 over bytes 0..6
  *
  * CRC: CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, xorout 0x0000), big-endian append
  ******************************************************************************
*/

#include <Arduino.h>

// ---------------- Protocol constants ----------------
const uint8_t START_BYTE_CMD   = 0xAA;
const uint8_t START_BYTE_REPLY = 0xAB;

const uint8_t CMD_SET_SPEED  = 0x01;
const uint8_t CMD_STOP       = 0x02;
const uint8_t CMD_STATUS     = 0x03;
const uint8_t CMD_SET_TORQUE = 0x04;

const uint8_t RESP_STATUS = 0x10;

const int LED_PIN = LED_BUILTIN;   // Wheel "blink" indicator
#ifndef LED_PWR
#define LED_PWR 25                 // Fallback if not provided by board variant
#endif

// ---------------- Simulation parameters ----------------
// Unit helpers
constexpr float PI_F   = 3.14159265358979323846f;
constexpr float RPM2RAD = 2.0f * PI_F / 60.0f;
constexpr float RAD2RPM = 60.0f / (2.0f * PI_F);

// Physical model
const float   J_rw       = 0.001f;   // [kg m^2] wheel inertia
const int16_t TQ_MAX_mNm = 5;        // [mNm] command clamp (match FSFW clamp ±5 mNm)
const float   TQ_MAX_Nm  = TQ_MAX_mNm / 1000.0f;     // [N·m]
const float   alpha_max  = TQ_MAX_Nm / J_rw;         // [rad/s^2] -> 5.0 with values above
const float   dt         = 0.01f;    // [s] integrator step (10 ms)
const int16_t RPM_MAX    = 6000;     // [RPM] safety clamp

// ---------------- Runtime state ----------------
float   omega_rw        = 0.0f;  // [rad/s] current wheel speed
float   omega_rw_target = 0.0f;  // [rad/s] speed setpoint (speed mode)
float   alpha_rw        = 0.0f;  // [rad/s^2] current acceleration
bool    torqueMode      = false; // true if last command was CMD_SET_TORQUE
float   tau_cmd_Nm      = 0.0f;  // [N·m] commanded torque (torque mode)

bool    errorFlag       = false; // protocol/CRC error indicator
unsigned long lastUpdate = 0;

// ---------------- CRC-16/CCITT-FALSE ----------------
uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; ++b) {
      if (crc & 0x8000) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }
  return crc;
}

// Read exactly 'n' bytes (blocking up to timeout)
bool readExact(uint8_t* dst, size_t n, uint32_t timeoutMs = 50) {
  const uint32_t deadline = millis() + timeoutMs;
  size_t off = 0;
  while (off < n) {
    if (Serial.available()) {
      off += Serial.readBytes(dst + off, n - off);
    } else if (millis() > deadline) {
      return false;
    }
    delayMicroseconds(100);
  }
  return true;
}

void setup() {
  // Note: On Arduino Nano 33 BLE (USB CDC), baud is mostly ignored by USB but keep it consistent with host.
  Serial.begin(115200);
  while (!Serial) { /* wait for USB CDC */ }

  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_PWR, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(LED_PWR, LOW);

  randomSeed(analogRead(0));
}

void loop() {
  handleSerialPackets();
  updateWheelDynamics();
  visualizeSpeed();
  updateErrorLED();
}

// ---------------- Protocol handling ----------------
void handleSerialPackets() {
  // Each command frame is 6 bytes with CRC-16
  while (Serial.available() >= 6) {
    uint8_t packet[6];
    if (!readExact(packet, sizeof(packet), 10)) {
      break;
    }

    // Check start byte
    if (packet[0] != START_BYTE_CMD) {
      errorFlag = true;
      continue;
    }

    // Verify CRC over first 4 bytes; compare to trailing 2 bytes (big-endian)
    const uint16_t rxCrc = (static_cast<uint16_t>(packet[4]) << 8) | packet[5];
    const uint16_t caCrc = crc16_ccitt(packet, 4);
    if (rxCrc != caCrc) {
      errorFlag = true;
      continue;
    }

    errorFlag = false; // valid frame
    const uint8_t cmd = packet[1];
    const int16_t value = static_cast<int16_t>((packet[2] << 8) | packet[3]); // big-endian i16

    switch (cmd) {
      case CMD_SET_SPEED: {
        // RPM -> rad/s, exit torque mode
        int16_t rpm_cmd = value;
        if (rpm_cmd >  RPM_MAX) rpm_cmd =  RPM_MAX;
        if (rpm_cmd < -RPM_MAX) rpm_cmd = -RPM_MAX;
        omega_rw_target = static_cast<float>(rpm_cmd) * RPM2RAD;
        tau_cmd_Nm = 0.0f;
        torqueMode = false;
        break;
      }
      case CMD_SET_TORQUE: {
        // mNm -> N·m, clamp to ±TQ_MAX_mNm
        int16_t tq_mNm = value;
        if (tq_mNm >  TQ_MAX_mNm) tq_mNm =  TQ_MAX_mNm;
        if (tq_mNm < -TQ_MAX_mNm) tq_mNm = -TQ_MAX_mNm;
        tau_cmd_Nm = static_cast<float>(tq_mNm) / 1000.0f;
        torqueMode = true;
        break;
      }
      case CMD_STOP: {
        omega_rw_target = 0.0f;
        tau_cmd_Nm = 0.0f;
        torqueMode = false;
        break;
      }
      case CMD_STATUS: {
        sendStatusReply();
        break;
      }
      default: {
        errorFlag = true; // unknown command
        break;
      }
    }
  }
}

// ---------------- Dynamics & visualization ----------------
// Band-limited white noise torque (keep much smaller than 1 mNm)
float generateBLWN() {
  static float wn = 0.0f;
  const float noiseGain = 0.0002f; // [N·m] ~0.2 mNm
  const float alpha = 0.1f;        // low-pass factor
  const float white = static_cast<float>(random(-1000, 1000)) / 1000.0f;
  wn = (1.0f - alpha) * wn + alpha * white;
  return wn * noiseGain;
}

void updateWheelDynamics() {
  const unsigned long now = millis();
  if (now - lastUpdate < 10) return; // 10 ms step
  lastUpdate = now;

  // Small viscous friction to avoid endless glide
  const float c_fric  = 1e-5f;               // [N·m·s]
  const float tau_fric = -c_fric * omega_rw; // [N·m]
  const float tau_noise = generateBLWN();    // [N·m]

  if (torqueMode) {
    // alpha = (tau_cmd + friction + noise) / J, limited by alpha_max (equiv. to torque clamp)
    float alpha_cmd = (tau_cmd_Nm + tau_fric + tau_noise) / J_rw;
    if (alpha_cmd >  alpha_max) alpha_cmd =  alpha_max;
    if (alpha_cmd < -alpha_max) alpha_cmd = -alpha_max;
    alpha_rw = alpha_cmd;
  } else {
    // Speed mode: accel limiter towards target (simple capped first-order step)
    const float error = omega_rw_target - omega_rw;
    float desired_alpha = error / dt; // close gap in one step, then clamp
    if (desired_alpha >  alpha_max) desired_alpha =  alpha_max;
    if (desired_alpha < -alpha_max) desired_alpha = -alpha_max;
    alpha_rw = desired_alpha + (tau_fric + tau_noise) / J_rw;
  }

  // Integrate speed
  omega_rw += alpha_rw * dt;

  // Safety clamp in RPM
  const float rpm_now = omega_rw * RAD2RPM;
  if (rpm_now > RPM_MAX) {
    omega_rw = RPM_MAX * RPM2RAD;
  } else if (rpm_now < -RPM_MAX) {
    omega_rw = -RPM_MAX * RPM2RAD;
  }
}

void visualizeSpeed() {
  // Blink rate proportional to angular frequency (purely cosmetic)
  const float freq = fabs(omega_rw) / (2.0f * PI_F); // [Hz]
  const float delay_ms = (freq > 0.5f) ? (500.0f / freq) : 1000.0f;

  static unsigned long lastBlink = 0;
  static bool state = false;

  if (millis() - lastBlink >= delay_ms) {
    state = !state;
    digitalWrite(LED_PIN, state);
    lastBlink = millis();
  }
}

void updateErrorLED() {
  digitalWrite(LED_PWR, errorFlag ? HIGH : LOW);
}

// ---------------- Status reply (9 bytes, CRC-16) ----------------
void sendStatusReply() {
  uint8_t reply[9];
  reply[0] = START_BYTE_REPLY;
  reply[1] = RESP_STATUS;

  // Telemetry values (rounded to nearest to avoid truncation bias)
  const float rpm_f     = omega_rw * RAD2RPM;
  const float tq_mNm_f  = J_rw * alpha_rw * 1000.0f; // Nm -> mNm (actual = J*alpha)

  int16_t speed_rpm  = static_cast<int16_t>(rpm_f    + (rpm_f    >= 0 ? 0.5f : -0.5f));
  int16_t torque_mNm = static_cast<int16_t>(tq_mNm_f + (tq_mNm_f >= 0 ? 0.5f : -0.5f));
  uint8_t running    = 0;

  if (torqueMode) {
    running = (fabs(tau_cmd_Nm) > 1e-6f) ? 1 : 0;
  } else {
    running = (fabs(omega_rw_target) > 1e-6f) ? 1 : 0;
  }

  // Pack big-endian payload
  reply[2] = static_cast<uint8_t>((speed_rpm >> 8) & 0xFF);
  reply[3] = static_cast<uint8_t>( speed_rpm       & 0xFF);
  reply[4] = static_cast<uint8_t>((torque_mNm >> 8) & 0xFF);
  reply[5] = static_cast<uint8_t>( torque_mNm       & 0xFF);
  reply[6] = running;

  // CRC-16 over bytes 0..6, append big-endian
  const uint16_t crc = crc16_ccitt(reply, 7);
  reply[7] = static_cast<uint8_t>((crc >> 8) & 0xFF);
  reply[8] = static_cast<uint8_t>( crc       & 0xFF);

  Serial.write(reply, sizeof(reply));
}
