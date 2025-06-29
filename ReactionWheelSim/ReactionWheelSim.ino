// === Reaction Wheel Simulator mit realer Dynamik ===

const byte START_BYTE_CMD   = 0xAA;
const byte START_BYTE_REPLY = 0xAB;

const byte CMD_SET_SPEED    = 0x01;
const byte CMD_STOP         = 0x02;
const byte CMD_STATUS       = 0x03;

const byte RESP_STATUS      = 0x10;

const int LED_PIN = LED_BUILTIN;

// Physikalische Parameter
const float J_rw = 0.001f;         // [kg m^2] Trägheit des Reaktionsrads
const float J_sc = 0.5f;           // [kg m^2] Trägheit des Raumfahrzeugs (virtuell)
const float alpha_max = 5.0f;      // [rad/s^2] maximale Beschleunigung
const float dt = 0.01f;            // [s] Simulationszeit-Schritt (10ms)

// Zustände
float omega_rw = 0.0f;             // [rad/s] aktuelle Raddrehzahl
float omega_rw_target = 0.0f;      // [rad/s] Zielgeschwindigkeit
float alpha_rw = 0.0f;             // [rad/s^2] aktuelle Beschleunigung
bool errorFlag = false;            // Fehlerstatus (z. B. ungültiges Paket)

unsigned long lastUpdate = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_PWR, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(LED_PWR, LOW);
}

void loop() {
  handleSerialPackets();
  updateWheelDynamics();
  visualizeSpeed();
  updateErrorLED();
}

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

    errorFlag = false; // Paket korrekt, Fehlerstatus zurücksetzen
    byte cmd = packet[1];
    uint16_t value = (packet[2] << 8) | packet[3];

    switch (cmd) {
      case CMD_SET_SPEED:
        omega_rw_target = value * 2 * PI / 60.0f; // [U/min] zu [rad/s]
        break;
      case CMD_STOP:
        omega_rw_target = 0.0f;
        break;
      case CMD_STATUS:
        sendStatusReply();
        break;
      default:
        errorFlag = true; // unbekannter Befehl
        break;
    }
  }
}

void updateWheelDynamics() {
  unsigned long now = millis();
  if (now - lastUpdate < 10) return;
  lastUpdate = now;

  float error = omega_rw_target - omega_rw;
  float desired_alpha = constrain(error / dt, -alpha_max, alpha_max);

  alpha_rw = desired_alpha;
  omega_rw += alpha_rw * dt;
}

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

void updateErrorLED() {
  if (errorFlag) {
    digitalWrite(LED_PWR, HIGH); // Fehleranzeige EIN
  } else {
    digitalWrite(LED_PWR, LOW);  // Kein Fehler
  }
}

void sendStatusReply() {
  byte reply[8];
  reply[0] = START_BYTE_REPLY;
  reply[1] = RESP_STATUS;
  int16_t speed_rpm = omega_rw * 60.0f / (2 * PI); // rad/s → U/min
  int16_t torque_mNm = J_rw * alpha_rw * 1000.0f;   // Nm → mNm
  reply[2] = (speed_rpm >> 8) & 0xFF;
  reply[3] = speed_rpm & 0xFF;
  reply[4] = (torque_mNm >> 8) & 0xFF;
  reply[5] = torque_mNm & 0xFF;
  reply[6] = (omega_rw_target != 0.0f) ? 1 : 0;
  reply[7] = crc8(reply, 7);

  Serial.write(reply, 8);
}

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
