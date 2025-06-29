// === Reaction Wheel Simulator for FSFW Communication ===
// Features: CRC8, binary packet parser, LED blink for RPM visualization

const byte START_BYTE_CMD   = 0xAA;
const byte START_BYTE_REPLY = 0xAB;

const byte CMD_SET_SPEED    = 0x01;
const byte CMD_STOP         = 0x02;
const byte CMD_STATUS       = 0x03;

const byte RESP_STATUS      = 0x10;

const int LED_PIN = LED_BUILTIN;

int speed = 0;            // Abstract RPM
bool running = false;

void setup() {
  Serial.begin(9600);
  while (!Serial);  // Wait until Serial is ready
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  //Serial.println("Reaction Wheel ready (binary mode)");
}

void loop() {
  handleSerialPackets();
  simulateReactionWheel();
}

// === Handle incoming 5-byte binary commands ===
void handleSerialPackets() {
  if (Serial.available() >= 5) {
    byte packet[5];
    Serial.readBytes(packet, 5);

    if (packet[0] != START_BYTE_CMD) return;
    if (crc8(packet, 4) != packet[4]) return; // Invalid CRC

    byte cmd = packet[1];
    uint16_t value = (packet[2] << 8) | packet[3];

    switch (cmd) {
      case CMD_SET_SPEED:
        speed = constrain(value, 0, 1000);  // Cap to avoid division by 0
        running = true;
        break;

      case CMD_STOP:
        speed = 0;
        running = false;
        break;

      case CMD_STATUS:
        sendStatusReply();
        break;
    }
  }
}

// === Simulated reaction wheel blinking ===
void simulateReactionWheel() {
  if (running && speed > 0) {
    digitalWrite(LED_PIN, HIGH);
    delay(5000 / speed);  // Blink rate ~proportional to RPM
    digitalWrite(LED_PIN, LOW);
    delay(5000 / speed);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
}

// === Send a 6-byte reply with status and speed ===
void sendStatusReply() {
  byte reply[6];
  reply[0] = START_BYTE_REPLY;
  reply[1] = RESP_STATUS;
  reply[2] = (speed >> 8) & 0xFF;
  reply[3] = speed & 0xFF;
  reply[4] = running ? 1 : 0;
  reply[5] = crc8(reply, 5);

  Serial.write(reply, 6);
}

// === CRC8 using x^8 + x^2 + x + 1 polynomial (0x07) ===
byte crc8(const byte* data, byte len) {
  byte crc = 0x00;
  for (byte i = 0; i < len; i++) {
    crc ^= data[i];
    for (byte j = 0; j < 8; j++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x07;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}
