#include <Arduino.h>
HardwareSerial LaserSerial(1);

#define START_BYTE 0x7E
#define MAX_PAYLOAD 64
#define ACK 0x06
#define NAK 0x15

void sendPacket(const uint8_t *data, uint8_t len) {
  uint8_t checksum = len;
  for (uint8_t i = 0; i < len; i++) checksum += data[i];

  LaserSerial.write(START_BYTE);
  LaserSerial.write(len);
  LaserSerial.write(data, len);
  LaserSerial.write(checksum);
}

bool waitForAck(unsigned long timeout = 200) {
  unsigned long start = millis();
  while (millis() - start < timeout) {
    if (LaserSerial.available()) {
      uint8_t b = LaserSerial.read();
      if (b == ACK) return true;
      if (b == NAK) return false;
    }
  }
  return false; // timeout
}

void setup() {
  Serial.begin(115200);
  LaserSerial.begin(9600, SERIAL_8N1, 16, 17); // TX pin
  Serial.println("Laser UART TX ready");
}

void loop() {
  const char msg[] = "PACKET WITH ACK TEST!";
  bool success = false;

  while (!success) {
    sendPacket((uint8_t*)msg, strlen(msg));
    Serial.println("Sent packet, waiting for ACK...");
    success = waitForAck();
    if (!success) {
      Serial.println("No ACK (timeout or NAK), retrying...");
      delay(100); // short wait before retry
    }
  }

  Serial.println("Packet ACKed by receiver!");
  delay(1000); // send next packet in 1s
}
