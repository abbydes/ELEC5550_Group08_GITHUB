#include <Arduino.h>
#include <FS.h>
#include "SPIFFS.h"

HardwareSerial LaserSerial(1);

#define START_BYTE 0x7E
#define MAX_PAYLOAD 64
#define ACK 0x06
#define NAK 0x15

void sendPacket(uint8_t type, const uint8_t *data, uint8_t len) {
  uint8_t checksum = type + len;
  for (uint8_t i = 0; i < len; i++) checksum += data[i];

  LaserSerial.write(START_BYTE);
  LaserSerial.write(type);
  LaserSerial.write(len);
  LaserSerial.write(data, len);
  LaserSerial.write(checksum);
}

bool waitForAck(unsigned long timeout = 500) {
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

void reliableSend(uint8_t type, const uint8_t *data, uint8_t len) {
  bool success = false;
  while (!success) {
    sendPacket(type, data, len);
    success = waitForAck();
    if (!success) {
      Serial.println("Retrying packet...");
      delay(100);
    }
  }
}

void setup() {
  Serial.begin(115200);
  LaserSerial.begin(9600, SERIAL_8N1, 16, 17); // TX pin
  Serial.println("Laser UART TX ready");

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed");
    return;
  }

  // Example file to send
  File file = SPIFFS.open("/test.txt", "r");
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  // 1️⃣ Send FILE_INFO
  String filename = "test.txt";
  uint32_t filesize = file.size();
  uint8_t info[64];
  int idx = 0;
  info[idx++] = filename.length();
  memcpy(&info[idx], filename.c_str(), filename.length());
  idx += filename.length();
  memcpy(&info[idx], &filesize, sizeof(filesize));
  idx += sizeof(filesize);

  reliableSend(0x01, info, idx);
  Serial.println("Sent FILE_INFO");

  // 2️⃣ Send FILE_DATA in chunks
  uint8_t buffer[MAX_PAYLOAD];
  while (file.available()) {
    int len = file.read(buffer, MAX_PAYLOAD);
    reliableSend(0x02, buffer, len);
    Serial.printf("Sent %d bytes\n", len);
  }

  // 3️⃣ Send FILE_END
  reliableSend(0x03, NULL, 0);
  Serial.println("File transfer complete");

  file.close();
}

void loop() {}
