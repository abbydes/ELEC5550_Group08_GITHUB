#include <Arduino.h>
HardwareSerial LaserSerial(1);

#define START_BYTE 0x7E
#define MAX_PAYLOAD 64
#define ACK 0x06
#define NAK 0x15

void setup() {
  Serial.begin(115200);
  LaserSerial.begin(9600, SERIAL_8N1, 16, 17); // RX pin
  Serial.println("Laser UART RX ready");
}

void loop() {
  static enum { WAIT_START, WAIT_LEN, WAIT_DATA, WAIT_CHECKSUM } state = WAIT_START;
  static uint8_t buffer[MAX_PAYLOAD];
  static uint8_t len = 0, index = 0, checksum = 0;

  while (LaserSerial.available()) {
    uint8_t b = LaserSerial.read();

    switch (state) {
      case WAIT_START:
        if (b == START_BYTE) {
          state = WAIT_LEN;
          checksum = 0;
        }
        break;

      case WAIT_LEN:
        len = b;
        if (len > 0 && len <= MAX_PAYLOAD) {
          index = 0;
          checksum = len;
          state = WAIT_DATA;
        } else {
          state = WAIT_START;
        }
        break;

      case WAIT_DATA:
        buffer[index++] = b;
        checksum += b;
        if (index >= len) state = WAIT_CHECKSUM;
        break;

      case WAIT_CHECKSUM:
        if (checksum == b) {
          Serial.print("Received packet: ");
          for (int i = 0; i < len; i++) Serial.write(buffer[i]);
          Serial.println();
          LaserSerial.write(ACK); // tell TX success
        } else {
          Serial.println("Checksum error!");
          LaserSerial.write(NAK); // request resend
        }
        state = WAIT_START;
        break;
    }
  }
}
