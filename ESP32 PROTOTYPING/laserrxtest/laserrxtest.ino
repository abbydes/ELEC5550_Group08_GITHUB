#include <Arduino.h>

HardwareSerial LaserSerial(1);

void setup() {
  Serial.begin(115200);           // debug via USB
  LaserSerial.begin(115200, SERIAL_8N1, 16, -1); // RX on GPIO16
  Serial.println("Laser UART RX ready");
}

void loop() {
  if (LaserSerial.available()) {
    char c = LaserSerial.read();
    Serial.write(c);              // show directly on USB serial monitor
  }
}
