#include <Arduino.h>

HardwareSerial LaserSerial(1);

void setup() {
  Serial.begin(115200);           // debug via USB
  LaserSerial.begin(115200, SERIAL_8N1, -1, 17); // TX on GPIO17
  Serial.println("Laser UART TX ready");
}

void loop() {
  LaserSerial.println("HELLO");   // send test word
  Serial.println("Sent HELLO");
  delay(1000);                    // 1 second pause
}
