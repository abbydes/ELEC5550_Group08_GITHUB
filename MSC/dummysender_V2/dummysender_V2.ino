#include <Arduino.h>

#define LASER_UART   Serial1
#define LASER_TX_PIN 17   // change to your wiring
#define LASER_RX_PIN 16   // RX not needed for one-way send
#define BAUDRATE     115200

void setup() {
  Serial.begin(115200);
  LASER_UART.begin(115200, SERIAL_8N1, LASER_RX_PIN, LASER_TX_PIN);

  delay(2000);
  Serial.println("TX: Sending dummy file...");

  const char *filename = "dummy.txt";
  const char *content  = "Hello Laser World!\r\nThis is your transmitter speaking";

  LASER_UART.print("FILE:");
  LASER_UART.println(filename);
  LASER_UART.print("CONTENT:");
  LASER_UART.print(content);
  LASER_UART.println("END");

  // Wait for ACK/NAK
  unsigned long start = millis();
  while (!LASER_UART.available() && (millis() - start < 2000)) {
    delay(10);
  }
  if (LASER_UART.available()) {
    String resp = LASER_UART.readStringUntil('\n');
    Serial.print("RX replied: ");
    Serial.println(resp);
  } else {
    Serial.println("No reply from RX (timeout).");
  }
}

void loop() {
  // idle
}