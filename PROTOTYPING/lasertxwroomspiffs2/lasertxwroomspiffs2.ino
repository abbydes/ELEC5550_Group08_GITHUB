#include <Arduino.h>
#include <FS.h>
#include "SPIFFS.h"

HardwareSerial LaserSerial(1);

#define TX_PIN 17
#define RX_PIN 16
#define START_BYTE 0x7E
#define MAX_PAYLOAD 64
#define ACK 0x06
#define NAK 0x15

// Generate next available demo filename: demo1.txt, demo2.txt, ...
String getNextDemoFilename() {
  int num = 1;
  String fname;
  do {
    fname = "/demo" + String(num) + ".txt";
    num++;
  } while(SPIFFS.exists(fname));
  return fname;
}

void sendPacket(uint8_t type, const uint8_t *data, uint8_t len) {
  uint8_t checksum = type + len;
  for(uint8_t i=0;i<len;i++) checksum+=data[i];

  LaserSerial.write(START_BYTE);
  LaserSerial.write(type);
  LaserSerial.write(len);
  LaserSerial.write(data,len);
  LaserSerial.write(checksum);
}

bool waitForAck(unsigned long timeout=500){
  unsigned long start=millis();
  while(millis()-start < timeout){
    if(LaserSerial.available()){
      uint8_t b = LaserSerial.read();
      if(b==ACK) return true;
      if(b==NAK) return false;
    }
  }
  return false;
}

void reliableSend(uint8_t type,const uint8_t *data,uint8_t len){
  bool success=false;
  while(!success){
    sendPacket(type,data,len);
    success=waitForAck();
    if(!success){
      Serial.println("Retrying packet...");
      delay(100);
    }
  }
}

void setup(){
  Serial.begin(115200);
  LaserSerial.begin(9600,SERIAL_8N1,RX_PIN,TX_PIN);

  if(!SPIFFS.begin(true)){
    Serial.println("SPIFFS mount failed!");
    while(1);
  }

  Serial.println("Laser TX ready");

  // Generate unique demo filename
  String filename = getNextDemoFilename();

  // Create demo file in SPIFFS
  const char* demoText = "Hello from the Transmitter!\nThis is a second line.\n";
  File demoFile = SPIFFS.open(filename, FILE_WRITE);
  if(demoFile){
    demoFile.print(demoText);
    demoFile.close();
    Serial.println("Demo file created: " + filename);
  } else {
    Serial.println("Failed to create demo file!");
    return;
  }

  // Open demo file for sending
  File file = SPIFFS.open(filename,"r");
  if(!file){
    Serial.println("Failed to open demo file for reading");
    return;
  }

  // Send FILE_INFO
  uint32_t filesize=file.size();
  uint8_t info[64];
  int idx=0;
  info[idx++]=filename.length();
  memcpy(&info[idx],filename.c_str(),filename.length());
  idx+=filename.length();
  memcpy(&info[idx],&filesize,sizeof(filesize));
  idx+=sizeof(filesize);

  reliableSend(0x01,info,idx);

  // Send FILE_DATA
  uint8_t buffer[MAX_PAYLOAD];
  while(file.available()){
    int len=file.read(buffer,MAX_PAYLOAD);
    reliableSend(0x02,buffer,len);
  }

  // Send FILE_END
  reliableSend(0x03,NULL,0);
  Serial.println("Demo file transfer complete: " + filename);
  file.close();
}

void loop(){}
