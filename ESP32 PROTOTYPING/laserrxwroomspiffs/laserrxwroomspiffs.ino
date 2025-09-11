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

File rxFile;
String currentFile;
uint32_t expectedSize=0;
uint32_t receivedSize=0;

void sendAck(bool ok){
  LaserSerial.write(ok?ACK:NAK);
}

void handlePacket(uint8_t type,uint8_t *data,uint8_t len){
  if(type==0x01){ // FILE_INFO
    uint8_t nameLen = data[0];
    currentFile=String((char*)&data[1],nameLen);
    memcpy(&expectedSize,&data[1+nameLen],sizeof(expectedSize));
    receivedSize=0;
    if(rxFile) rxFile.close();
    rxFile=SPIFFS.open("/"+currentFile,"w");
    Serial.printf("Receiving file: %s (%d bytes)\n",currentFile.c_str(),expectedSize);
  } else if(type==0x02){ // FILE_DATA
    if(rxFile){
      rxFile.write(data,len);
      receivedSize+=len;
      Serial.printf("Received chunk (%d/%d)\n",receivedSize,expectedSize);
    }
  } else if(type==0x03){ // FILE_END
    if(rxFile){
      rxFile.close();
      Serial.printf("File %s saved (%d bytes)\n",currentFile.c_str(),receivedSize);
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
  Serial.println("Laser RX + USB-Serial Bridge ready");
}

void loop(){
  // --- Laser packet handling with ACK/NAK ---
  static enum { WAIT_START, WAIT_TYPE, WAIT_LEN, WAIT_DATA, WAIT_CHECKSUM } state=WAIT_START;
  static uint8_t buffer[MAX_PAYLOAD];
  static uint8_t type=0,len=0,index=0,checksum=0;

  while(LaserSerial.available()){
    uint8_t b=LaserSerial.read();
    switch(state){
      case WAIT_START:
        if(b==START_BYTE){state=WAIT_TYPE;checksum=0;}
        break;
      case WAIT_TYPE:
        type=b; checksum+=b; state=WAIT_LEN;
        break;
      case WAIT_LEN:
        len=b; checksum+=b; index=0;
        if(len>0 && len<=MAX_PAYLOAD) state=WAIT_DATA;
        else if(len==0) state=WAIT_CHECKSUM;
        else state=WAIT_START;
        break;
      case WAIT_DATA:
        buffer[index++]=b; checksum+=b;
        if(index>=len) state=WAIT_CHECKSUM;
        break;
      case WAIT_CHECKSUM:
        if(checksum==b){ handlePacket(type,buffer,len); sendAck(true);}
        else { Serial.println("Checksum error!"); sendAck(false);}
        state=WAIT_START;
        break;
    }
  }

  // --- USB Serial Bridge commands (LIST, READ, DELETE) ---
  static String command;
  while(Serial.available()){
    char c=Serial.read();
    if(c=='\n'||c=='\r'){
      command.trim();
      if(command.equalsIgnoreCase("LIST")){
        Serial.println("Files on SPIFFS:");
        File root=SPIFFS.open("/");
        File file=root.openNextFile();
        while(file){
          Serial.println(file.name());
          file=root.openNextFile();
        }
      } else if(command.startsWith("READ ")){
        String fname=command.substring(5);
        File f=SPIFFS.open("/"+fname,"r");
        if(f){
          uint32_t size=f.size();
          Serial.write((const uint8_t*)&size,sizeof(size));
          uint8_t buf[128];
          while(f.available()){
            size_t n=f.read(buf,sizeof(buf));
            Serial.write(buf,n);
          }
          f.close();
        } else {
          uint32_t zero=0;
          Serial.write((const uint8_t*)&zero,sizeof(zero));
        }
      } else if(command.startsWith("DELETE ")){
        String fname=command.substring(7);
        if(SPIFFS.exists("/"+fname)){
          SPIFFS.remove("/"+fname);
          Serial.printf("Deleted file: %s\n",fname.c_str());
        } else Serial.printf("File not found: %s\n",fname.c_str());
      } else {
        Serial.println("Unknown command!");
      }
      command="";
    } else command+=c;
  }
}
