#include "USB.h"
#include "USBMSC.h"

USBMSC MSC;

#define DISK_SECTOR_SIZE 512
#define DISK_SECTOR_COUNT 128   // 128 * 512 = 64 kb

uint8_t disk[DISK_SECTOR_COUNT * DISK_SECTOR_SIZE];

int32_t onRead(uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize) {
  memcpy(buffer, disk + lba * DISK_SECTOR_SIZE + offset, bufsize);
  return bufsize;
}

int32_t onWrite(uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize) {
  memcpy(disk + lba * DISK_SECTOR_SIZE + offset, buffer, bufsize);
  return bufsize;
}

bool onStartStop(uint8_t power_condition, bool start, bool load_eject) {
  return true;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting MSC RAM disk (1 MB)...");

  MSC.vendorID("ESP32");
  MSC.productID("RAMDisk");
  MSC.productRevision("1.0");

  MSC.onRead(onRead);
  MSC.onWrite(onWrite);
  MSC.onStartStop(onStartStop);

  MSC.mediaPresent(true);
  MSC.begin(DISK_SECTOR_COUNT, DISK_SECTOR_SIZE);

  USB.begin();
}

void loop() {
}
