# ELEC5550 Group08 GITHUB Repository

## USB Host and Device Function
The USB host and device functionality is based on the ESP-IDF library which can be found [here](https://github.com/espressif/esp-idf?tab=readme-ov-file). 
The project requires an ESP32-S3 to host for both [USB HID](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/usb/host/hid) and [USB MSC](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/usb/host/msc) devices while another ESP32-S3 acts as a [USB HID](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/usb/device/tusb_hid) and/or [USB MSC](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/usb/device/tusb_msc) device to an external laptop for system transparency.

Code regarding the USB host & device functionality can be found in the USB folder.

## Handshake Protocol
The project outlines the use of two modules that can communicate with one another through free-space optical communication. The handshake protocol is the method used to establish the laser connection in the case of initialisation and will be used if the laser is interrupted and connection must be reestablished. 

Code regarding the Handshake protocol can be found in the HANDSHAKE folder.

## USB MSC Virtual Storage
A virtual storage system will be required on the module connected to the host laptop as the USB MSC protocol may be too strenuous to implement over serial laser communication. As such, the contents of the USB file will be transferred over the laser and reconstructed in the virtual storage.

Code regarding the Virtual Storage can be found in the VIRTUAL_STORAGE folder.

## Error Correction
For the data to be transferred correctly between modules via the laser a method of error correction must be implemented. The type of error correction that will be implemented is a combination of cyclical redundancy check (CRC) checksum with stop-and-wait Automatic Repeat request (ARQ).

Code regarding the Error Correction can be found in the ERROR_CORRECTION folder.

## USB HID Encoding/Decoding
The project specifies the connection of a mouse or keyboard into a module which then encodes the HID data, transfers it over the laser, decodes it and provides it to the host laptop.

Code regarding HID Encoding/decoding can be found in the HID folder.

## USB MSC Encoding/Decoding
The project specifies the connection of a USB flashdrive into a module which then encodes the MSC data, transfers it over the laser, decodes it and provides it to the host laptop.

Code regarding MSC Encoding/decoding can be found in the MSC folder.

## ESP32 Prototyping
This project includes the creation of both hardware and software. Rudimentary pieces of code were used to verify hardware design using the ESP32-S3 as a stepping stone to PCB design.

Code regarding ESP32-S3 hardware verification can be found in the ESP32 PROTOTYPING folder.

## PCB Prototyping
Similary to the ESP32-S3 prototype testing, rudimentary pieces of code were required to verify the PCBs functionality in comparison to the ESP32-S3 prototype it was based on.

Code regarding PCB hardware verification can be found in the PCB PROTOTYPING folder.

The PCB prototyping folder also includes the KICAD design documents which are located in the KICAD DESIGN folder under PCB Prototyping

## Finalised
All of the code must be finalised and submitted in a finalised github repository.

Finalised code can be found in the FINALISED folder.

## File Management
All files are to be uploaded to their respective folders. New file versions to be created each week to allow for proper version control.
