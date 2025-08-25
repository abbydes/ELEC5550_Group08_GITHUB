# ELEC5550 Group08 GITHUB Repository

## USB Host and Device Function
The USB host and device functionality is based on the ESP-IDF library which can be found [here](https://github.com/espressif/esp-idf?tab=readme-ov-file). 
The project requires an ESP32-S3 to host for both [USB HID](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/usb/host/hid) and [USB MSC](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/usb/host/msc) devices while another ESP32-S3 acts as a [USB HID](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/usb/device/tusb_hid) and/or [USB MSC](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/usb/device/tusb_msc) device to an external laptop for system transparency.

Code regarding the USB host & device functionality can be found in the USB folder.

## Handshake Protocol
The project outlines the use of two modules that can communicate with one another through free-space optical communication. The handshake protocol is the method used to establish the laser connection in the case of initialisation and will be used if the laser is interrupted and connection must be reestablished. 

Code regarding the Handshake protocol can be found in the Handshake folder.

## USB MSC Virtual Storage

## Error Correction

## USB HID Encoding/Decoding

## USB MSC Encoding/Decoding
