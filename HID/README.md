This folder contains code for HID Encoding/Decoding

### hid_device_v2
* For the receiver side
* Does not include dummy testing of random serialised data to test output of receiver
* Validates the checksum from the transmitter data
* The deserialization and logging is already integrated.
* It prints human-readable keyboard/mouse data directly to the UART.
* It also sends the actual HID report via TinyUSB, so the PC sees real input.
* It also prints the same string to the command monitor using ESP_LOGI so you can see live activity.

### hid_host_esp-idf
* UNMODIFIED official example code from espressif ESP-IDF

### hid_host_v3
* For transmitter side
* Fully tested and verified on keyboard and mouse HID devices
* Serialisation of raw data is integrated
* Checksum appends every byte except header and checksum bytes

### hid_keyboard_test
* keyboard not tested
