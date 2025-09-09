This folder contains code for HID Encoding/Decoding

hid_device_v1_no_test
* for the receiver side
* does not incude dummy testing of random serialised data to test output of receiver

hid_host_esp-idf
* UNMODIFIED official example code from espressif ESP-IDF

hid_host_v3
* fully tested on keyboard and mouse HID devices
* last update: added serialisation of raw data, added checksum that appends every byte except header and checksum bytes

hid_keyboard_test
* keyboard not tested
