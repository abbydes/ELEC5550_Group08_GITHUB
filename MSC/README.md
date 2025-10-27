This folder contains code for MSC encoding/decoding

The host and device code is a modified version of the ESP-IDF host and device examples which can be found here:

[USB-MSC Host Example](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/usb/host/msc)

[USB-MSC Device Example](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/usb/device/tusb_msc_)


| MSC Module B | Description | MSC Module A | Description |
| :-------: | :------ | :-------: | :------- |
| -------------  | x | `dev_main_V9` | based off hid_device_v9 with added LED functionality |
| `host_main_V8` | increased baud rate to 8000000 | `dev_main_V8` | <ul><li>Works for keyboard & mouse on macbook using ESP32S3DevKitC-1 directly connecting Tx/Rx pins</li><li>Run instructions from hid_device_v7 --> idf.py menuconfig</li><li>fixed TUD_HID_DESCRIPTOR calls</li><li>corrected byte offsets for mouse data to include wheel</li><li>corrected interface numbering mismatch</li><li>added tud_mounted() check to ensure USB is fully enumerated before sending reports - using TinyUSB API calls tud_hid_n_ready(1) and tud_hid_n_mouse_report(1,...)</li><li>prints status every 5 seconds for USB UART connection status reporting</li></ul> |
| `host_main_V7` | cleaned up comments | `dev_main_V7` | <ul><li>Added UART framing errors and a valid frame timeout</li><li>Globals have been added to track the state</li><li>When no valid frame has arrived for 200ms or if framing errors occur a UART Frame error is pushed into the UART queue</li></ul> |
| `host_main_V6` | <ul><li>tested and validated sent data of up to baud rate of 912600 (in conjunction with hid_device_v3)</li><li>cleaned up comments (unfinished)</li></ul> | `dev_main_V6` | <ul><li>based off hid_device_v5</li><li>edited menuconfig - works with keyboard acting as device on mac using ESP32S3DevKitC-1 directly connecting Tx/Rx pins</li></ul>On the Receiver side:</li></ul><ul><li>Run in terminal - idf.py menuconfig</li><li>Component config -->TinyUSB Stack -->TinyUSB DCD -->DCD Mode -->Confirm that the x is in: (X) Buffer DMA</li><li>Go back to TinyUSB Stack--></li><li>Human Interface Device Class (HID) -->TinyUSB HID interfaces count -->Press 'Enter' to change the count number to 2 (one for keyboard, one for mouse)</li><li>Go back to TinyUSB Stack--></li><li>USB Peripheral (OTG1.1)-->Confirm that the x is in: (X) OTG1.1</li><li>Press S to save, press Q to quit back to terminal, Rebuild</li></ul> |
| `host_main_V5` | <ul><li>Fully tested and verified on keyboard and mouse HID devices</li><li>Integrated serialisation of raw data</li><li>Checksum appends every byte except header and checksum bytes</li></ul> | `dev_main_V5` | changed layout of keyboard/mouse ESP_LOGI prints |
| `host_main_V4` | x | `dev_main_V4` | changed from 1 interface to 2 interfaces |
| `host_main_V3` | x | `dev_main_V3` | <ul><li>edited code to receive UART HID frames and send to computer to act as USB HID keyboard/mouse</li><li>removed app_send_hid_demo block, mouse_draw_square_next_delta(), constants DISTANCE_MAX, DELTA_SCALAR and its associated calls</li><li>cleaned up comments</li></ul> |
| `host_main_V2` | x | `dev_main_V2` | <ul><li>correct packet framing for received data</li><li>tested and validated received data of up to baud rate of 912600 (in conjunction with hid_host_v4)</li></ul> |
| `host_main_V1` | x | `dev_main_V1` | <ul><li>Integrated deserialisation and logging</li><li>Validates the checksum from the transmitter data</li><li>It prints human-readable keyboard/mouse data directly to the UART.</li><li>It also prints the same string to the command monitor using ESP_LOGI so you can see live activity.</li><li>Does not include dummy testing of random serialised data to test output of receiver</li></ul> |

