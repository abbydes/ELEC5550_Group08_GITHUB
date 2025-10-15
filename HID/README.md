This folder contains code for HID Encoding/Decoding

| HID Transmitter | Description | HID Receiver | Description |
| :-------: | :------ | :-------: | :------- |
| `hid_host_v7` | removed excess UART framing code for simplicity | `hid_device_v10` | based off hid_device_v9 with added LED functionality |
| `hid_host_v6` | increased baud rate to 8000000 | `hid_device_v9` | <ul><li>Works for keyboard & mouse on macbook using ESP32S3DevKitC-1 directly connecting Tx/Rx pins</li><li>Run instructions from hid_device_v7 --> idf.py menuconfig</li><li>fixed TUD_HID_DESCRIPTOR calls</li><li>corrected byte offsets for mouse data to include wheel</li><li>corrected interface numbering mismatch</li><li>added tud_mounted() check to ensure USB is fully enumerated before sending reports - using TinyUSB API calls tud_hid_n_ready(1) and tud_hid_n_mouse_report(1,...)</li><li>prints status every 5 seconds for USB UART connection status reporting</li></ul> |
| `hid_host_v5` | cleaned up comments | `hid_device_v8` | <ul><li>Added UART framing errors and a valid frame timeout</li><li>Globals have been added to track the state</li><li>When no valid frame has arrived for 200ms or if framing errors occur a UART Frame error is pushed into the UART queue</li></ul> |
| `hid_host_v4` | <ul><li>tested and validated sent data of up to baud rate of 912600 (in conjunction with hid_device_v3)</li><li>cleaned up comments (unfinished)</li></ul> | `hid_device_v7` | <ul><li>based off hid_device_v5</li><li>edited menuconfig - works with keyboard acting as device on mac using ESP32S3DevKitC-1 directly connecting Tx/Rx pins</li></ul>On the Receiver side:</li></ul><ul><li>Run in terminal - idf.py menuconfig</li><li>Component config -->TinyUSB Stack -->TinyUSB DCD -->DCD Mode -->Confirm that the x is in: (X) Buffer DMA</li><li>Go back to TinyUSB Stack--></li><li>Human Interface Device Class (HID) -->TinyUSB HID interfaces count -->Press 'Enter' to change the count number to 2 (one for keyboard, one for mouse)</li><li>Go back to TinyUSB Stack--></li><li>USB Peripheral (OTG1.1)-->Confirm that the x is in: (X) OTG1.1</li><li>Press S to save, press Q to quit back to terminal, Rebuild</li></ul> |
| `hid_host_v3` | <ul><li>Fully tested and verified on keyboard and mouse HID devices</li><li>Integrated serialisation of raw data</li><li>Checksum appends every byte except header and checksum bytes</li></ul> | `hid_device_v6` | changed layout of keyboard/mouse ESP_LOGI prints |
| `hid_host_v2` | x | `hid_device_v5` | changed from 1 interface to 2 interfaces |
| `hid_host_v1` | x | `hid_device_v4` | <ul><li>edited code to receive UART HID frames and send to computer to act as USB HID keyboard/mouse</li><li>removed app_send_hid_demo block, mouse_draw_square_next_delta(), constants DISTANCE_MAX, DELTA_SCALAR and its associated calls</li><li>cleaned up comments</li></ul> |
| ------------- | x | `hid_device_v3` | <ul><li>correct packet framing for received data</li><li>tested and validated received data of up to baud rate of 912600 (in conjunction with hid_host_v4)</li></ul> |
| ------------- | x | `hid_device_v2` | <ul><li>Integrated deserialisation and logging</li><li>Validates the checksum from the transmitter data</li><li>It prints human-readable keyboard/mouse data directly to the UART.</li><li>It also prints the same string to the command monitor using ESP_LOGI so you can see live activity.</li><li>Does not include dummy testing of random serialised data to test output of receiver</li></ul> |
| ------------- | x | `hid_device_v1` | <ul><li>x

| <ul><li>list item 1</li><li>list item 2</li></ul> | <ul><li>list item 1</li><li>list item 2</li></ul> |


`hid_host_esp-idf`
* UNMODIFIED official example code from espressif ESP-IDF
