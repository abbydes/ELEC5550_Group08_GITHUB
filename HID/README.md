This folder contains code for HID Encoding/Decoding

| HID Transmitter | Description | HID Receiver | Description |
| :-------: | :------: | :-------: | :-------: |
| Row 1 Col 1 | Row 1 Col 2 | ### hid_device_v10 | * based off hid_device_v9 with added LED functionality |
| Row 2 Col 1 | Row 2 Col 2 | Row 2 Col 3 | Row 1 Col 3 |

### hid_device_v10
* based off hid_device_v9 with added LED functionality

### hid_device_v9
* Works for keyboard & mouse on macbook using ESP32S3DevKitC-1 directly connecting Tx/Rx pins
* Run instructions from hid_device_v7 --> idf.py menuconfig
* fixed TUD_HID_DESCRIPTOR calls
* corrected byte offsets for mouse data to include wheel
* corrected interface numbering mismatch
* added tud_mounted() check to ensure USB is fully enumerated before sending reports - using TinyUSB API calls tud_hid_n_ready(1) and tud_hid_n_mouse_report(1,...)
* prints status every 5 seconds for USB UART connection status reporting

### hid_host_v7
* removed excess UART framing code for simplicity

### hid_device_v8
* Added UART framing errors and a valid frame timeout
* Globals have been added to track the state
* When no valid frame has arrived for 200ms or if framing errors occur a UART Frame error is pushed into the UART queue
* Needs to be tested

### hid_device_v7
* based off hid_device_v5
* edited menuconfig - works with keyboard acting as device on mac using ESP32S3DevKitC-1 directly connecting Tx/Rx pins
*  On the Receiver side:
	1. Run in terminal - idf.py menuconfig
	2. Component config -->TinyUSB Stack -->TinyUSB DCD -->DCD Mode -->Confirm that the x is in: (X) Buffer DMA

	1. Go back to TinyUSB Stack-->
	2. Human Interface Device Class (HID) -->TinyUSB HID interfaces count -->Press 'Enter' to change the count number to 2 (one for keyboard, one for mouse)

	1. Go back to TinyUSB Stack-->
	2. USB Peripheral (OTG1.1)-->Confirm that the x is in: (X) OTG1.1
	3. Press S to save, press Q to quit back to terminal, Rebuild

### hid_device_v6
* changed layout of keyboard/mouse ESP_LOGI prints

### hid_device_v5
* changed from 1 interface to 2 interfaces

### hid_host_v5
* cleaned up comments

### hid_device_v4
* edited code to receive UART HID frames and send to computer to act as USB HID keyboard/mouse
  * remove app_send_hid_demo block, mouse_draw_square_next_delta(), constants DISTANCE_MAX, DELTA_SCALAR and its associated calls
* cleaned up comments

### hid_host_v4
* tested and validated sent data of up to baud rate of 912600 (in conjunction with hid_device_v3) - changed baud rate manually
* cleaned up comments (unfinished)

### hid_device_v3
* correct packet framing for received data
* tested and validated received data of up to baud rate of 912600 (in conjunction with hid_host_v4)

### hid_host_v3
* For transmitter side
* Fully tested and verified on keyboard and mouse HID devices
* Serialisation of raw data is integrated
* Checksum appends every byte except header and checksum bytes

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

### hid_keyboard_test
* keyboard not tested
