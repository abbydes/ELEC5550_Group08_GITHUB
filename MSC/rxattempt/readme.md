This file is a modified version of tusb_rx_V1 that attempts to mount a block-level USB to the computer. Unfortunately, Windows recognises there is a disc, but it does not open. The modified code includes:

main/CMakeLists

tusb_msc_main.c

and potentially c_cpp_properties.json

To verify code, modify the main/CmakeLists and tusb_msc_main.c as required. If it does not build, then change the c_cpp_properties.json with the correct corresponding paths.
