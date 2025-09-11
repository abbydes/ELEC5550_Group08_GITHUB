idf_component_register(
    SRCS "msc_example_main.c"
    INCLUDE_DIRS "."   # Keep as "." if headers are in main
    PRIV_REQUIRES usb usb_host_msc fatfs esp_driver_gpio esp_timer esp_driver_uart
)
