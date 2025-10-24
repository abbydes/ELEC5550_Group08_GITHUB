idf_component_register(SRCS "msc_example_main.c"
                    INCLUDE_DIRS ""
                    REQUIRES driver
                    PRIV_REQUIRES usb fatfs esp_driver_gpio esp_timer
                    )
