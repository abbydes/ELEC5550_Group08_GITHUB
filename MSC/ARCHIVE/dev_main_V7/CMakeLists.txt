set(priv_requires fatfs console esp_tinyusb)
 
if(CONFIG_EXAMPLE_STORAGE_MEDIA_SPIFLASH)
    list(APPEND priv_requires wear_levelling esp_partition)
endif()
 
idf_component_register(
    SRCS "tusb_msc_main.c"
    INCLUDE_DIRS "."
    PRIV_INCLUDE_DIRS "C:/Users/abbyd/msc_dev/managed_components/espressif__esp_tinyusb/include_private"
    REQUIRES driver tinyusb freertos
    PRIV_REQUIRES ${priv_requires}
)


