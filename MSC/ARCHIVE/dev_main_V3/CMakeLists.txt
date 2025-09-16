set(priv_requires fatfs console)

if(CONFIG_EXAMPLE_STORAGE_MEDIA_SPIFLASH)
    list(APPEND priv_requires wear_levelling esp_partition)
endif()

idf_component_register(
    SRCS "tusb_msc_main.c"
    INCLUDE_DIRS "."
    REQUIRES driver
    PRIV_REQUIRES ${priv_requires}
)
