idf_component_register(SRCS avb.c avbutils.c avbnet.c avbcodec.c avtp.c atdecc.c avbpll.c
                       PRIV_REQUIRES esp_codec_dev esp_driver_i2c esp_driver_i2s esp_driver_gpio esp_eth esp_hal_clock esp_hw_support esp_netif esp_timer nvs_flash
                       INCLUDE_DIRS "." "./include"
                       EMBED_FILES logo.png)

target_sources(${COMPONENT_LIB} PRIVATE)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-function)
