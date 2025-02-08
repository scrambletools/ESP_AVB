idf_component_register(SRCS avb.c avbutils.c avbnet.c avbcodec.c avtp.c atdecc.c 
                       PRIV_REQUIRES esp_codec_dev esp_driver_i2s esp_driver_gpio esp_eth esp_netif esp_timer
                       INCLUDE_DIRS "." "./include" 
                       EMBED_FILES logo.png)

target_sources(${COMPONENT_LIB} PRIVATE)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-function)
