idf_component_register(SRCS avb.c avbutils.c avbnet.c avtp.c atdecc.c 
                       PRIV_REQUIRES esp_eth esp_netif esp_timer driver
                       INCLUDE_DIRS "." "./include")

target_sources(${COMPONENT_LIB} PRIVATE)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-function)
