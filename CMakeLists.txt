# Source list.
#
# Always-compiled core: avb.c, avbutils.c, avbnet.c, msrp.c, avbstats.c.
#   - msrp.c holds the SRP family (MSRP / MVRP / MAAP) plus the
#     bridge-only admission control. Used by both endpoint and bridge
#     builds. The MSRP/MVRP/MAAP code lived in avtp.c historically;
#     it was misfiled there because AVTP per IEEE 1722 is the audio/
#     video transport, while SRP is IEEE 802.1Q reservation.
#
# Endpoint-only:
#   - avtp.c (now AVTPDU stream construction + talker/listener tasks
#     after the SRP move). Excluded from the bridge build because the
#     bridge has no talker/listener.
#   - atdecc.c, avbcodec.c, avbpll.c — bridge has no ATDECC entity,
#     no codec, no media-clock PLL.
#
# Bridge-only:
#   - avbbridge.c (L2 forwarding + ATDECC/codec/PLL no-op stubs)
#   - avbfqtss.c (802.1Qav credit-based shaper)

set(avb_srcs avb.c avbutils.c avbnet.c msrp.c avtp.c avbstats.c)

if(CONFIG_ESP_AVB_ROLE_BRIDGE)
    list(APPEND avb_srcs avbbridge.c avbfqtss.c)
else()
    list(APPEND avb_srcs avbcodec.c atdecc.c avbpll.c)
endif()

idf_component_register(SRCS ${avb_srcs}
                       REQUIRES esp_eth
                       PRIV_REQUIRES esp_codec_dev esp_driver_i2c esp_driver_i2s esp_driver_gpio esp_hal_clock esp_hw_support esp_netif esp_timer nvs_flash
                       INCLUDE_DIRS "." "./include"
                       EMBED_FILES logo.png)

target_sources(${COMPONENT_LIB} PRIVATE)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-function)
