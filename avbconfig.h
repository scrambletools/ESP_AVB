/*
 * Copyright 2024-2026 Scramble Tools
 * License: MIT
 *
 * ESP_AVB Component
 *
 * This component provides an implementation of an AVB talker and listener.
 *
 * This file provides the common definitions and types for the AVB protocol.
 */

#ifndef ESP_AVB_CONFIG_H_
#define ESP_AVB_CONFIG_H_

#if CONFIG_ESP_AVB_MILAN
#define AVB_DEFAULT_MILAN_COMPLIANT true
#else
#define AVB_DEFAULT_MILAN_COMPLIANT false
#endif

#if CONFIG_ESP_AVB_AVB_LITE_COMPLIANT
#define AVB_DEFAULT_AVB_LITE_COMPLIANT true
#else
#define AVB_DEFAULT_AVB_LITE_COMPLIANT false
#endif

/* Boot-time audio test. When AVB_AUDIO_TEST_BOOT_RATE_HZ is defined and
 * non-zero, the AVB task plays a 1 kHz sine through the codec for
 * AVB_AUDIO_TEST_DURATION_MS milliseconds at startup, after I2S/codec
 * init and before the main control loop begins. The I2S clock is
 * temporarily switched to the test rate and restored to
 * config.default_sample_rate after the tone ends. Useful for verifying
 * that each supported sample rate produces audible output through the
 * full I2S → codec path independent of any AVTP streaming. */
#ifndef AVB_AUDIO_TEST_BOOT_RATE_HZ
#define AVB_AUDIO_TEST_BOOT_RATE_HZ 0 /* 0 = test disabled */
#endif
#ifndef AVB_AUDIO_TEST_DURATION_MS
#define AVB_AUDIO_TEST_DURATION_MS 10000 /* 10 seconds */
#endif

/*
 * default AVB configuration
 *
 * note: dout and din represent data out and data in on on the mcu
 * dout should map to din on the codec, din should map to dout on the codec
 *
 * for ESP32-P4-ETH board using onboard es8311 the following pins are used:
 * MCLK: GPIO 13
 * BCLK: GPIO 12
 * WS: GPIO 10
 * DOUT: GPIO 9
 * DIN: GPIO 11
 * I2C_SDA: GPIO 7
 * I2C_SCL: GPIO 8
 * PA: GPIO 53 (active high)
 *
 * for ESP32-P4-ETH board using a Scramble Hat the following pins are used:
 * MCLK: GPIO 16
 * BCLK: GPIO 17
 * WS: GPIO 19
 * DOUT: GPIO 18
 * DIN: GPIO 54
 * I2C_SDA: GPIO 21
 * I2C_SCL: GPIO 20
 * PA: GPIO -1
 *
 * for ESP32-C6 test board using onboard es8311 the following pins are used:
 * MCLK: GPIO 19
 * BCLK: GPIO 20
 * WS: GPIO 22
 * DOUT: GPIO 21
 * DIN: GPIO 23
 * I2C_SDA: GPIO 8
 * I2C_SCL: GPIO 7
 * PA: GPIO 6
 *
 * pa_reverted: set true if the PA enable pin on the board is wired
 * through an inverter (active-low). The codec driver and the
 * identify/audio-test paths both honor this setting.
 */
#define AVB_DEFAULT_CONFIG()                                                   \
  {.talker = true,                                                             \
   .listener = true,                                                           \
   .atdecc_control = true,                                                     \
   .milan_compliant = AVB_DEFAULT_MILAN_COMPLIANT,                             \
   .avb_lite_compliant = AVB_DEFAULT_AVB_LITE_COMPLIANT,                       \
   .association_id = 0xffffffffffffffff,                                       \
   .model_id = 0x0000007468696e67,                                             \
   .port_id = 0x0001,                                                          \
   .entity_name = "Simple Talker/Listener",                                    \
   .vendor_name = "ACME",                                                      \
   .model_name = "AVB Device Model 1",                                         \
   .group_name = "",                                                           \
   .firmware_version = "1.0.0",                                                \
   .serial_number = "12345678",                                                \
   .eth_interface = DEF_ETH_IF,                                                \
   .i2s_port = 0,                                                              \
   .codec_pins = {.mclk = 13,                                                  \
                  .bclk = 12,                                                  \
                  .ws = 10,                                                    \
                  .dout = 9,                                                   \
                  .din = 11,                                                   \
                  .i2c_scl = 8,                                                \
                  .i2c_sda = 7,                                                \
                  .pa = 53,                                                    \
                  .pa_reverted = false},                                       \
   .eth_handle = NULL,                                                         \
   .codec_type = avb_codec_type_es8311,                                        \
   .default_sample_rate = 48000,                                               \
   .default_presentation_time_offset_ns = 2000000,                             \
   .default_bits_per_sample = 24,                                              \
   .input_channels_usable = 1,                                                 \
   .output_channels_usable = 1,                                                \
   .channels_per_stream = 8,                                                   \
   .num_allowed_sample_rates = 3,                                              \
   .allowed_sample_rates = {48000, 96000, 192000},                             \
   .num_allowed_bits_per_sample = 1,                                           \
   .allowed_bits_per_sample = {24},                                            \
   .default_mic_gain_tenth_db = 60,                                            \
   .default_speaker_vol_tenth_db = -100}

#define AVB_LOCALIZED_STRINGS_PER_DESCRIPTOR 7
#define AVB_LOCALIZED_STRINGS_DESCRIPTORS 3

typedef struct {
  const char *locale_identifier;
  const char *strings[AVB_LOCALIZED_STRINGS_DESCRIPTORS]
                     [AVB_LOCALIZED_STRINGS_PER_DESCRIPTOR];
} avb_locale_strings_s;

static const avb_locale_strings_s AVB_LOCALIZED_STRINGS[] = {
    {
        .locale_identifier = "en",
        .strings =
            {
                /* Strings descriptor 0: localized refs 0..6 */
                {
                    "Configuration",    /* 0: Configuration Name */
                    "ESP-AVB",          /* 1: Audio Unit Name */
                    "Mono Audio In",    /* 2: Stream Port Input Cluster Name */
                    "Mono Audio Out",   /* 3: Stream Port Output Cluster Name */
                    "Audio Stream In",  /* 4: Audio Stream Input Name */
                    "Audio Stream Out", /* 5: Audio Stream Output Name */
                    "Ethernet",         /* 6: AVB Interface Name */
                },
                /* Strings descriptor 1: localized refs 8..14 */
                {
                    "Logo",           /* 8: Memory Object Name */
                    "Identify",       /* 9: Identify Control Name */
                    "Clock Domain",   /* 10: Clock Domain Name */
                    "Internal Clock", /* 11: Internal Clock Source Name */
                    "",               /* 12: Vendor Name (from config) */
                    "",               /* 13: Model Name (from config) */
                    "Speaker Volume", /* 14: Speaker Volume Control Name */
                },
                /* Strings descriptor 2: localized refs 16..22 */
                {
                    "Mic Gain",            /* 16: Mic Gain Control Name */
                    "CRF Media Clock In",  /* 17: CRF Clock Input Name */
                    "CRF Media Clock Out", /* 18: CRF Clock Output Name */
                    "Volume",              /* 19: Speaker Volume Value Name */
                    "Gain",                /* 20: Mic Gain Value Name */
                    "Identify",            /* 21: Identify Value Name */
                    "",
                },
            },
    },
};

#define AVB_LOCALIZED_LOCALE_COUNT                                             \
  (sizeof(AVB_LOCALIZED_STRINGS) / sizeof(AVB_LOCALIZED_STRINGS[0]))

#endif /* ESP_AVB_CONFIG_H_ */
