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

/**
 * default AVB configuration
 */
#define AVB_DEFAULT_CONFIG()                                                   \
  {.talker = true,                                                             \
   .listener = true,                                                           \
   .atdecc_control = true,                                                     \
   .milan_compliant = AVB_DEFAULT_MILAN_COMPLIANT,                             \
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
                  .pa = 53},                                                   \
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
