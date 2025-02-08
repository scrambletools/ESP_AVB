/*
 * Copyright 2024 Scramble Tools
 * License: MIT
 *
 * ESP_AVB Component
 *
 * This component provides an implementation of an AVB talker and listener.
 * 
 * This file provides the public API for the ESP_AVB component.
 */

#ifndef _ESP_AVB_H_
#define _ESP_AVB_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <time.h>

/****************************************************************************
 * Compiler Definitions
 ****************************************************************************/

/* Default Ethernet interface */
#define DEF_ETH_IF "ETH_0"

/**
 * @brief get default AVB configuration
 */
#define AVB_DEFAULT_CONFIG() { \
    .talker = false, \
    .listener = false, \
    .eth_interface = DEF_ETH_IF, \
    .i2s_port = 0, \
    .output_pa_pin = 53, \
    .i2c_handle = NULL, \
    .codec_handle = NULL, \
    .eth_handle = NULL, \
    .codec_type = avb_codec_type_es8311, \
    .default_sample_rate = 48000, \
    .default_bits_per_sample = 24, \
    .num_channels_input = 1, \
    .num_channels_output = 1, \
    .supported_sample_rates = { \
        .sample_rates = {44100, 48000, 96000}, \
        .num_rates = 3}, \
    .supported_bits_per_sample = { \
        .bit_rates = {16, 24}, \
        .num_rates = 2} \
}

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Codec types */
typedef enum {
  avb_codec_type_es8311 // Everest Semiconductor ES8311 (currently only one supported)
} avb_codec_type_t;

/* Sample rates struct */
typedef struct {
    uint32_t sample_rates[8];
    uint8_t num_rates;
} avb_sample_rates_s;

/* Bit rates struct */
typedef struct {
    uint8_t bit_rates[8]; // bits per sample
    uint8_t num_rates;
} avb_bit_rates_s;

/* AVB configuration structure
 * Currently sample rate and bit rate must be same for
 * for all inputs and outputs.
 */
typedef struct {
    bool                   talker;                    // enable talker
    bool                   listener;                  // enable listener
    char *                 eth_interface;             // ethernet interface name
    uint8_t                i2s_port;                  // i2s port number
    uint8_t                output_pa_pin;             // output PA pin
    void *                 i2c_handle;                // i2c handle
    void *                 codec_handle;              // codec handle
    esp_eth_handle_t *     eth_handle;                // ethernet handle
    const avb_codec_type_t codec_type;                // codec type
    uint32_t               default_sample_rate;       // default sample rate
    uint8_t                default_bits_per_sample;   // default bits per sample
    uint8_t                num_channels_input;        // number of input channels
    uint8_t                num_channels_output;       // number of output channels
    avb_sample_rates_s     supported_sample_rates;    // supported sample rates
    avb_bit_rates_s        supported_bits_per_sample; // supported bits per sample
} avb_config_s;

/* AVB status information structure */
typedef struct {
  bool clock_source_valid;      // clock source valid
  struct {
    uint8_t id[8];              // Entity ID
  } entity;
  struct timespec last_started; // when AVB was last started
} avb_status_s;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* @brief Start the AVB task and bind it to a specified interface
 * 
 * @param config Configuration structure
 * 
 * @return OK on success, ERROR on failure
 * 
 * @note Make sure i2c is initialized before starting AVB task. 
 *       AVB task will handle i2s and codec initialization.
 *       Only one instance of AVB task can run at a time. 
 *       Attempting to start multiple instances will fail with an error.
 */
int avb_start(avb_config_s *config);

/* @brief Query status from a running AVB task
 * 
 * @param status Pointer to status storage structure
 * 
 * @return OK on success, negative errno on failure
 * 
 * @note Multiple threads with priority less than CONFIG_ESP_AVB_TASKPRIO
 *       can request status simultaneously. If higher priority threads 
 *       request status simultaneously, some of the requests may timeout.
 */
int avb_status(avb_status_s *status);

/* @brief Stop the AVB task
 * 
 * @return OK on success, negative errno on failure
 **/
int avb_stop();

/* @brief Get the codec handle
 * 
 * @return codec handle
 **/
void *avb_get_codec_handle();

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* _ESP_AVB_H_ */
