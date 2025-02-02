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
    .sample_rate = 48000, \
    .bits_per_sample = 24, \
    .num_channels = 1, \
}

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Codec types */
typedef enum {
  avb_codec_type_es8311 // Everest Semiconductor ES8311 (currently only one supported)
} avb_codec_type_t;

/* AVB configuration structure */
struct avb_config_s {
  
  bool talker;                        // enable talker
  bool listener;                      // enable listener
  char *eth_interface;                // ethernet interface name
  int i2s_port;                       // i2s port number
  int output_pa_pin;                  // output PA pin
  void *i2c_handle;                   // i2c handle
  void *codec_handle;                 // codec handle
  esp_eth_handle_t *eth_handle;       // ethernet handle
  const avb_codec_type_t codec_type;  // codec type
  int sample_rate;                    // sample rate
  int bits_per_sample;                // bits per sample
  int num_channels;                   // number of channels



};

/* AVB status information structure */
struct avb_status_s {
  bool clock_source_valid;           // clock source valid

  struct {
    uint8_t id[8];                   // Entity ID
  } entity;

  struct timespec last_started;      // when AVB was last started
};

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
int avb_start(struct avb_config_s *config);

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
int avb_status(struct avb_status_s *status);

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
