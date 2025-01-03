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
#include <driver/i2s_std.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* AVB configuration structure */
struct avb_config_s {
  const char *interface;
  i2s_chan_handle_t i2s_tx_handle; // handle to i2s tx channel
  i2s_chan_handle_t i2s_rx_handle; // handle to i2s rx channel
  bool talker;
  bool listener;
  bool controller;
};

/* AVB status information structure */
struct avb_status_s {
  bool clock_source_valid;

  struct {
    uint8_t id[8];     /* Entity identity */
  } entity;

  struct timespec last_started; // when AVB was last started
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

int avb_start(struct avb_config_s *config);

int avb_status(struct avb_status_s *status);

int avb_stop();

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* _ESP_AVB_H_ */
