/*
 * Copyright 2024 Scramble Tools
 * License: MIT
 *
 * ESP_AVB Component
 *
 * This component provides an implementation of an AVB talker and listener.
 */

#ifndef _ESP_AVB_H_
#define _ESP_AVB_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* AVB status information structure */

struct avb_status_s
{
  bool clock_source_valid;

  struct
  {
    uint8_t id[8];     /* Entity identity */
  } entity;

  struct timespec last_started; // when AVB was last started
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

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

int avb_start(const char *interface);

int avb_status(int pid, struct avb_status_s *status);

int avb_stop(int pid);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* _ESP_AVB_H_ */
