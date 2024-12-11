/*
 * Copyright 2024 Scramble Tools
 * License: MIT
 *
 * ESP_AVB Component
 *
 * This component provides an implementation of an AVB talker and listener.
 */

#ifndef _ESP_AVB_AVB_H_
#define _ESP_AVB_AVB_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Multicast MAC addresses for AVB FIXME */
#define IEEE1722_MULTICAST_ADDR (uint8_t[6]){0x01, 0x80, 0xC2, 0x00, 0x00, 0x0e} // for all messages in case of gPTP
#define MRP_MULTICAST_ADDR (uint8_t[6]){0x01, 0x1B, 0x19, 0x00, 0x00, 0x00} // for sync, announce, follow_up (non-gPTP)

/* MSRP Message types FIXME */ 
#define MSRP_MSGTYPE_MASK          0x0F // mask for message type
#define MSRP_MSGTYPE_SYNC          0x00 // sync message

/* Message flags FIXME */
#define MSRP_FLAGS0_TWOSTEP        (1 << 1) // flag indicating there will be a follow-up message
#define MSRP_FLAGS1_PTP_TIMESCALE  (1 << 3) // flag indicating use of PTP timescale (gPTP required)
#define MSRP_MSGTYPE_SDOID_GPTP    (1 << 4) // flag indicating a gPTP message

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* AVB Entity */
typedef struct {
  uint8_t mac_addr[6];
  uint8_t entity_id[2];
} avb_entity_s;

/* AVTP Message Types
 * The following data structures are defined in IEEE 1722-2016
 * All multi-byte fields are big-endian.
 */

/* AVTP Header */
typedef struct {
  uint8_t nuthin[8];
} avtp_header_s;

/* AVTP Message */
typedef struct {
  avtp_header_s header;
  uint8_t payload[8];
} avtp_message_s;

typedef union
{
  struct avtp_header_s               header;
  struct avtp_message_s              message;
  uint8_t                            raw[128];
} avtp_msgbuf;

/* MSRP Message Types
 * The following data structures are defined in IEEE 802.1Q
 * All multi-byte fields are big-endian.
 */

/* MSRP Header */
typedef struct {
  uint8_t nuthin[8];
} avb_msrp_header_s;

/* MSRP Message */
typedef struct {
  avb_msrp_header_s header;
  uint8_t payload[8];
} avb_msrp_message_s;

typedef union
{
  struct msrp_header_s               header;
  struct msrp_message_s              message;
  uint8_t                            raw[128];
} msrp_msgbuf;

/* MVRP Message Types
 * The following data structures are defined in IEEE 802.1Q
 * All multi-byte fields are big-endian.
 */

/* MVRP Header */
typedef struct {
  uint8_t nuthin[8];
} avb_mvrp_header_s;

/* MVRP Message */
typedef struct {
  avb_mvrp_header_s header;
  uint8_t payload[8];
} avb_mvrp_message_s;

typedef union
{
  struct mvrp_header_s               header;
  struct mvrp_message_s              message;
  uint8_t                            raw[128];
} mvrp_msgbuf;

#endif /* _ESP_AVB_AVB_H_ */
