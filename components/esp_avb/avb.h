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
#include <stdbool.h>
#include <stdint.h>
//#include <sys/l2if.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <esp_eth_driver.h>
#include <esp_vfs_l2tap.h>
#include <semaphore.h>
#include <esp_log.h>
#include <esp_err.h>
#include <lwip/prot/ethernet.h> // Ethernet headers
#include <esp_eth_time.h>
#include <ptpd.h>
#include "utils.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Number of protocols (AVTP, MSRP, MVRP) */
#define AVB_NUM_PROTOCOLS 3

/* Protocol identifiers */
#define AVTP 0
#define MSRP 1
#define MVRP 2

/* Ethertypes */
#define ETHERTYPE_AVTP 0x22f0
#define ETHERTYPE_MSRP 0x22ea
#define ETHERTYPE_MVRP 0x88f5

/* AVTP Message types */
#define AVTP_MSGTYPE_ANNOUNCE 0x00
#define AVTP_MSGTYPE_SYNC 0x01

/* MSRP Message types */
#define MSRP_MSGTYPE_SYNC 0x00
#define MSRP_MSGTYPE_ANNOUNCE 0x01
#define MSRP_MSGTYPE_FOLLOW_UP 0x02

/* MVRP Message types */
#define MVRP_MSGTYPE_SYNC 0x00
#define MVRP_MSGTYPE_ANNOUNCE 0x01
#define MVRP_MSGTYPE_FOLLOW_UP 0x02

/* Periodic message intervals */
#define MSRP_DOMAIN_INTERVAL_MSEC 9000
#define MVRP_VLAN_IDENTIFIER_INTERVAL_MSEC 2000
#define MSRP_TALKER_ADVERTISE_INTERVAL_MSEC 3000
#define MSRP_LISTENER_READY_INTERVAL_MSEC 3000
#define ADP_ENTITY_AVAILABLE_INTERVAL_MSEC 10000

// Commonly used mac addresses
#define BCAST_MAC_ADDR (uint8_t[6]){ 0x91, 0xe0, 0xf0, 0x01, 0x00, 0x00 } // adp
#define MAAP_MCAST_MAC_ADDR (uint8_t[6]){ 0x91, 0xe0, 0xf0, 0x00, 0xff, 0x00 } // maap
#define SPANTREE_MAC_ADDR (uint8_t[6]){ 0x01, 0x80, 0xc2, 0x00, 0x00, 0x21 } // 
#define LLDP_MCAST_MAC_ADDR (uint8_t[6]){ 0x01, 0x80, 0xc2, 0x00, 0x00, 0x0e } // msrp,mvrp

/* Poll interval for checking for incoming messages on L2TAP FDs */
#define AVB_POLL_INTERVAL_MS 1000

/* Max message length */
#define AVB_MAX_MSG_LEN 500

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

/* Network */

// for l2tap
#define INVALID_FD     -1
#define ETH_INTERFACE "ETH_DEF"
#define PRINT_FRAME_FORMAT 0 // 0=short, 1=long

typedef uint8_t eth_addr_t[ETH_ADDR_LEN];

/* MSRP types */

/* The following data structures are defined in IEEE 802.1Q
 * All multi-byte fields are big-endian.
 */

/* MSRP domain message */
typedef struct {
  uint8_t protocol_ver; // protocol version
  uint8_t attr1_type; // attribute type
  uint8_t attr1_len; // attribute length
  uint8_t attr1_list_len[2]; // attribute list length
  uint8_t attr1_vector_header[2]; // vector header
  uint8_t attr1_sr_class_id; // sr class ID
  uint8_t attr1_sr_class_priority; // sr class priority
  uint8_t attr1_sr_class_vid[2]; // sr class VID
  uint8_t attr1_event; // attribute event
  uint8_t attr1_end_mark_list[2]; // end mark for list
  uint8_t attr2_type; // attribute type
  uint8_t attr2_len; // attribute length
  uint8_t attr2_list_len[2]; // attribute list length
  uint8_t attr2_vector_header[2]; // vector header
  uint8_t attr2_sr_class_id; // sr class ID
  uint8_t attr2_sr_class_priority; // sr class priority
  uint8_t attr2_sr_class_vid[2]; // sr class VID
  uint8_t attr2_event; // attribute event
  uint8_t attr2_end_mark_list[2]; // end mark for list
  uint8_t end_mark_pdu[2]; // end mark for pdu
} msrp_domain_message_s; // 29 bytes

/* MSRP talker advertise message */
typedef struct { // in comments: numbers in brackets are bit lengths
  uint8_t protocol_ver; // protocol version
  uint8_t attr_type; // attribute type
  uint8_t attr_len; // attribute length
  uint8_t attr_list_len[2]; // attribute list length
  uint8_t vector_header[2]; // vector header
  uint8_t stream_id[8]; // stream ID
  uint8_t stream_dest_addr[6]; // stream destination address
  uint8_t vlan_id[2]; // vlan ID
  uint8_t tspec_max_frame_size[2]; // tspec max frame size
  uint8_t tspec_max_frame_interval[2]; // tspec max frame interval
  uint8_t priority : 3; // 3 bits (0-7)
  uint8_t rank : 4; // 4 bits (0-15)
  uint8_t reserved : 1; // 1 bit (0-1)
  uint8_t accumulated_latency[2]; // accumulated latency
  uint8_t end_mark_list[2]; // end mark for list
  uint8_t end_mark_pdu[2]; // end mark for pdu
} msrp_talker_adv_message_s; // 34 bytes

/* MSRP listener ready message */
typedef struct {
  uint8_t protocol_ver; // protocol version
  uint8_t attr_type; // attribute type
  uint8_t attr_len; // attribute length
  uint8_t attr_list_len[2]; // attribute list length
  uint8_t vector_header[2]; // vector header
  uint8_t stream_id[8]; // stream ID
  uint8_t end_mark_list[2]; // end mark for list
  uint8_t end_mark_pdu[2]; // end mark for pdu
} msrp_listener_ready_message_s; // 19 bytes

/* MSRP message buffer */
typedef union {
  msrp_domain_message_s         domain;
  msrp_talker_adv_message_s     talker_adv;
  msrp_listener_ready_message_s listener_ready;
  uint8_t                       raw[34];
} msrp_msgbuf_u;

/* MSRP attribute types and their values */
typedef enum {
  msrp_attr_type_talker_advertise = 0x01,
  msrp_attr_type_talker_failed = 0x02,
  msrp_attr_type_listener = 0x03,
  msrp_attr_type_domain = 0x04
} msrp_attr_type_t;

/* MSRP reservation declaration types in enumerated order */
typedef enum {
  msrp_decl_type_talker_advertise,
  msrp_decl_type_talker_failed,
  msrp_decl_type_listener_asking_failed,
  msrp_decl_type_listener_ready,
  msrp_decl_type_listener_ready_failed
} msrp_decl_type_t;

/* MSRP reservation failure codes in enumerated order */
typedef enum {
  no_failure,
  insufficient_bandwidth,
  insufficient_bridge_resources,
  insufficient_bandwidth_for_traffic_class,
  stream_id_in_use_by_another_talker,
  stream_destination_address_already_in_use,
  stream_preempted_by_higher_rank,
  reported_latency_has_changed,
  egress_port_is_not_avb_capable,
  use_a_different_destination_address,
  out_of_msrp_resources,
  out_of_mmrp_resources,
  cannot_store_destination_address,
  requested_priority_is_not_an_sr_class_priority,
  max_frame_size_is_too_large_for_media,
  max_fan_in_ports_limit_has_been_reached,
  changes_in_first_value_for_a_registered_stream_id,
  vlan_is_blocked_on_this_egress_port__registration_forbidden,
  vlan_tagging_is_disabled_on_this_egress_port__untagged_set,
  sr_class_priority_mismatch
} msrp_reservation_failure_code_t;

/* MVRP types */

/* The following data structures are defined in IEEE 802.1Q
 * All multi-byte fields are big-endian.
 */

/* MVRP VLAN identifier message */
typedef struct {
  uint8_t protocol_ver; // protocol version
  uint8_t attr_type; // attribute type
  uint8_t attr_len; // attribute length
  uint8_t vector_header[2]; // vector header
  uint8_t vlan_id[2]; // vlan ID
  uint8_t attr_event; // attribute event
  uint8_t end_mark_list[2]; // end mark for list
  uint8_t end_mark_pdu[2]; // end mark for pdu
} mvrp_vlan_identifier_message_s; // 12 bytes

typedef union {
  mvrp_vlan_identifier_message_s vlan_identifier;
  uint8_t                        raw[12];
} mvrp_msgbuf_u;

/* MVRP attribute types and their values */
typedef enum {
  mvrp_attr_type_vlan_identifier = 0x01
} mvrp_attr_type_t;

/* AVTP types */

/* The following data structures are defined in IEEE 1722-2016
 * All multi-byte fields are big-endian.
 */

/* AAF Message */
typedef struct { // in comments: numbers in brackets are bit lengths
  uint8_t subtype; // AVTP message subtype
  uint8_t sv_ver_mr_rsv_tv; // stream_id valid[1], version[3], media clock restart [1], reserved[2], avtp_ts valid[1]
  uint8_t seq_num; // sequence number
  uint8_t rsv_tu; // reserved[7], avtp_ts uncertain[1]
  uint8_t stream_id[8]; // stream ID
  uint8_t avtp_ts[4]; // AVTP timestamp
  uint8_t format; // format
  uint8_t nsr_rsv_cpf[2]; // nominal sample rate[4], reserved[2], channels per frame[10]
  uint8_t bit_depth; // cannot be larger than what is specified in format
  uint8_t stream_data_len[2]; // stream data length
  uint8_t rsv_sp_evt; // reserved[3], sparse timestamp[1], upper-level event[4]
  uint8_t reserved; // reserved
  uint8_t stream_data[80]; // variable length
} aaf_pcm_message_s;

/* MAAP message */
typedef struct {
  uint8_t subtype; // AVTP message subtype
  uint8_t sv_ver_msgtype; // stream_id valid[1], version[3], message type[4]
  uint8_t maapver_controldatalen; // maap version[5], control data length[11]
  uint8_t stream_id[8]; // stream ID
  uint8_t req_start_addr[6]; // requested start address
  uint8_t req_count[2]; // requested count
  uint8_t confl_start_addr[6]; // conflict start address
  uint8_t confl_count[2]; // conflict count
} maap_message_s;

/* AVTP subtypes and their values */
typedef enum {
  avtp_subtype_aaf = 0x02,
  avtp_subtype_adp = 0xfa,
  avtp_subtype_aecp = 0xfb,
  avtp_subtype_acmp = 0xfc,
  avtp_subtype_maap = 0xfe
} avtp_subtype_t;

/* MAAP message types and their values */
typedef enum {
  maap_msg_type_probe = 0x01,
  maap_msg_type_defend = 0x02,
  maap_msg_type_announce = 0x03
} maap_msg_type_t;

/* ATDECC types*/

#define ATDECC_TEMPLATE_MAX_SIZE 500

/* AVB Entity Summary */
typedef struct {
  uint8_t entity_id[8];
  uint8_t entity_model_id[8];
  uint8_t entity_capabilities[4];
  uint8_t talker_stream_sources[2];
  uint8_t talker_capabilities[2];
  uint8_t listener_stream_sinks[2];
  uint8_t listener_capabilities[2];
  uint8_t controller_capabilities[4];
  uint8_t available_index[4];
} avb_entity_summary_s; // 36 bytes

/* AVB Entity Detail */
typedef struct {
  uint8_t association_id[8];
  uint8_t entity_name[64];
  uint8_t vendor_name_ptr[2];
  uint8_t model_name_ptr[2];
  uint8_t firmware_version[64];
  uint8_t group_name[64];
  uint8_t serial_number[64];
  uint8_t configurations_count[2];
  uint8_t current_configuration[2];
} avb_entity_detail_s; // 308 bytes

/* AVB Entity */
typedef struct {
  avb_entity_summary_s summary;
  avb_entity_detail_s detail;
} avb_entity_s;

/* ADP message */
typedef struct {
  uint8_t subtype; // AVTP message subtype
  uint8_t h_ver_msgtype; // header specific[1], version[3], message type[4]
  uint8_t validtime_controldatalen[2]; // valid time[5], control data length[1]
  avb_entity_summary_s entity; // avb entity summary
  uint8_t gptp_gm_id[8]; // gptp grand master ID
  uint8_t gptp_domain_num; // gptp domain number
  uint8_t reserved1; // reserved
  uint8_t current_config_index[2]; // current configuration index
  uint8_t identity_control_index[2]; // identity control index
  uint8_t interface_index[2]; // interface index
  uint8_t association_id[8]; // association ID
  uint8_t reserved2[4]; // reserved
} adp_message_s; // 

/* AECP message */
typedef struct {
  uint8_t subtype; // AVTP message subtype
  uint8_t h_ver_msgtype; // header specific[1], version[3], message type[4]
  uint8_t status_controldatalen[2]; // status[5], control data length[11]
  uint8_t target_entity_id[8]; // target entity ID
  uint8_t controller_entity_id[8]; // controller entity ID
  uint8_t seq_id[2]; // sequence ID
  uint8_t msg_data[80]; // variable length
} aecp_message_s;

/* ACMP Message */
typedef struct {
  uint8_t subtype; // AVTP message subtype
  uint8_t h_ver_msgtype; // header specific[1], version[3], message type[4]
  uint8_t status_controldatalen[2]; // status[5], control data length[11]
  uint8_t stream_id[8]; // stream ID
  uint8_t controller_entity_id[8]; // controller entity ID
  uint8_t talker_entity_id[8]; // talker entity ID
  uint8_t listener_entity_id[8]; // listener entity ID
  uint8_t talker_uid[2]; // talker UID
  uint8_t listener_uid[2]; // listener UID
  uint8_t stream_dest_addr[6]; // stream destination address
  uint8_t connection_count[2]; // connection count
  uint8_t seq_id[2]; // sequence ID
  uint8_t flags[2]; // flags
  uint8_t stream_vlan_id[2]; // stream VLAN ID
  uint8_t conn_listeners_entries[2]; // connection listeners entries
  uint8_t ip_flags[2]; // IP flags
  uint8_t reserved[2]; // reserved
  uint8_t src_port[2]; // source port
  uint8_t dest_port[2]; // destination port
  uint8_t src_ip_addr[4]; // source IP address
  uint8_t dest_ip_addr[4]; // destination IP address
} acmp_message_s; // 96 bytes

/* AVTP message buffer */
typedef union {
  uint8_t           subtype;
  aaf_pcm_message_s aaf;
  maap_message_s    maap;
  adp_message_s     adp;
  aecp_message_s    aecp;
  acmp_message_s    acmp;
  uint8_t           raw[500];
} avtp_msgbuf_u;

// ADP message types in enumerated order (1722.1 Clause 6.2)
typedef enum {
	adp_msg_type_entity_available,
	adp_msg_type_entity_departing,
	adp_msg_type_entity_discover
} adp_msg_type_t;

// AECP message types (subset) in enumerated order (1722.1 Clause 9)
typedef enum {
	aecp_msg_type_aem_command,
	aecp_msg_type_aem_response
} aecp_msg_type_t;

// AECP command codes (subset) in enumerated order (1722.1 Clause 7.4)
typedef enum {
	aecp_cmd_code_acquire_entity,
	aecp_cmd_code_lock_entity,
	aecp_cmd_code_entity_available,
	aecp_cmd_code_controller_available,
	aecp_cmd_code_read_descriptor
} aecp_cmd_code_t;

// ACMP message types (subset) in enumerated order (1722.1 Clause 8.2)
typedef enum {
	acmp_msg_type_connect_tx_command,
	acmp_msg_type_connect_tx_response,
	acmp_msg_type_disconnect_tx_command,
  acmp_msg_type_disconnect_tx_response,
	acmp_msg_type_get_tx_state_command,
	acmp_msg_type_get_tx_state_response,
	acmp_msg_type_connect_rx_command,
  acmp_msg_type_connect_rx_response,
  acmp_msg_type_disconnect_rx_command,
	acmp_msg_type_disconnect_rx_response,
	acmp_msg_type_get_rx_state_command,
	acmp_msg_type_get_rx_state_response
} acmp_msg_type_t;

/* General */

/* Generic message buffer */
typedef union {
  avtp_msgbuf_u avtp;
  msrp_msgbuf_u msrp;
  mvrp_msgbuf_u mvrp;
  uint8_t       raw[128];
} avb_msgbuf_u;

/* Carrier structure for querying AVB status */

struct avb_statusreq_s {
  sem_t *done;
  struct avb_status_s *dest;
};

/* Main AVB state storage */

struct avb_state_s {
  /* Request for AVB task to stop or report status */

  bool stop;
  struct avb_statusreq_s status_req;

  uint8_t internal_mac_addr[ETH_ADDR_LEN];
  int l2if[AVB_NUM_PROTOCOLS]; // 3 L2TAP interfaces (FDs) for AVTP, MSRP, and MVRP

  /* Our own entity */
  avb_entity_s own_entity;

  /* Latest received packet and its timestamp (CLOCK_REALTIME) 
   * 3 elements, 1 for each protocol (AVTP, MSRP, MVRP)
   */
  struct timespec rxtime[AVB_NUM_PROTOCOLS];
  avb_msgbuf_u rxbuf[AVB_NUM_PROTOCOLS];

  /* Last time we sent a periodic message */
  struct timespec last_transmitted_msrp_domain;
  struct timespec last_transmitted_msrp_talker_advertise;
  struct timespec last_transmitted_msrp_listener_ready;
  struct timespec last_transmitted_mvrp_vlan_identifier;
  struct timespec last_transmitted_adp_entity_available;
};

/* Network functions */
void set_eth_handle(esp_eth_handle_t handle);
void avb_create_eth_frame(uint8_t *eth_frame, eth_addr_t *dest_addr, struct avb_state_s *state, ethertype_t ethertype, void *msg, uint16_t msg_len);
int avb_net_send(struct avb_state_s *state, int l2if, void *msg, uint16_t msg_len, struct timespec *ts);
int avb_net_recv(struct avb_state_s *state, int l2if, void *msg, uint16_t msg_len, struct timespec *ts);
void print_frame(eth_frame_t *frame, int format);
void print_frame_of_type(avb_frame_type_t type, eth_frame_t *frame, int format);

/* AVTP Functions */
void append_avtpdu(avtp_subtype_t type, eth_frame_t *frame);
void append_msrpdu(msrp_attr_type_t type, eth_frame_t *frame);
void append_mvrpdu(mvrp_attr_type_t type, eth_frame_t *frame);
avb_frame_type_t detect_avtp_frame_type(ethertype_t *ethertype, eth_frame_t *frame, ssize_t size);
void print_avtp_frame(avb_frame_type_t type, eth_frame_t *frame, int format);

/* ATDECC Functions */
void append_adpdu(adp_msg_type_t type, eth_frame_t *frame);
void append_aecpdu(aecp_msg_type_t type, eth_frame_t *frame);
void append_acmpdu(acmp_msg_type_t type, eth_frame_t *frame);
avb_frame_type_t detect_atdecc_frame_type(eth_frame_t *frame, ssize_t size);
void print_atdecc_frame(avb_frame_type_t type, eth_frame_t *frame, int format);

#endif /* _ESP_AVB_AVB_H_ */
