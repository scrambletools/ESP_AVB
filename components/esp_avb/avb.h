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
#include "avbutils.h"
#include "config.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Number of protocols to use for L2TAP (AVTP, MSRP, MVRP) */
#define AVB_NUM_PROTOCOLS 3

/* Maximum number of talkers to remember */
#define AVB_NUM_TALKERS 10

/* Maximum number of listeners to remember */
#define AVB_NUM_LISTENERS 10

/* Maximum number of connections to remember */
#define AVB_NUM_CONNECTIONS 2

/* Protocol identifiers */
#define AVTP 0
#define MSRP 1
#define MVRP 2

/* Periodic message intervals */
#define MSRP_DOMAIN_INTERVAL_MSEC 9000
#define MVRP_VLAN_ID_INTERVAL_MSEC 10000
#define MSRP_TALKER_ADV_INTERVAL_MSEC 3000
#define MSRP_LISTENER_READY_INTERVAL_MSEC 3000
#define ADP_ENTITY_AVAIL_INTERVAL_MSEC 5000
#define MAAP_ANNOUNCE_INTERVAL_MSEC 10000
#define PTP_STATUS_UPDATE_INTERVAL_MSEC 3000

// Commonly used mac addresses
#define BCAST_MAC_ADDR      (uint8_t[6]){ 0x91, 0xe0, 0xf0, 0x01, 0x00, 0x00 } // adp
#define MAAP_MCAST_MAC_ADDR (uint8_t[6]){ 0x91, 0xe0, 0xf0, 0x00, 0xff, 0x00 } // maap
#define MAAP_START_MAC_ADDR (uint8_t[6]){ 0x91, 0xe0, 0xf0, 0x00, 0xf2, 0x00 } // maap, can change
#define SPANTREE_MAC_ADDR   (uint8_t[6]){ 0x01, 0x80, 0xc2, 0x00, 0x00, 0x21 } // mvrp
#define LLDP_MCAST_MAC_ADDR (uint8_t[6]){ 0x01, 0x80, 0xc2, 0x00, 0x00, 0x0e } // msrp

/* Empty ID */
#define EMPTY_ID (uint8_t[8]){0,0,0,0,0,0,0,0}

/* Poll interval for checking for incoming messages on L2TAP FDs */
#define AVB_POLL_INTERVAL_MS 1000

/* Maximum message length */
#define AVB_MAX_MSG_LEN 500

/* Timespec functions */
#define clock_timespec_subtract(ts1, ts2, ts3) timespecsub(ts1, ts2, ts3)
#define clock_timespec_add(ts1, ts2, ts3) timespecadd(ts1, ts2, ts3)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Network types */

typedef uint8_t eth_addr_t[ETH_ADDR_LEN];

/* MRP types */

/* MRP three packed event (IEEE 802.1Q-2018 Clause 10.8.2.10)
 * (event1 * 36) + (event2 * 6) + event3 = 3pe value
 */
typedef uint8_t mrp_3pe_event_t;

/* MRP four packed event (IEEE 802.1Q-2018 Clause 10.8.2.11)
 * (event1 * 64) + (event2 * 16) + (event3 * 4) + event4 = 4pe value
 * The formula is not needed, as the struct below is sufficient.
 */
typedef struct {
  uint8_t event4 : 2; // 4th event
  uint8_t event3 : 2; // 3rd event
  uint8_t event2 : 2; // 2nd event
  uint8_t event1 : 2; // 1st event
} mrp_4pe_event_s; // 1 byte

/* Union of 3PE and 4PE events */
typedef union {
  mrp_3pe_event_t event;
  mrp_4pe_event_s declaration;
} mrp_event_u;

/* MVRP types */

/* The data structures in this section are defined in IEEE 802.1Q-2018
 * All multi-byte fields are big-endian.
 */

/* MVRP attribute header */
typedef struct {
  uint8_t attr_type; // attribute type
  uint8_t attr_len; // attribute length
  uint8_t vechead_padding : 5; // padding (ignored part of num_vals)
  uint8_t vechead_leaveall : 3; // 0 or 1, if 0 then num_vals is non-zero
  uint8_t vechead_num_vals; // # of events (div by 3, round up for # of 3pes)
} mvrp_attr_header_s; // 6 bytes

/* MVRP VLAN identifier message */
typedef struct {
  uint8_t            protocol_ver; // protocol version
  mvrp_attr_header_s header; // attribute header
  uint8_t            vlan_id[2]; // vlan ID
  mrp_3pe_event_t    attr_event[20]; // allow up to 20 events, ignore the rest
} mvrp_vlan_id_message_s; // 23 bytes limit

/* MSRP types */

/* The data structures in this section are defined in IEEE 802.1Q-2018
 * All multi-byte fields are big-endian.
 */

/* MSRP attribute header */
typedef struct {
  uint8_t attr_type; // attribute type
  uint8_t attr_len; // attribute length
  uint8_t attr_list_len[2]; // attribute list length
  uint8_t vechead_padding : 5; // padding (ignored part of num_vals)
  uint8_t vechead_leaveall : 3; // 0 or 1, if 0 then num_vals is non-zero
  uint8_t vechead_num_vals; // # of events (div by 3, round up for # of 3pes)
} msrp_attr_header_s; // 6 bytes

/* MSRP domain message */
typedef struct {
  msrp_attr_header_s header; // attribute header
  uint8_t            sr_class_id; // sr class ID
  uint8_t            sr_class_priority; // sr class priority
  uint8_t            sr_class_vid[2]; // sr class VID
  mrp_3pe_event_t    attr_event[20]; // allow up to 20 events, ignore the rest
} msrp_domain_message_s; // 25 bytes limit

/* MSRP talker advertise message */
typedef struct { // in comments: numbers in brackets are bit lengths
  msrp_attr_header_s header; // attribute header
  uint8_t            stream_id[8]; // stream ID
  uint8_t            stream_dest_addr[6]; // stream destination address
  uint8_t            vlan_id[2]; // vlan ID
  uint8_t            tspec_max_frame_size[2]; // tspec max frame size
  uint8_t            tspec_max_frame_interval[2]; // tspec max frame interval
  uint8_t            reserved : 4;
  uint8_t            rank : 1;
  uint8_t            priority : 3;
  uint8_t            accumulated_latency[2]; // accumulated latency
  mrp_3pe_event_t    event_data[20]; // up to 20 events, ignore the rest
} msrp_talker_adv_message_s; // 44 bytes limit

/* MSRP talker failed message */
typedef struct { // in comments: numbers in brackets are bit lengths
  msrp_attr_header_s header; // attribute header
  uint8_t            stream_id[8]; // stream ID
  uint8_t            stream_dest_addr[6]; // stream destination address
  uint8_t            vlan_id[2]; // vlan ID
  uint8_t            tspec_max_frame_size[2]; // tspec max frame size
  uint8_t            tspec_max_frame_interval[2]; // tspec max frame interval
  uint8_t            reserved : 4;
  uint8_t            rank : 1; 
  uint8_t            priority : 3; 
  uint8_t            accumulated_latency[2]; // accumulated latency
  uint8_t            failure_bridge_id[8]; // failure bridge ID
  uint8_t            failure_code; // failure code
  mrp_3pe_event_t    event_data[20]; // up to 20 events, ignore the rest
} msrp_talker_failed_message_s; // 53 bytes limit

/* MSRP listener message */
typedef struct {
  msrp_attr_header_s header; // attribute header
  uint8_t            stream_id[8]; // stream ID
  mrp_event_u        event_decl_data[20]; // up to 20 event/decl, ignore the rest
} msrp_listener_message_s; // 29 bytes limit

/* MSRP attribute union */
typedef union {
  msrp_attr_header_s            header;
  msrp_domain_message_s         domain;
  msrp_talker_adv_message_s     talker_adv;
  msrp_talker_failed_message_s  talker_failed;
  msrp_listener_message_s       listener;
  uint8_t                       raw[53];
} msrp_attribute_u; // 53 bytes limit

/* MSRP message buffer */
typedef struct {
  uint8_t protocol_ver; // protocol version
  uint8_t messages_raw[200]; // variable length depending on attributes
} msrp_msgbuf_s; // 266 bytes limit

/* AVTP types */

/* The data structures in this section are defined in IEEE 1722-2016
 * All multi-byte fields are big-endian.
 */

/* AAF PCM Message */
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
} aaf_pcm_message_s; // 

/* MAAP message */
typedef struct {
  uint8_t    subtype; // AVTP message subtype
  uint8_t    sv_ver_msgtype; // stream_id valid[1], version[3], message type[4]
  uint8_t    maapver_controldatalen; // maap version[5], control data length[11]
  uint8_t    stream_id[8]; // stream ID
  eth_addr_t req_start_addr; // requested start address
  uint8_t    req_count[2]; // requested count
  eth_addr_t confl_start_addr; // conflict start address
  uint8_t    confl_count[2]; // conflict count
} maap_message_s;

/* Talker */
typedef struct {
  uint8_t    stream_id[8]; // stream ID
  eth_addr_t stream_dest_addr; // stream destination address
  uint8_t    vlan_id[2]; // vlan ID
  uint8_t    tspec_max_frame_size[2]; // tspec max frame size
  uint8_t    tspec_max_frame_interval[2]; // tspec max frame interval
  int64_t    accumulated_latency; // as stated by the talker
  uint8_t    entity_id[8]; // entity ID
  uint8_t    model_id[8]; // model ID
  uint8_t    failure_code; // failure code
  bool       streaming; // streaming status
  bool       ready; // general status
} avb_talker_s;

/* Listener */
typedef struct {
  uint8_t stream_id[8]; // stream ID
  uint8_t vlan_id[2]; // vlan ID
  uint8_t entity_id[8]; // entity ID
  uint8_t model_id[8]; // model ID
  uint8_t last_event; // last listener event
  bool    ready; // status
} avb_listener_s;

/* Connection */
typedef struct {
  bool            as_talker; // we are talker or listener
  uint8_t         stream_id[8]; // stream ID
  uint8_t         vlan_id[2]; // vlan ID
  uint8_t         entity_id[8]; // other entity ID
  uint8_t         model_id[8]; // other model ID
  struct timespec started; // last start timestamp
  int64_t         accumulated_latency; // observed latency
  int64_t         accumulated_jitter; // observed jitter
  bool            active; // status
} avb_connection_s;

/* ATDECC types*/

/* The data structures in this section are defined in IEEE 1722.1-2021
 * All multi-byte fields are big-endian.
 */

/* AVB Entity Capabilities */
typedef struct {
  char reserved[2];
  char class_a : 1;
  char class_b : 1;
  char gptp_supported : 1;
  char padding : 5;
  char efu_mode : 1;
  char address_access : 1;
  char gateway_entity : 1;
  char aem : 1;
  char legacy_avc : 1;
  char assoc_id_supported : 1;
  char assoc_id_valid : 1;
  char vendor_unique : 1;
} avb_entity_cap_s; // 4 bytes

/* AVB Talker Capabilities */
typedef struct {
  uint8_t padding : 1;
  uint8_t other_source : 1;
  uint8_t control_source : 1;
  uint8_t media_clock_source : 1;
  uint8_t smpte_source : 1;
  uint8_t midi_source : 1;
  uint8_t audio_source : 1;
  uint8_t video_source : 1;
  uint8_t implemented : 1;
  uint8_t reserved : 7;
} avb_talker_cap_s; // 2 bytes

/* AVB Listener Capabilities */
typedef struct {
  uint8_t padding : 1;
  uint8_t other_sink : 1;
  uint8_t control_sink : 1;
  uint8_t media_clock_sink : 1;
  uint8_t smpte_sink : 1;
  uint8_t midi_sink : 1;
  uint8_t audio_sink : 1;
  uint8_t video_sink : 1;
  uint8_t implemented : 1;
  uint8_t reserved : 7;
} avb_listener_cap_s; // 2 bytes

/* AVB Controller Capabilities */
typedef struct {
  uint8_t reserved[3];
  uint8_t implemented : 1;
  uint8_t layer3_proxy : 1;
  uint8_t padding : 6;
} avb_controller_cap_s; // 4 bytes

/* AVB Entity Summary */
typedef struct {
  uint8_t              entity_id[8];
  uint8_t              entity_model_id[8];
  avb_entity_cap_s     entity_capabilities;
  uint8_t              talker_stream_sources[2];
  avb_talker_cap_s     talker_capabilities;
  uint8_t              listener_stream_sinks[2];
  avb_listener_cap_s   listener_capabilities;
  avb_controller_cap_s controller_capabilities;
  uint8_t              available_index[4];
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
  avb_entity_detail_s  detail;
} avb_entity_s;

/* ADP message */
typedef struct {
  uint8_t              subtype; // AVTP message subtype
  uint8_t              h_ver_msgtype; // header specific[1], version[3], message type[4]
  uint8_t              validtime_controldatalen[2]; // valid time[5], control data length[1]
  avb_entity_summary_s entity; // avb entity summary
  uint8_t              gptp_gm_id[8]; // gptp grand master ID
  uint8_t              gptp_domain_num; // gptp domain number
  uint8_t              reserved1; // reserved
  uint8_t              current_config_index[2]; // current configuration index
  uint8_t              identity_control_index[2]; // identity control index
  uint8_t              interface_index[2]; // interface index
  uint8_t              association_id[8]; // association ID
  uint8_t              reserved2[4]; // reserved
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
  uint8_t    subtype; // AVTP message subtype
  uint8_t    h_ver_msgtype; // header specific[1], version[3], message type[4]
  uint8_t    status_controldatalen[2]; // status[5], control data length[11]
  uint8_t    stream_id[8]; // stream ID
  uint8_t    controller_entity_id[8]; // controller entity ID
  uint8_t    talker_entity_id[8]; // talker entity ID
  uint8_t    listener_entity_id[8]; // listener entity ID
  uint8_t    talker_uid[2]; // talker UID
  uint8_t    listener_uid[2]; // listener UID
  eth_addr_t stream_dest_addr; // stream destination address
  uint8_t    connection_count[2]; // connection count
  uint8_t    seq_id[2]; // sequence ID
  uint8_t    flags[2]; // flags
  uint8_t    stream_vlan_id[2]; // stream VLAN ID
  uint8_t    conn_listeners_entries[2]; // connection listeners entries
  uint8_t    ip_flags[2]; // IP flags
  uint8_t    reserved[2]; // reserved
  uint8_t    src_port[2]; // source port
  uint8_t    dest_port[2]; // destination port
  uint8_t    src_ip_addr[4]; // source IP address
  uint8_t    dest_ip_addr[4]; // destination IP address
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

/* General */

/* Generic AVB message buffer */
typedef union {
  avtp_msgbuf_u          avtp;
  msrp_msgbuf_s          msrp;
  mvrp_vlan_id_message_s mvrp;
  uint8_t                raw[128];
} avb_msgbuf_u;

/* AVB Enums*/

/* Ethertypes (convert to big-endian before writing to Ethernet header) */
typedef enum {
	ethertype_msrp = 0x22ea,
	ethertype_avtp = 0x22f0,
	ethertype_mvrp = 0x88f5,
	ethertype_gptp = 0x88f7
} ethertype_t;

/* MVRP attribute types in enumerated order */
typedef enum {
  mvrp_attr_type_none,
  mvrp_attr_type_vlan_identifier
} mvrp_attr_type_t;

/* MSRP attribute types in enumerated order */
typedef enum {
  msrp_attr_type_none,
  msrp_attr_type_talker_advertise,
  msrp_attr_type_talker_failed,
  msrp_attr_type_listener,
  msrp_attr_type_domain
} msrp_attr_type_t;

/* MSRP attribute events in enumerated order */
typedef enum {
  msrp_attr_event_new,
  msrp_attr_event_join_in,
  msrp_attr_event_in,
  msrp_attr_event_join_mt,
  msrp_attr_event_mt,
  msrp_attr_event_lv,
  msrp_attr_event_none // no event
} msrp_attr_event_t;

/* MSRP listener events in enumerated order */
typedef enum {
  msrp_listener_event_ignore,
  msrp_listener_event_asking_failed,
  msrp_listener_event_ready,
  msrp_listener_event_ready_failed
} msrp_listener_event_t;

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

/* ADP message types in enumerated order (1722.1 Clause 6.2) */
typedef enum {
	adp_msg_type_entity_available,
	adp_msg_type_entity_departing,
	adp_msg_type_entity_discover
} adp_msg_type_t;

/* AVTP subtypes and their values */
typedef enum {
  avtp_subtype_aaf  = 0x02,
  avtp_subtype_adp  = 0xfa,
  avtp_subtype_aecp = 0xfb,
  avtp_subtype_acmp = 0xfc,
  avtp_subtype_maap = 0xfe
} avtp_subtype_t;

/* MAAP message types and their values */
typedef enum {
  maap_msg_type_probe    = 0x01,
  maap_msg_type_defend   = 0x02,
  maap_msg_type_announce = 0x03
} maap_msg_type_t;

/* AECP message types (subset) in enumerated order (1722.1 Clause 9) */
typedef enum {
	aecp_msg_type_aem_command,
	aecp_msg_type_aem_response
} aecp_msg_type_t;

/* AECP command codes (subset) in enumerated order (1722.1 Clause 7.4) */
typedef enum {
	aecp_cmd_code_acquire_entity,
	aecp_cmd_code_lock_entity,
	aecp_cmd_code_entity_available,
	aecp_cmd_code_controller_available,
	aecp_cmd_code_read_descriptor
} aecp_cmd_code_t;

/* AECP descriptor types in enumerated order */
typedef enum {
  aecp_desc_type_entity,
  aecp_desc_type_configuration
} aecp_desc_type_t;

/* ACMP message types (subset) in enumerated order (1722.1 Clause 8.2) */
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

/* AVB status structures*/

/* Carrier structure for querying AVB status */
struct avb_statusreq_s {
  sem_t               *done;
  struct avb_status_s *dest;
};

/* Main AVB state storage */
struct avb_state_s {

  /* Request for AVB task to stop or report status */
  bool stop;
  struct avb_statusreq_s status_req;
  struct ptpd_status_s   ptp_status;

  uint8_t internal_mac_addr[ETH_ADDR_LEN];
  int l2if[AVB_NUM_PROTOCOLS]; // 3 L2TAP interfaces (FDs) for AVTP, MSRP, and MVRP

  /* Our own entity */
  avb_entity_s own_entity;

  /* Talkers that we are aware of */
  avb_talker_s talkers[AVB_NUM_TALKERS];

  /* Listeners that we are aware of */
  avb_listener_s listeners[AVB_NUM_LISTENERS];

  /* Connections that we are in progress */
  avb_connection_s connections[AVB_NUM_CONNECTIONS];

  /* Latest received packet and its timestamp (CLOCK_REALTIME) 
   * 3 elements, 1 for each protocol (AVTP, MSRP, MVRP)
   */
  avb_msgbuf_u rxbuf[AVB_NUM_PROTOCOLS];
  struct timespec rxtime[AVB_NUM_PROTOCOLS];
  eth_addr_t rxsrc[AVB_NUM_PROTOCOLS];

  /* Last time we sent a periodic message */
  struct timespec last_transmitted_adp_entity_avail;
  struct timespec last_transmitted_mvrp_vlan_id;
  struct timespec last_transmitted_msrp_domain;
  struct timespec last_transmitted_msrp_talker_adv;
  struct timespec last_transmitted_msrp_listener_ready;
  struct timespec last_transmitted_maap_announce;
  struct timespec last_ptp_status_update;
};

/* AVB Functions */

/* Network functions */
int avb_net_init(struct avb_state_s *state, const char *interface);
void avb_create_eth_frame(uint8_t *eth_frame, 
                          eth_addr_t *dest_addr, 
                          struct avb_state_s *state, 
                          ethertype_t ethertype, 
                          void *msg, 
                          uint16_t msg_len);
int avb_net_send_to(struct avb_state_s *state, 
                 ethertype_t ethertype, 
                 void *msg, 
                 uint16_t msg_len, 
                 struct timespec *ts,
                 eth_addr_t *dest_addr);
int avb_net_send(struct avb_state_s *state, 
                 ethertype_t ethertype, 
                 void *msg, 
                 uint16_t msg_len, 
                 struct timespec *ts);
int avb_net_recv(struct avb_state_s *state, 
                 int l2if, 
                 void *msg, 
                 uint16_t msg_len, 
                 struct timespec *ts,
                 eth_addr_t *src_addr);

/* AVTP send functions */
int avb_send_mvrp_vlan_id(struct avb_state_s *state);
int avb_send_msrp_domain(struct avb_state_s *state);
int avb_send_msrp_talker_adv(struct avb_state_s *state, msrp_attr_event_t event);
int avb_send_msrp_talker_failed(struct avb_state_s *state, msrp_attr_event_t event);
int avb_send_msrp_listener(struct avb_state_s *state, 
                           msrp_attr_event_t attr_event, 
                           msrp_listener_event_t listener_event);
int avb_send_maap_announce(struct avb_state_s *state);
int avb_send_aaf_pcm(struct avb_state_s *state);

/* AVTP processing functions */
int avb_process_mvrp_vlan_id(struct avb_state_s *state, mvrp_vlan_id_message_s *msg);
int avb_process_msrp_domain(struct avb_state_s *state,
                            msrp_msgbuf_s *msg,
                            int offset,
                            size_t length);
int avb_process_msrp_talker(struct avb_state_s *state,
                            msrp_msgbuf_s *msg,
                            int offset,
                            size_t length,
                            bool is_failed);
int avb_process_msrp_listener(struct avb_state_s *state,
                              msrp_msgbuf_s *msg,
                              int offset,
                              size_t length);
int avb_process_maap(struct avb_state_s *state, maap_message_s *msg);
int avb_process_aaf(struct avb_state_s *state, aaf_pcm_message_s *msg);

/* ATDECC send functions */
int avb_send_adp_entity_available(struct avb_state_s *state);
int avb_send_acmp_connect_rx_command(struct avb_state_s *state);
int avb_send_acmp_connect_tx_command(struct avb_state_s *state);
int avb_send_aecp_cmd_get_stream_info(struct avb_state_s *state);
int avb_send_aecp_rsp_get_stream_info(struct avb_state_s *state); // could be notification
int avb_send_aecp_cmd_get_counters(struct avb_state_s *state);
int avb_send_aecp_rsp_get_counters(struct avb_state_s *state); // could be notification

/* ATDECC processing functions */
int avb_process_adp(struct avb_state_s *state, adp_message_s *msg); // handle all adp messages
int avb_process_aecp(struct avb_state_s *state, aecp_message_s *msg); // route to specific func
int avb_process_acmp(struct avb_state_s *state, acmp_message_s *msg); // route to specific func
int avb_process_aecp_cmd_get_stream_info(struct avb_state_s *state);
int avb_process_aecp_rsp_get_stream_info(struct avb_state_s *state);
int avb_process_aecp_cmd_get_counters(struct avb_state_s *state);
int avb_process_aecp_rsp_get_counters(struct avb_state_s *state);
int avb_process_acmp_connect_rx_command(struct avb_state_s *state);
int avb_process_acmp_connect_tx_command(struct avb_state_s *state);

/* Helper functions */
const char* get_adp_message_type_name(adp_msg_type_t message_type);
const char* get_aecp_command_code_name(aecp_cmd_code_t command_code);
const char* get_acmp_message_type_name(acmp_msg_type_t message_type);

#endif /* _ESP_AVB_AVB_H_ */
