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

/* Size of a unique ID */
#define UNIQUE_ID_LEN 8

/* Timespec functions */
#define clock_timespec_subtract(ts1, ts2, ts3) timespecsub(ts1, ts2, ts3)
#define clock_timespec_add(ts1, ts2, ts3) timespecadd(ts1, ts2, ts3)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Network types */

typedef uint8_t eth_addr_t[ETH_ADDR_LEN];
typedef uint8_t unique_id_t[UNIQUE_ID_LEN];

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

/* Talker advertise information */
typedef struct {
  unique_id_t    stream_id; // stream ID
  eth_addr_t     stream_dest_addr; // stream destination address
  uint8_t        vlan_id[2]; // vlan ID
  uint8_t        tspec_max_frame_size[2]; // tspec max frame size
  uint8_t        tspec_max_frame_interval[2]; // tspec max frame interval
  uint8_t        reserved : 4;
  uint8_t        rank : 1;
  uint8_t        priority : 3;
  uint8_t        accumulated_latency[4]; // as stated by the talker
} talker_adv_info_s;

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
  talker_adv_info_s  info; // talker advertise information
  mrp_3pe_event_t    event_data[20]; // up to 20 events, ignore the rest
} msrp_talker_adv_message_s; // 44 bytes limit

/* MSRP talker failed message */
typedef struct { // in comments: numbers in brackets are bit lengths
  msrp_attr_header_s header; // attribute header
  talker_adv_info_s  info; // talker advertise information
  uint8_t            failure_bridge_id[8]; // failure bridge ID or padded mac
  uint8_t            failure_code; // failure code
  mrp_3pe_event_t    event_data[20]; // up to 20 events, ignore the rest
} msrp_talker_failed_message_s; // 53 bytes limit

/* MSRP talker message union */
typedef union {
  msrp_attr_header_s           header;
  msrp_talker_adv_message_s    talker;
  msrp_talker_failed_message_s talker_failed;
} msrp_talker_message_u;

/* MSRP listener message */
typedef struct {
  msrp_attr_header_s header; // attribute header
  unique_id_t        stream_id; // stream ID
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
  uint8_t     subtype; // AVTP message subtype
  uint8_t     timestamp_valid: 1; // source timestamp valid
  uint8_t     reserved1: 2; // reserved
  uint8_t     media_clock_restart: 1; // media clock restart
  uint8_t     version: 3; // AVTP version
  uint8_t     sv: 1; // stream id valid
  uint8_t     seq_num; // sequence number
  uint8_t     timestamp_uncertain: 1; // avtp timestamp uncertain
  uint8_t     reserved2 : 7; // reserved
  unique_id_t stream_id; // stream ID
  uint8_t     avtp_ts[4]; // AVTP timestamp
  uint8_t     format; // format
  uint8_t     padding : 2; // ignored part of channels per frame
  uint8_t     reserved3 : 2; // reserved
  uint8_t     sample_rate : 4; // nominal sample rate
  uint8_t     chan_per_frame; // channels per frame; using 1 byte for convenience
  uint8_t     bit_depth; // cannot be larger than what is specified in format
  uint8_t     stream_data_len[2]; // stream data length
  uint8_t     evt : 4; // upper-level event
  uint8_t     sparse_ts : 1; // sparse timestamp
  uint8_t     reserved4 : 3; // reserved
  uint8_t     reserved5; // reserved
  uint8_t     stream_data[80]; // variable length
} aaf_pcm_message_s;

/* MAAP message */
typedef struct {
  uint8_t     subtype; // AVTP message subtype
  uint8_t     msg_type: 4; // MAAP message type
  uint8_t     version: 3; // AVTP version
  uint8_t     sv: 1; // always 0 for MAAP messages
  uint8_t     padding : 3; // ignored part of control data length
  uint8_t     maap_version : 5; // maap version
  uint8_t     control_data_len; // control data length
  unique_id_t stream_id; // stream ID
  eth_addr_t  req_start_addr; // requested start address
  uint8_t     req_count[2]; // requested count
  eth_addr_t  confl_start_addr; // conflict start address
  uint8_t     confl_count[2]; // conflict count
} maap_message_s;

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

/* AVB Entity Summary 
 * used in entity available message and entity descriptor
 */
typedef struct {
  unique_id_t          entity_id;
  unique_id_t          entity_model_id;
  avb_entity_cap_s     entity_capabilities;
  uint8_t              talker_stream_sources[2];
  avb_talker_cap_s     talker_capabilities;
  uint8_t              listener_stream_sinks[2];
  avb_listener_cap_s   listener_capabilities;
  avb_controller_cap_s controller_capabilities;
  uint8_t              available_index[4];
} avb_entity_summary_s; // 36 bytes

/* AVB Entity Detail 
 * used in entity descriptor
 */
typedef struct {
  unique_id_t association_id;
  uint8_t     entity_name[64];
  uint8_t     vendor_name_ref[2];
  uint8_t     model_name_ref[2];
  uint8_t     firmware_version[64];
  uint8_t     group_name[64];
  uint8_t     serial_number[64];
  uint8_t     configurations_count[2];
  uint8_t     current_configuration[2];
} avb_entity_detail_s; // 308 bytes

/* AVB Entity descriptor */
typedef struct {
  uint8_t descriptor_type[2];             
  uint8_t descriptor_index[2];
  avb_entity_summary_s summary;
  avb_entity_detail_s  detail;
} avb_entity_s;

/* ATDECC header */
typedef struct {
  uint8_t subtype; // AVTP message subtype
  uint8_t msg_type: 4; // ADP message type
  uint8_t version: 3; // AVTP version
  uint8_t sv: 1; // always 0 for ADP messages
  uint8_t padding : 3; // ignored part of control data length
  uint8_t status_valtime : 5; // status or valid time in 2sec increments
  uint8_t control_data_len; // control data length
} atdecc_header_s; // 4 bytes

/* ADP message */
typedef struct {
  atdecc_header_s      header;
  avb_entity_summary_s entity; // avb entity summary
  unique_id_t          gptp_gm_id; // gptp grand master ID
  uint8_t              gptp_domain_num; // gptp domain number
  uint8_t              reserved1; // reserved
  uint8_t              current_config_index[2]; // current configuration index
  uint8_t              identity_control_index[2]; // identity control index
  uint8_t              interface_index[2]; // interface index
  unique_id_t          association_id; // association ID
  uint8_t              reserved2[4]; // reserved
} adp_message_s; 

/* AECP common data */
typedef struct {
  atdecc_header_s header;
  unique_id_t     target_entity_id; // target entity ID
  unique_id_t     controller_entity_id; // controller entity ID
  uint8_t         seq_id[2]; // sequence ID
  uint8_t         cr : 1; // request for controller to perform an action
  uint8_t         unsolicited : 1; // unsolicited notification
  uint8_t         padding2 : 6; // ignored part of command type
  uint8_t         command_type; // command type
} aecp_common_s; // 24 bytes

/* AECP stream flags
 * used in stream descriptor
 */
typedef struct {
  uint8_t clock_sync_source : 1;           // Indicates that the Stream is a preferred clock synchronization source.
  uint8_t class_a : 1;                     // Indicates that the Stream supports streaming at Class A.
  uint8_t class_b : 1;                     // Indicates that the Stream supports streaming at Class B.
  uint8_t supports_encrypted : 1;          // Indicates that the Stream supports streaming with encrypted PDUs.
  uint8_t primary_backup_supported : 1;    // Indicates that the backup_talker_entity_id_0 and the backup_talker_entity_id_0 fields are supported.
  uint8_t primary_backup_valid : 1;        // Indicates that the backup_talker_entity_id_0 and the backup_talker_entity_id_0 fields are valid.
  uint8_t secondary_backup_supported : 1;  // Indicates that the backup_talker_entity_id_1 and the backup_talker_entity_id_1 fields are supported.
  uint8_t secondary_backup_valid : 1;      // Indicates that the backup_talker_entity_id_1 and the backup_talker_entity_id_1 fields are valid.
  uint8_t tertiary_backup_supported : 1;   // Indicates that the backup_talker_entity_id_2 and the backup_talker_entity_id_2 fields are supported.
  uint8_t tertiary_backup_valid : 1;       // Indicates that the backup_talker_entity_id_2 and the backup_talker_entity_id_2 fields are valid.
  uint8_t supports_avtp_udp_v4 : 1;        // Indicates that the Stream supports streaming using AVTP over UDP/IPv4 (17222016AnnexJ).
  uint8_t supports_avtp_udp_v6 : 1;        // Indicates that the Stream supports streaming using AVTP over UDP/IPv6 (17222016AnnexJ).
  uint8_t no_support_avtp_native : 1;      // Indicates that the Stream does not support streaming with native (L2,Ethertype0x22f0)AVTPDUs.
  uint8_t timing_field_valid : 1;          // Indicates that the timing field contains a valid TIMING descriptor index
  uint8_t no_media_clock : 1;              // Indicates that the stream does not use a media clock and so the clock_domain_index field does not contain a valid index.
  uint8_t supports_no_srp : 1;             // Indicates that the Stream supports streaming without an SRP preservation.
} stream_flags_s; // 2 bytes

/* AECP stream info flags 
 * used in get stream info response
*/
typedef struct {
  uint8_t not_registering_srp : 1;        // For a STREAM_INPUT, indicates that the Listener is not registering an SRP TalkerAdvertise or TalkerFailed attribute for the stream. For a STREAM_OUTPUT, indicates that the Talker is declaring an SRP TalkerAdvertise or TalkerFailed attribute and not registering a matching Listener attribute for the stream.
  uint8_t stream_vlan_id_valid : 1;       // Indicates that the stream_vlan_id field is valid.
  uint8_t connected : 1;                  // The Stream has been connected with ACMP. This may only be set in a response.
  uint8_t msrp_failure_valid : 1;         // The values in the msrp_failure_code and msrp_failure_bridge_id fields are valid.
  uint8_t stream_dest_mac_valid : 1;      // The value in the stream_dest_mac field is valid.
  uint8_t msrp_acc_lat_valid : 1;         // The value in the msrp_accumulated_latency field is valid.
  uint8_t stream_id_valid : 1;            // The value in the stream_id field is valid.
  uint8_t stream_format_valid : 1;        // The value in the stream_format field is valid and is to be used to change the StreamFormat if it is a SET_STREAM_INFO command.
  uint8_t reserved1 : 3;                  // Reserved bits
  uint8_t ip_flags_valid : 1;             // The value in the ip_flags field is valid.
  uint8_t ip_src_port_valid : 1;          // The value in the source_port field is valid.
  uint8_t ip_dst_port_valid : 1;          // The value in the destination_port field is valid.
  uint8_t ip_src_addr_valid : 1;          // The value in the source_ip_address field is valid.
  uint8_t ip_dst_addr_valid : 1;          // The value in the destination_ip_address field is valid.
  uint8_t reserved2 : 3;                  // Reserved bits
  uint8_t class_b : 1;                    // Indicates that the Stream is Class B instead of Class A (default 0 is classA)
  uint8_t fast_connect : 1;               // Reserved for backward compatibility. This flag used to indicate that the Stream was connected in Fast Connect Mode or is presently trying to connect in Fast Connect Mode.
  uint8_t saved_state : 1;                // Reserved for backward compatibility. This flag used to indicate that the connection has saved ACMP state associated with FAST_CONNECT.
  uint8_t streaming_wait : 1;             // The Stream is presently in STREAMING_WAIT, either it was connected with STREAMING_WAIT flag set or it was stopped with STOP_STREAMING command.
  uint8_t supports_encrypted : 1;         // Indicates that the Stream supports streaming with encrypted PDUs.
  uint8_t talker_failed : 1;              // Indicates that the Listener has registered an SRP TalkerFailed attribute for the Stream.
  uint8_t reserved3 : 3;                  // Reserved bits
  uint8_t no_srp : 1;                     // Indicates that SRP is not being used for the stream. The Talker will not register a TalkerAdvertise or wait for a Listener registration before streaming.
} stream_info_flags_s; // 4 bytes

/* Stream descriptor */
typedef struct {
  uint8_t descriptor_type[2];              // The type of the descriptor. Always set to STREAM_INPUT or STREAM_OUTPUT.
  uint8_t descriptor_index[2];             // The index of the descriptor.This is the index of the Stream.
  uint8_t object_name[64];                 // UTF8 string containing a Stream name.
  uint8_t localized_description[2];        // The localized string reference pointing to the localized Stream name. See 7.3.7.
  uint8_t clock_domain_index[2];           // The descriptor_index of the Clock Domain providing the media clock for the Stream. See 7.2.9.
  stream_flags_s stream_flags;             // Flags describing capabilities or features of the Stream. See Table79.
  uint8_t current_format[8];               // The Stream format of the current format, as defined in 7.3.3.
  uint8_t formats_offset[2];               // The offset from the start of the descriptor for the first octet of the formats. This field is 138 for this version of AEM.
  uint8_t number_of_formats[2];            // The number of formats supported by this audio Stream. The value of this field is referred to as N. The maximum value for this field is 46 for this version of AEM.
  unique_id_t backup_talker_entity_id_0;   // The primary backup ATDECCTalker's EntityID.
  uint8_t backup_talker_unique_id_0[2];    // The primary backup ATDECCTalker's UniqueID.
  unique_id_t backup_talker_entity_id_1;   // The secondary backup ATDECCTalker's EntityID.
  uint8_t backup_talker_unique_id_1[2];    // The secondary backup ATDECCTalker's UniqueID.
  unique_id_t backup_talker_entity_id_2;   // The tertiary backup ATDECCTalker's EntityID.
  uint8_t backup_talker_unique_id_2[2];    // The tertiary backup ATDECCTalker's UniqueID.
  unique_id_t backedup_talker_entity_id;   // The EntityID of the ATDECCTalker that this Stream is backing up.
  uint8_t backedup_talker_unique_id[2];    // The UniqueID of the ATDECCTalker that this Stream is backing up.
  uint8_t avb_interface_index[2];          // The descriptor_index of the AVB_INTERFACE from which this Stream is sourced or to which it is sinked.
  uint8_t buffer_length[4];                // The length in nanoseconds of the MAC's ingress or egress buffer as defined in IEEE Std1722-2016 Figure5.4. For a STREAM_INPUT this is the MAC's ingress buffer size and for a STREAM_OUTPUT this is the MAC's egress buffer size. This is the length of the buffer between the IEEE Std1722-2016 reference plane and the MAC.
  uint8_t redundant_offset[2];             // The offset from the start of the descriptor for the first octet of the redundant_streams array. This field is 138+8*N for this version of AEM.
  uint8_t number_of_redundant_streams[2];  // The number of redundant streams supported by this audio Stream. The value of this field is referred to as R. The maximum value for this field is 8 for this version of AEM.
  uint8_t timing[2];                       // The TIMING descriptor index which represents the source of gPTP time for the stream.
  uint8_t formats[4][8];                   // 4x8 Array of Stream formats of the supported formats, as defined in 7.3.3.
  //uint8_t redundant_streams[2*R];        // Array of redundant STREAM_INPUT or STREAM_OUTPUT descriptor indices. The current version of AEM doesn’t specify an ordering for the elements of this array.
} stream_desc_s; 

/* AECP stream summary 
 * used in get stream info response and talker list
 */
typedef struct {
  stream_info_flags_s flags; // stream descriptor flags
  uint8_t             stream_format[8]; // stream format
  unique_id_t         stream_id; // stream ID
  uint8_t             msrp_accumulated_latency[4]; // MSRP accumulated latency
  eth_addr_t          dest_addr; // stream destination MAC address
  uint8_t             msrp_failure_code;
} stream_summary_s; // 17 bytes

/* AECP get stream info command */
typedef struct {
  aecp_common_s common;
  uint8_t       descriptor_type[2]; // descriptor type
  uint8_t       descriptor_index[2]; // descriptor index
} aecp_get_stream_info_s; // 28 bytes

/* AECP get stream info response */
typedef struct {
  aecp_common_s       common;
  uint8_t             descriptor_type[2]; // descriptor type
  uint8_t             descriptor_index[2]; // descriptor index
  stream_summary_s    stream; // stream summary
  uint8_t             reserved;
  uint8_t             msrp_failure_bridge_id[8]; // MSRP failure bridge ID
  uint8_t             vlan_id[2]; // stream VLAN ID
  uint8_t             ip_flags[2]; // stream destination port
  uint8_t             src_port[2]; // stream source port
  uint8_t             dest_port[2]; // stream destination port
  uint8_t             src_ip_addr[16]; // stream source IP address
  uint8_t             dest_ip_addr[16]; // stream destination IP address
} aecp_get_stream_info_rsp_s; // 108 bytes

/* AECP Stream input counters valid flags */
typedef struct {
  uint8_t     entity_specific8 : 1;
  uint8_t     entity_specific7 : 1;
  uint8_t     entity_specific6 : 1;
  uint8_t     entity_specific5 : 1;
  uint8_t     entity_specific4 : 1;
  uint8_t     entity_specific3 : 1;
  uint8_t     entity_specific2 : 1;
  uint8_t     entity_specific1 : 1;
  uint8_t     reserved;
  uint8_t     unsupported_format : 1;
  uint8_t     late_ts : 1;
  uint8_t     early_ts : 1;
  uint8_t     frames_rx : 1;
  uint8_t     padding : 4;
  uint8_t     media_locked : 1;
  uint8_t     media_unlocked : 1;
  uint8_t     stream_interrupted : 1;
  uint8_t     seq_num_mismatch : 1;
  uint8_t     media_reset : 1;
  uint8_t     ts_uncertain : 1;
  uint8_t     ts_valid : 1;
  uint8_t     ts_not_valid : 1;
} stream_in_counters_val_s; // 4 bytes

/* AECP Stream output counters valid flags */
typedef struct {
  uint8_t     entity_specific8 : 1;
  uint8_t     entity_specific7 : 1;
  uint8_t     entity_specific6 : 1;
  uint8_t     entity_specific5 : 1;
  uint8_t     entity_specific4 : 1;
  uint8_t     entity_specific3 : 1;
  uint8_t     entity_specific2 : 1;
  uint8_t     entity_specific1 : 1;
  uint8_t     reserved[2];
  uint8_t     stream_start : 1;
  uint8_t     stream_stop : 1;
  uint8_t     stream_interrupted : 1;
  uint8_t     media_reset : 1;
  uint8_t     ts_uncertain : 1;
  uint8_t     ts_valid : 1;
  uint8_t     ts_not_valid : 1;
  uint8_t     frames_tx : 1;
} stream_out_counters_val_s; // 4 bytes

/* AECP Stream input counters block */
typedef struct {
  uint8_t     media_locked : 1;
  uint8_t     media_unlocked : 1;
  uint8_t     stream_interrupted : 1;
  uint8_t     seq_num_mismatch : 1;
  uint8_t     media_reset : 1;
  uint8_t     ts_uncertain : 1;
  uint8_t     ts_valid : 1;
  uint8_t     ts_not_valid : 1;
  uint8_t     unsupported_format : 1;
  uint8_t     late_ts : 1;
  uint8_t     early_ts : 1;
  uint8_t     frames_rx : 1;
  uint8_t     reserved[48];
  uint8_t     entity_specific8 : 1;
  uint8_t     entity_specific7 : 1;
  uint8_t     entity_specific6 : 1;
  uint8_t     entity_specific5 : 1;
  uint8_t     entity_specific4 : 1;
  uint8_t     entity_specific3 : 1;
  uint8_t     entity_specific2 : 1;
  uint8_t     entity_specific1 : 1;
} stream_in_counters_s; // 32 bytes

/* AECP Stream output counters block */
typedef struct {
  uint8_t     stream_start[4];
  uint8_t     stream_stop[4];
  uint8_t     stream_interrupted[4];
  uint8_t     media_reset[4];
  uint8_t     ts_uncertain[4];
  uint8_t     ts_valid[4];
  uint8_t     ts_not_valid[4];
  uint8_t     frames_tx[4];
  uint8_t     reserved[64];
  uint8_t     entity_specific8 : 1;
  uint8_t     entity_specific7 : 1;
  uint8_t     entity_specific6 : 1;
  uint8_t     entity_specific5 : 1;
  uint8_t     entity_specific4 : 1;
  uint8_t     entity_specific3 : 1;
  uint8_t     entity_specific2 : 1;
  uint8_t     entity_specific1 : 1;
} stream_out_counters_s; // 128 bytes

/* AECP stream counters valid flags union */
typedef union {
  stream_in_counters_val_s  stream_in_counters_val;
  stream_out_counters_val_s stream_out_counters_val;
} stream_counters_val_u; // 4 bytes

/* AECP stream counters block union */
typedef union {
  stream_in_counters_s  stream_in_counters;
  stream_out_counters_s stream_out_counters;
} stream_counters_u; // 128 bytes

/* AECP get counters command */
typedef struct {
  aecp_common_s common;
  uint8_t       descriptor_type[2]; // descriptor type
  uint8_t       descriptor_index[2]; // descriptor index
} aecp_get_counters_s; // 36 bytes

/* AECP get counters response */
typedef struct {
  aecp_common_s         common;
  uint8_t               descriptor_type[2]; // descriptor type
  uint8_t               descriptor_index[2]; // descriptor index
  stream_counters_val_u counters_valid; // counters valid
  stream_counters_u     counters_block; // counters block
} aecp_get_counters_rsp_s; // 36 bytes

/* AECP message union */
typedef union {
  atdecc_header_s         header;
  aecp_common_s           common;
  aecp_get_stream_info_s  get_stream_info;
  aecp_get_counters_s     get_counters;
  aecp_get_counters_rsp_s get_counters_rsp;
  uint8_t                 raw[500];
} aecp_message_u; // 36 bytes

/* ACMP Message */
typedef struct {
  atdecc_header_s header;
  unique_id_t     stream_id; // stream ID
  unique_id_t     controller_entity_id; // controller entity ID
  unique_id_t     talker_entity_id; // talker entity ID
  unique_id_t     listener_entity_id; // listener entity ID
  uint8_t         talker_uid[2]; // talker UID
  uint8_t         listener_uid[2]; // listener UID
  eth_addr_t      stream_dest_addr; // stream destination address
  uint8_t         connection_count[2]; // connection count
  uint8_t         seq_id[2]; // sequence ID
  uint8_t         flags[2]; // flags
  uint8_t         stream_vlan_id[2]; // stream VLAN ID
  uint8_t         conn_listeners_entries[2]; // connection listeners entries
  uint8_t         ip_flags[2]; // IP flags
  uint8_t         reserved[2]; // reserved
  uint8_t         src_port[2]; // source port
  uint8_t         dest_port[2]; // destination port
  uint8_t         src_ip_addr[4]; // source IP address
  uint8_t         dest_ip_addr[4]; // destination IP address
} acmp_message_s; // 96 bytes

/* AVTP message buffer */
typedef union {
  uint8_t           subtype;
  aaf_pcm_message_s aaf;
  maap_message_s    maap;
  adp_message_s     adp;
  aecp_message_u    aecp;
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

/* Talker */
typedef struct {
  unique_id_t       entity_id; // entity ID
  unique_id_t       model_id; // model ID
  eth_addr_t        mac_addr; // mac address
  talker_adv_info_s info; // from talker advertise
  stream_summary_s  stream; // from get stream info
  uint8_t           failure_code; // msrp failure code
  bool              streaming; // streaming status
  bool              ready; // general status
} avb_talker_s;

/* Listener */
typedef struct {
  unique_id_t stream_id; // stream ID
  uint8_t     vlan_id[2]; // vlan ID
  unique_id_t entity_id; // entity ID
  unique_id_t model_id; // model ID
  eth_addr_t  mac_addr; // mac address
  uint8_t     last_event; // last listener event
  bool        ready; // status
} avb_listener_s;

/* Connection */
typedef struct {
  bool            as_talker; // we are talker or listener
  unique_id_t     stream_id; // stream ID
  uint8_t         vlan_id[2]; // vlan ID
  unique_id_t     entity_id; // other entity ID
  unique_id_t     model_id; // other model ID
  struct timespec started; // last start timestamp
  int64_t         accumulated_latency; // observed latency
  int64_t         accumulated_jitter; // observed jitter
  bool            active; // status
} avb_connection_s;

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
  sr_class_priority_mismatch,
  enhanced_feature_cannot_be_propagated_to_original_port,
  max_latency_exceeded,
  nearest_bridge_cannot_provide_network_identification_for_stream_transformation,
  stream_transformation_not_supported,
  stream_identification_type_not_supported_for_stream_transformation,
  enhanced_feature_cannot_be_supported_without_a_cnc
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
	aecp_cmd_code_get_configuration,
	aecp_cmd_code_read_descriptor,
	aecp_cmd_code_get_stream_info,
	aecp_cmd_code_get_counters,
	aecp_cmd_code_register_unsol_notif,
	aecp_cmd_code_deregister_unsol_notif
} aecp_cmd_code_t;

/* AECP descriptor types in enumerated order */
typedef enum {
  aecp_desc_type_entity,
  aecp_desc_type_configuration
} aecp_desc_type_t;

/* AECP statuses in enumerated order */
typedef enum {
  aecp_status_success, // The ATDECC Entity successfully performed the command and has valid results.
  aecp_status_not_implemented, // The ATDECC Entity does not support the command type.
  aecp_status_no_such_descriptor, // A descriptor with the descriptor_type and descriptor_index specified does not exist.
  aecp_status_entity_locked, // The ATDECC Entity has been locked by another ATDECC Controller.
  aecp_status_entity_acquired, // The ATDECC Entity has been acquired by another ATDECC Controller.
  aecp_status_not_authenticated, // The ATDECC Controller is not authenticated with the ATDECC Entity.
  aecp_status_authentication_disabled, // The ATDECC Controller is trying to use an authentication command when authentication is not enabled on the ATDECC Entity.
  aecp_status_bad_arguments, // One or more of the values in the fields of the frame were deemed to be bad by the ATDECC Entity (unsupported, incorrect combination, etc.).
  aecp_status_no_resources, // The ATDECC Entity cannot complete the command because it does not have the resources to support it.
  aecp_status_in_progress, // The ATDECC Entity is processing the command and will send a second response at a later time with the result of the command.
  aecp_status_entity_misbehaving, // The ATDECC Entity generated an internal error while trying to process the command.
  aecp_status_not_supported, // The command is implemented but the target of the command is not supported. For example, trying to set the value of a read-only control.
  aecp_status_stream_is_running // The stream is currently streaming and the command is one which cannot be executed on a streaming stream.
} aecp_status_t;

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

/* ACMP statuses in enumerated order */
typedef enum {
  acmp_status_success, // Command executed successfully
  acmp_status_listener_unknown_id, // Listener does not have the specified unique identifier
  acmp_status_talker_unknown_id, // Talker does not have the specified unique identifier
  acmp_status_talker_dest_mac_fail, // Talker could not allocate a destination MAC for the stream
  acmp_status_talker_no_stream_index, // Talker does not have an available stream index for the stream
  acmp_status_talker_no_bandwidth, // Talker could not allocate bandwidth for the stream
  acmp_status_talker_exclusive, // Talker already has an established stream and only supports one Listener
  acmp_status_listener_talker_timeout, // Listener had timeout for all retries when trying to send command to Talker
  acmp_status_listener_exclusive, // The ATDECC Listener already has an established connection to stream
  acmp_status_state_unavailable, // Could not get the state from the ATDECC Entity
  acmp_status_not_connected, // Trying to disconnect when not connected or not connected to the ATDECC Talker specified
  acmp_status_no_such_connection, // Trying to obtain connection information for an ATDECC Talker connection which does not exist
  acmp_status_could_not_send_message, // The ATDECC Listener failed to send the message to the ATDECC Talker
  acmp_status_talker_misbehaving, // Talker was unable to complete the command because an internal error occurred
  acmp_status_listener_misbehaving, // Listener was unable to complete the command because an internal error occurred
  acmp_status_reserved, // Reserved for future use
  acmp_status_controller_not_authorized, // The ATDECC Controller with the specified Entity ID is not authorized to change stream connections
  acmp_status_incompatable_request, // The ATDECC Listener is trying to connect to an ATDECC Talker that is already streaming with a different traffic class, etc. or does not support the requested traffic class
  acmp_status_listener_invalid_connection, // ATDECC Listener is being asked to connect to something that it cannot listen to, e.g. it is being asked to listen to its own ATDECC Talker stream
  acmp_status_listener_can_only_listen_once, // The ATDECC Listener is being asked to connect to a stream that is already connected to another one of its streams sinks and it is only capable of listening on one of them
  acmp_status_not_supported // The command is not supported
} acmp_status_t;

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

/* AVB send functions */

/* MVRP send functions */
int avb_send_mvrp_vlan_id(struct avb_state_s *state);

/* MSRP send functions */
int avb_send_msrp_domain(struct avb_state_s *state);
int avb_send_msrp_talker_adv(struct avb_state_s *state, 
                             msrp_attr_event_t event);
int avb_send_msrp_talker_failed(struct avb_state_s *state, 
                                msrp_attr_event_t event);
int avb_send_msrp_listener(struct avb_state_s *state, 
                           msrp_attr_event_t attr_event, 
                           msrp_listener_event_t listener_event);

/* AVTP send functions */
int avb_send_maap_announce(struct avb_state_s *state);
int avb_send_aaf_pcm(struct avb_state_s *state);

/* ATDECC send functions */
int avb_send_adp_entity_available(struct avb_state_s *state);
int avb_send_aecp_cmd_controller_available(struct avb_state_s *state);
int avb_send_aecp_cmd_get_stream_info(struct avb_state_s *state);
int avb_send_aecp_cmd_get_counters(struct avb_state_s *state);
int avb_send_aecp_rsp_get_stream_info(struct avb_state_s *state); // as unsolicited notification
int avb_send_aecp_rsp_get_counters(struct avb_state_s *state); // as unsolicited notification
int avb_send_acmp_connect_rx_command(struct avb_state_s *state); // acting as controller
int avb_send_acmp_connect_tx_command(struct avb_state_s *state);
int avb_send_acmp_disconnect_rx_command(struct avb_state_s *state); // acting as controller
int avb_send_acmp_disconnect_tx_command(struct avb_state_s *state);

/* AVB processing functions */

/* MVRP processing functions */
int avb_process_mvrp_vlan_id(struct avb_state_s *state, 
                             mvrp_vlan_id_message_s *msg);

/* MSRP processing functions */
int avb_process_msrp_domain(struct avb_state_s *state,
                            msrp_msgbuf_s *msg,
                            int offset,
                            size_t length);
int avb_process_msrp_talker(struct avb_state_s *state,
                            msrp_msgbuf_s *msg,
                            int offset,
                            size_t length,
                            bool is_failed,
                            eth_addr_t *src_addr);
int avb_process_msrp_listener(struct avb_state_s *state,
                              msrp_msgbuf_s *msg,
                              int offset,
                              size_t length);

/* AVTP processing functions */
int avb_process_maap(struct avb_state_s *state, maap_message_s *msg);
int avb_process_aaf(struct avb_state_s *state, aaf_pcm_message_s *msg);

/* ADP processing functions */
int avb_process_adp(struct avb_state_s *state, 
                    adp_message_s *msg, 
                    eth_addr_t *src_addr); // handle all adp messages

/* AECP processing functions */
int avb_process_aecp(struct avb_state_s *state, 
                     aecp_message_u *msg); // route to specific func
int avb_process_acmp(struct avb_state_s *state, 
                     acmp_message_s *msg); // route to specific func
int avb_process_aecp_cmd_entity_available(struct avb_state_s *state,
                                          aecp_message_u *msg);
int avb_process_aecp_cmd_lock_entity(struct avb_state_s *state,
                                     aecp_message_u *msg);
int avb_process_aecp_cmd_acquire_entity(struct avb_state_s *state,
                                        aecp_message_u *msg);
int avb_process_aecp_cmd_get_configuration(struct avb_state_s *state,
                                           aecp_message_u *msg);
int avb_process_aecp_cmd_read_descriptor(struct avb_state_s *state,
                                         aecp_message_u *msg);
int avb_process_aecp_cmd_get_stream_info(struct avb_state_s *state,
                                         aecp_message_u *msg);
int avb_process_aecp_cmd_get_counters(struct avb_state_s *state,
                                      aecp_message_u *msg);
int avb_process_aecp_rsp_register_unsol_notif(struct avb_state_s *state,
                                              aecp_message_u *msg);
int avb_process_aecp_rsp_entity_available(struct avb_state_s *state,
                                          aecp_message_u *msg);
int avb_process_aecp_rsp_controller_available(struct avb_state_s *state,
                                              aecp_message_u *msg);
int avb_process_aecp_rsp_get_stream_info(struct avb_state_s *state,
                                         aecp_message_u *msg);
int avb_process_aecp_rsp_get_counters(struct avb_state_s *state,
                                      aecp_message_u *msg);

/* ACMP processing functions */
int avb_process_acmp_connect_rx_command(struct avb_state_s *state,
                                        acmp_message_s *msg);
int avb_process_acmp_connect_tx_command(struct avb_state_s *state,
                                        acmp_message_s *msg);
int avb_process_acmp_disconnect_rx_command(struct avb_state_s *state,
                                            acmp_message_s *msg);
int avb_process_acmp_disconnect_tx_command(struct avb_state_s *state,
                                            acmp_message_s *msg);
int avb_process_acmp_connect_rx_response(struct avb_state_s *state,
                                         acmp_message_s *msg);
int avb_process_acmp_connect_tx_response(struct avb_state_s *state,
                                         acmp_message_s *msg);
int avb_process_acmp_disconnect_rx_response(struct avb_state_s *state,
                                            acmp_message_s *msg);
int avb_process_acmp_disconnect_tx_response(struct avb_state_s *state,
                                            acmp_message_s *msg);

/* Helper functions */
int avb_add_entity(struct avb_state_s *state, 
                    avb_entity_summary_s *entity, 
                    eth_addr_t *entity_addr, 
                    bool is_talker);
int avb_find_entity(struct avb_state_s *state, 
                    unique_id_t *entity_id, 
                    bool is_talker);
const char* get_adp_message_type_name(adp_msg_type_t message_type);
const char* get_aecp_command_code_name(aecp_cmd_code_t command_code);
const char* get_acmp_message_type_name(acmp_msg_type_t message_type);

#endif /* _ESP_AVB_AVB_H_ */
