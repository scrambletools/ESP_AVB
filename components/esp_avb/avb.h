/*
 * Copyright 2024 Scramble Tools
 * License: MIT
 *
 * ESP_AVB Component
 *
 * This component provides an implementation of an AVB talker and listener.
 * 
 * This file provides the common definitions and types for the AVB protocol.
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
#include <esp_check.h>
#include <semaphore.h>
#include <esp_log.h>
#include <esp_err.h>
#include <lwip/prot/ethernet.h> // Ethernet headers
#include <esp_eth_time.h>
#include <ptpd.h>
#include <driver/i2s_std.h>
#include <driver/i2c_master.h>
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "es8311.h"
#include <esp_heap_caps.h>
#include "esp_avb.h"
#include "avbutils.h"
#include "config.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Number of protocols to use for L2TAP (AVTP, MSRP, MVRP) */
#define AVB_NUM_PROTOCOLS 3

/* Protocol indexes */
#define AVTP 0
#define MSRP 1
#define MVRP 2

/* Maximum number of endpoints to remember */
#define AVB_MAX_NUM_TALKERS 10
#define AVB_MAX_NUM_LISTENERS 10
#define AVB_MAX_NUM_CONTROLLERS 10

/* Maximum number of listeners connected to a talker stream */
#define AVB_MAX_NUM_CONNECTED_LISTENERS 10

/* Maximum number of inflight commands */
#define AVB_MAX_NUM_INFLIGHT_COMMANDS 20

/* Maximum number of streams */
#define AVB_MAX_NUM_INPUT_STREAMS 1
#define AVB_MAX_NUM_OUTPUT_STREAMS 1

/* Periodic message intervals */
#define MSRP_DOMAIN_INTERVAL_MSEC 2000
#define MVRP_VLAN_ID_INTERVAL_MSEC 10000
#define MSRP_TALKER_IDLE_INTERVAL_MSEC 3000 // when idle
#define MSRP_TALKER_CONN_INTERVAL_MSEC 3000 // when connected
#define MSRP_LISTENER_IDLE_INTERVAL_MSEC 3000 // when idle
#define MSRP_LISTENER_CONN_INTERVAL_MSEC 3000 // when connected
#define MSRP_LEAVEALL_INTERVAL_MSEC 10000  
#define ADP_ENTITY_AVAIL_INTERVAL_MSEC 5000
#define MAAP_ANNOUNCE_INTERVAL_MSEC 10000
#define PTP_STATUS_UPDATE_INTERVAL_MSEC 3000
#define UNSOL_NOTIF_INTERVAL_MSEC 2000

/* Default VLAN ID */
#define DEFAULT_VLAN_ID 0

// Commonly used mac addresses
#define BCAST_MAC_ADDR      (uint8_t[6]){ 0x91, 0xe0, 0xf0, 0x01, 0x00, 0x00 } // adp,acmp
#define MAAP_MCAST_MAC_ADDR (uint8_t[6]){ 0x91, 0xe0, 0xf0, 0x00, 0xff, 0x00 } // maap
#define MAAP_START_MAC_ADDR (uint8_t[6]){ 0x91, 0xe0, 0xf0, 0x00, 0xf2, 0x00 } // maap, can change
#define SPANTREE_MAC_ADDR   (uint8_t[6]){ 0x01, 0x80, 0xc2, 0x00, 0x00, 0x21 } // mvrp
#define LLDP_MCAST_MAC_ADDR (uint8_t[6]){ 0x01, 0x80, 0xc2, 0x00, 0x00, 0x0e } // msrp

/* Empty ID */
#define EMPTY_ID (uint8_t[8]){0,0,0,0,0,0,0,0}

/* MVU Protocol ID */
#define MVU_PROTOCOL_ID (uint8_t[6]){0x00, 0x1B, 0xC5, 0x0A, 0xC1, 0x00}

/* Poll interval for checking for incoming frames on L2TAP FDs */
#define AVB_POLL_INTERVAL_MS 1

/* Maximum message length */
#define AVB_MAX_MSG_LEN 600

/* Preamble before the control data */
#define AVTP_CDL_PREAMBLE_LEN 12

/* Preamble before the descriptor */
#define AECP_DESC_PREAMBLE_LEN 28

/* Size of a unique ID */
#define UNIQUE_ID_LEN 8

/* Configuration index settings */
#define DEFAULT_CONFIG_INDEX 0

/* MRP event limits */
#define MRP_MAX_NUM_EVENTS 20 // max number of events

/* AVTP data limits */
#define AVTP_STREAM_DATA_PER_MSG 80 // max number of stream data per message

/* AEM descriptor limits */
#define AEM_MAX_DESC_LEN 504 // descriptor max length (not including descriptor type and index)
#define AEM_MAX_NUM_CONFIGS 1 // only one config supported for now
#define AEM_MAX_NUM_DESC 20 // max number of descriptors in a config
#define AEM_MAX_DESC_COUNT 1 // max count of each descriptor in a config
#define AEM_MAX_NUM_SAMPLE_RATES 10 // max number of sample rates
#define AEM_MAX_NUM_FORMATS 10 // max number of formats
#define AEM_MAX_NUM_MAPPINGS 10 // max number of mappings per map descriptor (spec max is 62)
#define AEM_MAX_NUM_CLOCK_SOURCES 10 // max number of clock sources per clock domain descriptor (spec max is 216)
#define AEM_MAX_LEN_CONTROL_VAL_DETAILS 100 // max length of control value details (spec max is 404)

/* Timespec functions */
#define clock_timespec_subtract(ts1, ts2, ts3) timespecsub(ts1, ts2, ts3)
#define clock_timespec_add(ts1, ts2, ts3) timespecadd(ts1, ts2, ts3)

/* Default format */
#define AVB_DEFAULT_FORMAT(cip_sfc_sample_rate) { \
    .subtype = 0, \
    .vendor_defined = 0, \
    .format = 0x10, \
    .sf = 1, \
    .fdf_sfc = cip_sfc_sample_rate, \
    .fdf_evt = 0, \
    .dbs = 8, \
    .sc = 0, \
    .ut = 0, \
    .nb = 1, \
    .b = 0, \
    .label_iec_60958_cnt = 0, \
    .label_mbla_cnt = 8, \
    .label_smptecnt = 0, \
    .label_midi_cnt = 0 \
  }

  /* Default format */
#define AVB_DEFAULT_FORMAT_AAF(bit_rate, aaf_pcm_sample_rate) { \
    .subtype = 2, \
    .vendor_defined = 0, \
    .sample_rate = aaf_pcm_sample_rate, \
    .ut = 1, \
    .format = 0x02, \
    .bit_depth = bit_rate, \
    .chan_per_frame_h = 0, \
    .samples_per_frame_h = 0, \
    .chan_per_frame = 8, \
    .samples_per_frame = 6 \
  }

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* AVB Enums*/

/* Ethertypes 
 * must convert to big-endian before writing to Ethernet header
 */
typedef enum {
	ethertype_msrp = 0x22ea,
	ethertype_avtp = 0x22f0,
	ethertype_mvrp = 0x88f5,
	ethertype_gptp = 0x88f7
} ethertype_t;

/* AVB types of entities */
typedef enum {
    avb_entity_type_talker,
    avb_entity_type_listener,
    avb_entity_type_controller
} avb_entity_type_t;

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

/* MRP attribute events in enumerated order */
typedef enum {
    mrp_attr_event_new,
    mrp_attr_event_join_in,
    mrp_attr_event_in,
    mrp_attr_event_join_mt,
    mrp_attr_event_mt,
    mrp_attr_event_lv,
    mrp_attr_event_none // no event
} mrp_attr_event_t;

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

/* AVTP subtypes and their values */
typedef enum {
    avtp_subtype_61883 = 0x00,  
    avtp_subtype_aaf   = 0x02,
    avtp_subtype_adp   = 0xfa,
    avtp_subtype_aecp  = 0xfb,
    avtp_subtype_acmp  = 0xfc,
    avtp_subtype_maap  = 0xfe
} avtp_subtype_t;

/* MAAP message types and their values */
typedef enum {
    maap_msg_type_probe    = 0x01,
    maap_msg_type_defend   = 0x02,
    maap_msg_type_announce = 0x03
} maap_msg_type_t;

/* ADP message types in enumerated order (1722.1 Clause 6.2) */
typedef enum {
	adp_msg_type_entity_available,
	adp_msg_type_entity_departing,
	adp_msg_type_entity_discover
} adp_msg_type_t;

/* AECP message types (subset) and their values (1722.1 Clause 9) */
typedef enum {
	aecp_msg_type_aem_command            = 0,
	aecp_msg_type_aem_response           = 1,
    aecp_msg_type_addr_access_command    = 2,
    aecp_msg_type_addr_access_response   = 3,
    aecp_msg_type_vendor_unique_command  = 6,
    aecp_msg_type_vendor_unique_response = 7
} aecp_msg_type_t;

/* AECP command codes (subset) and their values (1722.1 Clause 7.4) 
 * must change to big-endian before writing to message body
 */
typedef enum {
	aecp_cmd_code_acquire_entity         = 0x0000,
	aecp_cmd_code_lock_entity            = 0x0001,
	aecp_cmd_code_entity_available       = 0x0002,
	aecp_cmd_code_controller_available   = 0x0003, // unsupported
    aecp_cmd_code_read_descriptor        = 0x0004,
	aecp_cmd_code_get_configuration      = 0x0007,
    aecp_cmd_code_set_stream_format      = 0x0008, // unsupported
    aecp_cmd_code_get_stream_format      = 0x0009, // unsupported
    aecp_cmd_code_get_stream_info        = 0x000f,
    aecp_cmd_code_set_name               = 0x0010, // unsupported
    aecp_cmd_code_get_name               = 0x0011, // unsupported
    aecp_cmd_code_set_clock_source       = 0x0016, // unsupported
    aecp_cmd_code_get_clock_source       = 0x0017,
    aecp_cmd_code_set_control            = 0x0018, // unsupported
    aecp_cmd_code_get_control            = 0x0019, 
    aecp_cmd_code_start_streaming        = 0x0022, // unsupported
    aecp_cmd_code_stop_streaming         = 0x0023, // unsupported
	aecp_cmd_code_register_unsol_notif   = 0x0024,
	aecp_cmd_code_deregister_unsol_notif = 0x0025,
    aecp_cmd_code_get_avb_info           = 0x0027,
    aecp_cmd_code_get_as_path            = 0x0028, // unsupported
    aecp_cmd_code_get_counters           = 0x0029,
    aecp_cmd_code_expansion              = 0xffff  // for milan
} aecp_cmd_code_t;

/* AECP statuses in enumerated order */
typedef enum {
    aecp_status_success,                    // The ATDECC Entity successfully performed the command and has valid results.
    aecp_status_not_implemented,            // The ATDECC Entity does not support the command type.
    aecp_status_no_such_descriptor,         // A descriptor with the descriptor_type and descriptor_index specified does not exist.
    aecp_status_entity_locked,              // The ATDECC Entity has been locked by another ATDECC Controller.
    aecp_status_entity_acquired,            // The ATDECC Entity has been acquired by another ATDECC Controller.
    aecp_status_not_authenticated,          // The ATDECC Controller is not authenticated with the ATDECC Entity.
    aecp_status_authentication_disabled,    // The ATDECC Controller is trying to use an authentication command when authentication is not enabled on the ATDECC Entity.
    aecp_status_bad_arguments,              // One or more of the values in the fields of the frame were deemed to be bad by the ATDECC Entity (unsupported, incorrect combination, etc.).
    aecp_status_no_resources,               // The ATDECC Entity cannot complete the command because it does not have the resources to support it.
    aecp_status_in_progress,                // The ATDECC Entity is processing the command and will send a second response at a later time with the result of the command.
    aecp_status_entity_misbehaving,         // The ATDECC Entity generated an internal error while trying to process the command.
    aecp_status_not_supported,              // The command is implemented but the target of the command is not supported. For example, trying to set the value of a read-only control.
    aecp_status_stream_is_running           // The stream is currently streaming and the command is one which cannot be executed on a streaming stream.
} aecp_status_t;

/* AEM descriptor types and their values 
 * must change to big-endian before writing to message body
 */
typedef enum {
    aem_desc_type_entity             = 0x0000, // The ATDECC Entity
    aem_desc_type_configuration      = 0x0001, // a configuration of the ATDECC Entity
    aem_desc_type_audio_unit         = 0x0002, // an Audio Unit
    aem_desc_type_video_unit         = 0x0003, // a Video Unit
    aem_desc_type_sensor_unit        = 0x0004, // a Sensor Unit with one or more sensors sampled with the same clock
    aem_desc_type_stream_input       = 0x0005, // an input stream to the ATDECC Entity
    aem_desc_type_stream_output      = 0x0006, // an output stream from the ATDECC Entity
    aem_desc_type_avb_interface      = 0x0009, // an AVB interface
    aem_desc_type_clock_source       = 0x000a, // an clock source
    aem_desc_type_memory_object      = 0x000b, // an memory object for files or firmware
    aem_desc_type_locale             = 0x000c, // a locale
    aem_desc_type_strings            = 0x000d, // a localized strings
    aem_desc_type_stream_port_input  = 0x000e, // an input stream port on a Unit
    aem_desc_type_stream_port_output = 0x000f, // an output stream port on a Unit
    aem_desc_type_audio_cluster      = 0x0014, // a cluster of channels within an audio Stream
    aem_desc_type_video_cluster      = 0x0015, // an element of the video Stream
    aem_desc_type_sensor_cluster     = 0x0016, // the sensor elements of a sensor stream
    aem_desc_type_audio_map          = 0x0017, // mapping between the channels of an audio Stream and the channels of the audio Port
    aem_desc_type_video_map          = 0x0018, // mapping between the components of a video Stream and the video clusters of the video Port
    aem_desc_type_sensor_map         = 0x0019, // mapping between a Sensor signal and the Sensor stream
    aem_desc_type_control            = 0x001a, // a generic Control
    aem_desc_type_signal_selector    = 0x001b, // a Signal Selector Control
    aem_desc_type_clock_domain       = 0x0024, // a Clock Domain
    aem_desc_type_invalid            = 0xffff  // there is no valid descriptor
} aem_desc_type_t;

/* AEM Clock Source Types and their values */
typedef enum {
    aem_clock_source_type_internal     = 0x0000,
    aem_clock_source_type_external     = 0x0001,
    aem_clock_source_type_input_stream = 0x0002,
    aem_clock_source_type_expansion    = 0xffff
} aem_clock_source_type_t;

/* AEM Memory Object Types and their values */
typedef enum {
    aem_memory_obj_type_firmware_image     = 0x0000,
    aem_memory_obj_type_vendor_specific    = 0x0001,
    aem_memory_obj_type_crash_dump         = 0x0002,
    aem_memory_obj_type_log_object         = 0x0003,
    aem_memory_obj_type_autostart_settings = 0x0004,
    aem_memory_obj_type_snapshot_settings  = 0x0005,
    aem_memory_obj_type_svg_manufacturer   = 0x0006,
    aem_memory_obj_type_svg_entity         = 0x0007,
    aem_memory_obj_type_svg_generic        = 0x0008,
    aem_memory_obj_type_png_manufacturer   = 0x0009,
    aem_memory_obj_type_png_entity         = 0x000a,
    aem_memory_obj_type_png_generic        = 0x000b,
    aem_memory_obj_type_dae_manufacturer   = 0x000c,
    aem_memory_obj_type_dae_entity         = 0x000d,
    aem_memory_obj_type_dae_generic        = 0x000e
} aem_memory_obj_type_t;

/* AEM Memory Object Operation Types and their values */
typedef enum {
    aem_memory_obj_op_type_store            = 0x0000,
    aem_memory_obj_op_type_store_and_reboot = 0x0001,
    aem_memory_obj_op_type_read             = 0x0002,
    aem_memory_obj_op_type_erase            = 0x0003,
    aem_memory_obj_op_type_upload           = 0x0004
} aem_memory_obj_op_type_t;

/* AEM Audio Cluster formats and their values */
typedef enum {
    aem_audio_cluster_format_iec_60958 = 0x00,
    aem_audio_cluster_format_mbla      = 0x40,
    aem_audio_cluster_format_midi      = 0x80,
    aem_audio_cluster_format_smpte     = 0x88
} aem_audio_cluster_format_t;

/* ACMP message types (subset) in enumerated order (1722.1 Clause 8.2) */
typedef enum {
	acmp_msg_type_connect_tx_command,      // Connect Talker source stream command
	acmp_msg_type_connect_tx_response,     // Connect Talker source stream response
	acmp_msg_type_disconnect_tx_command,   // Disconnect Talker source stream command
    acmp_msg_type_disconnect_tx_response,  // Disconnect Talker source stream response
	acmp_msg_type_get_tx_state_command,    // Get Talker source stream connection state command
	acmp_msg_type_get_tx_state_response,   // Get Talker source stream connection state response
	acmp_msg_type_connect_rx_command,      // Connect Listener sink stream command
    acmp_msg_type_connect_rx_response,     // Connect Listener sink stream response
    acmp_msg_type_disconnect_rx_command,   // Disconnect Listener sink stream command
	acmp_msg_type_disconnect_rx_response,  // Disconnect Listener sink stream response
	acmp_msg_type_get_rx_state_command,    // Get Listener sink stream connection state command
	acmp_msg_type_get_rx_state_response,   // Get Listener sink stream connection state response
    acmp_msg_type_connection_command,      // Get a specific Talker connection info command
    acmp_msg_type_connection_response,     // Get a specific Talker connection info response
    acmp_msg_type_count
} acmp_msg_type_t;

/* ACMP statuses in enumerated order */
typedef enum {
    acmp_status_success,                          // Command executed successfully
    acmp_status_listener_unknown_id,              // Listener does not have the specified unique identifier
    acmp_status_talker_unknown_id,                // Talker does not have the specified unique identifier
    acmp_status_talker_dest_mac_fail,             // Talker could not allocate a destination MAC for the stream
    acmp_status_talker_no_stream_index,           // Talker does not have an available stream index for the stream
    acmp_status_talker_no_bandwidth,              // Talker could not allocate bandwidth for the stream
    acmp_status_talker_exclusive,                 // Talker already has an established stream and only supports one Listener
    acmp_status_listener_talker_timeout,          // Listener had timeout for all retries when trying to send command to Talker
    acmp_status_listener_exclusive,               // The ATDECC Listener already has an established connection to stream
    acmp_status_state_unavailable,                // Could not get the state from the ATDECC Entity
    acmp_status_not_connected,                    // Trying to disconnect when not connected or not connected to the ATDECC Talker specified
    acmp_status_no_such_connection,               // Trying to obtain connection information for an ATDECC Talker connection which does not exist
    acmp_status_could_not_send_message,           // The ATDECC Listener failed to send the message to the ATDECC Talker
    acmp_status_talker_misbehaving,               // Talker was unable to complete the command because an internal error occurred
    acmp_status_listener_misbehaving,             // Listener was unable to complete the command because an internal error occurred
    acmp_status_reserved,                         // Reserved for future use
    acmp_status_controller_not_authorized,        // The ATDECC Controller with the specified Entity ID is not authorized to change stream connections
    acmp_status_incompatable_request,             // The ATDECC Listener is trying to connect to an ATDECC Talker that is already streaming with a different traffic class, etc. or does not support the requested traffic class
    acmp_status_listener_invalid_connection,      // ATDECC Listener is being asked to connect to something that it cannot listen to, e.g. it is being asked to listen to its own ATDECC Talker stream
    acmp_status_listener_can_only_listen_once,    // The ATDECC Listener is being asked to connect to a stream that is already connected to another one of its streams sinks and it is only capable of listening on one of them
    acmp_status_not_supported = 31                // The command is not supported
} acmp_status_t;

/* ACMP timeouts in milliseconds */
typedef enum {
    acmp_timeout_connect_tx     = 2000,
    acmp_timeout_disconnect_tx  = 200,
    acmp_timeout_get_tx_state   = 200,
    acmp_timeout_connect_rx     = 4500,
    acmp_timeout_disconnect_rx  = 500,
    acmp_timeout_get_rx_state   = 200,
    acmp_timeout_connection     = 200
} acmp_timeout_t;

/* 61883 formats and their values */
typedef enum {
    iec61883_format_bt_601   = 0x01,
    iec61883_format_audio    = 0x10,
    iec61883_format_mpeg2_ts = 0x20,
    iec61883_format_bo_1294  = 0x21
} iec61883_format_t;

/* AAF format values in enumerated order */
typedef enum {
    aaf_format_user,        // user defined
    aaf_format_float_32bit, // 32-bit floating point PCM
    aaf_format_int_32bit,   // 32-bit integer PCM; default for talker
    aaf_format_int_24bit,   // 24-bit integer PCM
    aaf_format_int_16bit,   // 16-bit integer PCM
    aaf_format_aes_32bit    // 32-bit AES3
} aaf_format_t;

/* AAF PCM sample rates in enumerated order  */
typedef enum {
    aaf_pcm_sample_rate_user,
    aaf_pcm_sample_rate_8k,
    aaf_pcm_sample_rate_16k,
    aaf_pcm_sample_rate_32k,
    aaf_pcm_sample_rate_44_1k,
    aaf_pcm_sample_rate_48k,
    aaf_pcm_sample_rate_88_2k,
    aaf_pcm_sample_rate_96k,
    aaf_pcm_sample_rate_176_4k,
    aaf_pcm_sample_rate_192k,
    aaf_pcm_sample_rate_24k
} aaf_pcm_sample_rate_t;

/* 61883-6 CIP SFC sample rates in enumerated order */
typedef enum {
    cip_sfc_sample_rate_32k,
    cip_sfc_sample_rate_44_1k,
    cip_sfc_sample_rate_48k,
    cip_sfc_sample_rate_88_2k,
    cip_sfc_sample_rate_96k,
    cip_sfc_sample_rate_176_4k,
    cip_sfc_sample_rate_192k
} cip_sfc_sample_rate_t;

/* Network types */

typedef uint8_t eth_addr_t[ETH_ADDR_LEN];
typedef uint8_t unique_id_t[UNIQUE_ID_LEN];
typedef struct {
    unique_id_t  id;     // entity id
    uint8_t      uid[2]; // unique id
} identity_pair_t;

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
  uint8_t attr_type;            // attribute type
  uint8_t attr_len;             // attribute length
  uint8_t vechead_padding : 5;  // padding (ignored part of num_vals)
  uint8_t vechead_leaveall : 3; // 0 or 1, if 0 then num_vals is non-zero
  uint8_t vechead_num_vals;     // # of events (div by 3, round up for # of 3pes)
} mrp_attr_header_s; // 6 bytes

/* MVRP VLAN identifier message */
typedef struct {
  uint8_t            protocol_ver;   // protocol version
  mrp_attr_header_s  header;         // attribute header
  uint8_t            vlan_id[2];     // vlan ID
  mrp_3pe_event_t    attr_event[20]; // allow up to 20 events, ignore the rest
} mvrp_vlan_id_message_s; // 23 bytes limit

/* MSRP types */

/* The data structures in this section are defined in IEEE 802.1Q-2018
 * All multi-byte fields are big-endian.
 */

/* MSRP attribute header */
typedef struct {
  uint8_t attr_type;            // attribute type
  uint8_t attr_len;             // attribute length
  uint8_t attr_list_len[2];     // attribute list length
  uint8_t vechead_padding : 5;  // padding (ignored part of num_vals)
  uint8_t vechead_leaveall : 3; // 0 or 1, if 0 then num_vals is non-zero
  uint8_t vechead_num_vals;     // # of events (div by 3, round up for # of 3pes)
} msrp_attr_header_s; // 6 bytes

/* MSRP domain message */
typedef struct {
  msrp_attr_header_s header;            // attribute header
  uint8_t            sr_class_id;       // sr class ID
  uint8_t            sr_class_priority; // sr class priority
  uint8_t            sr_class_vid[2];   // sr class VID
  mrp_3pe_event_t    attr_event[MRP_MAX_NUM_EVENTS]; // attribute events
} msrp_domain_message_s; // 25 bytes limit

/* Talker advertise information */
typedef struct {
  unique_id_t    stream_id;                   // stream ID
  eth_addr_t     stream_dest_addr;            // stream destination address
  uint8_t        vlan_id[2];                  // vlan ID
  uint8_t        tspec_max_frame_size[2];     // tspec max frame size
  uint8_t        tspec_max_frame_interval[2]; // tspec max frame interval
  uint8_t        reserved : 4;
  uint8_t        rank : 1;                    // 1 = non-emergency
  uint8_t        priority : 3;                // 3 = class A, 2 = class B
  uint8_t        accumulated_latency[4];      // as stated by the talker
} talker_adv_info_s;

/* MSRP talker advertise message */
typedef struct {
  msrp_attr_header_s header;         // attribute header
  talker_adv_info_s  info;           // talker advertise information
  mrp_3pe_event_t    event_data[MRP_MAX_NUM_EVENTS]; // attribute events
} msrp_talker_adv_message_s; // 44 bytes limit

/* MSRP talker failed message */
typedef struct {
  msrp_attr_header_s header;                 // attribute header
  talker_adv_info_s  info;                   // talker advertise information  
  uint8_t            failure_bridge_id[8];   // failure bridge ID or padded mac
  uint8_t            failure_code;           // failure code
  mrp_3pe_event_t    event_data[MRP_MAX_NUM_EVENTS]; // attribute events
} msrp_talker_failed_message_s; // 53 bytes limit

/* MSRP talker message union */
typedef union {
  msrp_attr_header_s           header;
  msrp_talker_adv_message_s    talker;
  msrp_talker_failed_message_s talker_failed;
} msrp_talker_message_u;

/* MSRP listener message */
typedef struct {
  msrp_attr_header_s header;              // attribute header
  unique_id_t        stream_id;           // stream ID
  mrp_event_u        event_decl_data[MRP_MAX_NUM_EVENTS]; // events and declarations
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
  uint8_t protocol_ver;      // protocol version
  uint8_t messages_raw[200]; // variable length depending on attributes
} msrp_msgbuf_s; // 266 bytes limit

/* AVTP types */

/* The data structures in this section are defined in IEEE 1722-2016
 * All multi-byte fields are big-endian.
 */

/* IEC 61883-6 Message */
typedef struct { 
    uint8_t     subtype;                // AVTP message subtype
    uint8_t     timestamp_valid: 1;     // source timestamp valid
    uint8_t     gateway_valid: 1;       // gateway valid
    uint8_t     reserved: 1;            // reserved
    uint8_t     media_clock_restart: 1; // media clock restart
    uint8_t     version: 3;             // AVTP version
    uint8_t     sv: 1;                  // stream id valid
    uint8_t     seq_num;                // sequence number
    uint8_t     timestamp_uncertain: 1; // avtp timestamp uncertain
    uint8_t     reserved2 : 7;          // reserved
    unique_id_t stream_id;              // stream ID
    uint8_t     avtp_ts[4];             // AVTP timestamp
    uint8_t     gateway_info[4];        // gateway info
    uint8_t     stream_data_len[2];     // stream data length
    uint8_t     channel : 6;            // IEEE 1394 channel (0-30, 32-63, 31=native AVTP)
    uint8_t     tag : 2;                // IEEE 1394 tag (00=no CIP, 01=CIP)
    uint8_t     sy : 4;                 // sy (for IIDC or DTCP)
    uint8_t     tcode : 4;              // IEEE 1394 tcode (must be 1010 on transmit)
    uint8_t     stream_data[AVTP_STREAM_DATA_PER_MSG];        // variable length (CIP header may be present)
} iec_61883_6_message_s;

/* AAF PCM Message */
typedef struct { 
    uint8_t     subtype;                // AVTP message subtype
    uint8_t     timestamp_valid: 1;     // source timestamp valid
    uint8_t     reserved1: 2;           // reserved
    uint8_t     media_clock_restart: 1; // media clock restart
    uint8_t     version: 3;             // AVTP version
    uint8_t     sv: 1;                  // stream id valid
    uint8_t     seq_num;                // sequence number
    uint8_t     timestamp_uncertain: 1; // avtp timestamp uncertain
    uint8_t     reserved2 : 7;          // reserved
    unique_id_t stream_id;              // stream ID
    uint8_t     avtp_ts[4];             // AVTP timestamp
    uint8_t     format;                 // format
    uint8_t     padding : 2;            // ignored part of channels per frame
    uint8_t     reserved3 : 2;          // reserved
    uint8_t     sample_rate : 4;        // nominal sample rate
    uint8_t     chan_per_frame;         // channels per frame; using 1 byte for convenience
    uint8_t     bit_depth;              // cannot be larger than what is specified in format
    uint8_t     stream_data_len[2];     // stream data length
    uint8_t     evt : 4;                // upper-level event
    uint8_t     sparse_ts : 1;          // sparse timestamp
    uint8_t     reserved4 : 3;          // reserved
    uint8_t     reserved5;              // reserved
    uint8_t     stream_data[AVTP_STREAM_DATA_PER_MSG];        // variable length
} aaf_pcm_message_s;

/* MAAP message */
typedef struct {
    uint8_t     subtype;            // AVTP message subtype
    uint8_t     msg_type: 4;        // MAAP message type
    uint8_t     version: 3;         // AVTP version
    uint8_t     sv: 1;              // always 0 for MAAP messages
    uint8_t     padding : 3;        // ignored part of control data length
    uint8_t     maap_version : 5;   // maap version
    uint8_t     control_data_len;   // control data length (limited to 1 byte for convenience)
    unique_id_t stream_id;          // stream ID
    eth_addr_t  req_start_addr;     // requested start address
    uint8_t     req_count[2];       // requested count
    eth_addr_t  confl_start_addr;   // conflict start address
    uint8_t     confl_count[2];     // conflict count
} maap_message_s; // 28 bytes

/* ATDECC types*/

/* The data structures in this section are defined in IEEE 1722.1-2021
 * All multi-byte fields are big-endian.
 */

/* AVB Entity Capabilities */
typedef struct {
  uint8_t multiple_ptp_instances : 1;           // The Entity has multiple PTP Instances using an interface.
  uint8_t aem_config_index_valid : 1;           // The current_configuration_index field contains a valid index of an AEM CONFIGURATION descriptor for the current Configuration.
                                                // This flag shall only be set if the AEM_SUPPORTED flag is set.
  uint8_t reserved : 6;                         // Reserved for future use
  uint8_t general_controller_ignore : 1;        // General purpose ATDECC Controllers ignore the presence of this ATDECC Entity when this flag is set.
  uint8_t entity_not_ready : 1;                 // The ATDECC Entity is not ready to be enumerated or connected by an ATDECC Controller.
  uint8_t acmp_acquire_with_aem : 1;            // ACMP respects any acquisition made with the ACQUIRE_ENTITY command.
  uint8_t acmp_auth_with_aem : 1;               // ACMP requires that the ATDECC Controller authenticate using the AEM AUTHENTICATE command.
  uint8_t supports_udpv4_atdecc : 1;            // The Entity supports ATDECC via AVTP over UDP using IPv4.
  uint8_t supports_udpv4_streaming : 1;         // The Entity supports streaming via AVTP over UDP using IPv4.
  uint8_t supports_udpv6_atdecc : 1;            // The Entity supports ATDECC via AVTP over UDP using IPv6.
  uint8_t supports_udpv6_streaming : 1;         // The Entity supports streaming via AVTP over UDP using IPv6.
  uint8_t class_a : 1;                          // Supports sending and/or receiving Class A streams.
  uint8_t class_b : 1;                          // Supports sending and/or receiving Class B streams.
  uint8_t gptp_supported : 1;                   // The ATDECC Entity implements IEEE Std 802.1AS-2020.
  uint8_t aem_auth_supported : 1;               // Supports using AEM Authentication via the AUTHENTICATE command as defined in 7.4.66. This flag shall only be set if the AEM_SUPPORTED flag is set.
  uint8_t aem_auth_required : 1;                // Requires the use of AEM Authentication via the AUTHENTICATE command as defined in 7.4.66. This flag shall only be set if the AEM_SUPPORTED flag is set.
  uint8_t aem_persistent_acquire_supported : 1; // Supports the use of the PERSISTENT flag in the ACQUIRE command as defined in 7.4.1. This flag shall only be set if the AEM_SUPPORTED flag is set.
  uint8_t aem_identify_control_index_valid : 1; // The identify_control_index field contains a valid index of an AEM CONTROL descriptor for the primary IDENTIFY control in the current Configuration. 
                                                // This flag shall only be set if the AEM_SUPPORTED flag is set.
  uint8_t aem_interface_index_valid : 1;        // The interface_index field contains a valid index of an AEM AVB_INTERFACE descriptor for interface in the current Configuration which is transmitting the ADPDU. 
                                                // This flag shall only be set if the AEM_SUPPORTED flag is set.
  uint8_t efu_mode : 1;                         // Entity Firmware Upgrade mode is enabled on the ATDECC Entity. When this flag is set, the ATDECC Entity is in the mode to perform an ATDECC Entity firmware upgrade.
  uint8_t address_access_supported : 1;         // Supports receiving the ADDRESS_ACCESS commands as definedin 1722.1-2021 section 9.4.
  uint8_t gateway_entity : 1;                   // ATDECC Entity serves as a gateway to a device on another typeof media (typically a IEEE Std 1394 device) by proxying control services for it
  uint8_t aem_supported : 1;                    // Supports receiving the ATDECC Entity Model (AEM) AECP commands as defined in 9.3.
  uint8_t legacy_avc : 1;                       // Supports using IEEE Std 1394 AV/C protocol (For example, for a IEEE Std 1394 device through a gateway).
  uint8_t assoc_id_supported : 1;               // The ATDECC Entity supports the use of the association_id field for associating the ATDECC Entity with other ATDECC entities.
  uint8_t assoc_id_valid : 1;                   // The association_id field contains a valid value. This bit shall only be set in conjunction with ASSOCIATION_ID_SUPPORTED.
  uint8_t vendor_unique_supported : 1;          // Supports receiving the AEM VENDOR_UNIQUE commands as defined in 9.5.3.
} avb_entity_cap_s; // 4 bytes

/* AVB Talker Capabilities */
typedef struct {
  uint8_t reserved1 : 1;
  uint8_t other_source : 1;
  uint8_t control_source : 1;
  uint8_t media_clock_source : 1;
  uint8_t smpte_source : 1;
  uint8_t midi_source : 1;
  uint8_t audio_source : 1;
  uint8_t video_source : 1;
  uint8_t implemented : 1;
  uint8_t reserved2 : 7;
} avb_talker_cap_s; // 2 bytes

/* AVB Listener Capabilities */
typedef struct {
  uint8_t reserved1 : 1;
  uint8_t other_sink : 1;
  uint8_t control_sink : 1;
  uint8_t media_clock_sink : 1;
  uint8_t smpte_sink : 1;
  uint8_t midi_sink : 1;
  uint8_t audio_sink : 1;
  uint8_t video_sink : 1;
  uint8_t implemented : 1;
  uint8_t reserved2 : 7;
} avb_listener_cap_s; // 2 bytes

/* AVB Controller Capabilities */
typedef struct {
  uint8_t reserved1[3];
  uint8_t implemented : 1;
  uint8_t layer3_proxy : 1;
  uint8_t reserved2 : 6;
} avb_controller_cap_s; // 4 bytes

/* AVB Entity Summary 
 * used in entity available message and entity descriptor
 */
typedef struct {
  unique_id_t          entity_id;
  unique_id_t          model_id;
  avb_entity_cap_s     entity_capabilities;
  uint8_t              talker_stream_sources[2];
  avb_talker_cap_s     talker_capabilities;
  uint8_t              listener_stream_sinks[2];
  avb_listener_cap_s   listener_capabilities;
  avb_controller_cap_s controller_capabilities;
  uint8_t              available_index[4];
} avb_entity_summary_s; // 40 bytes

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
} avb_entity_detail_s; // 272 bytes

/* ATDECC header */
typedef struct {  
  uint8_t subtype;                // AVTP message subtype
  uint8_t msg_type: 4;            // ATDECC message type  
  uint8_t version: 3;             // AVTP version
  uint8_t sv: 1;                  // always 0 for ADP messages
  uint8_t control_data_len_h : 3; // 3 high order bits of control data length
  uint8_t status_valtime : 5;     // status or valid time in 2sec increments
  uint8_t control_data_len;       // control data length (8 low order bits)
} atdecc_header_s;                // 4 bytes

/* ADP message */
typedef struct {
  atdecc_header_s      header;
  avb_entity_summary_s entity;                   // avb entity summary
  unique_id_t          gptp_gm_id;               // gptp grand master ID
  uint8_t              gptp_domain_num;          // gptp domain number
  uint8_t              reserved1;                // reserved
  uint8_t              current_config_index[2];  // current configuration index
  uint8_t              identify_control_index[2];// identify control index
  uint8_t              interface_index[2];       // interface index
  unique_id_t          association_id;           // association ID
  uint8_t              reserved2[4];             // reserved
} adp_message_s; 

/* AECP common data */
typedef struct {
  atdecc_header_s  header;
  unique_id_t      target_entity_id;     // target entity ID
  unique_id_t      controller_entity_id; // controller entity ID
  uint8_t          seq_id[2];            // sequence ID
} aecp_common_s; // 22 bytes

/* AECP address access command and response */
typedef struct {
  aecp_common_s  common;
  uint8_t        tlv_count[2];
  uint8_t        tlv_data[AVB_MAX_MSG_LEN];
} aecp_addr_access_s; // 624 bytes

/* AECP Milan vendor unique command */
typedef struct {
  aecp_common_s  common;
  uint8_t        protocol_id[6];      // protocol ID
  uint8_t        reserved1 : 1;
  uint8_t        command_type_h : 7;  // command type high order bits
  uint8_t        command_type;        // command type
  uint8_t        reserved2[2]; 
} aecp_mvu_s; // 32 bytes

/* AECP Milan vendor unique response */
typedef struct {
  aecp_common_s  common;
  uint8_t        protocol_id[6];      // protocol ID
  uint8_t        reserved : 1;
  uint8_t        command_type_h : 7;  // command type high order bits
  uint8_t        command_type;        // command type
  uint8_t        reserved2[2];
  uint8_t        protocol_version[4]; // protocol version = 1
  uint8_t        features_flags[4];   // only one flag (lsb): 1 = supports Milan redundancy
  uint8_t        certification_verison[4]; // The certification_version field shall be set to the version number of 
                                           // the Milan certification that the PAAD-AE has passed. It is composed of 
                                           // four 8-bit numbers which, when read from MSB to LSB and noted as dot-separated 
                                           // decimal numbers, are the human-readable representation of the version. This 
                                           // field is set to 0 if the PAAD-AE has not passed any Milan
} aecp_mvu_rsp_s; // 44 bytes

/* AECP AEM common data */
typedef struct {
  uint8_t  cr : 1;               // request for controller to perform an action
  uint8_t  unsolicited : 1;      // unsolicited notification
  uint8_t  command_type_h : 6;   // high order bits of command type
  uint8_t  command_type;         // command type
} aecp_common_aem_s; // 2 bytes

/* AECP AEM basic command */
// used for some commands
typedef struct {
  aecp_common_s     common;
  aecp_common_aem_s aem;        
} aecp_aem_basic_s; 

/* AECP AEM short command */
typedef struct {
  aecp_common_s     common;
  aecp_common_aem_s aem;
  uint8_t           descriptor_type[2];  // descriptor type
  uint8_t           descriptor_index[2]; // descriptor index
} aecp_aem_short_s; // 28 bytes

/* AECP acquire entity command and response */
typedef struct {
  aecp_common_s     common;
  aecp_common_aem_s aem;
  uint8_t           reserved1 : 7 ;      // reserved
  uint8_t           release : 1;         // release the acquired entity
  uint8_t           reserved2[2];        // reserved
  uint8_t           persistent : 1;      // Acquire the ATDECC Entity and disable the CONTROLLER_AVAILABLE
                                         // test for future ACQUIRE_ENTITY commands until released.
                                         // The ATDECC Entity returns an ENTITY_ACQUIRED response immediately
                                         // to any other Controller.
  uint8_t           reserved3 : 7;       // reserved
  unique_id_t       owner_id;            // 0 for command, owner id for response
  uint8_t           descriptor_type[2];  // descriptor type being acquired
  uint8_t           descriptor_index[2]; // descriptor index being acquired
} aecp_acquire_entity_s; // 40 bytes

/* AECP lock entity command and response */
typedef struct {
  aecp_common_s     common;
  aecp_common_aem_s aem;
  uint8_t           reserved1[3];        // reserved
  uint8_t           unlock : 1;          // Unlock the entity
  uint8_t           reserved2 : 7;       // reserved
  unique_id_t       locked_id;           // 0 for command, id of locked entity for response
  uint8_t           descriptor_type[2];  // descriptor type being locked
  uint8_t           descriptor_index[2]; // descriptor index being locked
} aecp_lock_entity_s; // 40 bytes

/* AECP entity available command
 * uses common AEM basic command format
 */
typedef aecp_aem_basic_s aecp_entity_available_s; // 24 bytes

/* AECP entity available response */
typedef struct {
  aecp_common_s     common;
  aecp_common_aem_s aem;
  uint8_t       reserved1[3];              // reserved
  uint8_t       entity_acquired : 1;       // Entity is acquired
  uint8_t       entity_locked : 1;         // Entity is locked
  uint8_t       subentity_acquired : 1;    // subentity is acquired
  uint8_t       subentity_locked : 1;      // subentity is locked
  uint8_t       reserved2 : 4;             // reserved
  unique_id_t   acquired_controller_id;    // acquired controller id
  unique_id_t   locked_controller_id;      // locked controller id
} aecp_entity_available_rsp_s; // 44 bytes

/* AECP controller available command and response
 * both use common format with no command specific data
 */
typedef aecp_aem_basic_s aecp_controller_available_s; // 24 bytes

/* AECP read descriptor command */
typedef struct {
  aecp_common_s     common;
  aecp_common_aem_s aem;
  uint8_t      configuration_index[2]; // configuration index
  uint8_t      reserved[2];            // reserved
  uint8_t      descriptor_type[2];     // descriptor type
  uint8_t      descriptor_index[2];    // descriptor index
} aecp_read_descriptor_s; // 32 bytes

/* AECP read descriptor response */
typedef struct {
  aecp_common_s     common;
  aecp_common_aem_s aem;
  uint8_t      configuration_index[2];   // configuration index
  uint8_t      reserved[2];              // reserved
  uint8_t      descriptor_type[2];
  uint8_t      descriptor_index[2];
  uint8_t      descriptor_data[AEM_MAX_DESC_LEN]; // descriptor data; variable length
} aecp_read_descriptor_rsp_s; // 536 bytes

/* AECP get configuration command 
 * uses common format with no command specific data
 */
typedef aecp_aem_basic_s aecp_get_configuration_s; // 24 bytes

/* AECP get configuration response */
typedef struct {
  aecp_common_s     common;
  aecp_common_aem_s aem;
  uint8_t      reserved[2];              // reserved
  uint8_t      configuration_index[2];   // configuration index
} aecp_get_configuration_rsp_s; // 28 bytes

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
  uint8_t supports_avtp_udp_v4 : 1;        // Indicates that the Stream supports streaming using AVTP over UDP/IPv4 (1722-2016 Annex J).
  uint8_t supports_avtp_udp_v6 : 1;        // Indicates that the Stream supports streaming using AVTP over UDP/IPv6 (1722-2016 Annex J).
  uint8_t no_support_avtp_native : 1;      // Indicates that the Stream does not support streaming with native (L2, Ethertype 0x22f0) AVTPDUs.
  uint8_t timing_field_valid : 1;          // Indicates that the timing field contains a valid TIMING descriptor index
  uint8_t no_media_clock : 1;              // Indicates that the stream does not use a media clock and so the clock_domain_index field does not contain a valid index.
  uint8_t supports_no_srp : 1;             // Indicates that the Stream supports streaming without an SRP preservation.
} aem_stream_flags_s; // 2 bytes

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
} aem_stream_info_flags_s; // 4 bytes

/* IEC 61883-6 AM824 stream format
 * used in stream summary and stream descriptor
 */
typedef struct {
  uint8_t subtype : 7;             // 0 for 61883
  uint8_t vendor_defined : 1;      // 0 for AVTP standard, 1 for vendor or ATDECC defined
  uint8_t reserved1 : 1;           // 
  uint8_t format : 6;              // 61883 format ID; 0x10 for IEC 61883-6 (1722 Clause I.2.2.3)
  uint8_t sf : 1;                  // 1=61883, 0=IIDC
  uint8_t fdf_sfc : 3;             // See Clause 9.1
  uint8_t fdf_evt : 5;             // 00000=AM824, 00100=32-bit float, 00110= 32-bit fixed-point
  uint8_t dbs;                     // num of data blocks, same as sum of all label_ fields (Clause 9.2)
  uint8_t reserved2 : 4;           // 
  uint8_t sc : 1;                  // packetization clock and media clock are synchronous; should be 0 (async)
  uint8_t ut : 1;                  // can source or sink a stream with less than num of data blocks indicated in dbs
  uint8_t nb : 1;                  // supports non-blocking mode (Annex A)
  uint8_t b : 1;                   // supports blocking mode (Annex A)
  uint8_t label_iec_60958_cnt;     // count of IEC 60958 quadlets (see Clause 8.2.2)
  uint8_t label_mbla_cnt;          // count of multi-bit linear audio quadlets (see Clause 8.2.3)
  uint8_t label_smptecnt : 4;      // count of SMPTE quadlets (see Clause 8.2.6)
  uint8_t label_midi_cnt : 4;      // count of MIDI quadlets (see Clause 8.2.5)
} avtp_stream_format_am824_s; // 8 bytes

/* AAF PCM stream format
 * used in stream summary and stream descriptor
 */
typedef struct {
  uint8_t subtype : 7;             // 0x02 for AAF
  uint8_t vendor_defined : 1;      // 0 for AVTP standard, 1 for vendor or ATDECC defined
  uint8_t sample_rate : 4;         // nominal base freq; same as sample_rate in stream message
  uint8_t ut : 1;                  // capable of handling less than channels_per_frame amount
  uint8_t reserved1 : 3;
  uint8_t format;                  // the AAF format value; 0x02 for 32-bit integer PCM
  uint8_t bit_depth;               // num bits in each sample; same as bit_depth in stream message
  uint8_t chan_per_frame_h;        // hight 8 bits of channels_per_frame
  uint8_t samples_per_frame_h : 6; // high 6 bits of samples_per_frame
  uint8_t chan_per_frame : 2;      // num channels in each frame; same as in stream message
  uint8_t reserved2 : 4;
  uint8_t samples_per_frame : 4;   // num samples per channel per frame
  uint8_t reserved3;
} avtp_stream_format_aaf_pcm_s; // 8 bytes

/* AVTP Stream Format */
typedef union {
  uint8_t                      subtype : 7;
  uint8_t                      vendor_defined : 1;
  avtp_stream_format_am824_s   am824;
  avtp_stream_format_aaf_pcm_s aaf_pcm;
} avtp_stream_format_s; // 64 bytes

/* Stream descriptor (input or output) */
typedef struct {
  uint8_t              object_name[64];                 // UTF8 string containing a Stream name.
  uint8_t              localized_description[2];        // The localized string reference pointing to the localized Stream name. See 7.3.7.
  uint8_t              clock_domain_index[2];           // The descriptor_index of the Clock Domain providing the media clock for the Stream. See 7.2.9.
  aem_stream_flags_s   stream_flags;                    // Flags describing capabilities or features of the Stream. See Table 79.
  avtp_stream_format_s current_format;                  // The Stream format of the current format, as defined in 7.3.3.
  uint8_t              formats_offset[2];               // The offset from the start of the descriptor for the first octet of the formats. This field is 138 for this version of AEM.
  uint8_t              number_of_formats[2];            // The number of formats supported by this audio Stream. The value of this field is referred to as N. The maximum value for this field is 46 for this version of AEM.
  unique_id_t          backup_talker_entity_id_0;       // The primary backup ATDECCTalker's EntityID.
  uint8_t              backup_talker_unique_id_0[2];    // The primary backup ATDECCTalker's UniqueID.
  unique_id_t          backup_talker_entity_id_1;       // The secondary backup ATDECCTalker's EntityID.
  uint8_t              backup_talker_unique_id_1[2];    // The secondary backup ATDECCTalker's UniqueID.
  unique_id_t          backup_talker_entity_id_2;       // The tertiary backup ATDECCTalker's EntityID.
  uint8_t              backup_talker_unique_id_2[2];    // The tertiary backup ATDECCTalker's UniqueID.
  unique_id_t          backedup_talker_entity_id;       // The EntityID of the ATDECCTalker that this Stream is backing up.
  uint8_t              backedup_talker_unique_id[2];    // The UniqueID of the ATDECCTalker that this Stream is backing up.
  uint8_t              avb_interface_index[2];          // The descriptor_index of the AVB_INTERFACE from which this Stream is sourced or to which it is sinked.
  uint8_t              buffer_length[4];                // The length in nanoseconds of the MAC's ingress or egress buffer as defined in IEEE Std1722-2016 Figure5.4. For a STREAM_INPUT this is the MAC's ingress buffer size and for a STREAM_OUTPUT this is the MAC's egress buffer size. This is the length of the buffer between the IEEE Std1722-2016 reference plane and the MAC.
  uint8_t              redundant_offset[2];             // The offset from the start of the descriptor for the first octet of the redundant_streams array. This field is 138+8*N for this version of AEM.
  uint8_t              number_of_redundant_streams[2];  // The number of redundant streams supported by this audio Stream. The value of this field is referred to as R. The maximum value for this field is 8 for this version of AEM.
  uint8_t              timing[2];                       // The TIMING descriptor index which represents the source of gPTP time for the stream.
  avtp_stream_format_s formats[AEM_MAX_NUM_FORMATS];    // Array of Stream formats of the supported formats, as defined in 7.3.3.
  //uint8_t redundant_streams[2*R];                     // NOT YET SUPPORTED. Array of redundant STREAM_INPUT or STREAM_OUTPUT descriptor indices. The current version of AEM doesn’t specify an ordering for the elements of this array.
} aem_stream_desc_s; 

/* AEM stream summary 
 * used in get stream info response and talker/listener list
 */
typedef struct {
  aem_stream_info_flags_s flags;                       // stream descriptor flags
  avtp_stream_format_s    stream_format;               // stream format
  unique_id_t             stream_id;                   // stream ID
  uint8_t                 msrp_accumulated_latency[4]; // MSRP accumulated latency
  eth_addr_t              dest_addr;                   // stream destination MAC address
  uint8_t                 msrp_failure_code;           // MSRP failure code
} aem_stream_summary_s; // 17 bytes

/* AECP get stream info command */
typedef aecp_aem_short_s aecp_get_stream_info_s; // 28 bytes

/* AECP set stream info command  
 * also used for get/set stream info response
 */
typedef struct {
  aecp_common_s        common;
  aecp_common_aem_s    aem;
  uint8_t              descriptor_type[2];          // descriptor type
  uint8_t              descriptor_index[2];         // descriptor index
  aem_stream_summary_s stream;                      // stream summary
  uint8_t              reserved;
  uint8_t              msrp_failure_bridge_id[8];   // MSRP failure bridge ID
  uint8_t              vlan_id[2];                  // stream VLAN ID
  uint8_t              ip_flags[2];                 // stream destination port
  uint8_t              src_port[2];                 // stream source port
  uint8_t              dest_port[2];                // stream destination port
  uint8_t              src_ip_addr[16];             // stream source IP address
  uint8_t              dest_ip_addr[16];            // stream destination IP address
} aecp_set_stream_info_s; // 108 bytes

/* AECP get stream info response */
// same as set stream info command
typedef aecp_set_stream_info_s aecp_get_stream_info_rsp_s;

/* AECP set stream info response */
// same as set stream info command
typedef aecp_set_stream_info_s aecp_set_stream_info_rsp_s;

/* AEM Entity counters valid flags */
typedef struct {
  uint8_t     entity_specific8 : 1;
  uint8_t     entity_specific7 : 1;
  uint8_t     entity_specific6 : 1;
  uint8_t     entity_specific5 : 1;
  uint8_t     entity_specific4 : 1;
  uint8_t     entity_specific3 : 1;
  uint8_t     entity_specific2 : 1;
  uint8_t     entity_specific1 : 1;
  uint8_t     reserved[3];
} aem_entity_counters_val_s; // 4 bytes

/* AEM Stream input counters valid flags */
typedef struct {
  uint8_t     entity_specific8 : 1;
  uint8_t     entity_specific7 : 1;
  uint8_t     entity_specific6 : 1;
  uint8_t     entity_specific5 : 1;
  uint8_t     entity_specific4 : 1;
  uint8_t     entity_specific3 : 1;
  uint8_t     entity_specific2 : 1;
  uint8_t     entity_specific1 : 1;
  uint8_t     reserved1;
  uint8_t     unsupported_format : 1;
  uint8_t     late_ts : 1;
  uint8_t     early_ts : 1;
  uint8_t     frames_rx : 1;
  uint8_t     reserved2 : 4;
  uint8_t     media_locked : 1;
  uint8_t     media_unlocked : 1;
  uint8_t     stream_interrupted : 1;
  uint8_t     seq_num_mismatch : 1;
  uint8_t     media_reset : 1;
  uint8_t     ts_uncertain : 1;
  uint8_t     ts_valid : 1;
  uint8_t     ts_not_valid : 1;
} aem_stream_in_counters_val_s; // 4 bytes

/* AEM Stream output counters valid flags */
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
} aem_stream_out_counters_val_s; // 4 bytes

/* AEM AVB interface counters valid flags */
typedef struct {
    uint8_t     entity_specific8 : 1;
    uint8_t     entity_specific7 : 1;
    uint8_t     entity_specific6 : 1;
    uint8_t     entity_specific5 : 1;
    uint8_t     entity_specific4 : 1;
    uint8_t     entity_specific3 : 1;
    uint8_t     entity_specific2 : 1;
    uint8_t     entity_specific1 : 1;
    uint8_t     reserved1[2];
    uint8_t     link_up : 1;
    uint8_t     link_down : 1;
    uint8_t     frames_tx : 1;
    uint8_t     frames_rx : 1;
    uint8_t     rx_crc_error : 1;
    uint8_t     gptp_gm_changed : 1;
    uint8_t     reserved2 : 2;
} aem_avb_interface_counters_val_s; // 4 bytes

/* AEM clock domain counters valid flags */
typedef struct {
    uint8_t     entity_specific8 : 1;
    uint8_t     entity_specific7 : 1;
    uint8_t     entity_specific6 : 1;
    uint8_t     entity_specific5 : 1;
    uint8_t     entity_specific4 : 1;
    uint8_t     entity_specific3 : 1;
    uint8_t     entity_specific2 : 1;
    uint8_t     entity_specific1 : 1;
    uint8_t     reserved1[2];
    uint8_t     locked : 1;
    uint8_t     unlocked : 1;
    uint8_t     reserved2 : 6;
} aem_clock_domain_counters_val_s; // 4 bytes

/* AEM entity counters block */
typedef struct {
  uint8_t     reserved[96];
  uint8_t     entity_specific8[4];
  uint8_t     entity_specific7[4];
  uint8_t     entity_specific6[4];
  uint8_t     entity_specific5[4];
  uint8_t     entity_specific4[4];
  uint8_t     entity_specific3[4];
  uint8_t     entity_specific2[4];
  uint8_t     entity_specific1[4];
} aem_entity_counters_s; // 128 bytes

/* AEM Stream input counters block */
typedef struct {
  uint8_t     media_locked[4];         // Increments on a Stream media clock locking.
  uint8_t     media_unlocked[4];       // Increments on a Stream media clock unlocking.
  uint8_t     stream_interrupted[4];   // Increments when Stream playback is interrupted.
  uint8_t     seq_num_mismatch[4];     // Increments when a Stream data AVTPDU is received with a non-sequential sequence_num field.
  uint8_t     media_reset[4];          // Increments on a toggle of the mr bit in the Stream data AVTPDU.
  uint8_t     ts_uncertain[4];         // Increments on a toggle of the tu bit in the Stream data AVTPDU.
  uint8_t     ts_valid[4];             // Increments on receipt of a Stream data AVTPDU with the tv bit set.
  uint8_t     ts_not_valid[4];         // Increments on receipt of a Stream data AVTPDU with the tv bit cleared.
  uint8_t     unsupported_format[4];   // Increments on receipt of a Stream data AVTPDU that contains an unsupported media type.
  uint8_t     late_ts[4];              // Increments on receipt of a Stream data AVTPDU with an avtp_timestamp field that is in the past.
  uint8_t     early_ts[4];             // Increments on receipt of a Stream data AVTPDU with an avtp_timestamp field that is too far in the future to process.
  uint8_t     frames_rx[4];            // Increments on each Stream data AVTPDU received.
  uint8_t     reserved[48];            // Reserved for future use
  uint8_t     entity_specific8[4];
  uint8_t     entity_specific7[4];
  uint8_t     entity_specific6[4];
  uint8_t     entity_specific5[4];
  uint8_t     entity_specific4[4];
  uint8_t     entity_specific3[4];
  uint8_t     entity_specific2[4];
  uint8_t     entity_specific1[4];
} aem_stream_in_counters_s; // 128 bytes

/* AEM Stream output counters block */
typedef struct {
  uint8_t     stream_start[4];       // Increments when a stream is started.
  uint8_t     stream_stop[4];        // Increments when a stream is stopped.
  uint8_t     stream_interrupted[4]; // Increments when Stream playback is interrupted.
  uint8_t     media_reset[4];        // Increments on a toggle of the mr bit in the Stream data AVTPDU.
  uint8_t     ts_uncertain[4];       // Increments on a toggle of the tu bit in the Stream data AVTPDU.
  uint8_t     ts_valid[4];           // Increments on receipt of a Stream data AVTPDU with the tv bit set.
  uint8_t     ts_not_valid[4];       // Increments on receipt of a Stream data AVTPDU with the tv bit cleared.
  uint8_t     frames_tx[4];          // Increments on each Stream data AVTPDU transmitted.
  uint8_t     reserved[64];          // Reserved for future use
  uint8_t     entity_specific8[4];
  uint8_t     entity_specific7[4];
  uint8_t     entity_specific6[4];
  uint8_t     entity_specific5[4];
  uint8_t     entity_specific4[4];
  uint8_t     entity_specific3[4];
  uint8_t     entity_specific2[4];
  uint8_t     entity_specific1[4];
} aem_stream_out_counters_s; // 128 bytes

/* AEM AVB interface counters block */
typedef struct {
    uint8_t     link_up[4];          // Total number of network link up events.
    uint8_t     link_down[4];        // Total number of network link down events.
    uint8_t     frames_tx[4];        // Total number of network frames transmitted. 
    uint8_t     frames_rx[4];        // Total number of network frames received.
    uint8_t     rx_crc_error[4];     // Total number of network frames received with incorrect CRC.
    uint8_t     gptp_gm_changed[4];  // GPTP GM change count.
    uint8_t     reserved[72];        // Reserved for future use
    uint8_t     entity_specific8[4];
    uint8_t     entity_specific7[4];
    uint8_t     entity_specific6[4];
    uint8_t     entity_specific5[4];
    uint8_t     entity_specific4[4];
    uint8_t     entity_specific3[4];
    uint8_t     entity_specific2[4];
    uint8_t     entity_specific1[4];
} aem_avb_interface_counters_s; // 128 bytes

/* AEM clock domain counters block */
typedef struct {
    uint8_t     locked[4];          // Increments on a clock locking event.
    uint8_t     unlocked[4];        // Increments on a clock unlocking event.
    uint8_t     reserved[88];       // Reserved for future use
    uint8_t     entity_specific8[4];
    uint8_t     entity_specific7[4];
    uint8_t     entity_specific6[4];
    uint8_t     entity_specific5[4];
    uint8_t     entity_specific4[4];
    uint8_t     entity_specific3[4];
    uint8_t     entity_specific2[4];
    uint8_t     entity_specific1[4];
} aem_clock_domain_counters_s; // 128 bytes

/* AEM counters valid flags union */
typedef union {
  aem_entity_counters_val_s        entity_counters_val;
  aem_stream_in_counters_val_s     stream_in_counters_val;
  aem_stream_out_counters_val_s    stream_out_counters_val;
  aem_avb_interface_counters_val_s avb_interface_counters_val;
  aem_clock_domain_counters_val_s  clock_domain_counters_val;
} aem_counters_val_u; // 4 bytes

/* AEM counters block union */
typedef union {
  aem_entity_counters_s        entity_counters;
  aem_stream_in_counters_s     stream_in_counters;
  aem_stream_out_counters_s    stream_out_counters;
  aem_avb_interface_counters_s avb_interface_counters;
  aem_clock_domain_counters_s  clock_domain_counters;
} aem_counters_block_u; // 128 bytes

/* AECP get counters command uses basic command format */
typedef aecp_aem_short_s aecp_get_counters_s; // 28 bytes

/* AECP get counters response */
typedef struct {
  aecp_common_s     common;
  aecp_common_aem_s aem;
  uint8_t               descriptor_type[2];  // descriptor type
  uint8_t               descriptor_index[2]; // descriptor index
  aem_counters_val_u    counters_valid;      // counters valid
  aem_counters_block_u  counters_block;      // counters block
} aecp_get_counters_rsp_s; // 160 bytes

/* AECP register unsol flags */
typedef struct {
  uint8_t  reserved1[3];
  uint8_t  time_limited : 1;
  uint8_t  reserved2 : 7;
} aecp_register_unsol_flags_s; // 4 bytes

/* AECP register unsolicited notification command and response */

typedef struct {
  aecp_common_s      common;
  aecp_common_aem_s  aem;
  aecp_register_unsol_flags_s  flags;
} aecp_register_unsol_notif_s; // 28 bytes

/* AECP deregister unsolicited notification command and response 
 * uses common format with no command specific data
 */
typedef aecp_aem_basic_s aecp_deregister_unsol_notif_s; // 24 bytes

/* AECP message union */
typedef union {
    atdecc_header_s               header;
    aecp_aem_basic_s              basic;
    aecp_common_s                 common;
    aecp_acquire_entity_s         acquire_entity;
    aecp_lock_entity_s            lock_entity;
    aecp_entity_available_s       entity_available;
    aecp_controller_available_s   controller_available;
    aecp_get_configuration_s      get_configuration;
    aecp_read_descriptor_s        read_descriptor;
    aecp_get_stream_info_s        get_stream_info;
    aecp_get_stream_info_rsp_s    get_stream_info_rsp;
    aecp_set_stream_info_s        set_stream_info;     // not supporte
    aecp_set_stream_info_rsp_s    set_stream_info_rsp; // not supported
    aecp_get_counters_s           get_counters;
    aecp_get_counters_rsp_s       get_counters_rsp;
    aecp_register_unsol_notif_s   register_unsol_notif;
    aecp_deregister_unsol_notif_s deregister_unsol_notif;
    aecp_addr_access_s            addr_access;
    aecp_mvu_s                    mvu;
    uint8_t                       raw[AVB_MAX_MSG_LEN];
} aecp_message_u;

/* AEM configuration descriptor counts */
typedef struct {
    uint8_t  descriptor_type[2];  
    uint8_t  count[2];
} aem_config_desc_count_s; // 4 bytes

/* AEM descriptors */

/* AEM Entity descriptor */
typedef struct {
    avb_entity_summary_s summary;
    avb_entity_detail_s  detail;
} aem_entity_desc_s; // 312 bytes

/* AEM Configuration descriptor */
typedef struct {
    uint8_t                 object_name[64];             // 64-octet UTF8 string containing a Configuration name.
    uint8_t                 localized_description[2];    // The localized string reference pointing to the localized Configuration name. See 7.3.7.
    uint8_t                 descriptor_counts_count[2];  // The number of descriptor counts in the descriptor_countsfield. This is referred to as N.The maximum value for this field is 108 for this version of AEM.
    uint8_t                 descriptor_counts_offset[2]; // The offset to the descriptor_counts field from the start of the descriptor. This field is set to 74 for this version of AEM.
    aem_config_desc_count_s descriptor_counts[AEM_MAX_NUM_DESC];    // Counts of the top-level descriptors. See 7.2.2.1.
} aem_config_desc_s; // 114 bytes

/* AEM sample rate type */
typedef uint8_t aem_sample_rate_t[4];

/* AEM Audio Unit descriptor */
typedef struct {
    uint8_t  object_name[64];            // 64-octet UTF8 string containing a name.
    uint8_t  localized_description[2];   // Pointer to the localized name. See 7.3.7.
    uint8_t  clock_domain_index[2];      // 
    uint8_t  num_stream_input_ports[2];  // 
    uint8_t  base_stream_input_port[2];  // 
    uint8_t  num_stream_output_ports[2]; // 
    uint8_t  base_stream_output_port[2]; // 
    uint8_t  unused[56];                 // these fields not yet supported
    uint8_t  current_sampling_rate[4];   // 
    uint8_t  sampling_rate_offset[2];    // 144 for this version of AEM
    uint8_t  sampling_rates_count[2];    // 
    aem_sample_rate_t  sampling_rates[AEM_MAX_NUM_SAMPLE_RATES];
 } aem_audio_unit_desc_s; // 180 bytes

/* AEM Stream input and output descriptors are defined further above */

/* AEM AVB Interface flags (used for descriptor) */
typedef struct {
    uint8_t     reserved1;                    // Reserved for future use
    uint8_t     gptp_gm_supported : 1;        // Supports 802.1AS gPTP GM functionality
    uint8_t     gptp_supported : 1;           // Supports 802.1AS gPTP functionality
    uint8_t     srp_supported : 1;            // Supports 802.1Q clause for SRP
    uint8_t     fqtss_not_supported : 1;      // Does not support 802.1Q clause for FQTSS
    uint8_t     sched_traffic_supported : 1;  // Supports 802.1Q clauses for scheduled traffic
    uint8_t     can_listen_to_self : 1;       //  Listener stream sink on this interface can listen to a talker stream source on the sam interface.
    uint8_t     can_listen_to_other_self : 1; // Listener stream sink on this interface can listen to a talker stream source of another interface within same Entity
    uint8_t     reserved2 : 1;                // Reserved for future use
} aem_avb_interface_flags_s; // 2 bytes

/* AEM AVB Interface descriptor */
typedef struct {
    uint8_t                   object_name[64];               // 64-octet UTF8 string containing a name.
    uint8_t                   localized_description[2];      // Pointer to the localized name. See 7.3.7.
    eth_addr_t                mac_address;                   // 
    aem_avb_interface_flags_s interface_flags;               // 
    unique_id_t               clock_identity;                // 
    uint8_t                   priority1;                     // 
    uint8_t                   clock_class;                   // 
    uint8_t                   offset_scaled_log_variance[2]; // 
    uint8_t                   clock_accuracy;                // 
    uint8_t                   priority2;                     // 
    uint8_t                   domain_number;                 // 
    uint8_t                   log_sync_interval;             // 
    uint8_t                   log_announce_interval;         // 
    uint8_t                   log_pdelay_interval;           // 
    uint8_t                   port_number[2];                // 
    uint8_t                   number_of_controls[2];         // 
    uint8_t                   base_control[2];               // 
 } aem_avb_interface_desc_s; // 98 bytes

/* AEM Clock Source flags (used for descriptor) */
typedef struct {
    uint8_t     reserved1;       // Reserved for future use
    uint8_t     stream_id : 1;   // The input stream clock source is identified by the stream ID.
    uint8_t     local_id : 1;    // The input stream clock source is identified by its local ID.
    uint8_t     reserved2 : 6;   // Reserved for future use
} aem_clock_source_flags_s; // 2 bytes

/* AEM Clock Source descriptor */
typedef struct {
    uint8_t                  object_name[64];                // 64-octet UTF8 string containing a name.
    uint8_t                  localized_description[2];       // Pointer to the localized name. See 7.3.7.
    aem_clock_source_flags_s clock_source_flags;             //
    uint8_t                  clock_source_type[2];           // must be one of aem_clock_source_type_t
    unique_id_t              clock_source_id;                //
    uint8_t                  clock_source_location_type[2];  // must be one of aem_desc_type_t
    uint8_t                  clock_source_location_index[2]; //
 } aem_clock_source_desc_s; // 82 bytes

/* AEM Memory Object descriptor */
typedef struct {
    uint8_t object_name[64];             // 64-octet UTF8 string containing a name.
    uint8_t localized_description[2];    // Pointer to the localized name. See 7.3.7.
    uint8_t memory_object_type[2];       // must be one of aem_memory_obj_type_t
    uint8_t target_descriptor_type[2];   // must be one of aem_desc_type_t
    uint8_t target_descriptor_index[2];  // 
    uint8_t start_address[8];            // 
    uint8_t maximum_length[8];           // 
    uint8_t length[8];                   // 
    uint8_t maximum_segment_length[8];   // 
 } aem_memory_object_desc_s; // 104 bytes

/* AEM Locale descriptor */
typedef struct {
    uint8_t locale_identifier[64]; // 64-octet UTF8 string containing the locale identifier.
    uint8_t number_of_strings[2];  // Number of strings descriptors in this locale. This is the same value for all locales in an ATDECC Entity.
    uint8_t base_strings[2];       // Descriptor index of the first Strings descriptor for this locale.
 } aem_locale_desc_s; // 68 bytes

/* AEM Strings descriptor */
typedef struct {
    uint8_t string_0[64];        // 64-octet UTF8 string at index 0.
    uint8_t string_1[64];        // 64-octet UTF8 string at index 1.
    uint8_t string_2[64];        // 64-octet UTF8 string at index 2.
    uint8_t string_3[64];        // 64-octet UTF8 string at index 3.
    uint8_t string_4[64];        // 64-octet UTF8 string at index 4.
    uint8_t string_5[64];        // 64-octet UTF8 string at index 5.
    uint8_t string_6[64];        // 64-octet UTF8 string at index 6.
 } aem_strings_desc_s; // 448 bytes

/* AEM Stream Port flags (used for descriptor) */
typedef struct {
    uint8_t  reserved1;                  // Reserved for future use
    uint8_t  clock_sync_source : 1;      // 
    uint8_t  async_sample_rate_conv : 1; // 
    uint8_t  sync_sample_rate_conv : 1;  // 
    uint8_t  reserved2 : 5;              // Reserved for future use
} aem_stream_port_flags_s; // 2 bytes

/* AEM Stream Port descriptor (input and output) */
typedef struct {
    uint8_t                 clock_domain_index[2]; // 
    aem_stream_port_flags_s port_flags;            // 
    uint8_t                 number_of_controls[2]; //
    uint8_t                 base_control[2];       //
    uint8_t                 number_of_clusters[2]; //
    uint8_t                 base_cluster[2];       //
    uint8_t                 number_of_maps[2];     //
    uint8_t                 base_map[2];           //
 } aem_stream_port_desc_s; // 16 bytes

/* AEM Audio Cluster descriptor */
typedef struct {
    uint8_t object_name[64];             // 64-octet UTF8 string containing a name.
    uint8_t localized_description[2];    // Pointer to the localized name. See 7.3.7.
    uint8_t signal_type[2];              // must be one of aem_desc_type_t
    uint8_t signal_index[2];             //
    uint8_t signal_output[2];            //
    uint8_t path_latency[4];             //
    uint8_t block_latency[4];            //
    uint8_t channel_count[2];            //
    uint8_t format[2];                   // must be one of aem_audio_cluster_format_t
    uint8_t aes_data_type_reference[2];  //
    uint8_t aes_data_type[2];            // 
 } aem_audio_cluster_desc_s; // 86 bytes

 /* AEM Audio Mapping */
 typedef struct {
    uint8_t mapping_stream_index[2];    //
    uint8_t mapping_stream_channel[2];  //
    uint8_t mapping_cluster_offset[2];  // 
    uint8_t mapping_cluster_channel[2]; // 
 } aem_audio_mapping_s; // 8 bytes

/* AEM Audio Map descriptor */
typedef struct {
    uint8_t             mappings_offset[2];             // set to 8 for this version of AEM
    uint8_t             number_of_mappings[2];          // number of channel mappings in the descriptor (max is 62)
    aem_audio_mapping_s mappings[AEM_MAX_NUM_MAPPINGS];      
 } aem_audio_map_desc_s; // 84 bytes

/* AEM Identify Control value details */
typedef struct {
    uint8_t values[5];
    uint8_t units[2];
    uint8_t string_ref[2];
} aem_identify_control_value_s;

/* AEM Control descriptor */
typedef struct {
    uint8_t object_name[64];          // 64-octet UTF8 string containing a name.
    uint8_t localized_description[2]; // Pointer to the localized name. See 7.3.7.
    uint8_t block_latency[4];         // 
    uint8_t control_latency[4];       // 
    uint8_t control_domain[2];        // 
    uint8_t control_value_type[2];    // 
    uint8_t control_type[8];          // 
    uint8_t reset_time[4];            // 
    uint8_t values_offset[2];         // 
    uint8_t number_of_values[2];      // 
    uint8_t signal_type[2];           // 
    uint8_t signal_index[2];          // 
    uint8_t signal_output[2];         // 
    uint8_t value_details[AEM_MAX_LEN_CONTROL_VAL_DETAILS];
} aem_control_desc_s; // 68 bytes

 /* AEM Clock Source type used in Clock Domain descriptor */
 typedef uint8_t aem_clock_source_t[2];

/* AEM Clock Domain descriptor */
typedef struct {
  uint8_t                 object_name[64];            // 64-octet UTF8 string containing a name.
  uint8_t                 localized_description[2];   // Pointer to the localized name. See 7.3.7.
  uint8_t                 clock_source_index[2];      // 
  uint8_t                 clock_sources_offset[2];    // 
  uint8_t                 clock_sources_count[2];     // number of clock sources in the descriptor (max is 216)
  aem_clock_source_t      clock_sources[AEM_MAX_NUM_CLOCK_SOURCES]; // 2 bytes per clock source
 } aem_clock_domain_desc_s; // 92 bytes

/* ACMP Message */
typedef struct {
  atdecc_header_s header;
  unique_id_t     stream_id;                   // stream ID
  unique_id_t     controller_entity_id;        // controller entity ID  
  unique_id_t     talker_entity_id;            // talker entity ID
  unique_id_t     listener_entity_id;          // listener entity ID
  uint8_t         talker_uid[2];               // talker UID = stream output descr index
  uint8_t         listener_uid[2];             // listener UID = stream input descr index
  eth_addr_t      stream_dest_addr;            // stream destination address
  uint8_t         connection_count[2];         // connection count
  uint8_t         seq_id[2];                   // sequence ID
  uint8_t         flags[2];                    // flags
  uint8_t         stream_vlan_id[2];           // stream VLAN ID
  uint8_t         conn_listeners_entries[2];   // connection listeners entries
} acmp_message_s; // 56 bytes

/* ACMP Message Extended */
typedef struct {
  acmp_message_s  base;
  uint8_t         ip_flags[2];                 // IP flags
  uint8_t         reserved[2];                 // reserved
  uint8_t         src_port[2];                 // source port
  uint8_t         dest_port[2];                // destination port
  uint8_t         src_ip_addr[16];             // source IP address
  uint8_t         dest_ip_addr[16];            // destination IP address
} acmp_message_extended_s; // 96 bytes

/* AVTP message buffer */
typedef union {
  uint8_t               subtype;
  aaf_pcm_message_s     aaf;
  iec_61883_6_message_s iec;
  maap_message_s        maap;
  adp_message_s         adp;
  aecp_message_u        aecp;
  acmp_message_s        acmp;
  uint8_t               raw[AVB_MAX_MSG_LEN];
} avtp_msgbuf_u;

/* General */

/* Generic AVB message buffer */
typedef union {
  avtp_msgbuf_u          avtp;
  msrp_msgbuf_s          msrp;
  mvrp_vlan_id_message_s mvrp;
  uint8_t                raw[AVB_MAX_MSG_LEN];
} avb_msgbuf_u;

/* Talker */
typedef struct {
  unique_id_t          entity_id; // entity ID
  unique_id_t          model_id; // model ID
  eth_addr_t           mac_addr; // mac address
  talker_adv_info_s    info; // from talker advertise
  aem_stream_summary_s stream; // from get stream info
  uint8_t              talker_uid[2]; // stream output descr index
  uint8_t              failure_code; // msrp failure code
  bool                 streaming; // streaming status
  uint8_t              last_msrp_event; // last talker event
  bool                 ready; // general status
} avb_talker_s;

/* Listener */
typedef struct {
  unique_id_t stream_id; // stream ID
  uint8_t     vlan_id[2]; // vlan ID
  unique_id_t entity_id; // entity ID
  unique_id_t model_id; // model ID
  eth_addr_t  mac_addr; // mac address
  uint8_t     listener_uid[2]; // stream input descr index
  uint8_t     last_msrp_event; // last msrp event
  uint8_t     last_listener_event; // last listener event
  bool        ready; // status
} avb_listener_s;

/* Controller */
typedef struct {
  unique_id_t entity_id; // entity ID
  unique_id_t model_id; // model ID
  eth_addr_t  mac_addr; // mac address
  bool        ready; // status
} avb_controller_s;

/* Listener stream flags */
typedef struct {
  uint8_t class_b : 1;                 // connection is class B, instead of class A
  uint8_t fast_connect : 1;            // connection is using fast connect mode
  uint8_t saved_state : 1;             // connection has saved ACMP state for fast connect mode
  uint8_t streaming_wait : 1;          // for milan, this must be false
  uint8_t supports_encrypted : 1;      // stream supports encrypted PDU
  uint8_t encrypted_pdu : 1;           // stream is using encrypted PDU
  uint8_t srp_registration_failed : 1; // listener has registered an SRP talker failed attr or talker as registered an SRP listener asking failed attr
  uint8_t cl_entries_valid : 1;        // connected_listeners field is valid
  uint8_t no_srp : 1;                  // SRP not used for the stream
  uint8_t udp : 1;                     // stream using UDP transport
  uint8_t reserved : 6;                // reserved for future use
} avb_listener_stream_flags_s; // 2 bytes

/* Listener stream */
typedef struct {
  unique_id_t                 talker_id;          // talker entity ID
  uint8_t                     talker_uid[2];      // talker UID (same as stream output descr index)
  unique_id_t                 controller_id;      // controller entity ID
  unique_id_t                 stream_id;          // stream ID
  eth_addr_t                  stream_dest_addr;   // stream destination address
  avb_listener_stream_flags_s stream_flags;       // stream flags
  aem_stream_info_flags_s     stream_info_flags;  // stream info flags
  uint8_t                     vlan_id[2];         // vlan ID
  avtp_stream_format_s        stream_format;      // stream format
  uint8_t                     msrp_accumulated_latency[4]; // msrp accumulated latency
  uint8_t                     msrp_failure_code[2]; // msrp failure code
  bool                        connected;          // status as connected
  bool                        pending_connection; // status as pending connection
} avb_listener_stream_s;

/* Talker stream */
typedef struct {
  unique_id_t             stream_id;           // stream ID
  eth_addr_t              stream_dest_addr;    // stream destination address
  aem_stream_info_flags_s stream_info_flags; // stream info flags
  uint8_t                 vlan_id[2];          // vlan ID
  avtp_stream_format_s    stream_format;       // stream format
  uint8_t                 msrp_accumulated_latency[4]; // msrp accumulated latency
  uint8_t                 msrp_failure_code[2]; // msrp failure code
  uint8_t                 connection_count[2]; // number of connected listeners
  identity_pair_t         connected_listeners[AVB_MAX_NUM_CONNECTED_LISTENERS]; // list of connected listeners
} avb_talker_stream_s;

/* ATDECC command message union */
// used for inflight commands
typedef union {
  atdecc_header_s   header;
  aecp_aem_short_s  aecp;
  acmp_message_s    acmp;
  uint8_t           raw[56];
} atdecc_command_u;

/* Inflight command (state for processing an asynchronous response to a command) */
// used for AECP and ACMP command tracking
typedef struct {
  struct timeval   timeout;       // command timeout as a timeval
  bool             retried;       // indicates if the command has been retried
  uint16_t         aecp_seq_id;   // original AECP command sequence ID
  uint16_t         acmp_seq_id;   // original ACMP command sequence ID
  atdecc_command_u command;       // the command or partial command
  bool             inbound;       // indicates if the command is inbound
} atdecc_inflight_command_s;

/* Stream Input params */
struct stream_in_params_s {
  i2s_chan_handle_t i2s_tx_handle; // handle to i2s tx channel
  uint16_t buffer_size; // buffer size
  uint16_t interval; // interval in microseconds
  uint8_t l2if; // layer2 interface
  unique_id_t stream_id; // stream ID
  uint8_t bit_depth; // bit depth
  uint8_t channels; // channels
  uint8_t sample_rate; // sample rate
  uint8_t format; // aaf format
};

/* Stream Output params */
struct stream_out_params_s {
  i2s_chan_handle_t i2s_rx_handle; // handle to i2s rx channel
  uint16_t buffer_size; // buffer size
  uint16_t interval; // interval in microseconds
  uint8_t l2if; // layer2 interface
};

/* Carrier structure for querying AVB status */
typedef struct {
  sem_t *        done;
  avb_status_s * dest;
} avb_statusreq_s;

/* Main AVB state storage */
typedef struct {

  /* AVB configuration */
  avb_config_s config;

  /* Request for AVB task to stop or report status */
  bool stop;
  avb_statusreq_s        status_req;
  struct ptpd_status_s   ptp_status;

  eth_addr_t internal_mac_addr;
  esp_eth_handle_t eth_handle;
  int l2if[AVB_NUM_PROTOCOLS]; // 3 L2TAP interfaces (FDs) for AVTP, MSRP, and MVRP
  bool l2tap_receive; // receive L2TAP frames
  bool codec_enabled; // codec enabled
  
  /* Our own entity */
  aem_entity_desc_s own_entity;

  /* AVB interface */
  // only one interface is supported currently
  aem_avb_interface_desc_s avb_interface;

  /* Inflight commands */
  atdecc_inflight_command_s inflight_commands[AVB_MAX_NUM_INFLIGHT_COMMANDS];
  size_t num_inflight_commands;

  /* AVB streams */
  // index of input_streams is the listener_uid
  // index of output_streams is the talker_uid
  avb_listener_stream_s input_streams[AVB_MAX_NUM_INPUT_STREAMS];
  avb_talker_stream_s output_streams[AVB_MAX_NUM_OUTPUT_STREAMS];
  size_t num_input_streams;
  size_t num_output_streams;

  /* Endpoints that we are aware of */
  avb_talker_s talkers[AVB_MAX_NUM_TALKERS];
  avb_listener_s listeners[AVB_MAX_NUM_LISTENERS];
  avb_controller_s controllers[AVB_MAX_NUM_CONTROLLERS];
  size_t num_talkers;
  size_t num_listeners;
  size_t num_controllers;

  /* Access restriction */
  bool acquired;
  bool locked;
  unique_id_t acquired_by;
  unique_id_t locked_by;
  struct timeval last_acquired;
  struct timeval last_locked;

  /* Unsolicited notifications */
  bool unsol_notif_enabled;
  struct timespec last_unsol_notif;

  /* Latest received packet and its timestamp (CLOCK_REALTIME) 
   * 3 elements, 1 for each protocol (AVTP, MSRP, MVRP)
   */
  avb_msgbuf_u rxbuf[AVB_NUM_PROTOCOLS]; // received frame buffer
  size_t rxbuf_size[AVB_NUM_PROTOCOLS]; // size of the received frame
  struct timespec rxtime[AVB_NUM_PROTOCOLS]; // timestamp of the received frame
  eth_addr_t rxsrc[AVB_NUM_PROTOCOLS]; // source address of the received frame

  /* I2S handles */
  i2s_chan_handle_t i2s_tx_handle;
  i2s_chan_handle_t i2s_rx_handle;

  /* Last time we sent a periodic message */
  struct timespec last_transmitted_adp_entity_avail;
  struct timespec last_transmitted_mvrp_vlan_id;
  struct timespec last_transmitted_msrp_domain;
  struct timespec last_transmitted_msrp_talker_adv;
  struct timespec last_transmitted_msrp_listener;
  struct timespec last_transmitted_msrp_leaveall;
  struct timespec last_transmitted_maap_announce;
  struct timespec last_ptp_status_update;
  struct timespec last_transmitted_unsol_notif;

  /* Sequence IDs for outbound ATDECC messages */
  uint32_t adp_seq_id;
  uint16_t aecp_seq_id;
  int16_t  acmp_seq_id;

  /* Logo */
  const uint8_t *logo_start;
  uint32_t logo_length;

} avb_state_s;

/* AVB Functions */

/* Network functions */
int avb_net_init(avb_state_s *state);
void avb_create_eth_frame(
    uint8_t *eth_frame, 
    eth_addr_t *dest_addr, 
    avb_state_s *state, 
    ethertype_t ethertype, 
    void *msg, 
    uint16_t msg_len
);
int avb_net_send_to(
    avb_state_s *state, 
    ethertype_t ethertype, 
    void *msg, 
    uint16_t msg_len, 
    struct timespec *ts,
    eth_addr_t *dest_addr
);
int avb_net_send(
    avb_state_s *state, 
    ethertype_t ethertype, 
    void *msg, 
    uint16_t msg_len, 
    struct timespec *t
);
int avb_net_recv(
    int l2if, 
    void *msg, 
    uint16_t msg_len, 
    struct timespec *ts,
    eth_addr_t *src_addr
);
void eth_rx_callback(
    void *arg, 
    esp_eth_handle_t eth_handle, 
    uint8_t *buffer, 
    uint32_t length
);

/* AVB send functions */

/* MVRP send functions */
int avb_send_mvrp_vlan_id(
    avb_state_s *state,
    mrp_attr_event_t attr_event,
    bool leave_all
);
/* MSRP send functions */
int avb_send_msrp_domain(
    avb_state_s *state,
    mrp_attr_event_t attr_event,
    bool leave_all
);
int avb_send_msrp_talker(
    avb_state_s *state, 
    mrp_attr_event_t attr_event, 
    bool leave_all,
    bool is_failed,
    unique_id_t *stream_id
);
int avb_send_msrp_listener(
    avb_state_s *state, 
    mrp_attr_event_t attr_event, 
    msrp_listener_event_t listener_event,
    bool leave_all,
    unique_id_t *stream_id
);

/* AVTP send functions */
int avb_send_maap_announce(avb_state_s *state);
int avb_send_aaf_pcm(avb_state_s *state);

/* ATDECC send functions */
int avb_send_adp_entity_available(avb_state_s *state);
int avb_send_aecp_cmd_controller_available(
    avb_state_s *state, 
    unique_id_t *target_id
);
int avb_send_aecp_cmd_entity_available(
    avb_state_s *state, 
    unique_id_t *target_id
);
int avb_send_aecp_cmd_get_stream_info(
    avb_state_s *state, 
    unique_id_t *target_id
);
int avb_send_aecp_rsp_get_stream_info(
    avb_state_s *state, 
    aecp_get_stream_info_s *msg,
    eth_addr_t *dest_addr
);
int avb_send_aecp_unsol_get_stream_info(
    avb_state_s *state,
    uint16_t index,
    bool is_output
);
int avb_send_aecp_cmd_get_counters(
    avb_state_s *state, 
    unique_id_t *target_id
);
int avb_send_aecp_rsp_get_counters(
    avb_state_s *state,
    aecp_get_counters_s *msg,
    eth_addr_t *dest_addr
); 
int avb_send_aecp_unsol_get_counters(
    avb_state_s *state,
    aem_desc_type_t descriptor_type,
    uint16_t index
);
int avb_send_aecp_rsp_read_descr_entity(
    avb_state_s *state, 
    aecp_read_descriptor_rsp_s *msg,
    eth_addr_t *dest_addr
);
int avb_send_aecp_rsp_read_descr_configuration(
    avb_state_s *state, 
    aecp_read_descriptor_rsp_s *msg,
    eth_addr_t *dest_addr
);
int avb_send_aecp_rsp_read_descr_audio_unit(
    avb_state_s *state, 
    aecp_read_descriptor_rsp_s *msg,
    eth_addr_t *dest_addr
);
int avb_send_aecp_rsp_read_descr_stream(
    avb_state_s *state, 
    aecp_read_descriptor_rsp_s *msg,
    eth_addr_t *dest_addr,
    bool is_output
);
int avb_send_aecp_rsp_read_descr_avb_interface(
    avb_state_s *state, 
    aecp_read_descriptor_rsp_s *msg,
    eth_addr_t *dest_addr
);
int avb_send_aecp_rsp_read_descr_clock_source(
    avb_state_s *state, 
    aecp_read_descriptor_rsp_s *msg,
    eth_addr_t *dest_addr
);
int avb_send_aecp_rsp_read_descr_memory_obj(
    avb_state_s *state, 
    aecp_read_descriptor_rsp_s *msg,
    eth_addr_t *dest_addr
);
int avb_send_aecp_rsp_read_descr_locale(
    avb_state_s *state, 
    aecp_read_descriptor_rsp_s *msg,
    eth_addr_t *dest_addr
);
int avb_send_aecp_rsp_read_descr_strings(
    avb_state_s *state, 
    aecp_read_descriptor_rsp_s *msg,
    eth_addr_t *dest_addr
);
int avb_send_aecp_rsp_read_descr_stream_port(
    avb_state_s *state, 
    aecp_read_descriptor_rsp_s *msg,
    eth_addr_t *dest_addr,
    bool is_output
);
int avb_send_aecp_rsp_read_descr_audio_cluster(
    avb_state_s *state, 
    aecp_read_descriptor_rsp_s *msg,
    eth_addr_t *dest_addr
);
int avb_send_aecp_rsp_read_descr_audio_map(
    avb_state_s *state, 
    aecp_read_descriptor_rsp_s *msg,
    eth_addr_t *dest_addr
);
int avb_send_aecp_rsp_read_descr_control(
    avb_state_s *state, 
    aecp_read_descriptor_rsp_s *msg,
    eth_addr_t *dest_addr
);
int avb_send_aecp_rsp_read_descr_clock_domain(
    avb_state_s *state, 
    aecp_read_descriptor_rsp_s *msg,
    eth_addr_t *dest_addr
);
int avb_send_acmp_connect_rx_command(
    avb_state_s *state, 
    unique_id_t *talker_id, 
    unique_id_t *listener_id
); // acting as controller
int avb_send_acmp_connect_tx_command(
    avb_state_s *state, 
    unique_id_t *controller_id, 
    unique_id_t *talker_id
); // acting as listener
int avb_send_acmp_command(
    avb_state_s *state,
    acmp_msg_type_t msg_type, 
    acmp_message_s *command, 
    bool retried
);
int avb_send_acmp_response(
    avb_state_s *state, 
    acmp_msg_type_t msg_type, 
    acmp_message_s *response, 
    acmp_status_t status
);

/* AVB processing functions */

/* MVRP processing functions */
int avb_process_mvrp_vlan_id(
    avb_state_s *state, 
    mvrp_vlan_id_message_s *msg
);

/* MSRP processing functions */
int avb_process_msrp_domain(
    avb_state_s *state,
    msrp_msgbuf_s *msg,
    int offset,
    size_t length
);
int avb_process_msrp_talker(
    avb_state_s *state,
    msrp_msgbuf_s *msg,
    int offset,
    size_t length,
    bool is_failed,
    eth_addr_t *src_addr
);
int avb_process_msrp_listener(
    avb_state_s *state,
    msrp_msgbuf_s *msg,
    int offset,
    size_t length
);

/* AVTP processing functions */
int avb_process_iec_61883(
    avb_state_s *state, 
    iec_61883_6_message_s *msg
);
int avb_process_aaf(
    avb_state_s *state, 
    aaf_pcm_message_s *msg
);
int avb_process_maap(
    avb_state_s *state, 
    maap_message_s *msg
);

/* ADP processing functions */
int avb_process_adp(
    avb_state_s *state, 
    adp_message_s *msg, 
    eth_addr_t *src_addr
); // handle all adp messages

/* AECP processing functions */
int avb_process_aecp(
    avb_state_s *state, 
    aecp_message_u *msg,
    eth_addr_t *src_addr
); // route to specific func
int avb_process_acmp(
    avb_state_s *state, 
    acmp_message_s *msg
); // route to specific func
int avb_process_aecp_cmd_entity_available(
    avb_state_s *state,
    aecp_message_u *msg,
    eth_addr_t *src_addr
);
int avb_process_aecp_cmd_register_unsol_notif(
    avb_state_s *state,
    aecp_message_u *msg,
    eth_addr_t *src_addr
);
int avb_process_aecp_cmd_deregister_unsol_notif(
    avb_state_s *state,
    aecp_message_u *msg,
    eth_addr_t *src_addr
);
int avb_process_aecp_cmd_lock_entity(
    avb_state_s *state,
    aecp_message_u *msg,
    eth_addr_t *src_addr
);
int avb_process_aecp_cmd_acquire_entity(
    avb_state_s *state,
    aecp_message_u *msg,
    eth_addr_t *src_addr
);
int avb_process_aecp_cmd_get_configuration(
    avb_state_s *state,
    aecp_message_u *msg,
    eth_addr_t *src_add
);
int avb_process_aecp_cmd_read_descriptor(
    avb_state_s *state,
    aecp_message_u *msg,
    eth_addr_t *src_addr
);
int avb_process_aecp_cmd_get_stream_info(
    avb_state_s *state,
    aecp_message_u *msg,
    eth_addr_t *src_addr
);
int avb_process_aecp_cmd_get_counters(
    avb_state_s *state,
    aecp_message_u *msg,
    eth_addr_t *src_addr
);
int avb_process_aecp_rsp_register_unsol_notif(
    avb_state_s *state,
    aecp_message_u *msg
);
int avb_process_aecp_rsp_deregister_unsol_notif(
    avb_state_s *state,
    aecp_message_u *msg
);
int avb_process_aecp_rsp_entity_available(
    avb_state_s *state,
    aecp_message_u *msg
);
int avb_process_aecp_rsp_controller_available(
    avb_state_s *state,
    aecp_message_u *msg
);
int avb_process_aecp_rsp_get_stream_info(
    avb_state_s *state,
    aecp_message_u *msg
);
int avb_process_aecp_rsp_get_counters(
    avb_state_s *state,
    aecp_message_u *msg
);

/* ACMP processing functions */
int avb_process_acmp_connect_rx_command(
    avb_state_s *state,
    acmp_message_s *msg,
    bool disconnect
);
int avb_process_acmp_connect_tx_command(
    avb_state_s *state,
    acmp_message_s *msg,
    bool disconnect
);
int avb_process_acmp_connect_tx_response(
    avb_state_s *state,
    acmp_message_s *msg,
    bool disconnect
);
int avb_process_acmp_get_state_command(
    avb_state_s *state,
    acmp_message_s *msg,
    bool rx
);

/* Stream functions */
int avb_start_stream_in(avb_state_s *state, uint16_t index);
int avb_stop_stream_in(avb_state_s *state, uint16_t index);
int avb_start_stream_out(avb_state_s *state, uint16_t index);
int avb_stop_stream_out(avb_state_s *state, uint16_t index);

/* Codec functions */
esp_err_t avb_config_i2s(avb_state_s *state);
esp_err_t avb_config_codec(avb_state_s *state);
void i2s_task(void *arg);

/* Helper functions */
void stream_id_from_mac(
    eth_addr_t *mac_addr, 
    uint8_t *stream_id, 
    size_t uid
);
int avb_find_entity_by_id(
    avb_state_s *state, 
    unique_id_t *entity_id, 
    avb_entity_type_t entity_typ
);
int avb_find_entity_by_addr(
    avb_state_s *state, 
    eth_addr_t *entity_addr, 
    avb_entity_type_t entity_type
);
const char* get_adp_message_type_name(adp_msg_type_t message_type);
const char* get_aecp_command_code_name(aecp_cmd_code_t command_code);
const char* get_acmp_message_type_name(acmp_msg_type_t message_type);
bool avb_acquired_or_locked_by_other(
    avb_state_s *state, 
    unique_id_t *entity_id
);
bool avb_listener_is_connected(
    avb_state_s *state, 
    acmp_message_s *msg, 
    bool same_talker
);
bool avb_valid_talker_listener_uid(
    avb_state_s *state, 
    uint16_t uid, 
    avb_entity_type_t entity_type
);
int avb_add_inflight_command(
    avb_state_s *state, 
    atdecc_command_u *command, 
    bool inbound
);
int avb_find_inflight_command(
    avb_state_s *state, 
    uint16_t seq_id, 
    bool inbound
);
int avb_find_inflight_command_by_data(
    avb_state_s *state, 
    atdecc_command_u *data, 
    bool inbound
);
void avb_remove_inflight_command(
    avb_state_s *state, 
    uint16_t seq_id, 
    bool inbound
);
acmp_status_t avb_connect_listener(
    avb_state_s *state, 
    acmp_message_s *response
);
acmp_status_t avb_disconnect_listener(
    avb_state_s *state, 
    acmp_message_s *response
);
acmp_status_t avb_connect_talker(
    avb_state_s *state, 
    acmp_message_s *response
);
int avb_get_acmp_timeout_ms(acmp_msg_type_t msg_type);
IRAM_ATTR esp_err_t my_callback(esp_eth_handle_t eth_handle, uint8_t *buffer, uint32_t length, void *priv);

#endif /* _ESP_AVB_AVB_H_ */
