#ifndef ESP_AVB_SIMPLE_TALKER_INCLUDE_AVTP_H_
#define ESP_AVB_SIMPLE_TALKER_INCLUDE_AVTP_H_

#include <stdlib.h>
#include <esp_log.h>
#include "utils.h"
#include "config.h"

// AVTP subtypes and their values
typedef enum {
  avtp_subtype_aaf = 0x02,
  avtp_subtype_adp = 0xfa,
  avtp_subtype_aecp = 0xfb,
  avtp_subtype_acmp = 0xfc,
  avtp_subtype_maap = 0xfe
} avtp_subtype_t;

// MAAP message types and their values
typedef enum {
  maap_message_type_probe = 0x01,
  maap_message_type_defend = 0x02,
  maap_message_type_announce = 0x03
} maap_message_type_t;

// MSRP attribute types and their values
typedef enum {
  msrp_attribute_type_talker_advertise = 0x01,
  msrp_attribute_type_talker_failed = 0x02,
  msrp_attribute_type_listener = 0x03,
  msrp_attribute_type_domain = 0x04
} msrp_attribute_type_t;

// MSRP reservation declaration types in enumerated order
typedef enum {
  msrp_declaration_type_talker_advertise,
  msrp_declaration_type_talker_failed,
  msrp_declaration_type_listener_asking_failed,
  msrp_declaration_type_listener_ready,
  msrp_declaration_type_listener_ready_failed
} msrp_declaration_type_t;

// MSRP reservation failure codes in enumerated order
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

// MVRP attribute types and their values
typedef enum {
  mvrp_attribute_type_vlan_identifier = 0x01
} mvrp_attribute_type_t;

// Functions
void append_avtpdu(avtp_subtype_t type, eth_frame_t *frame);
void append_msrpdu(msrp_attribute_type_t type, eth_frame_t *frame);
void append_mvrpdu(msrp_attribute_type_t type, eth_frame_t *frame);
avb_frame_type_t detect_avtp_frame_type(ethertype_t *ethertype, eth_frame_t *frame, ssize_t size);
void print_avtp_frame(avb_frame_type_t type, eth_frame_t *frame, int format);

#endif /* ESP_AVB_SIMPLE_TALKER_INCLUDE_AVTP_H_ */