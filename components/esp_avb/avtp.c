/*
 * Copyright 2024 Scramble Tools
 * License: MIT
 *
 * ESP_AVB Component
 *
 * This component provides an implementation of an AVB talker and listener.
 */

#include "avb.h"

/* Send MVRP VLAN identifier message */
int avb_send_mvrp_vlan_id(struct avb_state_s *state) {
  mvrp_vlan_id_message_s msg;
  struct timespec ts;
  int ret;
  int vlan_id = CONFIG_ESP_AVB_VLAN_ID;
  memset(&msg, 0, sizeof(msg));

  // Populate the message
  msg.protocol_ver = 0;
  msg.header.attr_type = mvrp_attr_type_vlan_identifier;
  msg.header.attr_len = 2;
  msg.header.vechead_leaveall = 0;
  msg.header.vechead_padding = 0;
  msg.header.vechead_num_vals = 1;
  int_to_octets(&vlan_id, msg.vlan_id, 2);
  msg.attr_event[0] = int_to_3pe(msrp_attr_event_new, 0, 0);
  uint16_t msg_len = 8 + 2 + 2; // all of the above + end mark list and end mark msg
  
  ret = avb_net_send(state, ethertype_mvrp, &msg, msg_len, &ts);
  if (ret < 0) {
      avberr("send MVRP VLAN ID failed: %d", errno);
    }
  else {
      avbinfo("Sent MVRP VLAN ID message");
    }
  return ret;
}

/* Send MSRP domain message */
int avb_send_msrp_domain(struct avb_state_s *state) {
  msrp_domain_message_s msg;
  struct timespec ts;
  int ret;
  int vlan_id = CONFIG_ESP_AVB_VLAN_ID;
  memset(&msg, 0, sizeof(msg));

  // Populate the message
  msg.header.attr_type = msrp_attr_type_domain;
  msg.header.attr_len = 4;
  int attr_list_len = 9; // includes vechead, attr_event and vec end mark
  int_to_octets(&attr_list_len, msg.header.attr_list_len, 2);
  msg.header.vechead_leaveall = 0;
  msg.header.vechead_padding = 0;
  msg.header.vechead_num_vals = 1;
  msg.sr_class_id = 6; // class B
  msg.sr_class_priority = 3; // priority 2 for class B
  int_to_octets(&vlan_id, msg.sr_class_vid, 2);
  msg.attr_event[0] = int_to_3pe(msrp_attr_event_join_in, 0, 0);

  // Create an MSRP message buffer
  msrp_msgbuf_s msrp_msg;
  memset(&msrp_msg, 0, sizeof(msrp_msg));
  msrp_msg.protocol_ver = 0;
  memcpy(msrp_msg.messages_raw, &msg, sizeof(msg));
  uint16_t msg_len = 5 + attr_list_len + 2; // header + attr_list_len + end mark
  
  ret = avb_net_send(state, ethertype_msrp, &msrp_msg, msg_len, &ts);
  if (ret < 0) {
      avberr("send MSRP Domain failed: %d", errno);
    }
  else {
      avbinfo("Sent MSRP Domain message");
    }
  return ret;
}

/* Send MSRP talker advertise message with appropriate event */
int avb_send_msrp_talker_adv(struct avb_state_s *state, msrp_attr_event_t event) {
  msrp_talker_adv_message_s msg;
  struct timespec ts;
  int ret;
  int vlan_id = CONFIG_ESP_AVB_VLAN_ID;
  uint8_t stream_id[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  memset(&msg, 0, sizeof(msg));

  // Populate the message
  msg.header.attr_type = msrp_attr_type_talker_advertise;
  msg.header.attr_len = 25;
  int attr_list_len = 29; // includes vechead, attr_event and vec end mark
  int_to_octets(&attr_list_len, msg.header.attr_list_len, 2);
  msg.header.vechead_leaveall = 0;
  msg.header.vechead_padding = 0;
  msg.header.vechead_num_vals = 1;
  memcpy(msg.stream_id, stream_id, 8);
  memcpy(msg.stream_dest_addr, &MAAP_MCAST_MAC_ADDR, 6);
  int_to_octets(&vlan_id, msg.vlan_id, 2);
  int tspec_max_frame_size = 1024;
  int_to_octets(&tspec_max_frame_size, msg.tspec_max_frame_size, 2);
  int tspec_max_frame_interval = 1000;
  int_to_octets(&tspec_max_frame_interval, msg.tspec_max_frame_interval, 2);
  msg.priority = 3; // class A
  msg.rank = 1; // rank 1 for class A
  int accumulated_latency = 0;
  int_to_octets(&accumulated_latency, msg.accumulated_latency, 2);
  msg.event_data[0] = int_to_3pe(event, 0, 0);

  // Create an MSRP message buffer
  msrp_msgbuf_s msrp_msg;
  memset(&msrp_msg, 0, sizeof(msrp_msg));
  msrp_msg.protocol_ver = 0;
  memcpy(msrp_msg.messages_raw, &msg, sizeof(msg));
  uint16_t msg_len = 5 + attr_list_len + 2; // header + attr_list_len + end mark
  
  ret = avb_net_send(state, ethertype_msrp, &msrp_msg, msg_len, &ts);
  if (ret < 0) {
      avberr("send MSRP Talker Advertise failed: %d", errno);
    }
  else {
      avbinfo("Sent MSRP Talker Advertise message");
    }
  return ret;
}

/* Send MSRP listener message with appropriate event */
int avb_send_msrp_listener(struct avb_state_s *state, msrp_attr_event_t attr_event, msrp_listener_event_t listener_event) {
  msrp_listener_message_s msg;
  struct timespec ts;
  int ret;
  uint8_t stream_id[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  memset(&msg, 0, sizeof(msg));

  // Populate the message
  msg.header.attr_type = msrp_attr_type_listener;
  msg.header.attr_len = 8;
  int attr_list_len = 14; // includes vechead, attr_event and vec end mark
  int_to_octets(&attr_list_len, msg.header.attr_list_len, 2);
  msg.header.vechead_leaveall = 0;
  msg.header.vechead_padding = 0;
  msg.header.vechead_num_vals = 1; // only attribute event counts
  memcpy(msg.stream_id, stream_id, 8);
  msg.event_decl_data[0].event = int_to_3pe(attr_event, 0, 0);
  msg.event_decl_data[1].declaration.event1 = listener_event;

  // Create an MSRP message buffer
  msrp_msgbuf_s msrp_msg;
  memset(&msrp_msg, 0, sizeof(msrp_msg));
  msrp_msg.protocol_ver = 0;
  memcpy(msrp_msg.messages_raw, &msg, sizeof(msg));
  uint16_t msg_len = 5 + attr_list_len + 2; // header + attr_list_len + end mark
  
  ret = avb_net_send(state, ethertype_msrp, &msrp_msg, msg_len, &ts);
  if (ret < 0) {
      avberr("send MSRP Listener failed: %d", errno);
    }
  else {
      avbinfo("Sent MSRP Listener message");
    }
  return ret;
}

/* Send MAAP Announce message */
int avb_send_maap_announce(struct avb_state_s *state) {
  maap_message_s msg;
  struct timespec ts;
  int ret;
  memset(&msg, 0, sizeof(msg));

  // Populate the message TDB

  ret = avb_net_send(state, ethertype_avtp, &msg, sizeof(msg), &ts);
  if (ret < 0) {
      avberr("send MAAP Announce failed: %d", errno);
    }
  else {
      avbinfo("Sent MAAP Announce message");
    }

  return OK;
}

/* Process received MSRP domain message */
int avb_process_msrp_domain(struct avb_state_s *state,
                            msrp_msgbuf_s *msg,
                            int offset,
                            size_t length) {
  // TODO: Implement processing
  return OK;
}

/* Process received MSRP talker advertise message */
int avb_process_msrp_talker(struct avb_state_s *state,
                            msrp_msgbuf_s *msg,
                            int offset,
                            size_t length,
                            bool is_failed) {
  // TODO: Implement processing
  return OK;
}

/* Process received MSRP listener ready message */
int avb_process_msrp_listener(struct avb_state_s *state,
                              msrp_msgbuf_s *msg,
                              int offset,
                              size_t length) {
  // TODO: Implement processing
  return OK;
}

/* Process received MVRP VLAN identifier message */
int avb_process_mvrp_vlan_id(struct avb_state_s *state, mvrp_vlan_id_message_s *msg) {
  // TODO: Implement processing
  return OK;
}

/* Process received AVTP AAF PCM message */
int avb_process_aaf(struct avb_state_s *state, aaf_pcm_message_s *msg) {
  // TODO: Implement processing
  return OK;
}

/* Process received AVTP MAAP message */
int avb_process_maap(struct avb_state_s *state, maap_message_s *msg) {
  // TODO: Implement processing
  return OK;
}
