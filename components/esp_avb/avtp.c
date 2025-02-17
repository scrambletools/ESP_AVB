/*
 * Copyright 2024 Scramble Tools
 * License: MIT
 *
 * ESP_AVB Component
 *
 * This component provides an implementation of an AVB talker and listener.
 * 
 * This file provides the required features of the AVTP protocol including support for MSRP.
 */

#include "avb.h"

/* globals to optimize l2tap usage */
static bool avtp_active_stream_in = false;
static bool avtp_active_stream_out = false;

/* Send MVRP VLAN identifier message */
int avb_send_mvrp_vlan_id(avb_state_s *state) {
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
  msg.attr_event[0] = int_to_3pe(msrp_attr_event_join_in, 0, 0);
  uint16_t msg_len = 8 + 2 + 2; // all of the above + end mark list and end mark msg
  
  // send the message
  ret = avb_net_send(state, ethertype_mvrp, &msg, msg_len, &ts);
  if (ret < 0) {
    avberr("send MVRP VLAN ID failed: %d", errno);
    return ret;
  }
  return OK;
}

/* Send MSRP domain message */
int avb_send_msrp_domain(avb_state_s *state) {
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
  msg.sr_class_id = 6; // class A
  msg.sr_class_priority = 3; // priority 3 for class A
  int_to_octets(&vlan_id, msg.sr_class_vid, vlan_id);
  msg.attr_event[0] = int_to_3pe(msrp_attr_event_join_in, 0, 0);

  // Create an MSRP message buffer
  msrp_msgbuf_s msrp_msg;
  memset(&msrp_msg, 0, sizeof(msrp_msg));
  msrp_msg.protocol_ver = 0;
  memcpy(msrp_msg.messages_raw, &msg, sizeof(msg));
  uint16_t msg_len = 5 + attr_list_len + 2; // header + attr_list_len + end mark
  
  // send the message
  ret = avb_net_send(state, ethertype_msrp, &msrp_msg, msg_len, &ts);
  if (ret < 0) {
      avberr("send MSRP Domain failed: %d", errno);
    }
  return ret;
}

/* Send MSRP talker advertise message with appropriate event */
int avb_send_msrp_talker(
    avb_state_s *state, 
    msrp_attr_event_t attr_event, 
    bool leave_all,
    unique_id_t stream_id,
    bool is_failed
) {
  msrp_talker_message_u msg;
  struct timespec ts;
  int ret;
  int vlan_id = CONFIG_ESP_AVB_VLAN_ID;
  int attr_list_len;
  memset(&msg, 0, sizeof(msg));

  // Populate the message
  if (!is_failed) {   
    msg.header.attr_type = msrp_attr_type_talker_advertise;
    msg.header.attr_len = 25;
    attr_list_len = 29; // includes vechead, attr_event and vec end mark
  } else {
    msg.header.attr_type = msrp_attr_type_talker_failed;
    msg.header.attr_len = 34;
    attr_list_len = 38; // includes vechead, attr_event and vec end mark
  }
  int_to_octets(&attr_list_len, msg.header.attr_list_len, 2);
  msg.header.vechead_leaveall = leave_all;
  msg.header.vechead_num_vals = 1;
  memcpy(msg.talker.info.stream_id, stream_id, UNIQUE_ID_LEN);
  memcpy(msg.talker.info.stream_dest_addr, &MAAP_MCAST_MAC_ADDR, 6);
  int_to_octets(&vlan_id, msg.talker.info.vlan_id, 2);
  int tspec_max_frame_size = 1024;
  int_to_octets(&tspec_max_frame_size, msg.talker.info.tspec_max_frame_size, 2);
  int tspec_max_frame_interval = 1000;
  int_to_octets(&tspec_max_frame_interval, msg.talker.info.tspec_max_frame_interval, 2);
  msg.talker.info.priority = 3; // class A
  msg.talker.info.rank = 1; // rank 1 for class A
  int accumulated_latency = 0;
  int_to_octets(&accumulated_latency, msg.talker.info.accumulated_latency, 2);
  if (is_failed) {
    memcpy(msg.talker_failed.failure_bridge_id, &EMPTY_ID, UNIQUE_ID_LEN);
    msg.talker_failed.failure_code = 0;
    msg.talker_failed.event_data[0] = int_to_3pe(attr_event, 0, 0);
  }
  else {
    msg.talker.event_data[0] = int_to_3pe(attr_event, 0, 0);
  }

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
  return ret;
}

/* Send MSRP listener message with appropriate event */
int avb_send_msrp_listener(
    avb_state_s *state, 
    msrp_attr_event_t attr_event, 
    msrp_listener_event_t listener_event,
    bool leave_all,
    unique_id_t stream_id
) {
  msrp_listener_message_s msg;
  struct timespec ts;
  int ret; 
  memset(&msg, 0, sizeof(msg));
  if (stream_id == NULL) {
    stream_id = EMPTY_ID;
  }

  // Populate the message
  msg.header.attr_type = msrp_attr_type_listener;
  msg.header.attr_len = 8;
  int attr_list_len = 14; // includes vechead, attr_event and vec end mark
  int_to_octets(&attr_list_len, msg.header.attr_list_len, 2);
  msg.header.vechead_leaveall = leave_all;
  msg.header.vechead_padding = 0;
  msg.header.vechead_num_vals = 1; // only attribute event counts
  memcpy(msg.stream_id, stream_id, UNIQUE_ID_LEN);
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
  return ret;
}

/* Send MAAP Announce message */
int avb_send_maap_announce(avb_state_s *state) {
  maap_message_s msg;
  struct timespec ts;
  int ret;
  memset(&msg, 0, sizeof(msg));

  // Populate the message TDB

  ret = avb_net_send(state, ethertype_avtp, &msg, sizeof(msg), &ts);
  if (ret < 0) {
    avberr("send MAAP Announce failed: %d", errno);
  }
  return OK;
}

/* Process received MSRP domain message */
int avb_process_msrp_domain(
    avb_state_s *state,
    msrp_msgbuf_s *msg,
    int offset,
    size_t length
) {
  // not implemented
  return OK;
}

/* Process received MSRP talker advertise message */
int avb_process_msrp_talker(
    avb_state_s *state,
    msrp_msgbuf_s *msg_data,
    int offset,
    size_t length,
    bool is_failed, 
    eth_addr_t *src_addr
) {
  msrp_talker_message_u msg;
  memset(&msg, 0, sizeof(msrp_talker_message_u));
  memcpy(&msg, &msg_data->messages_raw[offset], sizeof(msrp_talker_message_u));

  // get the talker addr from the stream id
  eth_addr_t talker_addr;
  memcpy(&talker_addr, msg.talker.info.stream_id, ETH_ADDR_LEN);

  // If the talker is known then update talker info
  int index = avb_find_entity_by_addr(state, &talker_addr, avb_entity_type_talker);
  if (index >= 0) {
    memcpy(&state->talkers[index].info, &msg.talker.info, sizeof(talker_adv_info_s));
  }
  // If the talker is not known then remember it
  // else {
  //   // create a new talker entity
  //   avb_talker_s new_talker;
  //   memset(&new_talker, 0, sizeof(avb_talker_s));
  //   memcpy(&new_talker.info, &msg.talker.info, sizeof(talker_adv_info_s));
  //   // if talker list is not full, add the talker to the list
  //   if (state->num_talkers < AVB_MAX_NUM_TALKERS) {
  //     memcpy(&state->talkers[state->num_talkers], &new_talker, sizeof(avb_talker_s));
  //     state->num_talkers++;
  //   }
  //   // if talker list is full, replace the oldest talker
  //   else {
  //     memmove(&state->talkers[0], &state->talkers[1], (state->num_talkers - 1) * sizeof(avb_talker_s));
  //     memcpy(&state->talkers[state->num_talkers - 1], &new_talker, sizeof(avb_talker_s));
  //   }
  // }
  return OK;
}

/* Process received MSRP listener ready message */
int avb_process_msrp_listener(
    avb_state_s *state,
    msrp_msgbuf_s *msg,
    int offset,
    size_t length
) {
  // not implemented
  return OK;
}

/* Process received AVTP IEC 61883 message */
int avb_process_iec_61883(avb_state_s *state, iec_61883_6_message_s *msg) {
  //avbinfo("Got an AVTP IEC 61883 message");

  // check if the stream id is in the connection list and the stream is not yet active
//   int index = avb_find_connection_by_id(state, &msg->stream_id, avb_entity_type_listener);
//   if (index >= 0 && !state->connections[index].active) {
//     avbinfo("Stream id found in connection list");
//     // set the stream as active to avoid duplicate processing
//     state->connections[index].active = true;
//     // check if the connection has stream format info
//     if (state->connections[index].stream.stream_format.am824.fdf_evt == 0) {
      // set the stream format in the connection
    //   state->connections[index].stream.stream_format.format = msg->format;
    //   state->connections[index].stream.stream_format.sample_rate = msg->sample_rate;
    //   state->connections[index].stream.stream_format.chan_per_frame = msg->chan_per_frame;
    //   state->connections[index].stream.stream_format.bit_depth = msg->bit_depth;
    //}
    // if the stream format, sample rate or bit depth is not supported, then ignore the message
    // if (!in_array_of_int(state->connections[index].stream.stream_format.format, (int *)supported_aaf_formats, ARRAY_SIZE(supported_aaf_formats)) ||
    //     !in_array_of_int(state->connections[index].stream.stream_format.sample_rate, (int *)supported_sample_rates, ARRAY_SIZE(supported_sample_rates)) ||
    //     !in_array_of_int(state->connections[index].stream.stream_format.bit_depth, (int *)supported_bit_depths, ARRAY_SIZE(supported_bit_depths))) {
    //   avberr("Unsupported stream format, sample rate or bit depth");
    //   return ERROR;
    // }
    // start the stream input task
   //avb_start_stream_in(state, &msg->stream_id);
  //}
  // not implemented
  return OK;
}

/* Process received AVTP AAF PCM message */
int avb_process_aaf(avb_state_s *state, aaf_pcm_message_s *msg) {
  //avbinfo("Got an AVTP AAF PCM message");

  // check if the stream id is in the connection list and the stream is not yet active
//   int index = avb_find_connection_by_id(state, &msg->stream_id, avb_entity_type_listener);
//   if (index >= 0 && !state->connections[index].active) {
//     avbinfo("Stream id found in connection list");
//     // set the stream as active to avoid duplicate processing
//     state->connections[index].active = true;
    // check if the connection has stream format info
    // if (state->connections[index].stream.stream_format.bit_depth == 0) {
    //   // set the stream format in the connection
    //   state->connections[index].stream.stream_format.format = msg->format;
    //   state->connections[index].stream.stream_format.sample_rate = msg->sample_rate;
    //   state->connections[index].stream.stream_format.chan_per_frame = msg->chan_per_frame;
    //   state->connections[index].stream.stream_format.bit_depth = msg->bit_depth;
    // }
    // if the stream format, sample rate or bit depth is not supported, then ignore the message
    // if (!in_array_of_int(state->connections[index].stream.stream_format.format, (int *)supported_aaf_formats, ARRAY_SIZE(supported_aaf_formats)) ||
    //     !in_array_of_int(state->connections[index].stream.stream_format.sample_rate, (int *)supported_sample_rates, ARRAY_SIZE(supported_sample_rates)) ||
    //     !in_array_of_int(state->connections[index].stream.stream_format.bit_depth, (int *)supported_bit_depths, ARRAY_SIZE(supported_bit_depths))) {
    //   avberr("Unsupported stream format, sample rate or bit depth");
    //   return ERROR;
    // }
    // start the stream input task
    //avb_start_stream_in(state, &msg->stream_id);
  //}
  // not implemented
  return OK;
}

/* Process received AVTP MAAP message */
int avb_process_maap(avb_state_s *state, maap_message_s *msg) {
  // not implemented
  return OK;
}

void stream_id_from_mac(eth_addr_t *mac_addr, uint8_t *stream_id, size_t uid) {
  // copy the mac address octets to the stream id and fill the remaining octects with uid 
  memcpy(stream_id, mac_addr, ETH_ADDR_LEN);
  memset(stream_id + ETH_ADDR_LEN, uid, UNIQUE_ID_LEN - ETH_ADDR_LEN);
}

