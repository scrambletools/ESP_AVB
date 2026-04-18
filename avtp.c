/*
 * Copyright 2024-2026 Scramble Tools
 * License: MIT
 *
 * ESP_AVB Component
 *
 * This component provides an implementation of an AVB talker and listener.
 *
 * This file provides the required features of the AVTP protocol including
 * support for MSRP.
 */

#include "avb.h"
#include "esp_codec_dev.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/portmacro.h"

/* Note: Ethernet MAC DMA transmit is accessed concurrently by the AVTP stream
 * task (core 1) and the PTP daemon (core 0). The ESP-IDF driver lacks internal
 * locking. A lock here was attempted but the PTP timestamped TX path holds the
 * DMA for too long (busy-waits for TX completion), causing the AVTP real-time
 * task to miss its 125μs deadlines. The DMA ring appears to handle concurrent
 * access in practice (separate TX descriptors for each caller). */

/* Send MVRP VLAN identifier message */
int avb_send_mvrp_vlan_id(avb_state_s *state, mrp_attr_event_t attr_event,
                          bool leave_all) {
  mvrp_vlan_id_message_s msg;
  struct timespec ts;
  int ret;
  int vlan_id = CONFIG_ESP_AVB_STREAM_VLAN_ID;
  memset(&msg, 0, sizeof(msg));

  // Populate the message
  msg.protocol_ver = 0;
  msg.header.attr_type = mvrp_attr_type_vlan_identifier;
  msg.header.attr_len = 2;
  msg.header.vechead_leaveall = leave_all;
  msg.header.vechead_padding = 0;
  msg.header.vechead_num_vals = 1;
  int_to_octets(&vlan_id, msg.vlan_id, 2);
  msg.attr_event[0] = int_to_3pe(attr_event, 0, 0);
  uint16_t msg_len =
      8 + 2 + 2; // all of the above + end mark list and end mark msg

  // send the message
  ret = avb_net_send(state, ethertype_mvrp, &msg, msg_len, &ts);
  if (ret < 0) {
    avberr("send MVRP VLAN ID failed: %d", errno);
    return ret;
  }
  return OK;
}

/* Send MSRP domain message */
// TODO: add support for multiple domains
int avb_send_msrp_domain(avb_state_s *state, mrp_attr_event_t attr_event,
                         bool leave_all) {
  msrp_domain_message_s msg;
  struct timespec ts;
  int ret;
  int vlan_id = CONFIG_ESP_AVB_STREAM_VLAN_ID;
  memset(&msg, 0, sizeof(msg));

  // Populate the message
  msg.header.attr_type = msrp_attr_type_domain;
  msg.header.attr_len = 4;
  int attr_list_len = 9; // includes vechead, attr_event and vec end mark
  int_to_octets(&attr_list_len, msg.header.attr_list_len, 2);
  msg.header.vechead_leaveall = leave_all;
  msg.header.vechead_padding = 0;
  msg.header.vechead_num_vals = 1;
  msg.sr_class_id = 6;       // class A
  msg.sr_class_priority = 3; // priority 3 for class A
  int_to_octets(&vlan_id, msg.sr_class_vid, sizeof(msg.sr_class_vid));
  msg.attr_event[0] = int_to_3pe(attr_event, 0, 0);

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
    perror("send MSRP Domain failed");
  }
  return ret;
}

/* Send MSRP talker advertise message with appropriate event */
int avb_send_msrp_talker(avb_state_s *state, mrp_attr_event_t attr_event,
                         bool leave_all, bool is_failed, unique_id_t *stream_id,
                         eth_addr_t *stream_dest_addr, uint8_t *vlan_id) {
  msrp_talker_message_u msg;
  struct timespec ts;
  int ret;
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
  memcpy(msg.talker.info.stream_dest_addr, stream_dest_addr, ETH_ADDR_LEN);
  memcpy(msg.talker.info.vlan_id, vlan_id, 2);
  int tspec_max_frame_size = 850;
  int_to_octets(&tspec_max_frame_size, msg.talker.info.tspec_max_frame_size, 2);
  int tspec_max_frame_interval = 1;
  int_to_octets(&tspec_max_frame_interval,
                msg.talker.info.tspec_max_frame_interval, 2);
  msg.talker.info.priority = 3;    // class A
  msg.talker.info.rank = 1;        // rank 1 for class A
  int accumulated_latency = 15000; // ~15μs worst-case talker latency
  int_to_octets(&accumulated_latency, msg.talker.info.accumulated_latency, 4);
  if (is_failed) {
    memcpy(msg.talker_failed.failure_bridge_id, &EMPTY_ID, UNIQUE_ID_LEN);
    msg.talker_failed.failure_code = 0;
    msg.talker_failed.event_data[0] = int_to_3pe(attr_event, 0, 0);
  } else {
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
int avb_send_msrp_listener(avb_state_s *state, mrp_attr_event_t attr_event,
                           msrp_listener_event_t listener_event, bool leave_all,
                           unique_id_t *stream_id) {
  msrp_listener_message_s msg;
  struct timespec ts;
  int ret;
  memset(&msg, 0, sizeof(msg));

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

/* MAAP timing constants (IEEE 1722-2016 Annex B) */
#define MAAP_PROBE_RETRANSMITS 3
#define MAAP_PROBE_INTERVAL_BASE_MS 500
#define MAAP_PROBE_INTERVAL_VAR_MS 100
#define MAAP_ANNOUNCE_INTERVAL_BASE_MS 30000
#define MAAP_ANNOUNCE_INTERVAL_VAR_MS 2000
/* MAAP dynamic allocation pool: 91:E0:F0:00:00:00 to 91:E0:F0:00:FD:FF */
#define MAAP_POOL_SIZE 0xFE00

/* Generate a random address within the MAAP pool using device MAC as seed */
static void maap_generate_addr(avb_state_s *state, eth_addr_t *addr,
                               int stream_index) {
  uint32_t seed = state->internal_mac_addr[2] ^ state->internal_mac_addr[3];
  seed = (seed << 8) | state->internal_mac_addr[4];
  seed = (seed << 8) | state->internal_mac_addr[5];
  seed += (uint32_t)esp_timer_get_time() + stream_index;
  uint16_t offset = seed % MAAP_POOL_SIZE;
  uint8_t *a = (uint8_t *)addr;
  a[0] = 0x91;
  a[1] = 0xe0;
  a[2] = 0xf0;
  a[3] = 0x00;
  a[4] = (offset >> 8) & 0xFF;
  a[5] = offset & 0xFF;
}

/* Get a random timer jitter value */
static int64_t maap_jitter_us(int base_ms, int var_ms) {
  uint32_t r = (uint32_t)esp_timer_get_time();
  int jitter = (r % (var_ms + 1));
  return (int64_t)(base_ms + jitter) * 1000;
}

/* Check if two single-address ranges conflict */
static bool maap_addr_conflicts(eth_addr_t *a, eth_addr_t *b) {
  return memcmp(a, b, ETH_ADDR_LEN) == 0;
}

/* Send a MAAP message */
int avb_send_maap_msg(avb_state_s *state, maap_msg_type_t msg_type,
                      eth_addr_t *req_addr, uint16_t req_count,
                      eth_addr_t *confl_addr, uint16_t confl_count) {
  maap_message_s msg;
  struct timespec ts;
  memset(&msg, 0, sizeof(msg));

  msg.subtype = avtp_subtype_maap;
  msg.msg_type = msg_type;
  msg.sv = 0;
  msg.version = 0;
  msg.maap_version = 1;
  msg.control_data_len = 16;
  memcpy(msg.req_start_addr, req_addr, ETH_ADDR_LEN);
  int_to_octets(&req_count, msg.req_count, 2);
  if (confl_addr)
    memcpy(msg.confl_start_addr, confl_addr, ETH_ADDR_LEN);
  if (confl_count > 0)
    int_to_octets(&confl_count, msg.confl_count, 2);

  int ret = avb_net_send(state, ethertype_avtp, &msg, sizeof(msg), &ts);
  if (ret < 0) {
    avberr("send MAAP %s failed: %d",
           msg_type == maap_msg_type_probe     ? "probe"
           : msg_type == maap_msg_type_defend   ? "defend"
           : msg_type == maap_msg_type_announce ? "announce"
                                                : "unknown",
           errno);
  }
  return ret;
}

/* Initialize MAAP state for all output streams */
void avb_maap_init(avb_state_s *state) {
  for (int i = 0; i < state->num_output_streams; i++) {
    maap_stream_state_s *m = &state->maap[i];
    m->state = maap_state_initial;
    m->acquired = false;
    maap_generate_addr(state, &m->acquired_addr, i);
    m->probe_count = MAAP_PROBE_RETRANSMITS;
    m->timer_expiry_us =
        esp_timer_get_time() + maap_jitter_us(MAAP_PROBE_INTERVAL_BASE_MS,
                                              MAAP_PROBE_INTERVAL_VAR_MS);
    m->state = maap_state_probe;
    avb_send_maap_msg(state, maap_msg_type_probe, &m->acquired_addr, 1, NULL,
                      0);
    avbinfo("MAAP: probing %02x:%02x:%02x:%02x:%02x:%02x for stream %d",
            m->acquired_addr[0], m->acquired_addr[1], m->acquired_addr[2],
            m->acquired_addr[3], m->acquired_addr[4], m->acquired_addr[5], i);
  }
}

/* MAAP periodic tick — called from AVB main loop.
 * Handles probe retransmits and announce refresh. */
void avb_maap_tick(avb_state_s *state) {
  int64_t now = esp_timer_get_time();

  for (int i = 0; i < state->num_output_streams; i++) {
    maap_stream_state_s *m = &state->maap[i];
    if (now < m->timer_expiry_us)
      continue;

    if (m->state == maap_state_probe) {
      m->probe_count--;
      if (m->probe_count == 0) {
        /* Probing complete — address acquired */
        m->state = maap_state_defend;
        m->acquired = true;
        memcpy(state->output_streams[i].stream_dest_addr, m->acquired_addr,
               ETH_ADDR_LEN);
        avb_send_maap_msg(state, maap_msg_type_announce, &m->acquired_addr, 1,
                          NULL, 0);
        m->timer_expiry_us =
            now + maap_jitter_us(MAAP_ANNOUNCE_INTERVAL_BASE_MS,
                                 MAAP_ANNOUNCE_INTERVAL_VAR_MS);
        avbinfo("MAAP: acquired %02x:%02x:%02x:%02x:%02x:%02x for stream %d",
                m->acquired_addr[0], m->acquired_addr[1], m->acquired_addr[2],
                m->acquired_addr[3], m->acquired_addr[4], m->acquired_addr[5],
                i);
      } else {
        /* Send another probe */
        avb_send_maap_msg(state, maap_msg_type_probe, &m->acquired_addr, 1,
                          NULL, 0);
        m->timer_expiry_us =
            now + maap_jitter_us(MAAP_PROBE_INTERVAL_BASE_MS,
                                 MAAP_PROBE_INTERVAL_VAR_MS);
      }
    } else if (m->state == maap_state_defend) {
      /* Periodic announce refresh */
      avb_send_maap_msg(state, maap_msg_type_announce, &m->acquired_addr, 1,
                        NULL, 0);
      m->timer_expiry_us =
          now + maap_jitter_us(MAAP_ANNOUNCE_INTERVAL_BASE_MS,
                               MAAP_ANNOUNCE_INTERVAL_VAR_MS);
    }
  }
}

/* Process received MSRP domain message */
int avb_process_msrp_domain(avb_state_s *state, msrp_msgbuf_s *msg, int offset,
                            size_t length) {
  msrp_domain_message_s domain;
  memset(&domain, 0, sizeof(domain));
  memcpy(&domain, &msg->messages_raw[offset], sizeof(msrp_domain_message_s));

  uint16_t vid = octets_to_uint(domain.sr_class_vid, 2);
  uint16_t our_vid = octets_to_uint(state->msrp_mappings[0].vlan_id, 2);

  /* Log and validate — the network's SR class must match ours */
  if (domain.sr_class_id == 6 && vid != our_vid) {
    avberr("MSRP domain: network VLAN %d != our VLAN %d", vid, our_vid);
  }
  return OK;
}

/* Process received MSRP talker advertise message */
int avb_process_msrp_talker(avb_state_s *state, msrp_msgbuf_s *msg_data,
                            int offset, size_t length, bool is_failed,
                            eth_addr_t *src_addr) {
  msrp_talker_message_u msg;
  memset(&msg, 0, sizeof(msrp_talker_message_u));
  memcpy(&msg, &msg_data->messages_raw[offset], sizeof(msrp_talker_message_u));

  // get the talker addr from the stream id
  eth_addr_t talker_addr;
  memcpy(&talker_addr, msg.talker.info.stream_id, ETH_ADDR_LEN);

  // If the talker is known then update talker info
  int index =
      avb_find_entity_by_addr(state, &talker_addr, avb_entity_type_talker);
  if (index >= 0) {
    memcpy(&state->talkers[index].info, &msg.talker.info,
           sizeof(talker_adv_info_s));
  }

  /* Propagate the SRP-accumulated latency onto any listener input_stream
   * with a matching stream_id. This is the authoritative end-to-end
   * max_transit_time (talker origination + bridge hops), per
   * IEEE 802.1Qat / Milan §5.4. avb_start_stream_in reads it at connect
   * time to size the presentation-time startup fill.
   * Guard against a zero-stream-id malformed MSRP packet matching an
   * unconnected slot's zeroed stream_id. */
  static const uint8_t zero_stream_id[UNIQUE_ID_LEN] = {0};
  if (memcmp(msg.talker.info.stream_id, zero_stream_id, UNIQUE_ID_LEN) != 0) {
    for (int i = 0; i < AVB_MAX_NUM_INPUT_STREAMS; i++) {
      if (memcmp(state->input_streams[i].stream_id, msg.talker.info.stream_id,
                 UNIQUE_ID_LEN) == 0) {
        memcpy(state->input_streams[i].msrp_accumulated_latency,
               msg.talker.info.accumulated_latency, 4);
        break;
      }
    }
  }
  // If the talker is not known then remember it
  // else {
  //   // create a new talker entity
  //   avb_talker_s new_talker;
  //   memset(&new_talker, 0, sizeof(avb_talker_s));
  //   memcpy(&new_talker.info, &msg.talker.info, sizeof(talker_adv_info_s));
  //   // if talker list is not full, add the talker to the list
  //   if (state->num_talkers < AVB_MAX_NUM_TALKERS) {
  //     memcpy(&state->talkers[state->num_talkers], &new_talker,
  //     sizeof(avb_talker_s)); state->num_talkers++;
  //   }
  //   // if talker list is full, replace the oldest talker
  //   else {
  //     memmove(&state->talkers[0], &state->talkers[1], (state->num_talkers -
  //     1) * sizeof(avb_talker_s)); memcpy(&state->talkers[state->num_talkers -
  //     1], &new_talker, sizeof(avb_talker_s));
  //   }
  // }
  return OK;
}

/* Find a connected listener by MAC address in a stream's listener list.
 * Returns index or -1 if not found. */
static int find_connected_listener(avb_talker_stream_s *stream,
                                   eth_addr_t *mac_addr) {
  uint16_t count = octets_to_uint(stream->connection_count, 2);
  for (int i = 0; i < count && i < AVB_MAX_NUM_CONNECTED_LISTENERS; i++) {
    if (memcmp(stream->connected_listeners[i].mac_addr, mac_addr,
               ETH_ADDR_LEN) == 0) {
      return i;
    }
  }
  return -1;
}

/* Add a listener to a stream's connected_listeners list.
 * Returns index of added entry, or -1 if full. */
static int add_connected_listener(avb_talker_stream_s *stream,
                                  eth_addr_t *mac_addr) {
  uint16_t count = octets_to_uint(stream->connection_count, 2);
  if (count >= AVB_MAX_NUM_CONNECTED_LISTENERS)
    return -1;
  memcpy(stream->connected_listeners[count].mac_addr, mac_addr, ETH_ADDR_LEN);
  stream->connected_listeners[count].msrp_ready = true;
  count++;
  int_to_octets(&count, stream->connection_count, 2);
  return count - 1;
}

/* Remove a listener from a stream's connected_listeners list by index. */
static void remove_connected_listener(avb_talker_stream_s *stream, int index) {
  uint16_t count = octets_to_uint(stream->connection_count, 2);
  if (index < 0 || index >= count)
    return;
  /* Shift remaining entries down */
  for (int i = index; i < count - 1; i++) {
    stream->connected_listeners[i] = stream->connected_listeners[i + 1];
  }
  count--;
  int_to_octets(&count, stream->connection_count, 2);
  memset(&stream->connected_listeners[count], 0,
         sizeof(stream->connected_listeners[0]));
}

/* Process received MSRP listener message.
 * As a talker, this tells us a listener has declared ready/asking_failed
 * for one of our streams. Drives streaming start/stop decisions. */
int avb_process_msrp_listener(avb_state_s *state, msrp_msgbuf_s *msg,
                              int offset, size_t length,
                              eth_addr_t *src_addr) {
  msrp_listener_message_s listener_msg;
  memset(&listener_msg, 0, sizeof(listener_msg));
  memcpy(&listener_msg, &msg->messages_raw[offset],
         sizeof(msrp_listener_message_s));

  /* Decode the MRP attribute event and listener declaration */
  int attr_event = 0, unused1 = 0, unused2 = 0;
  three_pe_to_int(listener_msg.event_decl_data[0].event, &attr_event, &unused1,
                  &unused2);
  uint8_t listener_decl = listener_msg.event_decl_data[1].declaration.event1;

  /* Check if this listener declaration is for one of our output streams */
  for (int i = 0; i < state->num_output_streams; i++) {
    avb_talker_stream_s *stream = &state->output_streams[i];
    if (memcmp(listener_msg.stream_id, stream->stream_id, UNIQUE_ID_LEN) != 0)
      continue;

    /* Listener leaving — remove and possibly stop streaming */
    if (attr_event == mrp_attr_event_lv) {
      int idx = find_connected_listener(stream, src_addr);
      if (idx >= 0) {
        avbinfo("MSRP: listener leaving stream %d", i);
        remove_connected_listener(stream, idx);
        uint16_t count = octets_to_uint(stream->connection_count, 2);
        if (count == 0 && stream->streaming) {
          avb_stop_stream_out(state, i);
        }
      }
      return OK;
    }

    /* Listener ready — add if new and start streaming */
    if (listener_decl == msrp_listener_event_ready) {
      int idx = find_connected_listener(stream, src_addr);
      if (idx < 0) {
        /* New listener */
        idx = add_connected_listener(stream, src_addr);
        if (idx < 0) {
          avberr("MSRP: connected_listeners full for stream %d", i);
          return OK;
        }
        avbinfo("MSRP: listener ready for stream %d (count=%d)", i,
                octets_to_uint(stream->connection_count, 2));
        if (!stream->streaming) {
          avb_start_stream_out(state, i);
        }
      }
      /* Existing listener re-declaring — no action needed */
      return OK;
    }

    /* Listener asking_failed or ready_failed — log only */
    if (listener_decl == msrp_listener_event_asking_failed) {
      avbinfo("MSRP: listener asking_failed for stream %d", i);
    } else if (listener_decl == msrp_listener_event_ready_failed) {
      avbinfo("MSRP: listener ready_failed for stream %d", i);
    }
    return OK;
  }

  return OK;
}

/* Process received AVTP IEC 61883 message */
int avb_process_iec_61883(avb_state_s *state, iec_61883_6_message_s *msg) {
  // avbinfo("Got an AVTP IEC 61883 message");

  // check if the stream id is in the connection list and the stream is not yet
  // active
  //   int index = avb_find_connection_by_id(state, &msg->stream_id,
  //   avb_entity_type_listener); if (index >= 0 &&
  //   !state->connections[index].active) {
  //     avbinfo("Stream id found in connection list");
  //     // set the stream as active to avoid duplicate processing
  //     state->connections[index].active = true;
  //     // check if the connection has stream format info
  //     if (state->connections[index].stream.stream_format.am824.fdf_evt == 0)
  //     {
  // set the stream format in the connection
  //   state->connections[index].stream.stream_format.format = msg->format;
  //   state->connections[index].stream.stream_format.sample_rate =
  //   msg->sample_rate;
  //   state->connections[index].stream.stream_format.chan_per_frame =
  //   msg->chan_per_frame;
  //   state->connections[index].stream.stream_format.bit_depth =
  //   msg->bit_depth;
  //}
  // if the stream format, sample rate or bit depth is not supported, then
  // ignore the message if
  // (!in_array_of_int(state->connections[index].stream.stream_format.format,
  // (int *)supported_aaf_formats, ARRAY_SIZE(supported_aaf_formats)) ||
  //     !in_array_of_int(state->connections[index].stream.stream_format.sample_rate,
  //     (int *)supported_sample_rates, ARRAY_SIZE(supported_sample_rates)) ||
  //     !in_array_of_int(state->connections[index].stream.stream_format.bit_depth,
  //     (int *)supported_bit_depths, ARRAY_SIZE(supported_bit_depths))) {
  //   avberr("Unsupported stream format, sample rate or bit depth");
  //   return ERROR;
  // }
  // start the stream input task
  // avb_start_stream_in(state, &msg->stream_id);
  //}
  // not implemented
  return OK;
}

/* Process received AVTP AAF PCM message */
int avb_process_aaf(avb_state_s *state, aaf_pcm_message_s *msg) {
  // avbinfo("Got an AVTP AAF PCM message");

  // check if the stream id is in the connection list and the stream is not yet
  // active
  //   int index = avb_find_connection_by_id(state, &msg->stream_id,
  //   avb_entity_type_listener); if (index >= 0 &&
  //   !state->connections[index].active) {
  //     avbinfo("Stream id found in connection list");
  //     // set the stream as active to avoid duplicate processing
  //     state->connections[index].active = true;
  // check if the connection has stream format info
  // if (state->connections[index].stream.stream_format.bit_depth == 0) {
  //   // set the stream format in the connection
  //   state->connections[index].stream.stream_format.format = msg->format;
  //   state->connections[index].stream.stream_format.sample_rate =
  //   msg->sample_rate;
  //   state->connections[index].stream.stream_format.chan_per_frame =
  //   msg->chan_per_frame;
  //   state->connections[index].stream.stream_format.bit_depth =
  //   msg->bit_depth;
  // }
  // if the stream format, sample rate or bit depth is not supported, then
  // ignore the message if
  // (!in_array_of_int(state->connections[index].stream.stream_format.format,
  // (int *)supported_aaf_formats, ARRAY_SIZE(supported_aaf_formats)) ||
  //     !in_array_of_int(state->connections[index].stream.stream_format.sample_rate,
  //     (int *)supported_sample_rates, ARRAY_SIZE(supported_sample_rates)) ||
  //     !in_array_of_int(state->connections[index].stream.stream_format.bit_depth,
  //     (int *)supported_bit_depths, ARRAY_SIZE(supported_bit_depths))) {
  //   avberr("Unsupported stream format, sample rate or bit depth");
  //   return ERROR;
  // }
  // start the stream input task
  // avb_start_stream_in(state, &msg->stream_id);
  //}
  // not implemented
  return OK;
}

/* Process received AVTP MAAP message */
int avb_process_maap(avb_state_s *state, maap_message_s *msg) {
  for (int i = 0; i < state->num_output_streams; i++) {
    maap_stream_state_s *m = &state->maap[i];

    /* Only handle messages that conflict with our address */
    if (!maap_addr_conflicts(&m->acquired_addr, (eth_addr_t *)msg->req_start_addr))
      continue;

    switch (msg->msg_type) {
    case maap_msg_type_probe:
      /* Someone is probing our address — defend it if we own it */
      if (m->state == maap_state_defend) {
        avbinfo("MAAP: defending stream %d address against probe", i);
        avb_send_maap_msg(state, maap_msg_type_defend,
                          (eth_addr_t *)msg->req_start_addr, 1,
                          &m->acquired_addr, 1);
      } else if (m->state == maap_state_probe) {
        /* We're also probing — lower MAC yields. Compare our MAC vs sender.
         * For simplicity, restart with a new address. */
        avbinfo("MAAP: probe conflict on stream %d, selecting new address", i);
        maap_generate_addr(state, &m->acquired_addr, i);
        m->probe_count = MAAP_PROBE_RETRANSMITS;
        m->timer_expiry_us =
            esp_timer_get_time() +
            maap_jitter_us(MAAP_PROBE_INTERVAL_BASE_MS,
                           MAAP_PROBE_INTERVAL_VAR_MS);
        avb_send_maap_msg(state, maap_msg_type_probe, &m->acquired_addr, 1,
                          NULL, 0);
      }
      break;

    case maap_msg_type_defend:
      /* Our probe was defended — pick a new address */
      if (m->state == maap_state_probe) {
        avbinfo("MAAP: probe defended on stream %d, selecting new address", i);
        maap_generate_addr(state, &m->acquired_addr, i);
        m->probe_count = MAAP_PROBE_RETRANSMITS;
        m->timer_expiry_us =
            esp_timer_get_time() +
            maap_jitter_us(MAAP_PROBE_INTERVAL_BASE_MS,
                           MAAP_PROBE_INTERVAL_VAR_MS);
        avb_send_maap_msg(state, maap_msg_type_probe, &m->acquired_addr, 1,
                          NULL, 0);
      }
      break;

    case maap_msg_type_announce:
      /* Conflicting announce — yield and restart with new address */
      if (m->state == maap_state_defend) {
        avbinfo("MAAP: conflicting announce on stream %d, restarting", i);
        m->acquired = false;
        maap_generate_addr(state, &m->acquired_addr, i);
        m->state = maap_state_probe;
        m->probe_count = MAAP_PROBE_RETRANSMITS;
        m->timer_expiry_us =
            esp_timer_get_time() +
            maap_jitter_us(MAAP_PROBE_INTERVAL_BASE_MS,
                           MAAP_PROBE_INTERVAL_VAR_MS);
        avb_send_maap_msg(state, maap_msg_type_probe, &m->acquired_addr, 1,
                          NULL, 0);
      }
      break;

    default:
      break;
    }
  }
  return OK;
}

void stream_id_from_mac(eth_addr_t *mac_addr, uint8_t *stream_id, size_t uid) {
  // copy the mac address octets to the stream id and fill the remaining octects
  // with uid
  memcpy(stream_id, mac_addr, ETH_ADDR_LEN);
  memset(stream_id + ETH_ADDR_LEN, uid, UNIQUE_ID_LEN - ETH_ADDR_LEN);
}

/* Compute GCD for sine LUT sizing */
static uint32_t gcd(uint32_t a, uint32_t b) {
  while (b) {
    uint32_t t = b;
    b = a % b;
    a = t;
  }
  return a;
}

/* Build a precomputed sine wave lookup table (one exact cycle)
 *
 * Each entry is a big-endian PCM sample (all channels identical).
 * Returns the number of samples in one cycle, or 0 on allocation failure.
 * Caller must free *lut_out when done.
 */
static uint32_t build_sine_lut(uint8_t **lut_out, int channels, int bit_depth,
                               uint32_t sample_rate, float freq) {
  // One exact cycle length = sample_rate / gcd(sample_rate, freq)
  uint32_t freq_int = (uint32_t)freq;
  uint32_t cycle_samples = sample_rate / gcd(sample_rate, freq_int);

  int stride = (bit_depth == 24) ? 4 : (bit_depth / 8);
  int frame_size = stride * channels;
  uint8_t *lut = calloc(cycle_samples, frame_size);
  if (!lut) {
    *lut_out = NULL;
    return 0;
  }

  float amplitude =
      (bit_depth == 24) ? 20000.0f : 32767.0f; // quiet: match mic level (~±20k)
  float phase_inc = 2.0f * M_PI * freq / (float)sample_rate;
  float phase = 0.0f;

  for (uint32_t i = 0; i < cycle_samples; i++) {
    int32_t sample = (int32_t)(sinf(phase) * amplitude * 0.7f); // 70% amplitude

    for (int ch = 0; ch < channels; ch++) {
      int offset = (i * channels + ch) * stride;
      if (bit_depth == 24) {
        // IEEE 1722 AAF §7.3.5: 24-bit sample left-justified in 32-bit
        // big-endian container. AM824 path reads bytes 0-2 as 24-bit sample.
        lut[offset + 0] = (sample >> 16) & 0xFF; // MSB
        lut[offset + 1] = (sample >> 8) & 0xFF;
        lut[offset + 2] = (sample >> 0) & 0xFF; // LSB
        lut[offset + 3] = 0;                    // padding
      } else {
        lut[offset + 0] = (sample >> 8) & 0xFF;
        lut[offset + 1] = (sample >> 0) & 0xFF;
      }
    }
    phase += phase_inc;
    if (phase >= 2.0f * M_PI) {
      phase -= 2.0f * M_PI;
    }
  }

  *lut_out = lut;
  return cycle_samples;
}

/* Copy samples from the precomputed sine LUT into a packet buffer
 *
 * @param buf: output buffer for PCM samples
 * @param num_samples: number of samples per channel to copy
 * @param channels: number of channels
 * @param bit_depth: bits per sample (16 or 24)
 * @param lut: precomputed sine wave LUT (one cycle)
 * @param lut_samples: number of samples in the LUT
 * @param lut_pos: pointer to current LUT position (updated on return, wraps)
 */
static void copy_sine_from_lut(uint8_t *buf, int num_samples, int channels,
                               int bit_depth, const uint8_t *lut,
                               uint32_t lut_samples, uint32_t *lut_pos) {
  int stride = (bit_depth == 24) ? 4 : (bit_depth / 8);
  int frame_size = stride * channels;

  for (int i = 0; i < num_samples; i++) {
    memcpy(buf + i * frame_size, lut + (*lut_pos) * frame_size, frame_size);
    *lut_pos = (*lut_pos + 1) % lut_samples;
  }
}

/* Read PCM samples from an embedded file into a buffer
 *
 * Reads one packet's worth of samples from flash-mapped PCM data,
 * wrapping around to the beginning when the end is reached.
 *
 * @param buf: output buffer for PCM samples
 * @param num_samples: number of samples per channel to read
 * @param channels: number of channels
 * @param bit_depth: bits per sample (16 or 24)
 * @param src: pointer to embedded PCM file data (flash-mapped)
 * @param src_len: total length of embedded PCM data in bytes
 * @param offset: pointer to current read offset (updated on return, wraps)
 */
static void read_pcm_file(uint8_t *buf, int num_samples, int channels,
                          int bit_depth, const uint8_t *src, uint32_t src_len,
                          uint32_t *offset) {
  int bytes_per_sample = (bit_depth == 24) ? 4 : (bit_depth / 8);
  int frame_bytes = num_samples * channels * bytes_per_sample;
  int pos = 0;

  while (pos < frame_bytes) {
    int remaining = frame_bytes - pos;
    int available = src_len - *offset;
    int to_copy = (remaining < available) ? remaining : available;
    memcpy(buf + pos, src + *offset, to_copy);
    pos += to_copy;
    *offset += to_copy;
    if (*offset >= src_len) {
      *offset = 0;
    }
  }
}

/* Build and send an AAF PCM AVTP packet
 *
 * @param state: AVB state
 * @param stream_id: stream ID for the packet
 * @param pcm_data: raw PCM audio data
 * @param data_len: length of pcm_data in bytes
 * @param seq_num: sequence number (incremented by caller)
 * @param sample_rate: AAF sample rate enum value
 * @param channels: channels per frame
 * @param bit_depth: bit depth
 * @param dest_addr: destination MAC address
 * @param vlan_id: VLAN ID
 * @param avtp_ts: AVTP presentation timestamp (lower 32 bits of PTP ns count)
 */
static int avb_send_aaf_pcm_packet(avb_state_s *state, unique_id_t *stream_id,
                                   uint8_t *pcm_data, uint16_t data_len,
                                   uint8_t seq_num, uint8_t sample_rate_code,
                                   uint8_t channels, uint8_t bit_depth,
                                   eth_addr_t *dest_addr, uint8_t *vlan_id,
                                   uint32_t avtp_ts) {
  aaf_pcm_message_s msg;
  memset(&msg, 0, sizeof(msg));

  // Populate the AAF header
  msg.subtype = avtp_subtype_aaf;
  msg.sv = 1; // stream ID valid
  msg.version = 0;
  msg.timestamp_valid = 0;
  // IEEE 1722 §6.4: MCR=1 only on true media clock restart (stream startup),
  // not on every seq_num rollover. One-shot: set on first packet, then clear.
  static bool mcr_pending = true;
  msg.media_clock_restart = mcr_pending ? 1 : 0;
  if (mcr_pending)
    mcr_pending = false;
  msg.seq_num = seq_num;
  msg.timestamp_uncertain = 0;
  memcpy(msg.stream_id, stream_id, UNIQUE_ID_LEN);

  // Set AVTP presentation timestamp (lower 32 bits of PTP nanosecond count).
  // Add Class A max transit time (2ms) so the listener can buffer until the
  // presentation time to compensate for network jitter.
  uint32_t presentation_ts = avtp_ts + 4000000; // +4ms presentation offset
  msg.avtp_ts[0] = (presentation_ts >> 24) & 0xFF;
  msg.avtp_ts[1] = (presentation_ts >> 16) & 0xFF;
  msg.avtp_ts[2] = (presentation_ts >> 8) & 0xFF;
  msg.avtp_ts[3] = presentation_ts & 0xFF;
  msg.timestamp_valid = 1;

  msg.format = (bit_depth <= 16) ? aaf_format_int_16bit : aaf_format_int_32bit;
  msg.sample_rate = sample_rate_code;
  msg.chan_per_frame = channels;
  msg.bit_depth = bit_depth;
  msg.evt = 0; // normal
  msg.sparse_ts = 0;

  // Set stream data length
  uint16_t sdl = (data_len > AVTP_STREAM_DATA_PER_MSG)
                     ? AVTP_STREAM_DATA_PER_MSG
                     : data_len;
  msg.stream_data_len[0] = (sdl >> 8) & 0xFF;
  msg.stream_data_len[1] = sdl & 0xFF;

  // Copy PCM data into the message
  memcpy(msg.stream_data, pcm_data, sdl);

  // Calculate total message length (header + stream data)
  // AAF header is 24 bytes before stream_data
  uint16_t msg_len = sizeof(aaf_pcm_message_s) - AVTP_STREAM_DATA_PER_MSG + sdl;

  // Send to the stream destination address using the non-timestamped path.
  // The timestamped path (L2TAP_IREC_TIME_STAMP) busy-waits for DMA completion
  // while holding the EMAC transmit mutex, which blocks this real-time task
  // when PTP also transmits with timestamps.
  uint8_t eth_frame[msg_len + ETH_HEADER_LEN + sizeof(struct eth_vlan_hdr)];
  avb_create_eth_frame(eth_frame, dest_addr, state, ethertype_vlan, &msg,
                       msg_len, vlan_id);
  int ret = write(state->l2if[VLAN], eth_frame, sizeof(eth_frame));
  if (ret < 0) {
    avberr("send AAF PCM failed: %d", errno);
  }
  return ret;
}

/* Build and send an IEC 61883-6 AM824 AVTP packet
 *
 * AM824 wraps 24-bit PCM samples in IEC 61883-6 data blocks with a CIP header.
 * Each data block is 1 quadlet (4 bytes) per channel: label byte + 24-bit
 * sample. The CIP header is 2 quadlets (8 bytes) prepended to the data blocks.
 */
static int avb_send_iec_61883_packet(avb_state_s *state, unique_id_t *stream_id,
                                     uint8_t *pcm_data, uint16_t data_len,
                                     uint8_t seq_num, uint8_t dbs, uint8_t sfc,
                                     uint8_t channels, eth_addr_t *dest_addr,
                                     uint8_t *vlan_id, uint32_t avtp_ts,
                                     uint16_t dbc) {
  iec_61883_6_message_s msg;
  memset(&msg, 0, sizeof(msg));

  // Populate the IEC 61883 AVTP header
  msg.subtype = avtp_subtype_61883;
  msg.sv = 1;
  msg.version = 0;
  msg.timestamp_valid = 0;
  static bool mcr_pending_61883 = true;
  msg.media_clock_restart = mcr_pending_61883 ? 1 : 0;
  if (mcr_pending_61883)
    mcr_pending_61883 = false;
  msg.seq_num = seq_num;
  msg.timestamp_uncertain = 0;
  memcpy(msg.stream_id, stream_id, UNIQUE_ID_LEN);

  // AVTP presentation timestamp with presentation offset.
  // Class A max transit time is 2ms, but Apple CoreAudio needs extra headroom
  // for its jitter buffer. Use same offset as AAF path.
  uint32_t presentation_ts = avtp_ts + 2000000;
  msg.avtp_ts[0] = (presentation_ts >> 24) & 0xFF;
  msg.avtp_ts[1] = (presentation_ts >> 16) & 0xFF;
  msg.avtp_ts[2] = (presentation_ts >> 8) & 0xFF;
  msg.avtp_ts[3] = presentation_ts & 0xFF;
  msg.timestamp_valid = 1;

  // IEEE 1394 fields
  msg.tag = 1;      // CIP header present
  msg.channel = 31; // 31 = native AVTP (not IEEE 1394)
  msg.tcode = 0x0A; // must be 1010 on transmit
  msg.sy = 0;

  // Build CIP header (2 quadlets = 8 bytes) per IEC 61883-1 §2.3
  //
  // Quadlet 0: | EOH(1)=0 | Fmt_hi(1)=0 | SID(6)=0x3F | DBS(8) | FN(2)=0 |
  // QPC(3)=0 | SPH(1)=0 | rsv(1)=0 | DBC(8) | Quadlet 1: | EOH(1)=1 |
  // Fmt_hi(1)=0 | FMT(6)=0x10 | FDF(8)          | SYT(16)=0xFFFF          |
  //
  // SID=0x3F: no source ID info (standard for AVTP)
  // FMT=0x10: AM824 format (IEC 61883-6)
  // FDF: evt(3)=000 (AM824) | sfc(3) | reserved(2)=0
  msg.stream_data[0] = 0x3F; // EOH=0, Fmt_hi=0, SID=111111 (0x3F = no info)
  msg.stream_data[1] = dbs;  // data block size (quadlets per data block)
  msg.stream_data[2] = 0x00; // FN=00, QPC=000, SPH=0, rsv=0, DBC[7]=0
  msg.stream_data[3] = (uint8_t)(dbc & 0xFF); // DBC lower 8 bits
  msg.stream_data[4] = 0x90; // EOH=1, Fmt_hi=0, FMT=010000 (0x10 = AM824)
  msg.stream_data[5] =
      (sfc & 0x07);          // FDF: evt=00000 in bits[7:3], SFC in bits[2:0]
  msg.stream_data[6] = 0xFF; // SYT = 0xFFFF (no SYT info — use AVTP timestamp)
  msg.stream_data[7] = 0xFF;

  // Convert PCM data to AM824 format: each sample becomes a labeled quadlet
  // Label 0x40 = multi-bit linear audio (MBLA)
  // PCM data comes in big-endian 24-bit-in-32-bit format (same as AAF)
  uint16_t cip_header_len = 8;
  uint16_t pcm_offset = 0;
  uint16_t am824_offset = cip_header_len;
  int stride = 4; // 32-bit container for 24-bit audio
  int num_samples = data_len / (channels * stride);

  for (int s = 0; s < num_samples; s++) {
    for (int ch = 0; ch < channels; ch++) {
      if (am824_offset + 4 <= AVTP_STREAM_DATA_PER_MSG &&
          pcm_offset + 3 < data_len) {
        // AM824 quadlet: label(8) + sample(24), big-endian
        // PCM buffer is left-justified: [0]=MSB, [1]=MID, [2]=LSB, [3]=pad
        msg.stream_data[am824_offset + 0] = 0x40; // MBLA label
        msg.stream_data[am824_offset + 1] = pcm_data[pcm_offset + 0]; // MSB
        msg.stream_data[am824_offset + 2] = pcm_data[pcm_offset + 1];
        msg.stream_data[am824_offset + 3] = pcm_data[pcm_offset + 2]; // LSB
      }
      pcm_offset += stride;
      am824_offset += 4; // one quadlet per channel per sample
    }
  }

  // Stream data length = CIP header + AM824 data blocks
  uint16_t sdl = am824_offset;
  msg.stream_data_len[0] = (sdl >> 8) & 0xFF;
  msg.stream_data_len[1] = sdl & 0xFF;

  // Total message length
  uint16_t msg_len =
      sizeof(iec_61883_6_message_s) - AVTP_STREAM_DATA_PER_MSG + sdl;

  uint8_t eth_frame[msg_len + ETH_HEADER_LEN + sizeof(struct eth_vlan_hdr)];
  avb_create_eth_frame(eth_frame, dest_addr, state, ethertype_vlan, &msg,
                       msg_len, vlan_id);
  int ret = write(state->l2if[VLAN], eth_frame, sizeof(eth_frame));
  if (ret < 0) {
    avberr("send IEC 61883 failed: %d", errno);
  }
  return ret;
}

/* Map CIP SFC code to sample rate in Hz */
static uint32_t cip_sfc_to_sample_rate(uint8_t sfc) {
  switch (sfc) {
  case cip_sfc_sample_rate_32k:
    return 32000;
  case cip_sfc_sample_rate_44_1k:
    return 44100;
  case cip_sfc_sample_rate_48k:
    return 48000;
  case cip_sfc_sample_rate_88_2k:
    return 88200;
  case cip_sfc_sample_rate_96k:
    return 96000;
  case cip_sfc_sample_rate_176_4k:
    return 176400;
  case cip_sfc_sample_rate_192k:
    return 192000;
  default:
    return 48000;
  }
}

/* Map AAF sample rate enum to Hz */
uint32_t aaf_code_to_sample_rate(uint8_t code) {
  switch (code) {
  case aaf_pcm_sample_rate_8k:
    return 8000;
  case aaf_pcm_sample_rate_16k:
    return 16000;
  case aaf_pcm_sample_rate_24k:
    return 24000;
  case aaf_pcm_sample_rate_32k:
    return 32000;
  case aaf_pcm_sample_rate_44_1k:
    return 44100;
  case aaf_pcm_sample_rate_48k:
    return 48000;
  case aaf_pcm_sample_rate_88_2k:
    return 88200;
  case aaf_pcm_sample_rate_96k:
    return 96000;
  case aaf_pcm_sample_rate_176_4k:
    return 176400;
  case aaf_pcm_sample_rate_192k:
    return 192000;
  default:
    return 48000;
  }
}

/* Map sample rate in Hz to AAF sample rate enum */
static uint8_t sample_rate_to_aaf_code(uint32_t sample_rate) {
  switch (sample_rate) {
  case 8000:
    return aaf_pcm_sample_rate_8k;
  case 16000:
    return aaf_pcm_sample_rate_16k;
  case 24000:
    return aaf_pcm_sample_rate_24k;
  case 32000:
    return aaf_pcm_sample_rate_32k;
  case 44100:
    return aaf_pcm_sample_rate_44_1k;
  case 48000:
    return aaf_pcm_sample_rate_48k;
  case 88200:
    return aaf_pcm_sample_rate_88_2k;
  case 96000:
    return aaf_pcm_sample_rate_96k;
  case 176400:
    return aaf_pcm_sample_rate_176_4k;
  case 192000:
    return aaf_pcm_sample_rate_192k;
  default:
    return aaf_pcm_sample_rate_48k;
  }
}

/* AVB Stream output task - generates sine wave and sends as AVTP AAF stream
 *
 * This task generates a sine wave using the esp_codec_dev component (ES8311)
 * for I2S clocking, and packages the audio as AVTP AAF PCM packets sent
 * over Ethernet.
 *
 * For Class A streams: 125us intervals, 6 samples/packet at 48kHz
 * For Class B streams: 250us intervals, 12 samples/packet at 48kHz
 */
/* Convert I2S stereo 24-bit (3 bytes/sample) to multi-channel AVTP.
 * I2S Philips format on ESP32: MSB-first on wire, stored in DMA as
 * little-endian 24-bit: byte[0]=LSB, byte[1]=MID, byte[2]=MSB.
 * ES8311 is mono — L channel has mic data, R channel duplicated or silence.
 * Writes L channel data to ch0+ch1, pads remaining channels with silence.
 *
 * AM824: [0x40, MSB, MID, LSB] per channel (4 bytes)
 * AAF:   [MSB, MID, LSB, 0x00] per channel (4 bytes)
 */
static void i2s24_to_am824_mono(const uint8_t *in, uint8_t *out,
                                int num_samples, int stream_channels) {
  for (int s = 0; s < num_samples; s++) {
    /* L channel from I2S — byte[0]=LSB, byte[1]=MID, byte[2]=MSB */
    uint8_t lsb = in[0], mid = in[1], msb = in[2];
    in += 3; /* skip L */
    in += 3; /* skip R (mono codec, same or silence) */
    for (int ch = 0; ch < stream_channels; ch++) {
      if (ch < 2) {
        /* Duplicate mono mic to ch0 and ch1 */
        out[0] = 0x40;
        out[1] = msb;
        out[2] = mid;
        out[3] = lsb;
      } else {
        out[0] = 0x40;
        out[1] = 0;
        out[2] = 0;
        out[3] = 0;
      }
      out += 4;
    }
  }
}

static void i2s24_to_aaf_mono(const uint8_t *in, uint8_t *out, int num_samples,
                              int stream_channels) {
  for (int s = 0; s < num_samples; s++) {
    uint8_t lsb = in[0], mid = in[1], msb = in[2];
    in += 3; /* skip L */
    in += 3; /* skip R */
    for (int ch = 0; ch < stream_channels; ch++) {
      if (ch < 2) {
        out[0] = msb;
        out[1] = mid;
        out[2] = lsb;
        out[3] = 0;
      } else {
        out[0] = 0;
        out[1] = 0;
        out[2] = 0;
        out[3] = 0;
      }
      out += 4;
    }
  }
}

/* Sine LUT 32-bit BE [MSB,MID,LSB,pad] (4 bytes) → AM824 [0x40,MSB,MID,LSB] */
static inline void be32_to_am824(const uint8_t *in, uint8_t *out, int n) {
  for (int i = 0; i < n; i++) {
    out[0] = 0x40;
    out[1] = in[0];
    out[2] = in[1];
    out[3] = in[2];
    in += 4;
    out += 4;
  }
}

/* Sine LUT 32-bit BE [MSB,MID,LSB,pad] (4 bytes) → AAF (same layout, copy) */
static inline void be32_to_aaf(const uint8_t *in, uint8_t *out, int n) {
  memcpy(out, in, n * 4);
}

/* Frame layout constants for ETH+VLAN+AVTP */
#define TX_ETH_HDR_LEN 14  /* dst(6) + src(6) + ethertype(2) */
#define TX_VLAN_TAG_LEN 4  /* TCI(2) + inner ethertype(2) */
#define TX_AVTP_HDR_LEN 24 /* AVTP common header */
#define TX_CIP_HDR_LEN 8   /* IEC 61883 CIP header */
#define TX_HDR_LEN_AAF (TX_ETH_HDR_LEN + TX_VLAN_TAG_LEN + TX_AVTP_HDR_LEN)
#define TX_HDR_LEN_61883 (TX_HDR_LEN_AAF + TX_CIP_HDR_LEN)

static void avb_stream_out_task(void *task_param) {
  avbinfo("Starting stream out task");

  uint8_t *sine_lut = NULL;
  uint32_t lut_samples = 0;
  uint32_t lut_pos = 0;
  uint8_t *pcm_buf = NULL;
  uint8_t *i2s_buf = NULL;
  uint8_t *i2s_ring = NULL;
  uint8_t *tx_frame = NULL;
  esp_task_wdt_user_handle_t wdt_handle = NULL;
  struct stream_out_params_s *params = (struct stream_out_params_s *)task_param;
  if (params == NULL)
    goto err;

  avb_state_s *state = (avb_state_s *)params->state;
  uint8_t seq_num = 0;
  uint16_t dbc = 0;
  uint8_t sample_rate_code = sample_rate_to_aaf_code(params->sample_rate);
  bool is_am824 = (params->format_subtype == avtp_subtype_61883);

  /* I2S reads stereo (2ch) regardless of stream channel count.
   * Extra channels (ch2-7) are zero-padded in the AVTP conversion.
   * Codec configures I2S with 24-bit data / 24-bit slot = 3 bytes/sample. */
  int i2s_channels = 2;         /* ES8311 mic is stereo */
  int i2s_bytes_per_sample = 3; /* 24-bit slot */
  int i2s_read_size =
      params->samples_per_packet * i2s_channels * i2s_bytes_per_sample;

  i2s_buf = calloc(1, i2s_read_size);
  if (!i2s_buf) {
    avberr("Stream out: no memory for I2S buffer");
    goto err;
  }

  /* Sine wave fallback */
  if (params->use_sine_wave) {
    lut_samples = build_sine_lut(&sine_lut, params->channels, params->bit_depth,
                                 params->sample_rate, params->sine_freq);
    if (!sine_lut) {
      avberr("Stream out: Failed to build sine LUT");
      goto err;
    }
    int bps = (params->bit_depth == 24) ? 4 : (params->bit_depth / 8);
    pcm_buf = calloc(1, params->samples_per_packet * params->channels * bps);
    if (!pcm_buf)
      goto err;
  }

  /* Pre-build the TX frame — constant fields filled once.
   * Audio data offset depends on format (AAF vs AM824). */
  int audio_data_len = params->samples_per_packet * params->channels * 4;
  int stream_data_len =
      is_am824 ? (TX_CIP_HDR_LEN + audio_data_len) : audio_data_len;
  int audio_offset = is_am824 ? TX_HDR_LEN_61883 : TX_HDR_LEN_AAF;
  int frame_len = audio_offset + audio_data_len;
  tx_frame = calloc(1, frame_len);
  if (!tx_frame) {
    avberr("Stream out: no memory for TX frame");
    goto err;
  }

  /* ETH header: dst MAC + src MAC + 0x8100 (VLAN ethertype) */
  memcpy(tx_frame, &params->dest_addr, ETH_ADDR_LEN);
  memcpy(tx_frame + ETH_ADDR_LEN, state->internal_mac_addr, ETH_ADDR_LEN);
  tx_frame[12] = 0x81;
  tx_frame[13] = 0x00; /* VLAN ethertype */

  /* VLAN tag: PCP + VID, inner ethertype 0x22F0 */
  uint16_t vid = (params->vlan_id[0] << 8) | params->vlan_id[1];
  uint16_t pcp = state->msrp_mappings[0].priority;
  uint16_t tci = (pcp << 13) | (vid & 0x0FFF);
  tx_frame[14] = (tci >> 8) & 0xFF;
  tx_frame[15] = tci & 0xFF;
  tx_frame[16] = 0x22;
  tx_frame[17] = 0xF0; /* inner AVTP ethertype */

  /* AVTP header (starts at offset 18) */
  uint8_t *avtp = tx_frame + TX_ETH_HDR_LEN + TX_VLAN_TAG_LEN;
  avtp[0] = is_am824 ? avtp_subtype_61883 : avtp_subtype_aaf;
  avtp[1] = 0x81; /* sv=1, version=0, mr=1 (first pkt), tv=1 */
  /* avtp[2] = seq_num — set per-packet */
  avtp[3] = 0x00; /* tu=0 */
  memcpy(avtp + 4, &params->stream_id,
         UNIQUE_ID_LEN); /* stream_id at [4..11] */
  /* avtp[12..15] = avtp_ts — set per-packet */

  if (is_am824) {
    /* IEC 61883 specific fields */
    avtp[16] = 0x00;
    avtp[17] = 0x00;
    avtp[18] = 0x00;
    avtp[19] = 0x00; /* gateway */
    avtp[20] = (stream_data_len >> 8) & 0xFF;
    avtp[21] = stream_data_len & 0xFF;
    avtp[22] =
        (1 << 6) | 31;      /* tag=1 (CIP present), channel=31 (AVTP native) */
    avtp[23] = (0x0A << 4); /* tcode=1010, sy=0 */

    /* CIP header (at avtp + 24) */
    uint8_t *cip = avtp + TX_AVTP_HDR_LEN;
    cip[0] = 0x3F; /* SID=0x3F */
    cip[1] = params->dbs;
    cip[2] = 0x00; /* FN=0, QPC=0, SPH=0 */
    /* cip[3] = DBC — set per-packet */
    cip[4] = 0x90; /* EOH=1, FMT=0x10 (AM824) */
    cip[5] = params->cip_sfc & 0x07;
    cip[6] = 0xFF;
    cip[7] = 0xFF; /* SYT=0xFFFF */
  } else {
    /* AAF specific fields */
    avtp[16] = (params->bit_depth <= 16) ? 0x02 : 0x04; /* format */
    avtp[17] =
        (sample_rate_code << 4); /* sample_rate in upper nibble, padding=0 */
    /* Actually need to check AAF header layout more carefully */
    avtp[16] =
        (params->bit_depth <= 16) ? aaf_format_int_16bit : aaf_format_int_32bit;
    avtp[17] =
        (sample_rate_code & 0x0F); /* nsr in lower 4 bits, padding in upper */
    /* Rewriting properly using struct knowledge */
    avtp[16] =
        (params->bit_depth <= 16) ? aaf_format_int_16bit : aaf_format_int_32bit;
    avtp[17] =
        ((sample_rate_code & 0x0F) << 4); /* sample rate in upper nibble */
    avtp[18] = params->channels;
    avtp[19] = params->bit_depth;
    avtp[20] = (audio_data_len >> 8) & 0xFF;
    avtp[21] = audio_data_len & 0xFF;
    avtp[22] = 0x00; /* evt=0, sparse_ts=0 */
    avtp[23] = 0x00; /* reserved */
  }

  avbinfo("Stream out: rate=%luHz ch=%d samples/pkt=%d interval=%dus "
          "frame=%d bytes audio@%d %s",
          params->sample_rate, params->channels, params->samples_per_packet,
          params->interval, frame_len, audio_offset,
          params->use_sine_wave ? "sine" : "mic");

  uint32_t avtp_media_ts = 0;
  uint32_t avtp_ts_increment = (uint32_t)((uint64_t)params->samples_per_packet *
                                          1000000000ULL / params->sample_rate);

// Software PLL (unchanged)
#define PLL_MEASURE_FAST 500
#define PLL_MEASURE_SLOW 4000
#define PLL_FAST_DURATION 80000
#define PLL_SPREAD 16000
#define PLL_FILTER_SHIFT 4
#define PLL_FP_SHIFT 16
#define PLL_KI_SHIFT 8
  int64_t pll_correction_fp = 0;
  int64_t pll_frac_accum = 0;
  int64_t pll_integral_fp = 0;
  int32_t pll_offset_max = 0, pll_offset_min = 0;
  uint32_t pll_measure_count = 0, pll_skip_count = 0;

  esp_task_wdt_add_user("AVB-OUT", &wdt_handle);
  esp_log_level_set("ptpd", ESP_LOG_NONE);
  esp_log_level_set("esp.emac", ESP_LOG_NONE);

  uint32_t loop_count = 0;
  uint32_t overrun_count = 0;
  int64_t overrun_max = 0;
  uint32_t send_fail_count = 0;
  uint32_t i2s_zero_reads = 0;    /* reads that returned 0 bytes */
  uint32_t i2s_nonzero_audio = 0; /* reads with non-zero audio data */

  /* gPTP discontinuity detection — mr and tu bit management.
   * mr toggles on media clock restart, tu=1 on gPTP GM change.
   * Both held for MCR_HOLD_PACKETS after the event. */
#define MCR_HOLD_PACKETS 8
  uint8_t last_gm_id[8];
  memcpy(last_gm_id, state->ptp_status.clock_source_info.gm_id, 8);
  bool mr_state = true;         /* starts toggled (first packet = restart) */
  int mcr_hold_remaining = MCR_HOLD_PACKETS;
  bool tu_active = false;
  int tu_hold_remaining = 0;

/* I2S RX local ring — absorbs DMA buffer timing mismatch.
 * Read larger chunks when available, consume i2s_read_size per packet. */
  /* Ring size must be a multiple of i2s_read_size (bytes per AVTP packet)
   * to avoid partial-frame reads at ring wrap boundaries.  Target ~5ms. */
  int i2s_frame_bytes = i2s_channels * i2s_bytes_per_sample;
  int i2s_ring_size = (int)(params->sample_rate * i2s_frame_bytes * 5 / 1000);
  /* Round down to nearest multiple of i2s_read_size */
  i2s_ring_size -= i2s_ring_size % i2s_read_size;
  if (i2s_ring_size < i2s_read_size * 4)
    i2s_ring_size = i2s_read_size * 4; /* minimum 4 packets */
  int i2s_ring_head = 0, i2s_ring_tail = 0;
  if (!params->use_sine_wave) {
    i2s_ring = calloc(1, i2s_ring_size);
    if (!i2s_ring) {
      avberr("Stream out: no memory for I2S ring");
      goto err;
    }
    /* Pre-fill the I2S ring with blocking reads to establish buffer level.
     * I2S DMA returns in chunks larger than requested (driver adjusts
     * dma_frame_num), so we fill as much as we can. */
    int frame_size = i2s_channels * i2s_bytes_per_sample;
    int prefill_target = i2s_ring_size / 2; /* ~2.5ms of audio */
    while (i2s_ring_head < prefill_target) {
      int write_pos = i2s_ring_head % i2s_ring_size;
      int space = i2s_ring_size - i2s_ring_head;
      int chunk = i2s_ring_size - write_pos;
      if (chunk > space)
        chunk = space;
      /* Align to frame boundary to prevent partial-frame reads */
      chunk -= chunk % frame_size;
      if (chunk == 0)
        break;
      size_t got = 0;
      i2s_channel_read(params->i2s_rx_handle, i2s_ring + write_pos, chunk, &got,
                       100);
      got -= got % frame_size; /* discard any trailing partial frame */
      if (got == 0)
        break; /* timeout — give up pre-fill */
      i2s_ring_head += got;
    }
    avbinfo("Stream out: I2S ring pre-filled %d bytes", i2s_ring_head);
  }

  /* Sample PTP and send-time as late as possible — all logging and pre-fill
   * is done.  This ensures the first packet's presentation timestamp is
   * accurate rather than stale by the pre-fill duration.
   *
   * Stagger initial send phase by a device-unique offset derived from the MAC.
   * Without this, PTP-synchronized ESPs transmit in lockstep (same 125μs
   * phase), causing systematic DMA contention between TX and RX on the
   * receiving side — both fire at the same instant every interval. */
  uint8_t mac_hash = state->internal_mac_addr[4] ^ state->internal_mac_addr[5];
  int64_t phase_offset = (mac_hash % params->interval);
  int64_t next_send_time = esp_timer_get_time() + phase_offset;
  for (int init_try = 0; init_try < 5; init_try++) {
    struct timespec ptp_a, ptp_b;
    if (clock_gettime(CLOCK_PTP_SYSTEM, &ptp_a) == 0 &&
        clock_gettime(CLOCK_PTP_SYSTEM, &ptp_b) == 0) {
      uint32_t ts_a = (uint32_t)((uint64_t)ptp_a.tv_sec * 1000000000ULL +
                                 (uint64_t)ptp_a.tv_nsec);
      uint32_t ts_b = (uint32_t)((uint64_t)ptp_b.tv_sec * 1000000000ULL +
                                 (uint64_t)ptp_b.tv_nsec);
      int32_t diff = (int32_t)(ts_b - ts_a);
      if (diff >= 0 && diff < 500000) {
        avtp_media_ts = ts_a;
        break;
      }
    }
  }

  while (!state->output_streams[params->stream_index].stop_streaming) {
    /* Busy-wait until next send time */
    while (esp_timer_get_time() < next_send_time) {
    }

    int64_t now = esp_timer_get_time();
    int64_t overrun = now - next_send_time;
    if (overrun > params->interval / 2) {
      overrun_count++;
      if (overrun > overrun_max)
        overrun_max = overrun;
    }
    if (overrun > params->interval * 10) {
      next_send_time = now + params->interval;
    } else {
      next_send_time += params->interval;
    }

    /* Advance AVTP media clock with PLL correction */
    pll_frac_accum += pll_correction_fp;
    int32_t correction_ns = (int32_t)(pll_frac_accum >> PLL_FP_SHIFT);
    pll_frac_accum -= (int64_t)correction_ns << PLL_FP_SHIFT;
    avtp_media_ts += (uint32_t)((int32_t)avtp_ts_increment + correction_ns);

    /* Check for gPTP grandmaster change every ~1 second (8000 packets at 125us) */
    if ((loop_count & 0x1FFF) == 0 && loop_count > 0) {
      if (memcmp(last_gm_id, state->ptp_status.clock_source_info.gm_id, 8) != 0) {
        memcpy(last_gm_id, state->ptp_status.clock_source_info.gm_id, 8);
        mr_state = !mr_state;
        mcr_hold_remaining = MCR_HOLD_PACKETS;
        tu_active = true;
        tu_hold_remaining = MCR_HOLD_PACKETS;
      }
    }

    /* Update per-packet fields in tx_frame.
     * sv=1 always, tv=1 always, mr per state, tu per gPTP status. */
    uint8_t hdr1 = 0x81; /* sv=1, version=0, mr=0, tv=1 */
    if (mcr_hold_remaining > 0) {
      if (mr_state)
        hdr1 |= 0x08; /* mr=1 */
      mcr_hold_remaining--;
    }
    avtp[1] = hdr1;
    avtp[3] = tu_active ? 0x01 : 0x00;
    if (tu_hold_remaining > 0) {
      tu_hold_remaining--;
      if (tu_hold_remaining == 0)
        tu_active = false;
    }
    avtp[2] = seq_num++;
    uint32_t presentation_ts = avtp_media_ts + (is_am824 ? 2000000 : 4000000);
    avtp[12] = (presentation_ts >> 24) & 0xFF;
    avtp[13] = (presentation_ts >> 16) & 0xFF;
    avtp[14] = (presentation_ts >> 8) & 0xFF;
    avtp[15] = presentation_ts & 0xFF;

    if (is_am824) {
      /* Update DBC in CIP header */
      uint8_t *cip = avtp + TX_AVTP_HDR_LEN;
      cip[3] = (uint8_t)(dbc & 0xFF);
      dbc += params->samples_per_packet;
    }

    /* Get audio data and convert directly into tx_frame */
    uint8_t *audio_dst = tx_frame + audio_offset;
    int total_samples = params->samples_per_packet * params->channels;

    if (params->use_sine_wave) {
      copy_sine_from_lut(pcm_buf, params->samples_per_packet, params->channels,
                         params->bit_depth, sine_lut, lut_samples, &lut_pos);
      if (is_am824)
        be32_to_am824(pcm_buf, audio_dst, total_samples);
      else
        be32_to_aaf(pcm_buf, audio_dst, total_samples);
    } else {
      /* Read mic audio via local ring — refill from I2S when low,
       * consume i2s_read_size bytes per packet */
      int ring_avail = i2s_ring_head - i2s_ring_tail;
      if (ring_avail < i2s_read_size) {
        /* Refill: read as much as possible from I2S into ring */
        int ring_space = i2s_ring_size - ring_avail;
        int write_pos = i2s_ring_head % i2s_ring_size;
        int chunk = i2s_ring_size - write_pos; /* to end of buffer */
        if (chunk > ring_space)
          chunk = ring_space;
        size_t bytes_read = 0;
        /* Use short timeout — I2S DMA buffers arrive periodically.
         * If no data now, the ring has buffered audio from previous fills. */
        /* Align chunk to frame boundary (6 bytes = 1 stereo 24-bit frame)
         * to prevent partial-frame reads that desync the ring buffer */
        chunk -= chunk % (i2s_channels * i2s_bytes_per_sample);
        if (chunk == 0)
          goto skip_refill;
        i2s_channel_read(params->i2s_rx_handle, i2s_ring + write_pos, chunk,
                         &bytes_read, 0);
        /* Discard any trailing partial frame the driver might return */
        bytes_read -= bytes_read % (i2s_channels * i2s_bytes_per_sample);
        if (bytes_read > 0) {
          i2s_ring_head += bytes_read;
          i2s_nonzero_audio++;
        } else {
          i2s_zero_reads++;
        }
        ring_avail = i2s_ring_head - i2s_ring_tail;
      }
      skip_refill:
      /* Consume i2s_read_size bytes from ring */
      if (ring_avail >= i2s_read_size) {
        int read_pos = i2s_ring_tail % i2s_ring_size;
        int first = i2s_ring_size - read_pos;
        if (first >= i2s_read_size) {
          memcpy(i2s_buf, i2s_ring + read_pos, i2s_read_size);
        } else {
          memcpy(i2s_buf, i2s_ring + read_pos, first);
          memcpy(i2s_buf + first, i2s_ring, i2s_read_size - first);
        }
        i2s_ring_tail += i2s_read_size;
      } else {
        memset(i2s_buf, 0, i2s_read_size); /* underrun — silence */
        i2s_zero_reads++;
      }
      if (is_am824)
        i2s24_to_am824_mono(i2s_buf, audio_dst, params->samples_per_packet,
                            params->channels);
      else
        i2s24_to_aaf_mono(i2s_buf, audio_dst, params->samples_per_packet,
                          params->channels);
    } /* end else (mic path) */

    /* Transmit directly via EMAC — single copy into DMA ring */
    if (esp_eth_transmit(params->eth_handle, tx_frame, frame_len) != ESP_OK) {
      send_fail_count++;
    }

    loop_count++;

    /* Feed WDT every ~125ms */
    if (wdt_handle && loop_count % 1000 == 0) {
      esp_task_wdt_reset_user(wdt_handle);
    }

    /* Software PLL (unchanged) */
#if !PLL_DISABLED
    uint32_t pll_interval =
        (loop_count < PLL_FAST_DURATION) ? PLL_MEASURE_FAST : PLL_MEASURE_SLOW;
    if (loop_count % pll_interval == 0) {
      struct timespec ptp_now;
      if (clock_gettime(CLOCK_PTP_SYSTEM, &ptp_now) == 0) {
        uint32_t ptp_now_ts =
            (uint32_t)((uint64_t)ptp_now.tv_sec * 1000000000ULL +
                       (uint64_t)ptp_now.tv_nsec);
        struct timespec ptp_check;
        if (clock_gettime(CLOCK_PTP_SYSTEM, &ptp_check) == 0) {
          uint32_t check_ts =
              (uint32_t)((uint64_t)ptp_check.tv_sec * 1000000000ULL +
                         (uint64_t)ptp_check.tv_nsec);
          int32_t read_diff = (int32_t)(check_ts - ptp_now_ts);
          if (read_diff >= 0 && read_diff < 500000) {
            int32_t offset = (int32_t)(ptp_now_ts - avtp_media_ts);
            pll_measure_count++;
            if (offset > pll_offset_max)
              pll_offset_max = offset;
            if (offset < pll_offset_min)
              pll_offset_min = offset;
#define PLL_OUTLIER_NS 50000000
            if (offset > PLL_OUTLIER_NS || offset < -PLL_OUTLIER_NS) {
              pll_skip_count++;
              goto pll_skip;
            }
            int64_t prop_corr_fp =
                ((int64_t)offset << PLL_FP_SHIFT) / PLL_SPREAD;
            pll_integral_fp += ((int64_t)offset << PLL_FP_SHIFT) /
                               (PLL_SPREAD << PLL_KI_SHIFT);
            int64_t integral_max = (int64_t)125 << PLL_FP_SHIFT;
            if (pll_integral_fp > integral_max)
              pll_integral_fp = integral_max;
            if (pll_integral_fp < -integral_max)
              pll_integral_fp = -integral_max;
            int64_t new_corr_fp = prop_corr_fp + pll_integral_fp;
            int64_t filter_n = (1 << PLL_FILTER_SHIFT);
            pll_correction_fp =
                (pll_correction_fp * (filter_n - 1) + new_corr_fp) / filter_n;
          pll_skip:
            (void)0;
          }
        }
      }
    }
#endif
  }

  esp_log_level_set("*", ESP_LOG_INFO);
  avbinfo("Stream out stopped: %lu pkts, %lu fails, %lu overruns (max %lldus), "
          "i2s: %lu zero_reads, %lu nonzero_audio",
          loop_count, send_fail_count, overrun_count, overrun_max,
          i2s_zero_reads, i2s_nonzero_audio);
  avbinfo("PLL: %lu measures, %lu skipped, offset [%ldns, %ldns]",
          pll_measure_count, pll_skip_count, (long)pll_offset_min,
          (long)pll_offset_max);

err:
  esp_log_level_set("*", ESP_LOG_INFO);
  if (wdt_handle)
    esp_task_wdt_delete_user(wdt_handle);
  free(sine_lut);
  free(pcm_buf);
  free(i2s_buf);
  free(i2s_ring);
  free(tx_frame);
  free(params);
  vTaskDelete(NULL);
}

/****************************************************************************
 * Stream Input (Listener) — Jitter-buffered AVTP → I2S
 *
 * Architecture:
 *   EMAC RX handler → ring_write (non-blocking)
 *   esp_timer 1ms   → ring_read → i2s_channel_write
 *
 * The SPSC lock-free ring buffer decouples bursty EMAC packet arrival
 * from steady I2S DMA consumption. Milan-compliant: buffers ≥2.126ms.
 ****************************************************************************/

#include <stdatomic.h>

/* Time-scheduled packet queue for presentation-time rendering
 * (IEEE 1722 §AAF + Milan v1.3 §7.2): each received packet is stored
 * with its avtp_timestamp; the drain emits the packet whose
 * presentation time has come rather than in arrival order.
 *
 * Producer: AAF RX handler (EMAC task).
 * Consumer: drain callback (esp_timer task).
 * Single-producer / single-consumer ring of fixed-size packet slots.
 *
 * Class-agnostic: queue depth is sized for Class B's 50 ms
 * max_transit_time plus a safety margin, so the same queue works for
 * both Class A (2 ms presentation offset) and Class B (50 ms) without
 * recompilation. Packet duration and byte count are bounded per-slot
 * rather than hard-coded so multi-sample-rate or larger-packet formats
 * work too. */
#define AAF_MAX_PACKET_SAMPLES 64  /* generous upper bound for any AAF fmt */
#define AAF_MAX_PACKET_BYTES   (AAF_MAX_PACKET_SAMPLES * 2 /*ch*/ * 3 /*bytes*/)
#define AAF_DEFAULT_PACKET_DURATION_NS 125000 /* 6 sa @ 48 kHz — updated runtime */

/* Queue depth: covers Class B's 50 ms of in-flight presentation offset
 * at the worst-case AAF packet rate (8000 pps for 6-sample frames).
 * 50 ms × 8 pkts/ms = 400 → round up to next power of two for masking. */
#define AAF_QUEUE_CAPACITY     512

typedef struct {
  uint32_t presentation_ns; /* avtp_timestamp (lower 32 b of gPTP ns) */
  uint16_t pcm_len;         /* actual bytes in pcm_data, ≤ AAF_MAX_PACKET_BYTES */
  uint8_t  pcm_data[AAF_MAX_PACKET_BYTES]; /* converted stereo 24-bit samples */
} aaf_packet_slot_t;

typedef struct {
  aaf_packet_slot_t slots[AAF_QUEUE_CAPACITY];
  _Atomic uint32_t head; /* next write index (producer only) */
  _Atomic uint32_t tail; /* next read index (consumer only) */
} aaf_packet_queue_t;

static inline uint32_t aaf_queue_fill(const aaf_packet_queue_t *queue) {
  return atomic_load_explicit(&queue->head, memory_order_acquire) -
         atomic_load_explicit(&queue->tail, memory_order_relaxed);
}

/* Push a packet. Returns true on success, false if queue is full. */
static inline bool aaf_queue_push(aaf_packet_queue_t *queue,
                                  uint32_t presentation_ns,
                                  const uint8_t *pcm, uint16_t pcm_len) {
  if (aaf_queue_fill(queue) >= AAF_QUEUE_CAPACITY)
    return false;
  uint32_t head = atomic_load_explicit(&queue->head, memory_order_relaxed);
  aaf_packet_slot_t *slot = &queue->slots[head & (AAF_QUEUE_CAPACITY - 1)];
  slot->presentation_ns = presentation_ns;
  if (pcm_len > AAF_MAX_PACKET_BYTES)
    pcm_len = AAF_MAX_PACKET_BYTES;
  slot->pcm_len = pcm_len;
  memcpy(slot->pcm_data, pcm, pcm_len);
  atomic_store_explicit(&queue->head, head + 1, memory_order_release);
  return true;
}

/* Peek at the next packet due for consumption (oldest by arrival = oldest
 * by presentation_ns when packets arrive in order). Returns NULL if empty. */
static inline const aaf_packet_slot_t *
aaf_queue_peek(const aaf_packet_queue_t *queue) {
  if (aaf_queue_fill(queue) == 0)
    return NULL;
  uint32_t tail = atomic_load_explicit(&queue->tail, memory_order_relaxed);
  return &queue->slots[tail & (AAF_QUEUE_CAPACITY - 1)];
}

static inline void aaf_queue_pop(aaf_packet_queue_t *queue) {
  uint32_t tail = atomic_load_explicit(&queue->tail, memory_order_relaxed);
  atomic_store_explicit(&queue->tail, tail + 1, memory_order_release);
}

/* Stream RX handler context — file-static, accessed by EMAC task and
 * I2S on_sent ISR. Allocated by avb_start_stream_in. */
typedef struct {
  avb_state_s *state; /* for media_clock stats update */
  i2s_chan_handle_t i2s_tx_handle;
  aaf_packet_queue_t queue;
  uint8_t expected_stream_id[UNIQUE_ID_LEN]; /* filter: only accept this stream */
  /* Format-derived runtime constants (updated on first packet so the drain
   * stays class- and format-agnostic). */
  uint32_t packet_duration_ns;
  uint16_t packet_bytes;   /* stereo 24-bit bytes per packet */
  /* Presentation-time anchoring (Milan §5.3/§7.3). The drain holds off
   * until the queue has accumulated startup_fill_target packets — that
   * fill depth then becomes the listener-internal delay, giving a
   * *deterministic* offset between avtp_timestamp and DAC output so the
   * relative timing of samples is preserved. Rate-matched PLL (gap 2)
   * keeps the depth constant; only a gPTP discontinuity or real packet
   * loss can perturb it, at which point the next stream connect will
   * re-anchor. */
  uint16_t startup_fill_target;  /* packets to queue before draining */
  _Atomic bool drain_enabled;    /* ISR-safe flag: false until fill hit */
  bool drain_enabled_logged;     /* main loop has logged the transition */
  /* Partial-packet cursor — carries across on_sent events when a DMA
   * descriptor ends mid-AAF-packet. Bytes already copied out of the
   * current queue-head slot; 0 means "start of a fresh packet". Only
   * the on_sent ISR touches this. */
  uint16_t partial_bytes_drained;
  /* diagnostics */
  uint32_t pkt_count;          /* total AAF packets received */
  uint32_t queue_full_drops;   /* packet arrivals refused because queue full */
  uint32_t silence_inserts;    /* drain events where queue was empty */
  uint32_t on_time_emits;      /* drain events that emitted a real packet */
  uint32_t drain_count;        /* total on_sent events */
  uint32_t stream_id_mismatch; /* packets dropped due to wrong stream_id */
  uint32_t seq_num_mismatch;   /* sequence number gaps detected */
  uint8_t last_seq_num;        /* last received sequence number */
  bool seq_num_valid;          /* false until first packet received */
  int64_t drain_enabled_at_us; /* esp_timer_get_time when drain turned on */
  /* Milan GET_COUNTERS state (Table 5.13). All monotonic 32-bit. */
  uint32_t media_locked_count;     /* drain transitioned into locked state */
  uint32_t media_unlocked_count;   /* lost lock due to packet-arrival timeout */
  uint32_t ts_uncertain_count;     /* AVTPDU with tu=1 received */
  uint32_t late_ts_count;          /* avtp_timestamp already past at RX */
  uint32_t early_ts_count;         /* avtp_timestamp too far in future at RX */
  int64_t last_packet_us;          /* esp_timer at most recent packet RX */
  bool ever_locked;                /* first-lock gate for unlock detection */
  /* first-packet snapshot (written by handler, printed by main loop) */
  uint8_t diag_subtype;
  uint16_t diag_sdl;
  uint8_t diag_channels;
  uint8_t diag_samples;
  uint16_t diag_i2s_bytes;
  uint8_t diag_first_audio[8];
  uint8_t diag_captured; /* 0=waiting, 1=pending print, 2=printed */
} stream_rx_ctx_t;

static stream_rx_ctx_t *s_stream_rx_ctx = NULL;

/* CRF stream RX context — Milan v1.3 CRF media clock input. Allocated by
 * avb_start_stream_in when index == AVB_CRF_INPUT_INDEX. Unlike the AAF
 * context this has no I2S/jitter-buffer machinery, just packet reception
 * state for Phase 2 media-clock recovery. */
typedef struct {
  avb_state_s *state; /* for media_clock stats update */
  uint8_t expected_stream_id[UNIQUE_ID_LEN]; /* filter: CRF talker's stream_id */
  uint32_t pkt_count;
  uint32_t stream_id_mismatch;
  uint32_t seq_num_mismatch;
  uint8_t last_seq_num;
  bool seq_num_valid;
  /* Last CRF timestamp observed (IEEE 1722-2016 §10, 64-bit gPTP ns).
   * Reserved for Phase 2 PLL. */
  uint64_t last_timestamp_ns;
  uint32_t timestamp_count;
} crf_rx_ctx_t;

static crf_rx_ctx_t *s_crf_rx_ctx = NULL;

/* I2S on_sent callback — DMA-event-driven drain.
 *
 * Fires from I2S ISR every time a DMA descriptor finishes transmitting
 * (one descriptor = one AAF packet = 125 µs at 48 kHz / 6 frames). The
 * rate therefore tracks the APLL-tuned DAC clock exactly, so feeding
 * one packet per event keeps the DMA ring full without either overfill
 * or underrun — and i2s_bytes_written now measures actual DAC rate,
 * which is what the PLL needs to close the loop.
 *
 * The channel is configured with auto_clear_before_cb=true so
 * event->dma_buf arrives pre-zeroed; a short packet or empty queue
 * automatically becomes silence without us having to write anything.
 *
 * Registered once at i2s init (before channel_enable, which is a
 * precondition of i2s_channel_register_event_callback). The active
 * stream context is looked up via the file-static s_stream_rx_ctx so
 * listener start/stop doesn't touch the i2s callback registration.
 *
 * ISR restrictions apply: no blocking calls, no float, keep it short.
 * CONFIG_I2S_ISR_IRAM_SAFE is not set so flash code is acceptable. */
static bool IRAM_ATTR stream_in_i2s_sent_cb(i2s_chan_handle_t handle,
                                             i2s_event_data_t *event,
                                             void *user_ctx) {
  (void)handle;
  avb_state_s *state = (avb_state_s *)user_ctx;
  stream_rx_ctx_t *ctx = s_stream_rx_ctx;

  if (ctx) {
    ctx->drain_count++;

    /* Startup-fill gate: hold the drain until the queue has reached
     * the target depth. The resulting buffer level becomes the
     * deterministic listener-internal offset from avtp_timestamp to
     * DAC output — identical on every connect, preserved forever by
     * the rate-matched PLL. While waiting, the buffer stays zero
     * (auto_clear_before_cb), so the DAC emits silence. */
    if (!atomic_load_explicit(&ctx->drain_enabled, memory_order_acquire)) {
      if (aaf_queue_fill(&ctx->queue) >= ctx->startup_fill_target) {
        atomic_store_explicit(&ctx->drain_enabled, true, memory_order_release);
        ctx->drain_enabled_at_us = esp_timer_get_time();
        ctx->media_locked_count++; /* Milan media_locked counter */
      } else {
        ctx->silence_inserts++;
        goto pll_accounting;
      }
    }

    /* The DMA descriptor size is cache-line-aligned (192 B = 32 frames
     * at stereo 24-bit) which is not a whole number of AAF packets
     * (36 B each). So AAF packets straddle descriptor boundaries and
     * we carry a partial cursor across on_sent events:
     *   - start each event by finishing whatever partial packet from
     *     the queue head we had left over last time;
     *   - then copy whole packets while they fit;
     *   - if the last one only partially fits, copy its prefix and
     *     remember how far we got so the next event finishes it. */
    uint8_t *dst = (uint8_t *)event->dma_buf;
    size_t remaining = event->size;
    bool emitted_any = false;
    while (remaining > 0) {
      const aaf_packet_slot_t *slot = aaf_queue_peek(&ctx->queue);
      if (!slot)
        break;
      uint16_t bytes_left_in_slot =
          slot->pcm_len - ctx->partial_bytes_drained;
      size_t to_copy =
          (bytes_left_in_slot <= remaining) ? bytes_left_in_slot : remaining;
      memcpy(dst, slot->pcm_data + ctx->partial_bytes_drained, to_copy);
      dst += to_copy;
      remaining -= to_copy;
      ctx->partial_bytes_drained += to_copy;
      emitted_any = true;
      if (ctx->partial_bytes_drained >= slot->pcm_len) {
        /* Finished this packet — pop and move to the next. */
        aaf_queue_pop(&ctx->queue);
        ctx->partial_bytes_drained = 0;
        ctx->on_time_emits++;
      }
      /* If we didn't finish the packet, remaining must now be 0 and we
       * fall out of the loop naturally on the next iteration's check. */
    }
    if (!emitted_any)
      ctx->silence_inserts++;
  }
pll_accounting:;

  /* PLL counter: bytes about to be sent to the DAC on this descriptor.
   * Cadence now equals the APLL-tuned DAC rate, closing the PLL loop.
   * Updated even when no listener stream is active so the PLL has a
   * valid baseline if/when a stream connects. */
  if (state) {
    atomic_fetch_add_explicit(&state->media_clock.i2s_bytes_written,
                              (uint64_t)event->size, memory_order_relaxed);
  }
  return false;
}

/* Register the on_sent callback on the TX channel. Must be called
 * between i2s_channel_init_std_mode and i2s_channel_enable — the
 * driver rejects callback registration while the channel is running. */
int avb_stream_in_register_i2s_cb(avb_state_s *state) {
  i2s_event_callbacks_t cbs = { .on_sent = stream_in_i2s_sent_cb };
  return i2s_channel_register_event_callback(state->i2s_tx_handle, &cbs,
                                              state) == ESP_OK ? OK : ERROR;
}

/* CRF stream RX handler — counts CRF AVTPDUs and records the most recent
 * media-clock timestamp. Called inline from EMAC RX task via the
 * dispatcher; must return quickly. Phase 2 will use the collected
 * timestamps for a media-clock PLL. */
static void avb_crf_rx_handler(uint8_t *avtp_data, uint16_t len,
                               crf_rx_ctx_t *ctx) {
  if (!ctx || len < 28)
    return;

  ctx->pkt_count++;

  /* Track sequence number gaps (avtp_data[2] = sequence_num) */
  uint8_t seq_num = avtp_data[2];
  if (ctx->seq_num_valid) {
    uint8_t expected = ctx->last_seq_num + 1;
    if (seq_num != expected)
      ctx->seq_num_mismatch++;
  }
  ctx->last_seq_num = seq_num;
  ctx->seq_num_valid = true;

  /* CRF AVTPDU layout (IEEE 1722-2016 §10.4, Figure 13):
   *   bytes 0-3:   subtype, sv|version|mr, seq_num, type_specific
   *   bytes 4-11:  stream_id
   *   bytes 12-15: pull(3) | base_frequency(29)
   *   bytes 16-17: crf_data_length
   *   bytes 18-19: timestamp_interval
   *   bytes 20+:   CRF timestamps, 8 bytes each (big-endian uint64 ns)
   * For Milan's 1-timestamp-per-PDU format the 8 bytes at offset 20 hold
   * the single sample-time timestamp. */
  if (len < 28)
    return;
  uint64_t timestamp_ns = 0;
  for (int i = 0; i < 8; i++)
    timestamp_ns = (timestamp_ns << 8) | avtp_data[20 + i];
  ctx->last_timestamp_ns = timestamp_ns;
  ctx->timestamp_count++;

  /* Publish a (crf_ts, bytes_written) anchor for the PLL (seqlock).
   * Done before the drift log below so the snapshot captures
   * bytes_written as close to CRF arrival as possible. */
  if (ctx->state) {
    avb_state_s *state = ctx->state;
    uint64_t bytes_snapshot = atomic_load_explicit(
        &state->media_clock.i2s_bytes_written, memory_order_acquire);
    uint32_t seq = atomic_load_explicit(
        &state->media_clock.crf_anchor.seq, memory_order_relaxed);
    atomic_store_explicit(&state->media_clock.crf_anchor.seq, seq + 1,
                          memory_order_release); /* odd: write in progress */
    state->media_clock.crf_anchor.ts_ns = timestamp_ns;
    state->media_clock.crf_anchor.bytes = bytes_snapshot;
    atomic_store_explicit(&state->media_clock.crf_anchor.seq, seq + 2,
                          memory_order_release); /* even: complete */
  }

  /* Compute drift vs local gPTP for diagnostic log output */
  struct timespec now;
  if (ctx->state && clock_gettime(CLOCK_PTP_SYSTEM, &now) == 0) {
    uint64_t local_ns =
        (uint64_t)now.tv_sec * 1000000000ULL + (uint64_t)now.tv_nsec;
    int64_t drift = (int64_t)(timestamp_ns - local_ns);
    avb_state_s *state = ctx->state;
    state->media_clock.crf_last_drift_ns = drift;
    state->media_clock.crf_drift_sum_ns += drift;
    state->media_clock.crf_samples++;
    if (state->media_clock.crf_samples == 1 ||
        drift < state->media_clock.crf_drift_min_ns) {
      state->media_clock.crf_drift_min_ns = (int32_t)drift;
    }
    if (state->media_clock.crf_samples == 1 ||
        drift > state->media_clock.crf_drift_max_ns) {
      state->media_clock.crf_drift_max_ns = (int32_t)drift;
    }
  }
}

/* Stream RX handler — called inline from EMAC RX task for each
 * VLAN-tagged AVTP packet. Parses AVTP, converts audio to 24-bit
 * stereo, writes to jitter ring buffer. Must return quickly. */
static void avb_stream_rx_handler(uint8_t *avtp_data, uint16_t len, void *arg) {
  stream_rx_ctx_t *ctx = (stream_rx_ctx_t *)arg;
  if (!ctx || len < 24)
    return;

  /* Filter by stream_id — only accept packets from the connected talker.
   * Stream_id is at AVTP header bytes [4..11] for both AAF and IEC 61883. */
  if (memcmp(avtp_data + 4, ctx->expected_stream_id, UNIQUE_ID_LEN) != 0) {
    ctx->stream_id_mismatch++;
    return;
  }

  /* Track sequence number gaps (avtp_data[2] = sequence_num) */
  uint8_t seq_num = avtp_data[2];
  if (ctx->seq_num_valid) {
    uint8_t expected = ctx->last_seq_num + 1;
    if (seq_num != expected)
      ctx->seq_num_mismatch++;
  }
  ctx->last_seq_num = seq_num;
  ctx->seq_num_valid = true;

  /* Timestamp-uncertain (tu) bit — IEEE 1722-2016 §5.3, byte 3 bit 0.
   * When the talker sets tu=1 it's declaring the avtp_timestamp not
   * reliable (e.g. during gPTP re-lock). Milan counts these so a
   * controller can see that the stream is passing but timestamps
   * shouldn't be trusted. */
  if (avtp_data[3] & 0x01)
    ctx->ts_uncertain_count++;

  /* Packet-arrival timestamp for lock / unlock detection. */
  ctx->last_packet_us = esp_timer_get_time();

  /* Sample drift (avtp_timestamp vs local gPTP) every 10th packet —
   * AAF arrives at 8000/s and clock_gettime(CLOCK_PTP_SYSTEM) is
   * contended internally, so calling it on every packet burns real CPU
   * in the EMAC RX task and can starve the watchdog. 800/s is plenty
   * for averaging stats. */
  if (ctx->state && (ctx->pkt_count % 10) == 0) {
    struct timespec now;
    uint32_t avtp_timestamp =
        ((uint32_t)avtp_data[12] << 24) | ((uint32_t)avtp_data[13] << 16) |
        ((uint32_t)avtp_data[14] << 8) | (uint32_t)avtp_data[15];
    if (clock_gettime(CLOCK_PTP_SYSTEM, &now) == 0) {
      uint64_t now_ns = (uint64_t)now.tv_sec * 1000000000ULL +
                        (uint64_t)now.tv_nsec;
      uint32_t local_timestamp = (uint32_t)now_ns;
      int32_t drift = (int32_t)(avtp_timestamp - local_timestamp);
      avb_state_s *state = ctx->state;
      state->media_clock.aaf_last_drift_ns = drift;
      state->media_clock.aaf_drift_sum_ns += drift;
      state->media_clock.aaf_samples++;
      if (state->media_clock.aaf_samples == 1 ||
          drift < state->media_clock.aaf_drift_min_ns) {
        state->media_clock.aaf_drift_min_ns = drift;
      }
      if (state->media_clock.aaf_samples == 1 ||
          drift > state->media_clock.aaf_drift_max_ns) {
        state->media_clock.aaf_drift_max_ns = drift;
      }

      /* Milan late_ts / early_ts counters. drift is the signed
       * (avtp_ts − now) difference: negative = ts has already passed,
       * positive large = ts is unexpectedly far in the future. Sampled
       * at the same 1-in-10 rate as the drift stats; absolute counts
       * are therefore approximate, but the *trend* relative to
       * frames_rx still tells a controller whether the stream is
       * timing-healthy. */
      if (drift < 0)
        ctx->late_ts_count++;
      else if (drift > 50 * 1000 * 1000) /* 50 ms in future = anomalous */
        ctx->early_ts_count++;
    }
  }

  uint8_t subtype = avtp_data[0] & 0x7F;
  uint8_t *pcm_data = NULL;
  int pcm_len = 0;
  int channels = 0;
  int samples = 0;

  if (subtype == avtp_subtype_aaf) {
    aaf_pcm_message_s *aaf_msg = (aaf_pcm_message_s *)avtp_data;
    uint16_t stream_data_len =
        (aaf_msg->stream_data_len[0] << 8) | aaf_msg->stream_data_len[1];
    if (stream_data_len == 0 || stream_data_len > AVTP_STREAM_DATA_PER_MSG)
      return;
    channels = aaf_msg->chan_per_frame;
    if (channels == 0)
      channels = 8;
    samples = stream_data_len / (channels * 4);
    pcm_data = aaf_msg->stream_data;
    pcm_len = stream_data_len;

  } else if (subtype == avtp_subtype_61883) {
    iec_61883_6_message_s *iec_msg = (iec_61883_6_message_s *)avtp_data;
    uint16_t stream_data_len =
        (iec_msg->stream_data_len[0] << 8) | iec_msg->stream_data_len[1];
    if (stream_data_len <= 8)
      return;
    uint8_t dbs = iec_msg->stream_data[1];
    channels = dbs;
    if (channels == 0)
      channels = 8;
    int data_bytes = stream_data_len - 8;
    samples = data_bytes / (channels * 4);
    pcm_data = iec_msg->stream_data + 8;
    pcm_len = data_bytes;
  } else {
    return;
  }

  if (samples <= 0 || !pcm_data)
    return;

  /* Capture first packet diagnostics (main loop prints) */
  if (ctx->diag_captured == 0) {
    ctx->diag_subtype = subtype;
    ctx->diag_sdl =
        (subtype == avtp_subtype_61883)
            ? ((iec_61883_6_message_s *)avtp_data)->stream_data_len[0] << 8 |
                  ((iec_61883_6_message_s *)avtp_data)->stream_data_len[1]
            : 0;
    ctx->diag_channels = channels;
    ctx->diag_samples = samples;
    int copy = pcm_len < 8 ? pcm_len : 8;
    memcpy(ctx->diag_first_audio, pcm_data, copy);
    ctx->diag_captured = 1;
  }

  /* Convert AVTP 24-bit audio to 24-bit stereo for I2S into a local
   * packet buffer, then enqueue with its presentation time (bytes 12-15
   * of the AVTP header). The time-scheduled drain will emit the packet
   * when its presentation_ns matches the current gPTP time + DMA depth.
   * I2S slot config: 24-bit data, 24-bit slot (3 bytes/sample, little-endian).
   * Downmix multi-channel to stereo: ch0 → L, ch1 → R. */
  uint8_t packet_pcm[AAF_MAX_PACKET_BYTES];
  int offset = 0;
  for (int sample_idx = 0;
       sample_idx < samples && offset + 6 <= (int)sizeof(packet_pcm);
       sample_idx++) {
    for (int channel = 0; channel < 2; channel++) {
      int src_channel = (channel < channels) ? channel : 0;
      int src_offset = (sample_idx * channels + src_channel) * 4;

      if (subtype == avtp_subtype_aaf) {
        if (src_offset + 2 < pcm_len) {
          packet_pcm[offset + 0] = pcm_data[src_offset + 2]; // LSB
          packet_pcm[offset + 1] = pcm_data[src_offset + 1]; // MID
          packet_pcm[offset + 2] = pcm_data[src_offset + 0]; // MSB
        } else {
          packet_pcm[offset + 0] = 0;
          packet_pcm[offset + 1] = 0;
          packet_pcm[offset + 2] = 0;
        }
      } else {
        if (src_offset + 3 < pcm_len) {
          packet_pcm[offset + 0] = pcm_data[src_offset + 3]; // LSB
          packet_pcm[offset + 1] = pcm_data[src_offset + 2]; // MID
          packet_pcm[offset + 2] = pcm_data[src_offset + 1]; // MSB
        } else {
          packet_pcm[offset + 0] = 0;
          packet_pcm[offset + 1] = 0;
          packet_pcm[offset + 2] = 0;
        }
      }
      offset += 3;
    }
  }

  if (offset > 0) {
    uint32_t avtp_timestamp =
        ((uint32_t)avtp_data[12] << 24) | ((uint32_t)avtp_data[13] << 16) |
        ((uint32_t)avtp_data[14] << 8) | (uint32_t)avtp_data[15];

    /* Refine the drain scheduler's packet-duration window based on the
     * actual packet format we're receiving (class A/B and other AAF
     * sample counts work without recompilation; assumes 48 kHz). */
    uint16_t actual_bytes = (uint16_t)offset;
    if (ctx->packet_bytes != actual_bytes) {
      ctx->packet_bytes = actual_bytes;
      ctx->packet_duration_ns =
          (uint32_t)((uint64_t)samples * 1000000000ULL / 48000ULL);
    }

    if (!aaf_queue_push(&ctx->queue, avtp_timestamp, packet_pcm,
                        (uint16_t)offset)) {
      ctx->queue_full_drops++;
    }
    if (!ctx->diag_i2s_bytes)
      ctx->diag_i2s_bytes = offset;
  }
  ctx->pkt_count++;
}

/* Print stream-in diagnostics — called from AVB main loop (safe for UART).
 * One-shot: prints first-packet info once, then suppressed. */
void avb_stream_in_print_diag(void) {
  stream_rx_ctx_t *ctx = s_stream_rx_ctx;
  if (!ctx)
    return;

  /* One-shot "drain enabled" snapshot — the startup fill has completed
   * and the drain is now presenting audio from a fixed-depth queue. The
   * reported fill_ms is the deterministic listener-internal offset
   * between avtp_timestamp and DAC output. */
  if (!ctx->drain_enabled_logged &&
      atomic_load_explicit(&ctx->drain_enabled, memory_order_acquire)) {
    ctx->drain_enabled_logged = true;
    ctx->ever_locked = true;
    uint32_t fill_us = (uint32_t)(ctx->startup_fill_target *
                                  (uint64_t)ctx->packet_duration_ns / 1000);
    avbinfo("STREAM: drain enabled (fill=%u pkts ≈ %lu us internal delay)",
            ctx->startup_fill_target, (unsigned long)fill_us);
  }

  /* Milan media_unlocked detection — if we have been locked and no AAF
   * packet has arrived in UNLOCK_TIMEOUT_US, the stream has stalled.
   * Tear down the drain_enabled flag so that a resumption of traffic
   * re-runs the startup fill, re-anchors, and increments
   * media_locked_count again. */
  const int64_t UNLOCK_TIMEOUT_US = 125000; /* 10× Class A packet period */
  if (ctx->ever_locked &&
      atomic_load_explicit(&ctx->drain_enabled, memory_order_acquire) &&
      ctx->last_packet_us > 0 &&
      (esp_timer_get_time() - ctx->last_packet_us) > UNLOCK_TIMEOUT_US) {
    atomic_store_explicit(&ctx->drain_enabled, false, memory_order_release);
    ctx->drain_enabled_logged = false;
    ctx->media_unlocked_count++;
    /* Drain whatever stale packets remain so the next lock re-anchors
     * from freshly-arriving packets. */
    uint32_t head = atomic_load_explicit(&ctx->queue.head,
                                         memory_order_acquire);
    atomic_store_explicit(&ctx->queue.tail, head, memory_order_release);
    avbinfo("STREAM: media unlocked (no packets for %lld ms)",
            (long long)((esp_timer_get_time() - ctx->last_packet_us) / 1000));
  }

  /* One-shot "first packet received" snapshot */
  if (ctx->diag_captured == 1) {
    ctx->diag_captured = 2;
    avbinfo("STREAM: first packet sub=%d sdl=%d ch=%d samp=%d i2s=%d "
            "audio=[%02x %02x %02x %02x %02x %02x %02x %02x]",
            ctx->diag_subtype, ctx->diag_sdl, ctx->diag_channels,
            ctx->diag_samples, ctx->diag_i2s_bytes, ctx->diag_first_audio[0],
            ctx->diag_first_audio[1], ctx->diag_first_audio[2],
            ctx->diag_first_audio[3], ctx->diag_first_audio[4],
            ctx->diag_first_audio[5], ctx->diag_first_audio[6],
            ctx->diag_first_audio[7]);
  }

  /* Periodic drain-health snapshot (~every 5 s) — shows whether the
   * DMA-driven drain is staying locked: emits should match packet
   * arrivals, silence/full should stay near zero in steady state.
   * Uses deltas so each window is independent. */
  static int64_t next_print_us = 0;
  static uint32_t last_pkt = 0, last_emit = 0, last_silence = 0;
  static uint32_t last_full = 0, last_drain = 0;
  int64_t now_us = esp_timer_get_time();
  if (now_us < next_print_us)
    return;
  next_print_us = now_us + 5 * 1000 * 1000;

  uint32_t packets = ctx->pkt_count, emits = ctx->on_time_emits,
           silence = ctx->silence_inserts,
           full = ctx->queue_full_drops, drain = ctx->drain_count;
  avbinfo("STREAM: pkts=%lu emit=%lu silence=%lu "
          "full_drop=%lu drain=%lu qfill=%lu id_skip=%lu seq_gap=%lu",
          packets - last_pkt, emits - last_emit, silence - last_silence,
          full - last_full, drain - last_drain,
          aaf_queue_fill(&ctx->queue), ctx->stream_id_mismatch,
          ctx->seq_num_mismatch);
  last_pkt = packets;
  last_emit = emits;
  last_silence = silence;
  last_full = full;
  last_drain = drain;
}

/* Fill stream input counters for GET_COUNTERS response (Milan Table 5.13) */
void avb_get_stream_in_counters(aem_stream_in_counters_val_s *valid,
                                aem_stream_in_counters_s *counters) {
  memset(valid, 0, sizeof(*valid));
  memset(counters, 0, sizeof(*counters));

  stream_rx_ctx_t *ctx = s_stream_rx_ctx;

  /* Set valid flags for all Milan mandatory counters (Table 5.13) */
  valid->media_locked = true;
  valid->media_unlocked = true;
  valid->stream_interrupted = true;
  valid->seq_num_mismatch = true;
  valid->media_reset = true;
  valid->ts_uncertain = true;
  valid->unsupported_format = true;
  valid->late_ts = true;
  valid->early_ts = true;
  valid->frames_rx = true;

  if (!ctx)
    return;

  /* Populate counters from stream RX context + media_clock state.
   * All are 32-bit monotonic rollovers per Milan §5.4.8. The
   * valid->unsupported_format flag MUST stay true (set above) — some
   * Milan controllers reject enumeration if any of the mandatory
   * Table 5.13 flags are false, even when the counter itself is zero. */
  uint32_t counter_val;
  counter_val = ctx->pkt_count;
  int_to_octets(&counter_val, counters->frames_rx, 4);
  counter_val = ctx->seq_num_mismatch;
  int_to_octets(&counter_val, counters->seq_num_mismatch, 4);
  counter_val = ctx->queue_full_drops;
  int_to_octets(&counter_val, counters->stream_interrupted, 4);
  counter_val = ctx->media_locked_count;
  int_to_octets(&counter_val, counters->media_locked, 4);
  counter_val = ctx->media_unlocked_count;
  int_to_octets(&counter_val, counters->media_unlocked, 4);
  counter_val = ctx->ts_uncertain_count;
  int_to_octets(&counter_val, counters->ts_uncertain, 4);
  counter_val = ctx->late_ts_count;
  int_to_octets(&counter_val, counters->late_ts, 4);
  counter_val = ctx->early_ts_count;
  int_to_octets(&counter_val, counters->early_ts, 4);
  if (ctx->state) {
    counter_val = ctx->state->media_clock.media_reset_count;
    int_to_octets(&counter_val, counters->media_reset, 4);
  }
}

/* Stream RX dispatcher — registered with the net layer; routes incoming
 * VLAN AVTP frames by stream_id to either the AAF handler or the CRF
 * handler. Both contexts are checked because either may be active. */
static void avb_stream_rx_dispatcher(uint8_t *avtp_data, uint16_t len,
                                     void *unused_ctx) {
  (void)unused_ctx;
  if (!avtp_data || len < 12)
    return;
  /* stream_id lives at bytes [4..11] in all AVTP stream subtypes */
  if (s_crf_rx_ctx != NULL &&
      memcmp(avtp_data + 4, s_crf_rx_ctx->expected_stream_id,
             UNIQUE_ID_LEN) == 0) {
    avb_crf_rx_handler(avtp_data, len, s_crf_rx_ctx);
    return;
  }
  if (s_stream_rx_ctx != NULL) {
    avb_stream_rx_handler(avtp_data, len, s_stream_rx_ctx);
  }
}

/* Start CRF media clock input (Milan v1.3 §7.2.2).
 * Lightweight compared to AAF: no I2S, no jitter buffer — just a context
 * for packet reception and stats. Phase 2 will use the recorded
 * timestamps to drive a media-clock PLL. */
static int avb_start_stream_in_crf(avb_state_s *state, uint16_t index) {
  if (s_crf_rx_ctx != NULL) {
    avberr("CRF stream-in already running");
    return ERROR;
  }
  crf_rx_ctx_t *ctx = calloc(1, sizeof(crf_rx_ctx_t));
  if (!ctx) {
    avberr("CRF stream in: no memory for context");
    return ERROR;
  }
  ctx->state = state;
  memcpy(ctx->expected_stream_id, state->input_streams[index].stream_id,
         UNIQUE_ID_LEN);
  s_crf_rx_ctx = ctx;
  state->input_streams[index].connected = true;
  /* Ensure the dispatcher is registered even if AAF isn't running yet */
  avb_net_set_stream_rx_handler(avb_stream_rx_dispatcher, NULL);
  avbinfo("CRF stream in started (stream index %d)", index);
  return OK;
}

/* Start AVB stream input — opens codec, creates jitter buffer,
 * starts drain timer, registers stream RX handler. */
int avb_start_stream_in(avb_state_s *state, uint16_t index) {

  if (index == AVB_CRF_INPUT_INDEX) {
    return avb_start_stream_in_crf(state, index);
  }

  if (state->stream_in_active) {
    avberr("Another instance of stream-in is already running");
    return ERROR;
  }

  // Codec already opened at startup by avb_config_codec (ADC+DAC active).
  // No per-stream open needed — avoids killing the other direction.

  // Allocate stream handler context
  stream_rx_ctx_t *ctx = calloc(1, sizeof(stream_rx_ctx_t));
  if (!ctx) {
    avberr("Stream in: no memory for context");
    return ERROR;
  }
  ctx->state = state;
  ctx->i2s_tx_handle = state->i2s_tx_handle;
  memcpy(ctx->expected_stream_id, state->input_streams[index].stream_id,
         UNIQUE_ID_LEN);

  /* Packet queue starts empty; packet_bytes/duration are filled in by
   * the first received packet. Seed them with the Class A default so a
   * sensible value is available before the first packet arrives. */
  atomic_store(&ctx->queue.head, 0);
  atomic_store(&ctx->queue.tail, 0);
  ctx->packet_bytes = 6 /*samples*/ * 2 /*ch*/ * 3 /*bytes*/;
  ctx->packet_duration_ns = AAF_DEFAULT_PACKET_DURATION_NS;

  /* Presentation-time anchor: wait until the queue has buffered enough
   * packets to cover the max_transit_time the SRP reservation carries
   * for this stream. That makes listener-internal latency predictable
   * and controller-visible. Fallback = 8 packets (1 ms) when no
   * MSRP advertise has reached us yet — covers the case where a
   * controller ACMP-connects before the network delivers the SRP
   * talker-advertise. */
  uint32_t msrp_latency_ns =
      (uint32_t)octets_to_uint(
          state->input_streams[index].msrp_accumulated_latency, 4);
  uint16_t target_pkts;
  if (msrp_latency_ns > 0 && ctx->packet_duration_ns > 0) {
    uint32_t derived = msrp_latency_ns / ctx->packet_duration_ns;
    /* Floor at 2 packets (any less can't absorb a single DMA descriptor
     * worth of jitter) and cap at a safe fraction of the queue so the
     * fill always fits with headroom for arrival bursts. */
    if (derived < 2)
      derived = 2;
    if (derived > AAF_QUEUE_CAPACITY / 2)
      derived = AAF_QUEUE_CAPACITY / 2;
    target_pkts = (uint16_t)derived;
  } else {
    target_pkts = 8; /* Class A default */
  }
  ctx->startup_fill_target = target_pkts;
  atomic_store(&ctx->drain_enabled, false);
  ctx->drain_enabled_at_us = 0;
  ctx->partial_bytes_drained = 0;
  avbinfo("Stream in: MSRP latency=%lu ns, startup_fill_target=%u pkts",
          (unsigned long)msrp_latency_ns, (unsigned)target_pkts);

  /* The on_sent callback was already registered once at i2s init time
   * (avb_stream_in_register_i2s_cb). It checks s_stream_rx_ctx on each
   * fire, so all we need to do here is publish the new context. */
  s_stream_rx_ctx = ctx;
  state->stream_in_active = true;
  state->input_streams[index].connected = true;

  /* Register the shared dispatcher — routes to AAF or CRF by stream_id */
  avb_net_set_stream_rx_handler(avb_stream_rx_dispatcher, NULL);

  avbinfo("Stream in started (packet queue %d slots, DMA on_sent drain)",
          AAF_QUEUE_CAPACITY);
  return OK;
}

/* Stop CRF media clock input. Symmetric with avb_start_stream_in_crf. */
static void avb_stop_stream_in_crf(avb_state_s *state, uint16_t index) {
  if (!s_crf_rx_ctx)
    return;
  avbinfo("CRF stream in stopped: %lu pkts, ts=%lu, last_ts_ns=%llu, "
          "id_skip=%lu, seq_miss=%lu",
          s_crf_rx_ctx->pkt_count, s_crf_rx_ctx->timestamp_count,
          s_crf_rx_ctx->last_timestamp_ns, s_crf_rx_ctx->stream_id_mismatch,
          s_crf_rx_ctx->seq_num_mismatch);
  free(s_crf_rx_ctx);
  s_crf_rx_ctx = NULL;
  state->input_streams[index].connected = false;
  /* Unregister the dispatcher only if no other stream is active */
  if (!state->stream_in_active) {
    avb_net_set_stream_rx_handler(NULL, NULL);
  }
}

/* Stop AVB stream input — unregisters handler, stops timer, frees resources */
void avb_stop_stream_in(avb_state_s *state, uint16_t index) {

  if (index == AVB_CRF_INPUT_INDEX) {
    avb_stop_stream_in_crf(state, index);
    return;
  }

  if (!state->stream_in_active)
    return;

  state->stream_in_active = false;

  if (s_stream_rx_ctx) {
    /* The on_sent callback remains registered on the TX channel; it
     * becomes a no-op once s_stream_rx_ctx is cleared (see ISR). We
     * can't unregister it here anyway because the channel is still
     * running on behalf of the talker side. */
    stream_rx_ctx_t *ctx_to_free = s_stream_rx_ctx;
    s_stream_rx_ctx = NULL;

    avbinfo("Stream in stopped: pkts=%lu emit=%lu silence=%lu "
            "full_drop=%lu drain=%lu qfill=%lu id_skip=%lu",
            ctx_to_free->pkt_count, ctx_to_free->on_time_emits,
            ctx_to_free->silence_inserts,
            ctx_to_free->queue_full_drops, ctx_to_free->drain_count,
            aaf_queue_fill(&ctx_to_free->queue),
            ctx_to_free->stream_id_mismatch);

    free(ctx_to_free);
  }

  /* Unregister dispatcher only if no other stream is active */
  if (!s_crf_rx_ctx) {
    avb_net_set_stream_rx_handler(NULL, NULL);
  }

  // Codec stays open — shared with talker stream-out. Closed at AVB shutdown.
}

/****************************************************************************
 * Stream Output (Talker)
 ****************************************************************************/

/* Start the AVB stream output task (talker)
 *
 * Generates a sine wave, outputs it through the ES8311 codec via I2S,
 * and simultaneously sends it as an AVTP AAF PCM stream over Ethernet.
 *
 * @param state: AVB state
 * @param index: output stream index
 * @return OK on success, ERROR on failure
 */
int avb_start_stream_out(avb_state_s *state, uint16_t index) {

  if (state->output_streams[index].streaming) {
    avberr("Stream out %d is already active", index);
    return ERROR;
  }

  struct stream_out_params_s *params =
      calloc(1, sizeof(struct stream_out_params_s));
  if (!params) {
    avberr("Stream out: No memory for params");
    return ERROR;
  }

  params->state = state;
  params->stream_index = index;
  params->i2s_rx_handle = state->i2s_rx_handle;
  params->eth_handle = state->config.eth_handle;
  memcpy(&params->stream_id, state->output_streams[index].stream_id,
         UNIQUE_ID_LEN);
  memcpy(&params->dest_addr, &state->output_streams[index].stream_dest_addr,
         ETH_ADDR_LEN);
  memcpy(params->vlan_id, state->output_streams[index].vlan_id, 2);

  // Get format parameters from the stream format in state
  avtp_stream_format_s *fmt = &state->output_streams[index].stream_format;
  params->format_subtype = fmt->aaf_pcm.subtype; // 0=61883, 2=AAF

  if (params->format_subtype == avtp_subtype_61883) {
    // IEC 61883-6 AM824 format
    params->cip_sfc = fmt->am824.fdf_sfc;
    params->dbs = fmt->am824.dbs;
    params->channels = fmt->am824.dbs; // dbs = channels for AM824 PCM
    params->bit_depth = 24;            // AM824 always carries 24-bit samples
    params->sample_rate = cip_sfc_to_sample_rate(fmt->am824.fdf_sfc);
  } else {
    // AAF PCM format
    params->bit_depth = fmt->aaf_pcm.bit_depth;
    params->channels =
        (fmt->aaf_pcm.chan_per_frame_h << 2) | fmt->aaf_pcm.chan_per_frame;
    params->sample_rate = aaf_code_to_sample_rate(fmt->aaf_pcm.sample_rate);
  }

  // Audio source: mic input by default, sine wave for testing
  params->use_sine_wave = false;
  params->sine_freq = 1000.0f; // 1 kHz test tone (if sine enabled)

  // Calculate samples per packet based on stream class
  // Class A: 125us (8000 packets/sec), Class B: 250us (4000 packets/sec)
  bool class_b = state->output_streams[index].stream_info_flags.class_b;
  params->interval = class_b ? 250 : 125; // microseconds
  params->samples_per_packet =
      params->sample_rate / (1000000 / params->interval);

  // Calculate buffer size (PCM data in 32-bit containers for both formats)
  int bytes_per_sample =
      (params->bit_depth == 24) ? 4 : (params->bit_depth / 8);
  params->buffer_size =
      params->samples_per_packet * params->channels * bytes_per_sample;

  // Clear stop flag and start the stream output task
  state->output_streams[index].stop_streaming = false;
  state->output_streams[index].streaming = true;
  xTaskCreatePinnedToCore(avb_stream_out_task, "AVB-OUT", 8192, (void *)params,
                          configMAX_PRIORITIES - 1, NULL, 1);

  avbinfo("Stream out %d started: %s -> AVTP %s", index,
          params->use_sine_wave ? "sine wave" : "mic input",
          params->format_subtype == avtp_subtype_61883 ? "IEC 61883" : "AAF");
  return OK;
}

/* Stop the AVB stream output task */
int avb_stop_stream_out(avb_state_s *state, uint16_t index) {
  state->output_streams[index].stop_streaming = true;
  state->output_streams[index].streaming = false;
  avbinfo("Stream out %d stopped", index);
  return OK;
}
