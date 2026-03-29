/*
 * Copyright 2024 Scramble Tools
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

/* globals to optimize l2tap usage */
static bool avtp_active_stream_in = false;
static bool avtp_active_stream_out = false;

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
  int_to_octets(&vlan_id, msg.sr_class_vid, vlan_id);
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
  msg.talker.info.priority = 3; // class A
  msg.talker.info.rank = 1;     // rank 1 for class A
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
int avb_process_msrp_domain(avb_state_s *state, msrp_msgbuf_s *msg, int offset,
                            size_t length) {
  // not implemented
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

/* Process received MSRP listener ready message */
int avb_process_msrp_listener(avb_state_s *state, msrp_msgbuf_s *msg,
                              int offset, size_t length) {
  // not implemented
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
  // not implemented
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
      (bit_depth == 24) ? 8388607.0f : 32767.0f; // 2^23-1 or 2^15-1
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
        lut[offset + 2] = (sample >> 0) & 0xFF;  // LSB
        lut[offset + 3] = 0;                      // padding
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
  if (mcr_pending) mcr_pending = false;
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
 * Each data block is 1 quadlet (4 bytes) per channel: label byte + 24-bit sample.
 * The CIP header is 2 quadlets (8 bytes) prepended to the data blocks.
 */
static int avb_send_iec_61883_packet(avb_state_s *state, unique_id_t *stream_id,
                                     uint8_t *pcm_data, uint16_t data_len,
                                     uint8_t seq_num, uint8_t dbs,
                                     uint8_t sfc, uint8_t channels,
                                     eth_addr_t *dest_addr, uint8_t *vlan_id,
                                     uint32_t avtp_ts, uint16_t dbc) {
  iec_61883_6_message_s msg;
  memset(&msg, 0, sizeof(msg));

  // Populate the IEC 61883 AVTP header
  msg.subtype = avtp_subtype_61883;
  msg.sv = 1;
  msg.version = 0;
  msg.timestamp_valid = 0;
  static bool mcr_pending_61883 = true;
  msg.media_clock_restart = mcr_pending_61883 ? 1 : 0;
  if (mcr_pending_61883) mcr_pending_61883 = false;
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
  msg.tag = 1;   // CIP header present
  msg.channel = 31; // 31 = native AVTP (not IEEE 1394)
  msg.tcode = 0x0A; // must be 1010 on transmit
  msg.sy = 0;

  // Build CIP header (2 quadlets = 8 bytes) per IEC 61883-1 §2.3
  //
  // Quadlet 0: | EOH(1)=0 | Fmt_hi(1)=0 | SID(6)=0x3F | DBS(8) | FN(2)=0 | QPC(3)=0 | SPH(1)=0 | rsv(1)=0 | DBC(8) |
  // Quadlet 1: | EOH(1)=1 | Fmt_hi(1)=0 | FMT(6)=0x10 | FDF(8)          | SYT(16)=0xFFFF          |
  //
  // SID=0x3F: no source ID info (standard for AVTP)
  // FMT=0x10: AM824 format (IEC 61883-6)
  // FDF: evt(3)=000 (AM824) | sfc(3) | reserved(2)=0
  msg.stream_data[0] = 0x3F; // EOH=0, Fmt_hi=0, SID=111111 (0x3F = no info)
  msg.stream_data[1] = dbs;  // data block size (quadlets per data block)
  msg.stream_data[2] = 0x00; // FN=00, QPC=000, SPH=0, rsv=0, DBC[7]=0
  msg.stream_data[3] = (uint8_t)(dbc & 0xFF); // DBC lower 8 bits
  msg.stream_data[4] = 0x90; // EOH=1, Fmt_hi=0, FMT=010000 (0x10 = AM824)
  msg.stream_data[5] = (sfc & 0x07); // FDF: evt=00000 in bits[7:3], SFC in bits[2:0]
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
      if (am824_offset + 4 <= AVTP_STREAM_DATA_PER_MSG && pcm_offset + 3 < data_len) {
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
  uint16_t msg_len = sizeof(iec_61883_6_message_s) - AVTP_STREAM_DATA_PER_MSG + sdl;

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
  case cip_sfc_sample_rate_32k:    return 32000;
  case cip_sfc_sample_rate_44_1k:  return 44100;
  case cip_sfc_sample_rate_48k:    return 48000;
  case cip_sfc_sample_rate_88_2k:  return 88200;
  case cip_sfc_sample_rate_96k:    return 96000;
  case cip_sfc_sample_rate_176_4k: return 176400;
  case cip_sfc_sample_rate_192k:   return 192000;
  default: return 48000;
  }
}

/* Map AAF sample rate enum to Hz */
static uint32_t aaf_code_to_sample_rate(uint8_t code) {
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
static void avb_stream_out_task(void *task_param) {
  avbinfo("Starting stream out task");

  uint8_t *pcm_buf = NULL;
  uint8_t *sine_lut = NULL;
  uint32_t lut_samples = 0;
  uint32_t lut_pos = 0;
  esp_task_wdt_user_handle_t wdt_handle = NULL;
  struct stream_out_params_s *params = (struct stream_out_params_s *)task_param;
  if (params == NULL) {
    goto err;
  }

  avb_state_s *state = (avb_state_s *)params->state;
  esp_codec_dev_handle_t codec_dev = state->config.codec_handle;
  uint8_t seq_num = 0;
  uint16_t dbc = 0; // data block counter for IEC 61883
  uint32_t pcm_offset = 0;
  uint8_t sample_rate_code = sample_rate_to_aaf_code(params->sample_rate);

  // Precompute sine wave LUT for constant-time sample generation
  lut_samples = build_sine_lut(&sine_lut, params->channels, params->bit_depth,
                               params->sample_rate, params->sine_freq);
  if (!sine_lut) {
    avberr("Stream out: Failed to build sine LUT");
    goto err;
  }
  avbinfo("Stream out: sine LUT built with %lu samples/cycle", lut_samples);

  // Calculate bytes per sample frame (all channels)
  int bytes_per_sample =
      (params->bit_depth == 24) ? 4 : (params->bit_depth / 8);
  int frame_size = bytes_per_sample * params->channels;
  int pcm_buf_size = params->samples_per_packet * frame_size;

  // Allocate PCM buffer for one AVTP packet's worth of samples
  pcm_buf = calloc(1, pcm_buf_size);
  if (!pcm_buf) {
    avberr("Stream out: No memory for PCM buffer (%d bytes)", pcm_buf_size);
    goto err;
  }

  // Use esp_codec_dev to open the codec for output (this configures the ES8311
  // and I2S clocking even though we're generating data, not reading from mic)
  if (codec_dev) {
    esp_codec_dev_sample_info_t fs = {
        .bits_per_sample = params->bit_depth,
        .channel = params->channels,
        .sample_rate = params->sample_rate,
        .mclk_multiple = 384,
    };
    int ret = esp_codec_dev_open(codec_dev, &fs);
    if (ret != ESP_CODEC_DEV_OK) {
      avbwarn(
          "Stream out: codec_dev_open returned %d, continuing without codec",
          ret);
    }
  }

  avbinfo("Stream out: rate=%luHz depth=%d ch=%d samples/pkt=%d interval=%dus",
          params->sample_rate, params->bit_depth, params->channels,
          params->samples_per_packet, params->interval);

  int64_t next_send_time = esp_timer_get_time();

  // Initialize AVTP media clock from PTP. Double-read loop validates that
  // the seconds register didn't roll over between the HAL's non-atomic
  // sec + nsec reads. Retries up to 5 times to get a consistent reading.
  uint32_t avtp_media_ts = 0;
  for (int init_try = 0; init_try < 5; init_try++) {
    struct timespec ptp_a, ptp_b;
    if (esp_eth_clock_gettime(CLOCK_PTP_SYSTEM, &ptp_a) == 0 &&
        esp_eth_clock_gettime(CLOCK_PTP_SYSTEM, &ptp_b) == 0) {
      uint32_t ts_a = (uint32_t)((uint64_t)ptp_a.tv_sec * 1000000000ULL +
                                  (uint64_t)ptp_a.tv_nsec);
      uint32_t ts_b = (uint32_t)((uint64_t)ptp_b.tv_sec * 1000000000ULL +
                                  (uint64_t)ptp_b.tv_nsec);
      int32_t diff = (int32_t)(ts_b - ts_a);
      if (diff >= 0 && diff < 500000) { // consistent reads
        avtp_media_ts = ts_a;
        break;
      }
    }
  }
  // Per-packet timestamp increment in nanoseconds
  // For Class A 48kHz: 6 samples / 48000 Hz = 125us = 125000ns
  uint32_t avtp_ts_increment =
      (uint32_t)((uint64_t)params->samples_per_packet * 1000000000ULL /
                 params->sample_rate);
  // Software PLL (PI controller): measures drift vs PTP and applies a
  // filtered correction to the per-packet increment so timestamps stay locked.
  // Uses fixed-point (16.16) arithmetic to eliminate integer quantization
  // artifacts that cause periodic rate wobble.
  #define PLL_MEASURE_FAST  500   // fast lock: measure every ~62.5ms
  #define PLL_MEASURE_SLOW  4000  // steady-state: measure every ~500ms
  #define PLL_FAST_DURATION 80000 // use fast interval for first ~10s (80000 pkts)
  #define PLL_SPREAD        16000 // spread correction over ~2s of packets
  #define PLL_FILTER_SHIFT  4    // IIR filter: blend 1/16 new + 15/16 old
  #define PLL_FP_SHIFT      16   // fixed-point fractional bits
  #define PLL_KI_SHIFT      8    // integral gain = 1/256 of proportional
  int64_t pll_correction_fp = 0; // filtered correction in 16.16 fixed-point ns
  int64_t pll_frac_accum = 0;    // fractional accumulator for sub-ns distribution
  int64_t pll_integral_fp = 0;   // integral accumulator for steady-state error
  // PLL diagnostics
  int32_t pll_offset_max = 0, pll_offset_min = 0;
  uint32_t pll_measure_count = 0, pll_skip_count = 0;

  // Register a WDT user handle and feed it explicitly from the spin loop.
  // IDLE1 WDT check is disabled via sdkconfig since AVB-OUT starves it.
  esp_task_wdt_add_user("AVB-OUT", &wdt_handle);

  // Suppress all UART logging while streaming — any ESP_LOG call takes a
  // global spinlock that stalls this real-time loop on core 1. PTP daemon
  // logs sync/follow-up every ~1s which causes audible glitches.
  esp_log_level_set("*", ESP_LOG_NONE);

  // Timing instrumentation
  int64_t t_gen_total = 0, t_write_total = 0, t_send_total = 0;
  int64_t t_gen_max = 0, t_write_max = 0, t_send_max = 0;
  uint32_t loop_count = 0;
  uint32_t overrun_count = 0;
  int64_t overrun_max = 0;
  uint32_t send_fail_count = 0;

  while (octets_to_uint(
             state->output_streams[params->stream_index].connection_count, 2) >
         0) {
    // Pure busy-wait until next transmission time. No yielding — this task
    // is pinned to a dedicated core at max priority for real-time streaming.
    // The task watchdog is fed explicitly below instead of relying on IDLE.
    while (esp_timer_get_time() < next_send_time) {
    }

    // Detect overruns (late wakeup from interrupt preemption)
    int64_t now = esp_timer_get_time();
    int64_t overrun = now - next_send_time;
    if (overrun > params->interval / 2) {
      overrun_count++;
      if (overrun > overrun_max)
        overrun_max = overrun;
    }
    // If we've fallen more than 10 packet intervals behind, snap forward
    // instead of bursting back-to-back packets to catch up, which would
    // produce a gap followed by a burst on the wire.
    if (overrun > params->interval * 10) {
      next_send_time = now + params->interval;
    } else {
      next_send_time += params->interval;
    }

    // Advance the AVTP media clock with PLL-corrected increment.
    // Uses fixed-point accumulator to distribute sub-nanosecond corrections
    // evenly across packets, avoiding integer quantization wobble.
    pll_frac_accum += pll_correction_fp;
    int32_t correction_ns = (int32_t)(pll_frac_accum >> PLL_FP_SHIFT);
    pll_frac_accum -= (int64_t)correction_ns << PLL_FP_SHIFT;
    avtp_media_ts += (uint32_t)((int32_t)avtp_ts_increment + correction_ns);

    // Copy samples from precomputed sine LUT (constant time)
    int64_t t0 = esp_timer_get_time();
    copy_sine_from_lut(pcm_buf, params->samples_per_packet, params->channels,
                       params->bit_depth, sine_lut, lut_samples, &lut_pos);
    int64_t t1 = esp_timer_get_time();

    // Read PCM data from embedded file
    // read_pcm_file(pcm_buf, params->samples_per_packet, params->channels,
    //              params->bit_depth, state->config.pcm_data,
    //              state->config.pcm_data_length, &pcm_offset);

    int64_t t2 = t1; // I2S write disabled

    // Send the PCM data as AVTP packets over Ethernet
    int offset = 0;
    while (offset < pcm_buf_size) {
      int chunk = pcm_buf_size - offset;
      if (chunk > AVTP_STREAM_DATA_PER_MSG) {
        chunk = AVTP_STREAM_DATA_PER_MSG;
      }
      int send_ret;
      if (params->format_subtype == avtp_subtype_61883) {
        send_ret = avb_send_iec_61883_packet(
            state, &params->stream_id, pcm_buf + offset, chunk, seq_num++,
            params->dbs, params->cip_sfc, params->channels,
            &params->dest_addr, params->vlan_id, avtp_media_ts, dbc);
        dbc += params->samples_per_packet; // advance DBC by number of data blocks
      } else {
        send_ret = avb_send_aaf_pcm_packet(
            state, &params->stream_id, pcm_buf + offset, chunk, seq_num++,
            sample_rate_code, params->channels, params->bit_depth,
            &params->dest_addr, params->vlan_id, avtp_media_ts);
      }
      if (send_ret <= 0) send_fail_count++;
      offset += chunk;
    }
    int64_t t3 = esp_timer_get_time();

    int64_t dt_gen = t1 - t0;
    int64_t dt_write = t2 - t1;
    int64_t dt_send = t3 - t2;
    t_gen_total += dt_gen;
    t_write_total += dt_write;
    t_send_total += dt_send;
    if (dt_gen > t_gen_max)
      t_gen_max = dt_gen;
    if (dt_write > t_write_max)
      t_write_max = dt_write;
    if (dt_send > t_send_max)
      t_send_max = dt_send;
    loop_count++;

    // Feed the task watchdog every ~125ms (1000 cycles at 125us)
    if (wdt_handle && loop_count % 1000 == 0) {
      esp_task_wdt_reset_user(wdt_handle);
    }

    // Software PLL (PI controller): read PTP clock directly and compute
    // drift correction. Double-read guards against the HAL's non-atomic
    // seconds/nanoseconds register read at second boundaries.
#if !PLL_DISABLED
    uint32_t pll_interval = (loop_count < PLL_FAST_DURATION)
                                ? PLL_MEASURE_FAST : PLL_MEASURE_SLOW;
    if (loop_count % pll_interval == 0) {
      struct timespec ptp_now;
      if (esp_eth_clock_gettime(CLOCK_PTP_SYSTEM, &ptp_now) == 0) {
        uint32_t ptp_now_ts =
            (uint32_t)((uint64_t)ptp_now.tv_sec * 1000000000ULL +
                       (uint64_t)ptp_now.tv_nsec);
        // Re-read to verify the HAL's non-atomic sec+nsec read was consistent.
        // Compare full nanosecond values instead of just seconds, so we don't
        // skip every measurement that lands near the PTP second boundary.
        struct timespec ptp_check;
        if (esp_eth_clock_gettime(CLOCK_PTP_SYSTEM, &ptp_check) == 0) {
          uint32_t check_ts =
              (uint32_t)((uint64_t)ptp_check.tv_sec * 1000000000ULL +
                         (uint64_t)ptp_check.tv_nsec);
          // Both reads should be within ~10μs of each other. A bad HAL read
          // at the second boundary produces ~1s error in the uint32 domain.
          int32_t read_diff = (int32_t)(check_ts - ptp_now_ts);
          if (read_diff >= 0 && read_diff < 500000) { // 0-500μs apart = good
            // Signed offset: positive = we're behind PTP, negative = ahead
            int32_t offset = (int32_t)(ptp_now_ts - avtp_media_ts);

            // Track PLL diagnostics
            pll_measure_count++;
            if (offset > pll_offset_max) pll_offset_max = offset;
            if (offset < pll_offset_min) pll_offset_min = offset;

            // Reject outliers: offsets > 50ms indicate a bad HAL read or PTP
            // clock step — feeding these into the PI controller would cause a
            // massive transient that takes seconds to recover from.
            #define PLL_OUTLIER_NS 50000000 // 50ms
            if (offset > PLL_OUTLIER_NS || offset < -PLL_OUTLIER_NS) {
              pll_skip_count++;
              goto pll_skip;
            }

            // Proportional term in 16.16 fixed-point ns/packet
            int64_t prop_corr_fp =
                ((int64_t)offset << PLL_FP_SHIFT) / PLL_SPREAD;

            // Integral term: eliminates steady-state error from crystal offset.
            // Anti-windup clamp at ~100 ppm (125 ns/pkt at 125000 ns nominal).
            pll_integral_fp +=
                ((int64_t)offset << PLL_FP_SHIFT) / (PLL_SPREAD << PLL_KI_SHIFT);
            int64_t integral_max = (int64_t)125 << PLL_FP_SHIFT;
            if (pll_integral_fp > integral_max) pll_integral_fp = integral_max;
            if (pll_integral_fp < -integral_max) pll_integral_fp = -integral_max;

            // Combined PI correction, then IIR low-pass filter
            int64_t new_corr_fp = prop_corr_fp + pll_integral_fp;
            int64_t filter_n = (1 << PLL_FILTER_SHIFT);
            pll_correction_fp =
                (pll_correction_fp * (filter_n - 1) + new_corr_fp) / filter_n;
            pll_skip: (void)0;
          }
        }
      }
    }
#endif // !PLL_DISABLED

    // Reset stats every 8000 iterations (~1 second at Class A rate).
    // No logging here — UART output blocks the real-time loop.
    if (loop_count % 8000 == 0) {
      t_gen_total = t_write_total = t_send_total = 0;
      t_gen_max = t_write_max = t_send_max = 0;
      overrun_count = 0;
      overrun_max = 0;
    }
  }

  // Log stream summary before cleanup (variables are in scope here)
  esp_log_level_set("*", ESP_LOG_INFO);
  avbinfo("Stream out task stopped: %lu packets sent, %lu send failures, "
          "%lu overruns (max %lldus)",
          loop_count, send_fail_count, overrun_count, overrun_max);
  avbinfo("PLL: %lu measurements, %lu skipped, offset range [%ldns, %ldns], "
          "correction=%lld (fp16.16)",
          pll_measure_count, pll_skip_count,
          (long)pll_offset_min, (long)pll_offset_max,
          (long long)pll_correction_fp);

err:
  // Restore default logging now that real-time streaming has stopped
  esp_log_level_set("*", ESP_LOG_INFO);

  if (wdt_handle) {
    esp_task_wdt_delete_user(wdt_handle);
  }
  if (sine_lut) {
    free(sine_lut);
  }
  if (pcm_buf) {
    free(pcm_buf);
  }
  if (params && params->state) {
    avb_state_s *s = (avb_state_s *)params->state;
    if (s->config.codec_handle) {
      esp_codec_dev_close(s->config.codec_handle);
    }
  }
  free(params);
  vTaskDelete(NULL);
}

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

  if (octets_to_uint(state->output_streams[index].connection_count, 2) > 0) {
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
  params->l2if = state->l2if[VLAN];
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
    params->bit_depth = 24; // AM824 always carries 24-bit samples
    params->sample_rate = cip_sfc_to_sample_rate(fmt->am824.fdf_sfc);
  } else {
    // AAF PCM format
    params->bit_depth = fmt->aaf_pcm.bit_depth;
    params->channels =
        (fmt->aaf_pcm.chan_per_frame_h << 2) | fmt->aaf_pcm.chan_per_frame;
    params->sample_rate = aaf_code_to_sample_rate(fmt->aaf_pcm.sample_rate);
  }

  // Sine wave configuration
  params->use_sine_wave = true;
  params->sine_freq = 1000.0f; // 1 kHz test tone

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

  // Start the stream output task
  xTaskCreatePinnedToCore(avb_stream_out_task, "AVB-OUT", 8192, (void *)params,
                          configMAX_PRIORITIES - 1, NULL, 1);

  // Update connection count
  uint16_t count =
      octets_to_uint(state->output_streams[index].connection_count, 2) + 1;
  int_to_octets(&count, state->output_streams[index].connection_count, 2);

  avbinfo("Stream out %d started: sine wave %0.0f Hz -> AVTP %s", index,
          params->sine_freq,
          params->format_subtype == avtp_subtype_61883 ? "IEC 61883" : "AAF");
  return OK;
}

/* Stop the AVB stream output task */
int avb_stop_stream_out(avb_state_s *state, uint16_t index) {
  // Reset connection count to signal task should stop
  uint16_t count = 0;
  int_to_octets(&count, state->output_streams[index].connection_count, 2);
  avbinfo("Stream out %d stopped", index);
  return OK;
}
