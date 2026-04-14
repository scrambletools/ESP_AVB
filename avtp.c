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
  bool mcr_cleared = false;
  uint32_t i2s_zero_reads = 0;    /* reads that returned 0 bytes */
  uint32_t i2s_nonzero_audio = 0; /* reads with non-zero audio data */

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

  while (octets_to_uint(
             state->output_streams[params->stream_index].connection_count, 2) >
         0) {
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

    /* Update per-packet fields in tx_frame */
    avtp[1] = mcr_cleared ? 0x81 : 0x89; /* sv=1, tv=1, mr=1 on first pkt */
    if (!mcr_cleared) {
      avtp[1] = 0x89;
      mcr_cleared = true;
    } else {
      avtp[1] = 0x81;
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

/* Lock-free single-producer single-consumer ring buffer.
 * Capacity MUST be a power of 2 for fast index masking. */
typedef struct {
  uint8_t *buf;
  uint32_t capacity;     /* power of 2 */
  _Atomic uint32_t head; /* write position (producer only) */
  _Atomic uint32_t tail; /* read position (consumer only) */
} jitter_ring_t;

#define JITTER_RING_SIZE 2048 /* ~7ms at 48kHz stereo 24-bit */
#define JITTER_PREFILL 576    /* ~2ms of silence pre-fill */

static inline uint32_t ring_readable(const jitter_ring_t *r) {
  return atomic_load_explicit(&r->head, memory_order_acquire) -
         atomic_load_explicit(&r->tail, memory_order_relaxed);
}

static inline uint32_t ring_writable(const jitter_ring_t *r) {
  return r->capacity - ring_readable(r);
}

static inline uint32_t ring_write(jitter_ring_t *r, const uint8_t *data,
                                  uint32_t len) {
  uint32_t avail = ring_writable(r);
  if (len > avail)
    len = avail;
  if (len == 0)
    return 0;
  uint32_t h = atomic_load_explicit(&r->head, memory_order_relaxed);
  uint32_t mask = r->capacity - 1;
  uint32_t first = r->capacity - (h & mask);
  if (first > len)
    first = len;
  memcpy(r->buf + (h & mask), data, first);
  if (len > first)
    memcpy(r->buf, data + first, len - first);
  atomic_store_explicit(&r->head, h + len, memory_order_release);
  return len;
}

static inline uint32_t ring_read(jitter_ring_t *r, uint8_t *dst, uint32_t len) {
  uint32_t avail = ring_readable(r);
  if (len > avail)
    len = avail;
  if (len == 0)
    return 0;
  uint32_t t = atomic_load_explicit(&r->tail, memory_order_relaxed);
  uint32_t mask = r->capacity - 1;
  uint32_t first = r->capacity - (t & mask);
  if (first > len)
    first = len;
  memcpy(dst, r->buf + (t & mask), first);
  if (len > first)
    memcpy(dst + first, r->buf, len - first);
  atomic_store_explicit(&r->tail, t + len, memory_order_release);
  return len;
}

/* Stream RX handler context — file-static, accessed by EMAC task and
 * esp_timer task. Allocated by avb_start_stream_in. */
typedef struct {
  i2s_chan_handle_t i2s_tx_handle;
  uint8_t *i2s_drain_buf; /* buffer for drain callback reads */
  size_t i2s_drain_buf_size;
  uint8_t *i2s_convert_buf; /* buffer for AVTP→PCM conversion in handler */
  size_t i2s_convert_buf_size;
  jitter_ring_t ring;
  esp_timer_handle_t drain_timer;
  uint8_t expected_stream_id[UNIQUE_ID_LEN]; /* filter: only accept this stream */
  /* diagnostics */
  uint32_t pkt_count;
  uint32_t ring_write_fail; /* ring full — packets dropped */
  uint32_t ring_write_ok;
  uint32_t drain_count;
  uint32_t drain_underrun; /* drain found ring empty */
  uint32_t stream_id_mismatch; /* packets dropped due to wrong stream_id */
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

/* Drain timer callback — reads from ring, writes to I2S.
 * Runs in esp_timer task context (prio 22), fired every 1ms. */
static void stream_in_drain_cb(void *arg) {
  stream_rx_ctx_t *c = (stream_rx_ctx_t *)arg;
  c->drain_count++;

  uint32_t avail = ring_readable(&c->ring);
  if (avail == 0) {
    c->drain_underrun++;
    return;
  }

  uint32_t to_read = avail;
  if (to_read > c->i2s_drain_buf_size)
    to_read = c->i2s_drain_buf_size;
  /* Align to frame boundary (6 bytes per stereo 24-bit frame) */
  to_read = (to_read / 6) * 6;
  if (to_read == 0)
    return;

  uint32_t got = ring_read(&c->ring, c->i2s_drain_buf, to_read);
  size_t bytes_written = 0;
  i2s_channel_write(c->i2s_tx_handle, c->i2s_drain_buf, got, &bytes_written, 1);
}

/* Stream RX handler — called inline from EMAC RX task for each
 * VLAN-tagged AVTP packet. Parses AVTP, converts audio to 24-bit
 * stereo, writes to jitter ring buffer. Must return quickly. */
static void avb_stream_rx_handler(uint8_t *avtp_data, uint16_t len, void *ctx) {
  stream_rx_ctx_t *c = (stream_rx_ctx_t *)ctx;
  if (!c || len < 24)
    return;

  /* Filter by stream_id — only accept packets from the connected talker.
   * Stream_id is at AVTP header bytes [4..11] for both AAF and IEC 61883. */
  if (memcmp(avtp_data + 4, c->expected_stream_id, UNIQUE_ID_LEN) != 0) {
    c->stream_id_mismatch++;
    return;
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
  if (c->diag_captured == 0) {
    c->diag_subtype = subtype;
    c->diag_sdl =
        (subtype == avtp_subtype_61883)
            ? ((iec_61883_6_message_s *)avtp_data)->stream_data_len[0] << 8 |
                  ((iec_61883_6_message_s *)avtp_data)->stream_data_len[1]
            : 0;
    c->diag_channels = channels;
    c->diag_samples = samples;
    int copy = pcm_len < 8 ? pcm_len : 8;
    memcpy(c->diag_first_audio, pcm_data, copy);
    c->diag_captured = 1;
  }

  /* Convert AVTP 24-bit audio to 24-bit stereo for I2S.
   * I2S slot config: 24-bit data, 24-bit slot (3 bytes/sample, little-endian)
   * Downmix multi-channel to stereo: ch0 → L, ch1 → R */
  uint8_t *buf = c->i2s_convert_buf;
  int offset = 0;
  for (int s = 0; s < samples && offset + 6 <= (int)c->i2s_convert_buf_size;
       s++) {
    for (int ch = 0; ch < 2; ch++) {
      int src_ch = (ch < channels) ? ch : 0;
      int src_offset = (s * channels + src_ch) * 4;

      if (subtype == avtp_subtype_aaf) {
        if (src_offset + 2 < pcm_len) {
          buf[offset + 0] = pcm_data[src_offset + 2]; // LSB
          buf[offset + 1] = pcm_data[src_offset + 1]; // MID
          buf[offset + 2] = pcm_data[src_offset + 0]; // MSB
        } else {
          buf[offset + 0] = 0;
          buf[offset + 1] = 0;
          buf[offset + 2] = 0;
        }
      } else {
        if (src_offset + 3 < pcm_len) {
          buf[offset + 0] = pcm_data[src_offset + 3]; // LSB
          buf[offset + 1] = pcm_data[src_offset + 2]; // MID
          buf[offset + 2] = pcm_data[src_offset + 1]; // MSB
        } else {
          buf[offset + 0] = 0;
          buf[offset + 1] = 0;
          buf[offset + 2] = 0;
        }
      }
      offset += 3;
    }
  }

  /* Write converted PCM into jitter ring — non-blocking.
   * All-or-nothing to preserve 6-byte frame alignment. */
  if (offset > 0) {
    if (ring_writable(&c->ring) >= (uint32_t)offset) {
      ring_write(&c->ring, buf, offset);
      c->ring_write_ok++;
    } else {
      c->ring_write_fail++;
    }
    if (!c->diag_i2s_bytes)
      c->diag_i2s_bytes = offset;
  }
  c->pkt_count++;
}

/* Print stream-in diagnostics — called from AVB main loop (safe for UART).
 * One-shot: prints first-packet info once, then suppressed. */
void avb_stream_in_print_diag(void) {
  stream_rx_ctx_t *c = s_stream_rx_ctx;
  if (!c || c->diag_captured != 1)
    return;
  c->diag_captured = 2;
  avbinfo("STREAM: ok=%lu rfail=%lu drain=%lu underrun=%lu fill=%lu "
          "id_skip=%lu sub=%d sdl=%d ch=%d samp=%d i2s=%d "
          "audio=[%02x %02x %02x %02x %02x %02x %02x %02x]",
          c->ring_write_ok, c->ring_write_fail, c->drain_count,
          c->drain_underrun, ring_readable(&c->ring),
          c->stream_id_mismatch, c->diag_subtype,
          c->diag_sdl, c->diag_channels, c->diag_samples, c->diag_i2s_bytes,
          c->diag_first_audio[0], c->diag_first_audio[1],
          c->diag_first_audio[2], c->diag_first_audio[3],
          c->diag_first_audio[4], c->diag_first_audio[5],
          c->diag_first_audio[6], c->diag_first_audio[7]);
}

/* Start AVB stream input — opens codec, creates jitter buffer,
 * starts drain timer, registers stream RX handler. */
int avb_start_stream_in(avb_state_s *state, uint16_t index) {

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
  ctx->i2s_tx_handle = state->i2s_tx_handle;
  memcpy(ctx->expected_stream_id, state->input_streams[index].stream_id,
         UNIQUE_ID_LEN);

  // Conversion buffer — used by handler to build stereo PCM per packet.
  // All shared buffers use DMA-capable memory to avoid L1 cache coherency
  // issues between the EMAC RX handler (producer) and drain timer (consumer)
  // which may run on different CPU cores on ESP32-P4.
  ctx->i2s_convert_buf_size = (AVTP_STREAM_DATA_PER_MSG / 4) * 2 * 3;
  ctx->i2s_convert_buf =
      heap_caps_calloc(1, ctx->i2s_convert_buf_size, MALLOC_CAP_DMA);

  // Drain buffer — used by timer callback to read from ring
  ctx->i2s_drain_buf_size = 512; /* ~85 frames, well above 1ms worth */
  ctx->i2s_drain_buf = heap_caps_calloc(1, ctx->i2s_drain_buf_size, MALLOC_CAP_DMA);

  if (!ctx->i2s_convert_buf || !ctx->i2s_drain_buf) {
    avberr("Stream in: no memory for buffers");
    free(ctx->i2s_convert_buf);
    free(ctx->i2s_drain_buf);
    free(ctx);
    return ERROR;
  }

  // Allocate jitter ring buffer
  ctx->ring.buf = calloc(1, JITTER_RING_SIZE);
  if (!ctx->ring.buf) {
    avberr("Stream in: no memory for ring buffer");
    free(ctx->i2s_convert_buf);
    free(ctx->i2s_drain_buf);
    free(ctx);
    return ERROR;
  }
  ctx->ring.capacity = JITTER_RING_SIZE;
  atomic_store(&ctx->ring.head, JITTER_PREFILL); /* pre-fill silence */
  atomic_store(&ctx->ring.tail, 0);
  /* Ring buf is calloc'd (zeros) so pre-fill region is already silence */

  // Create and start drain timer (1ms period)
  esp_timer_create_args_t timer_args = {
      .callback = stream_in_drain_cb,
      .arg = ctx,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "avb_drain",
  };
  if (esp_timer_create(&timer_args, &ctx->drain_timer) != ESP_OK) {
    avberr("Stream in: failed to create drain timer");
    free(ctx->ring.buf);
    free(ctx->i2s_convert_buf);
    free(ctx->i2s_drain_buf);
    free(ctx);
    return ERROR;
  }
  esp_timer_start_periodic(ctx->drain_timer, 1000); /* 1000μs = 1ms */

  s_stream_rx_ctx = ctx;
  state->stream_in_active = true;
  state->input_streams[index].connected = true;

  // Register the stream handler — VLAN AVTP frames dispatched to handler
  avb_net_set_stream_rx_handler(avb_stream_rx_handler, ctx);

  avbinfo("Stream in started (jitter buffer %d bytes, pre-fill %d bytes)",
          JITTER_RING_SIZE, JITTER_PREFILL);
  return OK;
}

/* Stop AVB stream input — unregisters handler, stops timer, frees resources */
void avb_stop_stream_in(avb_state_s *state) {
  if (!state->stream_in_active)
    return;

  // Unregister handler first to stop new packets
  avb_net_set_stream_rx_handler(NULL, NULL);

  state->stream_in_active = false;

  if (s_stream_rx_ctx) {
    // Stop and delete drain timer
    esp_timer_stop(s_stream_rx_ctx->drain_timer);
    esp_timer_delete(s_stream_rx_ctx->drain_timer);

    avbinfo("Stream in stopped: %lu pkts, ring ok=%lu fail=%lu, "
            "drain=%lu underrun=%lu, ring fill=%lu, id_skip=%lu",
            s_stream_rx_ctx->pkt_count, s_stream_rx_ctx->ring_write_ok,
            s_stream_rx_ctx->ring_write_fail, s_stream_rx_ctx->drain_count,
            s_stream_rx_ctx->drain_underrun,
            ring_readable(&s_stream_rx_ctx->ring),
            s_stream_rx_ctx->stream_id_mismatch);

    free(s_stream_rx_ctx->ring.buf);
    free(s_stream_rx_ctx->i2s_convert_buf);
    free(s_stream_rx_ctx->i2s_drain_buf);
    free(s_stream_rx_ctx);
    s_stream_rx_ctx = NULL;
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

  // Start the stream output task
  xTaskCreatePinnedToCore(avb_stream_out_task, "AVB-OUT", 8192, (void *)params,
                          configMAX_PRIORITIES - 1, NULL, 1);

  // Update connection count
  uint16_t count =
      octets_to_uint(state->output_streams[index].connection_count, 2) + 1;
  int_to_octets(&count, state->output_streams[index].connection_count, 2);

  avbinfo("Stream out %d started: %s -> AVTP %s", index,
          params->use_sine_wave ? "sine wave" : "mic input",
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
