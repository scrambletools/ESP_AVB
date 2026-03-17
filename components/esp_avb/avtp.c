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
#include "esp_codec_dev.h"

/* globals to optimize l2tap usage */
static bool avtp_active_stream_in = false;
static bool avtp_active_stream_out = false;

/* Send MVRP VLAN identifier message */
int avb_send_mvrp_vlan_id(
    avb_state_s *state,
    mrp_attr_event_t attr_event,
    bool leave_all
) {
  mvrp_vlan_id_message_s msg;
  struct timespec ts;
  int ret;
  int vlan_id = CONFIG_ESP_AVB_VLAN_ID;
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
int avb_send_msrp_domain(
    avb_state_s *state,
    mrp_attr_event_t attr_event,
    bool leave_all
) {
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
  msg.header.vechead_leaveall = leave_all;
  msg.header.vechead_padding = 0;
  msg.header.vechead_num_vals = 1;
  msg.sr_class_id = 6; // class A
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
int avb_send_msrp_talker(
    avb_state_s *state, 
    mrp_attr_event_t attr_event, 
    bool leave_all,
    bool is_failed,
    unique_id_t *stream_id
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
    mrp_attr_event_t attr_event, 
    msrp_listener_event_t listener_event,
    bool leave_all,
    unique_id_t *stream_id
) {
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

/* Generate sine wave samples into a buffer
 *
 * @param buf: output buffer for PCM samples (24-bit packed into 32-bit words, stereo interleaved)
 * @param num_samples: number of samples per channel to generate
 * @param channels: number of channels (mono samples are duplicated to all channels)
 * @param bit_depth: bits per sample (16 or 24)
 * @param sample_rate: sample rate in Hz
 * @param freq: sine wave frequency in Hz
 * @param phase: pointer to running phase accumulator (updated on return)
 */
static void generate_sine_wave(uint8_t *buf, int num_samples, int channels,
                                int bit_depth, uint32_t sample_rate, float freq,
                                float *phase) {
    float phase_inc = 2.0f * M_PI * freq / (float)sample_rate;
    int bytes_per_sample = bit_depth / 8;
    // For 24-bit, we pack into 32-bit (4 bytes) per sample in I2S standard mode
    int stride = (bit_depth == 24) ? 4 : bytes_per_sample;
    float amplitude = (bit_depth == 24) ? 8388607.0f : 32767.0f; // 2^23-1 or 2^15-1

    for (int i = 0; i < num_samples; i++) {
        float sample_f = sinf(*phase) * amplitude * 0.7f; // 70% amplitude to avoid clipping
        int32_t sample = (int32_t)sample_f;

        for (int ch = 0; ch < channels; ch++) {
            int offset = (i * channels + ch) * stride;
            if (bit_depth == 24) {
                // 24-bit in 32-bit container, left-justified for I2S
                buf[offset + 0] = (sample >> 0) & 0xFF;
                buf[offset + 1] = (sample >> 8) & 0xFF;
                buf[offset + 2] = (sample >> 16) & 0xFF;
                buf[offset + 3] = 0; // padding byte
            } else {
                // 16-bit
                buf[offset + 0] = (sample >> 0) & 0xFF;
                buf[offset + 1] = (sample >> 8) & 0xFF;
            }
        }
        *phase += phase_inc;
        if (*phase >= 2.0f * M_PI) {
            *phase -= 2.0f * M_PI;
        }
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
 */
static int avb_send_aaf_pcm_packet(avb_state_s *state,
                                    unique_id_t *stream_id,
                                    uint8_t *pcm_data, uint16_t data_len,
                                    uint8_t seq_num, uint8_t sample_rate_code,
                                    uint8_t channels, uint8_t bit_depth) {
    aaf_pcm_message_s msg;
    struct timespec ts;
    memset(&msg, 0, sizeof(msg));

    // Populate the AAF header
    msg.subtype = avtp_subtype_aaf;
    msg.sv = 1;                     // stream ID valid
    msg.version = 0;
    msg.timestamp_valid = 0;        // no presentation timestamp for now
    msg.media_clock_restart = 0;
    msg.seq_num = seq_num;
    msg.timestamp_uncertain = 0;
    memcpy(msg.stream_id, stream_id, UNIQUE_ID_LEN);

    // Set AVTP timestamp from PTP clock
    struct timespec ptp_time;
    if (esp_eth_clock_gettime(CLOCK_PTP_SYSTEM, &ptp_time) == 0) {
        uint32_t avtp_ts = (uint32_t)(ptp_time.tv_nsec);
        msg.avtp_ts[0] = (avtp_ts >> 24) & 0xFF;
        msg.avtp_ts[1] = (avtp_ts >> 16) & 0xFF;
        msg.avtp_ts[2] = (avtp_ts >> 8) & 0xFF;
        msg.avtp_ts[3] = avtp_ts & 0xFF;
        msg.timestamp_valid = 1;
    }

    msg.format = (bit_depth <= 16) ? aaf_format_int_16bit : aaf_format_int_32bit;
    msg.sample_rate = sample_rate_code;
    msg.chan_per_frame = channels;
    msg.bit_depth = bit_depth;
    msg.evt = 0;       // normal
    msg.sparse_ts = 0;

    // Set stream data length
    uint16_t sdl = (data_len > AVTP_STREAM_DATA_PER_MSG) ? AVTP_STREAM_DATA_PER_MSG : data_len;
    msg.stream_data_len[0] = (sdl >> 8) & 0xFF;
    msg.stream_data_len[1] = sdl & 0xFF;

    // Copy PCM data into the message
    memcpy(msg.stream_data, pcm_data, sdl);

    // Calculate total message length (header + stream data)
    // AAF header is 24 bytes before stream_data
    uint16_t msg_len = sizeof(aaf_pcm_message_s) - AVTP_STREAM_DATA_PER_MSG + sdl;

    // Send to the stream destination multicast address
    eth_addr_t dest_addr;
    memcpy(&dest_addr, &MAAP_MCAST_MAC_ADDR, ETH_ADDR_LEN);
    int ret = avb_net_send_to(state, ethertype_avtp, &msg, msg_len, &ts, &dest_addr);
    if (ret < 0) {
        avberr("send AAF PCM failed: %d", errno);
    }
    return ret;
}

/* Map AAF sample rate enum to Hz */
static uint32_t aaf_code_to_sample_rate(uint8_t code) {
    switch (code) {
        case aaf_pcm_sample_rate_8k:     return 8000;
        case aaf_pcm_sample_rate_16k:    return 16000;
        case aaf_pcm_sample_rate_24k:    return 24000;
        case aaf_pcm_sample_rate_32k:    return 32000;
        case aaf_pcm_sample_rate_44_1k:  return 44100;
        case aaf_pcm_sample_rate_48k:    return 48000;
        case aaf_pcm_sample_rate_88_2k:  return 88200;
        case aaf_pcm_sample_rate_96k:    return 96000;
        case aaf_pcm_sample_rate_176_4k: return 176400;
        case aaf_pcm_sample_rate_192k:   return 192000;
        default:                         return 48000;
    }
}

/* Map sample rate in Hz to AAF sample rate enum */
static uint8_t sample_rate_to_aaf_code(uint32_t sample_rate) {
    switch (sample_rate) {
        case 8000:   return aaf_pcm_sample_rate_8k;
        case 16000:  return aaf_pcm_sample_rate_16k;
        case 24000:  return aaf_pcm_sample_rate_24k;
        case 32000:  return aaf_pcm_sample_rate_32k;
        case 44100:  return aaf_pcm_sample_rate_44_1k;
        case 48000:  return aaf_pcm_sample_rate_48k;
        case 88200:  return aaf_pcm_sample_rate_88_2k;
        case 96000:  return aaf_pcm_sample_rate_96k;
        case 176400: return aaf_pcm_sample_rate_176_4k;
        case 192000: return aaf_pcm_sample_rate_192k;
        default:     return aaf_pcm_sample_rate_48k;
    }
}

/* AVB Stream output task - generates sine wave and sends as AVTP AAF stream
 *
 * This task generates a sine wave using the esp_codec_dev component (ES8311)
 * for I2S clocking, and packages the audio as AVTP AAF PCM packets sent
 * over Ethernet.
 *
 * For Class A streams: 1ms intervals, ~48 samples/packet at 48kHz
 * For Class B streams: 4ms intervals, ~192 samples/packet at 48kHz
 */
static void avb_stream_out_task(void *task_param) {
    avbinfo("Starting stream out task");

    uint8_t *pcm_buf = NULL;
    struct stream_out_params_s *params = (struct stream_out_params_s *)task_param;
    if (params == NULL) { goto err; }

    avb_state_s *state = (avb_state_s *)params->state;
    esp_codec_dev_handle_t codec_dev = state->config.codec_handle;
    uint8_t seq_num = 0;
    uint32_t pcm_offset = 0;
    uint8_t sample_rate_code = sample_rate_to_aaf_code(params->sample_rate);

    // Calculate bytes per sample frame (all channels)
    int bytes_per_sample = (params->bit_depth == 24) ? 4 : (params->bit_depth / 8);
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
            avbwarn("Stream out: codec_dev_open returned %d, continuing without codec", ret);
        }
    }

    // Calculate the tick interval for FreeRTOS delay
    TickType_t interval_ticks = pdMS_TO_TICKS(params->interval / 1000);
    if (interval_ticks == 0) interval_ticks = 1;
    TickType_t last_wake = xTaskGetTickCount();

    avbinfo("Stream out: rate=%luHz depth=%d ch=%d samples/pkt=%d interval=%dus",
            params->sample_rate, params->bit_depth, params->channels,
            params->samples_per_packet, params->interval);

    while (octets_to_uint(state->output_streams[params->stream_index].connection_count, 2) > 0) {
        // // Generate sine wave PCM data
        // generate_sine_wave(pcm_buf, params->samples_per_packet, params->channels,
        //                    params->bit_depth, params->sample_rate,
        //                    params->sine_freq, &phase);

        // Read PCM data from embedded file
        read_pcm_file(pcm_buf, params->samples_per_packet, params->channels,
                      params->bit_depth, state->config.pcm_data,
                      state->config.pcm_data_length, &pcm_offset);

        // Also write to I2S/codec so the ES8311 DAC outputs the sine wave locally
        if (params->i2s_rx_handle) {
            // For the ES8311 output path, write the generated sine to the TX channel
            // (i2s_rx_handle in stream_out_params is actually used for reading mic;
            //  we write to the TX handle stored in state)
            size_t bytes_written = 0;
            i2s_channel_write(state->i2s_tx_handle, pcm_buf, pcm_buf_size,
                              &bytes_written, pdMS_TO_TICKS(100));
        }

        // Send the PCM data as AVTP AAF packets over Ethernet
        // If the PCM data exceeds AVTP_STREAM_DATA_PER_MSG, split into multiple packets
        int offset = 0;
        while (offset < pcm_buf_size) {
            int chunk = pcm_buf_size - offset;
            if (chunk > AVTP_STREAM_DATA_PER_MSG) {
                chunk = AVTP_STREAM_DATA_PER_MSG;
            }
            avb_send_aaf_pcm_packet(state, &params->stream_id,
                                     pcm_buf + offset, chunk, seq_num++,
                                     sample_rate_code, params->channels,
                                     params->bit_depth);
            offset += chunk;
        }

        // Wait for next interval using vTaskDelayUntil for precise timing
        vTaskDelayUntil(&last_wake, interval_ticks);
    }

err:
    avbinfo("Stream out task stopped");
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

    struct stream_out_params_s *params = calloc(1, sizeof(struct stream_out_params_s));
    if (!params) {
        avberr("Stream out: No memory for params");
        return ERROR;
    }

    params->state = state;
    params->stream_index = index;
    params->i2s_rx_handle = state->i2s_rx_handle;
    params->l2if = state->l2if[AVTP];
    memcpy(&params->stream_id, state->output_streams[index].stream_id, UNIQUE_ID_LEN);

    // Get format parameters from the stream format in state
    avtp_stream_format_s *fmt = &state->output_streams[index].stream_format;
    params->bit_depth = fmt->aaf_pcm.bit_depth;
    params->channels = (fmt->aaf_pcm.chan_per_frame_h << 2) |
                        fmt->aaf_pcm.chan_per_frame;
    params->sample_rate = aaf_code_to_sample_rate(fmt->aaf_pcm.sample_rate);

    // Sine wave configuration
    params->use_sine_wave = true;
    params->sine_freq = 1000.0f; // 1 kHz test tone

    // Calculate samples per packet based on stream class
    // Class A: 125us (8000 packets/sec), Class B: 250us (4000 packets/sec)
    bool class_b = state->output_streams[index].stream_info_flags.class_b;
    params->interval = class_b ? 250 : 125; // microseconds
    params->samples_per_packet = params->sample_rate / (1000000 / params->interval);

    // Calculate buffer size
    int bytes_per_sample = (params->bit_depth == 24) ? 4 : (params->bit_depth / 8);
    params->buffer_size = params->samples_per_packet * params->channels * bytes_per_sample;

    // Start the stream output task
    xTaskCreate(avb_stream_out_task, "AVB-OUT", 8192, (void *)params, 20, NULL);

    // Update connection count
    uint16_t count = octets_to_uint(state->output_streams[index].connection_count, 2) + 1;
    int_to_octets(&count, state->output_streams[index].connection_count, 2);

    avbinfo("Stream out %d started: sine wave %0.0f Hz -> AVTP AAF", index, params->sine_freq);
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

