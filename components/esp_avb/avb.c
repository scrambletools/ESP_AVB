/*
 * Copyright 2024-2026 Scramble Tools
 * License: MIT
 *
 * ESP_AVB Component
 *
 * This component provides an implementation of an AVB talker and listener.
 *
 * This file provides the main entry point for the AVB task.
 */

#include "avb.h" // Internal API
#include "esp_codec_dev.h"
#include "esp_timer.h"

/* Global state */
avb_state_s *s_state;

// logo.png
extern const char logo_png_start[] asm("_binary_logo_png_start");
extern const char logo_png_end[] asm("_binary_logo_png_end");

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* AVB callback function */
esp_err_t my_callback(esp_eth_handle_t eth_handle, uint8_t *buffer,
                      uint32_t length, void *priv) {
  // Process received packet
  if (buffer[12] == 0x22 && buffer[13] == 0xf0 && buffer[14] == 0x00) {
    avbinfo("stream frame %lu", length);
  }
  return ESP_OK;
}

/* Initialize AVB state and create L2TAP FDs */
static int avb_initialize_state(avb_state_s *state, avb_config_s *config) {
  // Copy config to state
  memcpy(&state->config, config, sizeof(avb_config_s));

  // Initialize the low level ethernet interface
  int ret = avb_net_init(state);
  if (ret < 0) {
    avberr("Failed to initialize AVB network interface");
    return ERROR;
  }

  // Set entity id based on MAC address and model id
  memcpy(state->own_entity.summary.entity_id, state->internal_mac_addr,
         ETH_ADDR_LEN);
  uint64_t model_id = CONFIG_EXAMPLE_AVB_MODEL_ID;
  int_to_octets(&model_id, state->own_entity.summary.model_id, 8);

  // Set entity capabilities
  avb_entity_cap_s entity_caps;
  memset(&entity_caps, 0, sizeof(avb_entity_cap_s));
  entity_caps.gptp_supported = true;
  entity_caps.class_a = true;
  entity_caps.class_b = true;
  entity_caps.aem_supported = true;
  entity_caps.aem_config_index_valid = true;
  memcpy(&state->own_entity.summary.entity_capabilities, &entity_caps,
         sizeof(avb_entity_cap_s));

  // Set AVB interface defaults
  memset(&state->avb_interface, 0, sizeof(aem_avb_interface_desc_s));
  strncpy((char *)state->avb_interface.object_name,
          CONFIG_ESP_AVB_INTERFACE_NAME,
          sizeof(state->avb_interface.object_name) - 1);
  memcpy(state->avb_interface.mac_address, state->internal_mac_addr,
         ETH_ADDR_LEN);
  state->avb_interface.flags.gptp_gm_supported = true;
  state->avb_interface.flags.gptp_supported = true;
  state->avb_interface.flags.srp_supported = true;
  state->avb_interface.domain_number = 0; // gPTP domain 0 per 802.1AS
  state->avb_interface.log_sync_interval = (int8_t)-3;
  state->avb_interface.log_announce_interval = 0;
  state->avb_interface.log_pdelay_interval = 0;
  uint16_t port_number = CONFIG_ESP_AVB_PORT_ID;
  int_to_octets(&port_number, state->avb_interface.port_number, 2);

  // Set default MSRP mappings (class A and class B)
  state->msrp_mappings_count = 2;
  state->msrp_mappings[0].traffic_class = 1;
  state->msrp_mappings[0].priority = CONFIG_ESP_AVB_VLAN_PRIO_CLASS_A;
  uint16_t msrp_vlan_id = CONFIG_ESP_AVB_STREAM_VLAN_ID;
  int_to_octets(&msrp_vlan_id, state->msrp_mappings[0].vlan_id, 2);
  state->msrp_mappings[1].traffic_class = 0;
  state->msrp_mappings[1].priority = CONFIG_ESP_AVB_VLAN_PRIO_CLASS_B;
  int_to_octets(&msrp_vlan_id, state->msrp_mappings[1].vlan_id, 2);

  // Set talker sources and capabilities
  if (config->talker) {
    uint16_t talker_sources = 1;
    int_to_octets(&talker_sources,
                  state->own_entity.summary.talker_stream_sources, 2);
    avb_talker_cap_s talker_caps;
    memset(&talker_caps, 0, sizeof(avb_talker_cap_s));
    talker_caps.implemented = true;
    talker_caps.audio_source = 1;
    memcpy(&state->own_entity.summary.talker_capabilities, &talker_caps,
           sizeof(avb_talker_cap_s));
    avbinfo("AVB endpoint configured as TALKER");
  }

  // Set listener sinks and capabilities
  if (config->listener) {
    uint16_t listener_sinks = 1;
    int_to_octets(&listener_sinks,
                  state->own_entity.summary.listener_stream_sinks, 2);
    avb_listener_cap_s listener_caps;
    memset(&listener_caps, 0, sizeof(avb_listener_cap_s));
    listener_caps.implemented = true;
    listener_caps.audio_sink = 1;
    memcpy(&state->own_entity.summary.listener_capabilities, &listener_caps,
           sizeof(avb_listener_cap_s));
    avbinfo("AVB endpoint configured as LISTENER");
  }

  // Set entity detail info
  int64_t association_id = CONFIG_ESP_AVB_ASSOCIATION_ID;
  int_to_octets(&association_id, state->own_entity.detail.association_id,
                UNIQUE_ID_LEN);
  uint8_t entity_name[64] = CONFIG_ESP_AVB_ENTITY_NAME;
  memcpy(state->own_entity.detail.entity_name, entity_name,
         sizeof(entity_name));
  uint8_t firmware_version[64] = CONFIG_ESP_AVB_FIRMWARE_VERSION;
  memcpy(state->own_entity.detail.firmware_version, firmware_version,
         sizeof(firmware_version));
  char group_name[64] = CONFIG_ESP_AVB_GROUP_NAME;
  memcpy(state->own_entity.detail.group_name, group_name, sizeof(group_name));
  char serial_number[64] = CONFIG_ESP_AVB_SERIAL_NUMBER;
  memcpy(state->own_entity.detail.serial_number, serial_number,
         sizeof(serial_number));
  size_t config_num = AEM_MAX_NUM_CONFIGS;
  int_to_octets(&config_num, state->own_entity.detail.configurations_count, 2);
  size_t config_index = DEFAULT_CONFIG_INDEX;
  int_to_octets(&config_index, state->own_entity.detail.current_configuration,
                2);

  // Build supported stream formats
  avtp_stream_format_am824_s sf0 =
      AVB_DEFAULT_FORMAT_AM824(cip_sfc_sample_rate_44_1k);
  avtp_stream_format_am824_s sf1 =
      AVB_DEFAULT_FORMAT_AM824(cip_sfc_sample_rate_48k);
  avtp_stream_format_am824_s sf2 =
      AVB_DEFAULT_FORMAT_AM824(cip_sfc_sample_rate_96k);
  uint8_t aaf_ch = 8; // 8 channels to match MOTU/Apple AVB expectations
  avtp_stream_format_aaf_pcm_s sf3 =
      AVB_DEFAULT_FORMAT_AAF(16, aaf_pcm_sample_rate_44_1k, aaf_ch, false);
  avtp_stream_format_aaf_pcm_s sf4 =
      AVB_DEFAULT_FORMAT_AAF(24, aaf_pcm_sample_rate_44_1k, aaf_ch, false);
  avtp_stream_format_aaf_pcm_s sf5 =
      AVB_DEFAULT_FORMAT_AAF(24, aaf_pcm_sample_rate_48k, aaf_ch, false);
  avtp_stream_format_aaf_pcm_s sf6 =
      AVB_DEFAULT_FORMAT_AAF(24, aaf_pcm_sample_rate_96k, aaf_ch, false);
  state->supported_formats[0].am824 = sf0;
  state->supported_formats[1].am824 = sf1;
  state->supported_formats[2].am824 = sf2;
  state->supported_formats[3].aaf_pcm = sf3;
  state->supported_formats[4].aaf_pcm = sf4;
  state->supported_formats[5].aaf_pcm = sf5;
  state->supported_formats[6].aaf_pcm = sf6;
  state->num_supported_formats = 7;
  avtp_stream_format_aaf_pcm_s format = sf5;

  // setup listener stream flags, and stream info flags, default vlan id and
  // stream format
  avb_listener_stream_flags_s flags = {0};
  // set any flags here
  aem_stream_info_flags_s info_flags = {.stream_vlan_id_valid = 1,
                                        .stream_format_valid = 1,
                                        .stream_id_valid = 1,
                                        .stream_dest_mac_valid = 1,
                                        .msrp_failure_valid = 1,
                                        .msrp_acc_lat_valid = 1};

  // Build input streams
  if (state->config.listener) {
    state->num_input_streams = AVB_MAX_NUM_INPUT_STREAMS;
    avb_listener_stream_s input_stream = {0};
    memcpy(&input_stream.stream_flags, &flags,
           sizeof(avb_listener_stream_flags_s));
    memcpy(&input_stream.stream_info_flags, &info_flags,
           sizeof(aem_stream_info_flags_s));
    int mapping_index = input_stream.stream_info_flags.class_b ? 1 : 0;
    memcpy(input_stream.vlan_id, state->msrp_mappings[mapping_index].vlan_id,
           2);
    memcpy(&input_stream.stream_format, &format, sizeof(avtp_stream_format_s));
    for (int i = 0; i < state->num_input_streams; i++) {
      memcpy(&state->input_streams[i], &input_stream,
             sizeof(avb_listener_stream_s));
    }
  }

  // Build output streams
  if (state->config.talker) {
    state->num_output_streams = AVB_MAX_NUM_OUTPUT_STREAMS;
    avb_talker_stream_s output_stream = {0};

    memcpy(&output_stream.stream_info_flags, &info_flags,
           sizeof(aem_stream_info_flags_s));
    int mapping_index = output_stream.stream_info_flags.class_b ? 1 : 0;
    memcpy(output_stream.vlan_id, state->msrp_mappings[mapping_index].vlan_id,
           2);
    memcpy(&output_stream.stream_format, &format, sizeof(avtp_stream_format_s));
    for (int i = 0; i < state->num_output_streams; i++) {
      memcpy(&state->output_streams[i], &output_stream,
             sizeof(avb_talker_stream_s));
      stream_id_from_mac(&state->internal_mac_addr,
                         state->output_streams[i].stream_id, i);
      memcpy(&state->output_streams[i].stream_dest_addr, &MAAP_START_MAC_ADDR,
             ETH_ADDR_LEN);
      state->output_streams[i].stream_dest_addr[5] += i;
    }
  }

  // Set logo start and length
  state->logo_start = (uint8_t *)logo_png_start;
  state->logo_length = logo_png_end - logo_png_start;

  // Get latest PTP status
  struct ptpd_status_s ptp_status;
  // if valid PTP status
  if (ptpd_status(0, &ptp_status) == 0) {
    state->ptp_status = ptp_status;
  }

  // Set stop to false
  state->stop = false;

  // Set unsolicited notifications disabled
  state->unsol_notif_enabled = false;

  // Set global state
  s_state = state;
  return OK;
}

/* Destroy l2ifs */
static int avb_destroy_state(avb_state_s *state) {

  for (int i = 0; i < AVB_NUM_PROTOCOLS; i++) {
    if (state->l2if[i] > 0) {
      close(state->l2if[i]);
      state->l2if[i] = -1;
    }
  }
  return OK;
}

/* Get latest PTP status */
static void avb_update_ptp_status(avb_state_s *state) {
  // if valid PTP status
  struct ptpd_status_s ptp_status;
  // if valid PTP status
  if (ptpd_status(0, &ptp_status) == 0) {
    memcpy(&state->ptp_status, &ptp_status, sizeof(struct ptpd_status_s));
    avb_update_avb_interface_from_ptp(state);
  }
}

/* Send periodic messages */
static int avb_periodic_send(avb_state_s *state) {
  struct timespec time_now;
  struct timespec delta;
  clock_gettime(CLOCK_MONOTONIC, &time_now);

  // PTP snapshot for the stream out PLL is no longer needed here — the
  // stream out task now reads the PTP clock directly on Core 1 with a
  // double-read consistency check instead of projecting a stale snapshot.

  // Send ADP entity available message
  clock_timespec_subtract(&time_now, &state->last_transmitted_adp_entity_avail,
                          &delta);
  if (timespec_to_ms(&delta) > ADP_ENTITY_AVAIL_INTERVAL_MSEC) {
    state->last_transmitted_adp_entity_avail = time_now;
    avb_send_adp_entity_available(state);
  }

  // Send MVRP VLAN ID message
  clock_timespec_subtract(&time_now, &state->last_transmitted_mvrp_vlan_id,
                          &delta);
  if (timespec_to_ms(&delta) > MVRP_VLAN_ID_INTERVAL_MSEC) {
    state->last_transmitted_mvrp_vlan_id = time_now;
    avb_send_mvrp_vlan_id(state, mrp_attr_event_join_mt, false);
  }

  // Send MSRP domain message
  clock_timespec_subtract(&time_now, &state->last_transmitted_msrp_domain,
                          &delta);
  if (timespec_to_ms(&delta) > MSRP_DOMAIN_INTERVAL_MSEC) {
    state->last_transmitted_msrp_domain = time_now;
    avb_send_msrp_domain(state, mrp_attr_event_join_mt, false);
  }

  // Send MSRP talker and AVTP MAAP announce messages
  if (state->config.talker) {
    clock_timespec_subtract(&time_now, &state->last_transmitted_msrp_talker_adv,
                            &delta);
    for (int i = 0; i < state->num_output_streams; i++) {
      // if connected and time to send
      if (octets_to_uint(state->output_streams[i].connection_count, 2) > 0 &&
          timespec_to_ms(&delta) > MSRP_TALKER_CONN_INTERVAL_MSEC) {
        state->last_transmitted_msrp_talker_adv = time_now;
        avb_send_msrp_talker(state, mrp_attr_event_join_in, false, false,
                             &state->output_streams[i].stream_id,
                             &state->output_streams[i].stream_dest_addr,
                             state->output_streams[i].vlan_id);
        // if idle and time to send
      } else if (timespec_to_ms(&delta) > MSRP_TALKER_IDLE_INTERVAL_MSEC) {
        state->last_transmitted_msrp_talker_adv = time_now;
        avb_send_msrp_talker(state, mrp_attr_event_join_mt, false, false,
                             &state->output_streams[i].stream_id,
                             &state->output_streams[i].stream_dest_addr,
                             state->output_streams[i].vlan_id);
      }
    }
    clock_timespec_subtract(&time_now, &state->last_transmitted_maap_announce,
                            &delta);
    if (timespec_to_ms(&delta) > MAAP_ANNOUNCE_INTERVAL_MSEC) {
      state->last_transmitted_maap_announce = time_now;
      avb_send_maap_announce(state);
    }
  }

  // Send MSRP listener message
  if (state->config.listener) {
    clock_timespec_subtract(&time_now, &state->last_transmitted_msrp_listener,
                            &delta);
    for (int i = 0; i < state->num_input_streams; i++) {
      // if connected and time to send
      if (state->input_streams[i].connected &&
          timespec_to_ms(&delta) > MSRP_LISTENER_CONN_INTERVAL_MSEC) {
        state->last_transmitted_msrp_listener = time_now;
        avb_send_msrp_listener(state, mrp_attr_event_join_in,
                               msrp_listener_event_ready, false,
                               &state->input_streams[i].stream_id);
        // if idle and time to send
      } else if (timespec_to_ms(&delta) > MSRP_LISTENER_IDLE_INTERVAL_MSEC) {
        // Only send idle Listener Mt if we have a valid (non-zero) stream ID.
        // Sending with a null stream ID confuses MSRP state machines on bridges.
        unique_id_t zero_id = {0};
        if (memcmp(&state->input_streams[i].stream_id, &zero_id,
                   sizeof(unique_id_t)) != 0) {
          state->last_transmitted_msrp_listener = time_now;
          avb_send_msrp_listener(state, mrp_attr_event_mt,
                                 msrp_listener_event_ready, false,
                                 &state->input_streams[i].stream_id);
        }
      }
    }
  }

  // if time to send leaveAll
  clock_timespec_subtract(&time_now, &state->last_transmitted_msrp_leaveall,
                          &delta);
  if (timespec_to_ms(&delta) > MSRP_LEAVEALL_INTERVAL_MSEC) {
    state->last_transmitted_msrp_leaveall = time_now;
    avb_send_msrp_domain(state, mrp_attr_event_join_in, true);

    // After leaveAll, immediately re-declare all active registrations so the
    // switch doesn't drop stream reservations while waiting for periodic timers
    if (state->config.talker) {
      for (int i = 0; i < state->num_output_streams; i++) {
        if (octets_to_uint(state->output_streams[i].connection_count, 2) > 0) {
          avb_send_msrp_talker(state, mrp_attr_event_join_in, false, false,
                               &state->output_streams[i].stream_id,
                               &state->output_streams[i].stream_dest_addr,
                               state->output_streams[i].vlan_id);
        }
      }
      state->last_transmitted_msrp_talker_adv = time_now;
    }
    if (state->config.listener) {
      for (int i = 0; i < state->num_input_streams; i++) {
        if (state->input_streams[i].connected) {
          avb_send_msrp_listener(state, mrp_attr_event_join_in,
                                 msrp_listener_event_ready, false,
                                 &state->input_streams[i].stream_id);
        }
      }
      state->last_transmitted_msrp_listener = time_now;
    }
    // Re-declare MVRP VLAN registration
    avb_send_mvrp_vlan_id(state, mrp_attr_event_join_in, false);
    state->last_transmitted_mvrp_vlan_id = time_now;
  }

  // Send Unsolicited notifications
  if (state->unsol_notif_enabled) {

    if (state->config.talker) {
      // Send get counters stream_output notification
      for (int i = 0; i < state->num_output_streams; i++) {
        // if the output stream is active
        if (octets_to_uint(state->output_streams[i].connection_count, 2) > 0) {
          clock_timespec_subtract(&time_now,
                                  &state->last_transmitted_unsol_notif, &delta);
          if (timespec_to_ms(&delta) > UNSOL_NOTIF_INTERVAL_MSEC) {
            state->last_transmitted_unsol_notif = time_now;
            avb_send_aecp_unsol_get_counters(state, aem_desc_type_stream_output,
                                             i);
          }
        }
      }
    }
    if (state->config.listener) {
      // Send get counters stream_input notification
      for (int i = 0; i < state->num_input_streams; i++) {
        // if the input stream is active
        if (state->input_streams[i].connected) {
          clock_timespec_subtract(&time_now,
                                  &state->last_transmitted_unsol_notif, &delta);
          if (timespec_to_ms(&delta) > UNSOL_NOTIF_INTERVAL_MSEC) {
            state->last_transmitted_unsol_notif = time_now;
            avb_send_aecp_unsol_get_counters(state, aem_desc_type_stream_input,
                                             i);
          }
        }
      }
    }
  }
  return OK;
} // avb_periodic_send

/* Determine received message type and process it */

static int avb_process_rx_message(avb_state_s *state, int protocol_idx,
                                  ssize_t length) {
  // Check if the message length is valid
  if (length <= ETH_HEADER_LEN) {
    avbwarn("Ignoring invalid message, length only %d bytes", (int)length);
    return OK;
  }
  eth_addr_t src_addr;
  memcpy(&src_addr, &state->rxsrc[protocol_idx], ETH_ADDR_LEN);

  // String representation of source address
  char src_addr_str[ETH_ADDR_LEN * 3 + 1];
  octets_to_hex_string((uint8_t *)src_addr, ETH_ADDR_LEN, src_addr_str, ':');

  /* Route the message to the appropriate handler */
  switch (protocol_idx) {
  case AVTP: {
    avtp_msgbuf_u *msg = (avtp_msgbuf_u *)&state->rxbuf[protocol_idx].avtp;
    switch (msg->subtype) {
    case avtp_subtype_61883:
      avbinfo("***** Got an IEC 61883 message from %s", src_addr_str);
      return avb_process_iec_61883(state, &msg->iec);
      break;
    case avtp_subtype_aaf:
      return avb_process_aaf(state, &msg->aaf);
      break;
    case avtp_subtype_maap:
      return avb_process_maap(state, &msg->maap);
      break;
    case avtp_subtype_adp:
      return avb_process_adp(state, &msg->adp, &src_addr);
      break;
    case avtp_subtype_aecp:
      return avb_process_aecp(state, &msg->aecp, &src_addr);
      break;
    case avtp_subtype_acmp:
      return avb_process_acmp(state, &msg->acmp);
      break;
    default:
      avbinfo("Ignoring unsupported AVTP message subtype: 0x%02x",
              msg->subtype);
    }
    break;
  }
  case MSRP: {
    msrp_msgbuf_s *msg = (msrp_msgbuf_s *)&state->rxbuf[protocol_idx].msrp;

    // Get the first attribute type
    msrp_attr_header_s header;
    int offset = 0;
    int count = 0;
    memcpy(&header, &msg->messages_raw[offset], sizeof(msrp_attr_header_s));

    // Process all attributes in the message
    while (header.attr_type != msrp_attr_type_none &&
           offset < sizeof(msrp_msgbuf_s)) {
      size_t attr_size = octets_to_uint(header.attr_list_len, 2) +
                         4; // 4 bytes for the header w/o vechead
      switch (header.attr_type) {
      case msrp_attr_type_domain:
        avb_process_msrp_domain(state, msg, offset, attr_size);
        break;
      case msrp_attr_type_talker_advertise:
        avb_process_msrp_talker(state, msg, offset, attr_size, false,
                                &src_addr);
        break;
      case msrp_attr_type_talker_failed:
        avb_process_msrp_talker(state, msg, offset, attr_size, true, &src_addr);
        break;
      case msrp_attr_type_listener:
        avb_process_msrp_listener(state, msg, offset, attr_size);
        break;
      default:
        avbinfo("Ignoring unknown MSRP message atribute type: 0x%02x",
                header.attr_type);
      }
      // Next attribute offset
      offset += attr_size;
      // Get the next attribute header
      memcpy(&header, &msg->messages_raw[offset], sizeof(msrp_attr_header_s));
      count++;
    }
    break;
  }
  }
  return OK;
}

/* AVB Stream input task */
static void avb_stream_in_task(void *task_param) {
  avbinfo("Starting stream in task");

  avb_state_s *state = NULL;
  uint8_t *stream_in_data = NULL;
  uint8_t *i2s_buf = NULL;

  struct stream_in_params_s *stream_in_params =
      (struct stream_in_params_s *)task_param;
  if (stream_in_params == NULL) {
    goto err;
  }
  state = (avb_state_s *)stream_in_params->state;

  // Open the codec for 16-bit stereo output (ES8311 works reliably at 16-bit;
  // 24-bit has bit alignment issues on this platform). AVTP 24-bit audio will
  // be downconverted to 16-bit before writing to I2S.
  if (state->codec_enabled && state->config.codec_handle) {
    esp_codec_dev_sample_info_t fs = {
        .bits_per_sample = 16,
        .channel = 2,
        .sample_rate = 48000,
        .mclk_multiple = 384,
    };
    int codec_ret = esp_codec_dev_open(state->config.codec_handle, &fs);
    if (codec_ret != ESP_CODEC_DEV_OK) {
      avbwarn("Stream in: codec_dev_open returned %d, continuing", codec_ret);
    }
    esp_codec_dev_set_out_vol(state->config.codec_handle, 50.0);
  }

  // Receive buffer must hold a full AVTP packet (header + max payload)
  size_t recv_buf_size = sizeof(iec_61883_6_message_s); // largest format
  stream_in_data = calloc(1, recv_buf_size);
  if (!stream_in_data) {
    avberr("Stream in: No memory for receive buffer");
    goto err;
  }

  // I2S output buffer — convert from AVTP big-endian 24-bit to 16-bit stereo.
  // Max 6 samples per packet, downmixed to stereo, 16-bit per sample.
  size_t i2s_buf_size = 6 * 2 * 2; // 6 samples * stereo * 16-bit
  i2s_buf = calloc(1, i2s_buf_size);
  if (!i2s_buf) {
    avberr("Stream in: No memory for I2S buffer");
    goto err;
  }

  int ret = 0;
  struct timespec ts;
  eth_addr_t src_addr;
  size_t bytes_write = 0;
  size_t timeout_ms = 1000;

  if (stream_in_params->l2if < 0) {
    avberr("Invalid file descriptor for VLAN interface");
    goto err;
  }

  // Disable the VLAN fd in the main loop's pollfds so this task owns it.
  // The main loop checks stream_in_active to skip VLAN reads.
  state->stream_in_active = true;

  // Brief delay to let the main loop finish any in-progress VLAN read
  vTaskDelay(pdMS_TO_TICKS(100));
  avbinfo("Stream in: listening for stream %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
          stream_in_params->stream_id[0], stream_in_params->stream_id[1],
          stream_in_params->stream_id[2], stream_in_params->stream_id[3],
          stream_in_params->stream_id[4], stream_in_params->stream_id[5],
          stream_in_params->stream_id[6], stream_in_params->stream_id[7]);

  while (1) {
    /* Read directly from the VLAN L2TAP fd (blocking).
     * This task owns the fd — the main loop skips it via stream_in_active. */
    ret = avb_net_recv(stream_in_params->l2if, stream_in_data, recv_buf_size,
                       &ts, &src_addr);
    if (ret <= 0) {
      continue;
    }

    // Log first few packets for debugging
    static int dbg_count = 0;
    if (dbg_count < 5) {
      avbinfo("Stream in: recv %d bytes, first 8: %02x %02x %02x %02x %02x %02x %02x %02x",
              ret, stream_in_data[0], stream_in_data[1], stream_in_data[2],
              stream_in_data[3], stream_in_data[4], stream_in_data[5],
              stream_in_data[6], stream_in_data[7]);
      dbg_count++;
    }

    // VLAN-tagged frames: after ETH header strip by avb_net_recv, the payload
    // starts with VLAN TCI(2) + inner ethertype(2) + AVTP data.
    // Skip the 4-byte VLAN+ethertype prefix to get to AVTP.
    uint8_t *avtp_data = stream_in_data;
    int avtp_len = ret;
    if (ret > 4 && stream_in_data[2] == 0x22 && stream_in_data[3] == 0xf0) {
      avtp_data = stream_in_data + 4;
      avtp_len = ret - 4;
    }

    if (avtp_len < 24) continue;

    uint8_t subtype = avtp_data[0] & 0x7F;
    uint8_t *pcm_data = NULL;
    int pcm_len = 0;
    int channels = 0;
    int samples = 0;

    if (dbg_count <= 5) {
      avbinfo("Stream in: subtype=0x%02x, stream_id=%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
              subtype, avtp_data[4], avtp_data[5], avtp_data[6], avtp_data[7],
              avtp_data[8], avtp_data[9], avtp_data[10], avtp_data[11]);
    }

    if (subtype == avtp_subtype_aaf) {
      /* AAF format */
      aaf_pcm_message_s *aaf_msg = (aaf_pcm_message_s *)avtp_data;
      if (memcmp(aaf_msg->stream_id, stream_in_params->stream_id,
                 UNIQUE_ID_LEN) != 0) {
        continue;
      }
      uint16_t stream_data_len =
          (aaf_msg->stream_data_len[0] << 8) | aaf_msg->stream_data_len[1];
      if (stream_data_len == 0 || stream_data_len > AVTP_STREAM_DATA_PER_MSG) {
        continue;
      }
      channels = aaf_msg->chan_per_frame;
      if (channels == 0) channels = 8;
      int stride = 4; // 32-bit container
      samples = stream_data_len / (channels * stride);
      pcm_data = aaf_msg->stream_data;
      pcm_len = stream_data_len;

    } else if (subtype == avtp_subtype_61883) {
      /* IEC 61883-6 AM824 format */
      iec_61883_6_message_s *iec_msg = (iec_61883_6_message_s *)avtp_data;
      if (memcmp(iec_msg->stream_id, stream_in_params->stream_id,
                 UNIQUE_ID_LEN) != 0) {
        continue;
      }
      uint16_t stream_data_len =
          (iec_msg->stream_data_len[0] << 8) | iec_msg->stream_data_len[1];
      if (stream_data_len <= 8) continue; // need at least CIP header
      // Skip 8-byte CIP header, remaining is AM824 quadlets
      uint8_t dbs = iec_msg->stream_data[1]; // DBS from CIP header
      channels = dbs;
      if (channels == 0) channels = 8;
      int data_bytes = stream_data_len - 8;
      samples = data_bytes / (channels * 4);
      pcm_data = iec_msg->stream_data + 8; // skip CIP header
      pcm_len = data_bytes;
    } else {
      continue;
    }

    if (samples <= 0 || !pcm_data) continue;

    // Convert AVTP 24-bit audio to 16-bit stereo for I2S output.
    // AVTP AAF: big-endian [MSB, MID, LSB, pad] per channel
    // AVTP AM824: [label, MSB, MID, LSB] per channel
    // I2S output: native-endian 16-bit interleaved stereo (L, R, L, R...)
    // Downmix multi-channel to stereo: ch0 -> L, ch1 -> R
    int i2s_offset = 0;
    for (int s = 0; s < samples && i2s_offset + 4 <= (int)i2s_buf_size; s++) {
      for (int ch = 0; ch < 2; ch++) {
        int src_ch = (ch < channels) ? ch : 0;
        int16_t sample_16 = 0;

        if (subtype == avtp_subtype_aaf) {
          int src_offset = (s * channels + src_ch) * 4;
          if (src_offset + 1 < pcm_len) {
            // Take top 16 bits of 24-bit left-justified: bytes [MSB, MID, ...]
            sample_16 = (int16_t)((pcm_data[src_offset] << 8) |
                                   pcm_data[src_offset + 1]);
          }
        } else {
          // AM824: [label, MSB, MID, LSB]
          int src_offset = (s * channels + src_ch) * 4;
          if (src_offset + 2 < pcm_len) {
            sample_16 = (int16_t)((pcm_data[src_offset + 1] << 8) |
                                   pcm_data[src_offset + 2]);
          }
        }

        // Write native-endian 16-bit
        i2s_buf[i2s_offset + 0] = (uint8_t)(sample_16 & 0xFF);
        i2s_buf[i2s_offset + 1] = (uint8_t)((sample_16 >> 8) & 0xFF);
        i2s_offset += 2;
      }
    }

    /* Write stereo PCM to I2S for codec playback */
    if (i2s_offset > 0) {
      ret = i2s_channel_write(stream_in_params->i2s_tx_handle, i2s_buf,
                              i2s_offset, &bytes_write, timeout_ms);
      if (ret != ESP_OK) {
        avbwarn("I2S write failed: %s",
                ret == ESP_ERR_TIMEOUT ? "timeout" : "error");
        // Don't exit on failure — keep trying
      }
    }
  }
err:
  if (state) {
    state->stream_in_active = false;
  }
  if (state && state->codec_enabled && state->config.codec_handle) {
    esp_codec_dev_close(state->config.codec_handle);
  }
  avberr("Stream in task stopped");
  free(stream_in_data);
  free(i2s_buf);
  free(stream_in_params);
  vTaskDelete(NULL);
}

/* Start the AVB stream input task */
int avb_start_stream_in(avb_state_s *state, uint16_t index) {

  if (!state->stream_in_active) {
    // Setup the stream input params
    struct stream_in_params_s *stream_in_params;
    stream_in_params = calloc(1, sizeof(struct stream_in_params_s));
    stream_in_params->state = state;
    stream_in_params->i2s_tx_handle = state->i2s_tx_handle;

    // Open a dedicated L2TAP socket for the listener to receive VLAN-tagged
    // AVTP stream packets. This must be separate from the main loop's VLAN fd
    // because two tasks cannot share reads on the same fd.
    int stream_fd = open("/dev/net/tap", 0);
    if (stream_fd < 0) {
      avberr("Stream in: failed to open L2TAP: %d", errno);
      free(stream_in_params);
      return ERROR;
    }
    ioctl(stream_fd, L2TAP_S_INTF_DEVICE, state->config.eth_interface);
    uint16_t vlan_etype = 0x8100;
    ioctl(stream_fd, L2TAP_S_RCV_FILTER, &vlan_etype);
    stream_in_params->l2if = stream_fd;
    memcpy(&stream_in_params->stream_id, state->input_streams[index].stream_id,
           UNIQUE_ID_LEN);
    stream_in_params->format =
        state->input_streams[index].stream_format.subtype;
    if (stream_in_params->format == 0x02) { // AAF
      stream_in_params->bit_depth =
          state->input_streams[index].stream_format.aaf_pcm.bit_depth;
      stream_in_params->channels =
          state->input_streams[index].stream_format.aaf_pcm.chan_per_frame;
      stream_in_params->sample_rate =
          state->input_streams[index].stream_format.aaf_pcm.sample_rate;
    } else { // assume IEC 61883-6 AM824
      stream_in_params->bit_depth = state->config.default_bits_per_sample; // ?
      stream_in_params->channels =
          state->input_streams[index].stream_format.am824.dbs;           // ?
      stream_in_params->sample_rate = state->config.default_sample_rate; // ?
    }
    int samples_per_interval = 0;
    // Class B (4ms intervals)
    if (state->input_streams[index].stream_flags.class_b) {
      samples_per_interval = (stream_in_params->sample_rate / 250);
      stream_in_params->interval = 4000;
      // Class A (1ms intervals)
    } else {
      samples_per_interval = (stream_in_params->sample_rate / 1000);
      stream_in_params->interval = 1000;
    }
    // Calculate the buffer size
    int buffer_size = 0;
    if (stream_in_params->format == 0x02) { // AAF
      buffer_size = samples_per_interval * stream_in_params->channels *
                    (stream_in_params->bit_depth / 8);
    } else { // assume IEC 61883-6 AM824
      buffer_size = samples_per_interval * stream_in_params->channels *
                    (stream_in_params->bit_depth / 8);
    }
    stream_in_params->buffer_size = buffer_size;
    if (!stream_in_params) {
      avberr("I2S: No memory for stream in params");
      return ERROR;
    }

    // Start the stream input task
    xTaskCreate(avb_stream_in_task, "AVB-IN", 6144, (void *)stream_in_params,
                20, NULL);
    state->input_streams[index].connected = true;
    return OK;
  }
  avberr("Another instance of AVB-IN is already running");
  return ERROR;
}

/* Process status information request */
static void avb_process_statusreq(avb_state_s *state) {
  avb_status_s *status;

  if (!state->status_req.dest) {
    return; /* No active request */
  }
  // Get the status structure
  status = state->status_req.dest;

  /* TODO: Copy required info to the state structure */
  // do this using status->... = ...

  /* Post semaphore to inform that we are done */
  if (state->status_req.done) {
    sem_post(state->status_req.done);
  }
  state->status_req.done = NULL;
  state->status_req.dest = NULL;
}

/* Main AVB task */
static void avb_task(void *task_param) {

  // Create AVB state structure
  avb_state_s *state = NULL;
  state = calloc(1, sizeof(avb_state_s));
  if (!state) {
    avberr("Failed to allocate memory for AVB state");
    goto err;
  }

  avbinfo("AVB state size: %d bytes", sizeof(avb_state_s));

  // Get configuration from task_param
  if (task_param == NULL) {
    avberr("No configuration provided");
    goto err;
  }

  // Fall back to default ethernet interface if not provided
  avb_config_s *config = (avb_config_s *)task_param;
  if (config->eth_interface == NULL) {
    config->eth_interface = "ETH_DEF";
  }

  // Initialize AVB state
  if (avb_initialize_state(state, config) != OK) {
    avberr("Failed to initialize AVB state, stopping AVB task");
    goto err;
  }

  state->codec_enabled = false;

  if (state->config.talker || state->config.listener) {

    /* Initialize i2s interface to codec */
    if (avb_config_i2s(state) != ESP_OK) {
      avberr("I2S init failed");
      goto err;
    }

    /* Initialize codec */
    if (avb_config_codec(state) != ESP_OK) {
      avberr("Codec init failed");
      goto err;
    } else {
      state->codec_enabled = true;
    }
  }

  // Set up pollfds for each L2TAP interface
  struct pollfd pollfds[AVB_NUM_PROTOCOLS];
  for (int i = 0; i < AVB_NUM_PROTOCOLS; i++) {
    pollfds[i].events = POLLIN;
    pollfds[i].fd = state->l2if[i];
    // Validate file descriptor
    if (pollfds[i].fd < 0) {
      avberr("Invalid file descriptor for interface %d: fd=%d", i,
             pollfds[i].fd);
      continue;
    }
  }
  // allow receive on L2TAP interfaces
  state->l2tap_receive = true;
  // ESP_ERROR_CHECK(esp_eth_update_input_path(state->config.eth_handle,
  // my_callback, NULL));

  bool first_time_receive_off = true;

  // Main AVB loop
  while (!state->stop) {

    if (state->l2tap_receive) {
      /* Wait for a message on any of the L2TAP interfaces */
      if (poll(pollfds, AVB_NUM_PROTOCOLS, AVB_POLL_INTERVAL_MS)) {
        /* Check for events on each L2TAP interface */
        for (int i = 0; i < AVB_NUM_PROTOCOLS; i++) {
          if (pollfds[i].revents) {
            // Skip AVTP fd when a stream input task is actively reading it
            if (i == AVTP && state->stream_in_active) {
              continue;
            }
            /* Get a message from the L2TAP interface */
            int ret =
                avb_net_recv(state->l2if[i], &state->rxbuf[i], AVB_MAX_MSG_LEN,
                             &state->rxtime[i], &state->rxsrc[i]);
            if (ret > 0) {
              /* Process the received message */
              avb_process_rx_message(state, i, ret);
            }
          }
        }
      }
    } else {
      if (first_time_receive_off) {
        vTaskDelay(100);
        first_time_receive_off = false;
        avbinfo("L2TAP receive disabled");
        char statsbuff[500];
        vTaskGetRunTimeStats(statsbuff);
        avbinfo("Run time stats: %s", statsbuff);
      }
    }

    if (state->l2tap_receive) {

      if (state->config.listener || state->config.talker) {
        // Get PTP status
        struct timespec time_now;
        struct timespec delta;
        clock_gettime(CLOCK_MONOTONIC, &time_now);
        clock_timespec_subtract(&time_now, &state->last_ptp_status_update,
                                &delta);
        if (timespec_to_ms(&delta) > PTP_STATUS_UPDATE_INTERVAL_MSEC) {
          state->last_ptp_status_update = time_now;
          avb_update_ptp_status(state);
        }
      }

      // Send periodic messages such as announcing entity available, etc
      avb_periodic_send(state);

      // do validation checking for timed out connections, etc
      // TBD

      // Process status requests
      avb_process_statusreq(state);
    }
  } // while (!state->stop)
err:
  if (state) {
    avb_destroy_state(state);
    free(state);
    s_state = NULL;
  }
  vTaskDelete(NULL);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Start the AVB task */
int avb_start(avb_config_s *config) {
  if (!config->talker && !config->listener) {
    avberr("No talker or listener enabled");
    return ERROR;
  }
  if (s_state == NULL) {
    xTaskCreate(avb_task, "AVB", 12288, (void *)config, 21, NULL);
    return OK;
  }
  avberr("Another instance of AVB is already running");
  return ERROR;
}

/* Query status from a running AVB task */
int avb_status(avb_status_s *status) {
  int ret = 0;
  sem_t donesem;
  avb_statusreq_s req;
  struct timespec timeout;

  /* Fill in the status request */
  memset(status, 0, sizeof(avb_status_s));
  sem_init(&donesem, 0, 0);
  req.done = &donesem;
  req.dest = status;
  s_state->status_req = req;

  /* Wait for status request to be handled */
  clock_gettime(CLOCK_REALTIME, &timeout); // sem_timedwait uses CLOCK_REALTIME
  timeout.tv_sec += 1;
  if (sem_timedwait(&donesem, &timeout) != 0) {
    req.done = NULL;
    req.dest = NULL;
    s_state->status_req = req;
    ret = -errno;
  }
  sem_destroy(&donesem);
  return ret;
}

/* Stop the AVB task */
int avb_stop() {
  s_state->stop = true;
  return OK;
}

/* Get the codec handle */
void *avb_get_codec_handle() {
  if (!s_state->codec_enabled) {
    return NULL;
  }
  return s_state->config.codec_handle;
}

/* Test codec playback task — runs at high priority with pre-computed LUT */
static void test_playback_task(void *param) {
  uint32_t duration_ms = (uint32_t)(uintptr_t)param;

  esp_codec_dev_handle_t codec = s_state->config.codec_handle;

  // Open codec for 16-bit stereo output
  esp_codec_dev_sample_info_t fs = {
      .bits_per_sample = 16,
      .channel = 2,
      .sample_rate = 48000,
      .mclk_multiple = 384,
  };
  int ret = esp_codec_dev_open(codec, &fs);
  if (ret != ESP_CODEC_DEV_OK) {
    avberr("Test playback: codec_dev_open failed: %d", ret);
    vTaskDelete(NULL);
    return;
  }
  esp_codec_dev_set_out_vol(codec, 50.0);

  // Pre-compute one full cycle of the sine wave (48 samples for 1kHz @ 48kHz)
  #define TEST_LUT_LEN 48
  #define TEST_AMP 16000
  int16_t lut[TEST_LUT_LEN];
  for (int i = 0; i < TEST_LUT_LEN; i++) {
    lut[i] = (int16_t)(sinf(2.0f * 3.14159265f * i / TEST_LUT_LEN) * TEST_AMP);
  }

  // Write buffer: 10ms = 480 stereo samples
  #define TEST_BUF_LEN 480
  int16_t buf[TEST_BUF_LEN * 2];
  int lut_pos = 0;

  avbinfo("Test playback: 1kHz sine, 16-bit stereo 48kHz, %lums", duration_ms);

  int64_t end_time = esp_timer_get_time() + (int64_t)duration_ms * 1000;

  while (esp_timer_get_time() < end_time) {
    for (int i = 0; i < TEST_BUF_LEN; i++) {
      int16_t s = lut[lut_pos];
      buf[i * 2 + 0] = s; // L
      buf[i * 2 + 1] = s; // R
      lut_pos = (lut_pos + 1) % TEST_LUT_LEN;
    }
    size_t written = 0;
    i2s_channel_write(s_state->i2s_tx_handle, buf, sizeof(buf), &written, 1000);
  }

  avbinfo("Test playback: done");
  esp_codec_dev_close(codec);
  vTaskDelete(NULL);
}

/* Test codec playback — generates a sine wave and writes to I2S */
esp_err_t avb_test_codec_playback(uint32_t duration_ms) {
  if (!s_state || !s_state->codec_enabled || !s_state->config.codec_handle) {
    avberr("Codec not initialized");
    return ESP_ERR_INVALID_STATE;
  }
  if (duration_ms == 0) duration_ms = 3000;

  xTaskCreatePinnedToCore(test_playback_task, "CODEC-TEST", 8192,
                          (void *)(uintptr_t)duration_ms,
                          configMAX_PRIORITIES - 2, NULL, 0);
  return ESP_OK;
}
