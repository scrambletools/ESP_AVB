/*
 * Copyright 2024 Scramble Tools
 * License: MIT
 *
 * ESP_AVB Component
 *
 * This component provides an implementation of an AVB talker and listener.
 * 
 * This file provides the main entry point for the AVB task.
 */

#include "avb.h" // Internal API

/* Global state */
avb_state_s *s_state;

// logo.png
extern const char logo_png_start[] asm("_binary_logo_png_start");
extern const char logo_png_end[] asm("_binary_logo_png_end");

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* AVB callback function */
IRAM_ATTR esp_err_t my_callback(esp_eth_handle_t eth_handle, uint8_t *buffer, uint32_t length, void *priv) {
    // Process received packet
    if (buffer[12] == 0x22 && buffer[13] == 0xf0 && buffer[14] == 0x00) {
        avbinfo("stream frame %lu",length);
    }
    return ESP_OK;
}

/* Initialize AVB state and create L2TAP FDs */
static int avb_initialize_state(
    avb_state_s *state,
    avb_config_s *config
) {
  // Copy config to state
  memcpy(&state->config, config, sizeof(avb_config_s));

  // Initialize the low level ethernet interface
  int ret = avb_net_init(state);
  if (ret < 0) {
    avberr("Failed to initialize AVB network interface");
    return ERROR;
  }

  // Set entity id based on MAC address and model id
  memcpy(state->own_entity.summary.entity_id, state->internal_mac_addr, ETH_ADDR_LEN);
  uint64_t model_id = CONFIG_EXAMPLE_AVB_MODEL_ID;
  int_to_octets(&model_id, state->own_entity.summary.model_id, 8);

  // Set entity capabilities
  avb_entity_cap_s entity_caps;
  memset(&entity_caps, 0, sizeof(avb_entity_cap_s));
  entity_caps.gptp_supported = 1;
  entity_caps.class_a = 1;
  entity_caps.class_b = 1;
  entity_caps.aem_supported = 1;
  entity_caps.aem_config_index_valid = 1;
  memcpy(&state->own_entity.summary.entity_capabilities, &entity_caps, sizeof(avb_entity_cap_s));

  // Set talker sources and capabilities
  if (config->talker) {
    uint16_t talker_sources = 1;
    int_to_octets(&talker_sources, state->own_entity.summary.talker_stream_sources, 2);
    avb_talker_cap_s talker_caps;
    memset(&talker_caps, 0, sizeof(avb_talker_cap_s));
    talker_caps.implemented = 1;
    talker_caps.audio_source = 1;
    memcpy(&state->own_entity.summary.talker_capabilities, &talker_caps, sizeof(avb_talker_cap_s));
    avbinfo("AVB endpoint configured as TALKER");
  }

  // Set listener sinks and capabilities
  if (config->listener) {
    uint16_t listener_sinks = 1;
    int_to_octets(&listener_sinks, state->own_entity.summary.listener_stream_sinks, 2);
    avb_listener_cap_s listener_caps;
    memset(&listener_caps, 0, sizeof(avb_listener_cap_s));
    listener_caps.implemented = 1;
    listener_caps.audio_sink = 1;
    memcpy(&state->own_entity.summary.listener_capabilities, &listener_caps, sizeof(avb_listener_cap_s));
    avbinfo("AVB endpoint configured as LISTENER");
  }

  // Set entity detail info
  int64_t association_id = CONFIG_ESP_AVB_ASSOCIATION_ID;
  int_to_octets(&association_id, state->own_entity.detail.association_id, UNIQUE_ID_LEN);
  uint8_t entity_name[64] = CONFIG_ESP_AVB_ENTITY_NAME;
  memcpy(state->own_entity.detail.entity_name, entity_name, sizeof(entity_name));
  uint8_t firmware_version[64] = CONFIG_ESP_AVB_FIRMWARE_VERSION;
  memcpy(state->own_entity.detail.firmware_version, firmware_version, sizeof(firmware_version));
  char group_name[64] = CONFIG_ESP_AVB_GROUP_NAME;
  memcpy(state->own_entity.detail.group_name, group_name, sizeof(group_name));
  char serial_number[64] = CONFIG_ESP_AVB_SERIAL_NUMBER;
  memcpy(state->own_entity.detail.serial_number, serial_number, sizeof(serial_number));
  size_t config_num = AEM_MAX_NUM_CONFIGS;
  int_to_octets(&config_num, state->own_entity.detail.configurations_count, 2);
  size_t config_index = DEFAULT_CONFIG_INDEX;
  int_to_octets(&config_index, state->own_entity.detail.current_configuration, 2);

  // setup listener stream flags, and stream info flags, default vlan id and stream format
  avb_listener_stream_flags_s flags = {0};
  // set any flags here
  aem_stream_info_flags_s info_flags = {0};
  // set any info flags here
  uint16_t vlan_id = DEFAULT_VLAN_ID;
  avtp_stream_format_am824_s format = AVB_DEFAULT_FORMAT(cip_sfc_sample_rate_48k);

if (state->config.listener) {
    // Build input streams
    state->num_input_streams = AVB_MAX_NUM_INPUT_STREAMS;
    avb_listener_stream_s input_stream = {0};
    memcpy(&input_stream.stream_flags, &flags, sizeof(avb_listener_stream_flags_s));
    memcpy(&input_stream.stream_info_flags, &info_flags, sizeof(aem_stream_info_flags_s));
    int_to_octets(&vlan_id, input_stream.vlan_id, 2);
    memcpy(&input_stream.stream_format, &format, sizeof(avtp_stream_format_s));
    for (int i = 0; i < state->num_input_streams; i++) {
        memcpy(&state->input_streams[i], &input_stream, sizeof(avb_listener_stream_s));
    }
  }

  if (state->config.talker) {
    // Build output streams
    state->num_output_streams = AVB_MAX_NUM_OUTPUT_STREAMS;
    avb_talker_stream_s output_stream = {0};
    memcpy(&output_stream.stream_info_flags, &info_flags, sizeof(aem_stream_info_flags_s));
    int_to_octets(&vlan_id, output_stream.vlan_id, 2);
    memcpy(&output_stream.stream_format, &format, sizeof(avtp_stream_format_s));
    for (int i = 0; i < state->num_output_streams; i++) {
        memcpy(&state->output_streams[i], &output_stream, sizeof(avb_talker_stream_s));
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
  }
}

/* Send periodic messages */
static int avb_periodic_send(avb_state_s *state) {
    struct timespec time_now;
    struct timespec delta;
    clock_gettime(CLOCK_MONOTONIC, &time_now);

    // Send ADP entity available message
    clock_timespec_subtract(&time_now, &state->last_transmitted_adp_entity_avail, &delta);
    if (timespec_to_ms(&delta) > ADP_ENTITY_AVAIL_INTERVAL_MSEC) {
        state->last_transmitted_adp_entity_avail = time_now;
        avb_send_adp_entity_available(state);
    }

    // Send MVRP VLAN ID message
    clock_timespec_subtract(&time_now, &state->last_transmitted_mvrp_vlan_id, &delta);
    if (timespec_to_ms(&delta) > MVRP_VLAN_ID_INTERVAL_MSEC) {
        state->last_transmitted_mvrp_vlan_id = time_now;
        avb_send_mvrp_vlan_id(state, mrp_attr_event_join_mt, false);
    }

    // Send MSRP domain message
    clock_timespec_subtract(&time_now, &state->last_transmitted_msrp_domain, &delta);
    if (timespec_to_ms(&delta) > MSRP_DOMAIN_INTERVAL_MSEC) {
        state->last_transmitted_msrp_domain = time_now;
        avb_send_msrp_domain(state, mrp_attr_event_join_mt, false);
    }

    // Send MSRP talker and AVTP MAAP announce messages
    if (state->config.talker) {
        // clock_timespec_subtract(&time_now, &state->last_transmitted_msrp_talker_adv, &delta);
        // if (timespec_to_ms(&delta) > MSRP_TALKER_ADV_INTERVAL_MSEC) {
        //   state->last_transmitted_msrp_talker_adv = time_now;
        //   unique_id_t stream_id = {0};
        //   avb_send_msrp_talker(state, mrp_attr_event_join_in, stream_id, false);
        // }
        // clock_timespec_subtract(&time_now, &state->last_transmitted_maap_announce, &delta);
        // if (timespec_to_ms(&delta) > MAAP_ANNOUNCE_INTERVAL_MSEC) {
        //   state->last_transmitted_maap_announce = time_now;
        //   avb_send_maap_announce(state);
        // }
    }

    // Send MSRP listener message
    if (state->config.listener) {
        clock_timespec_subtract(&time_now, &state->last_transmitted_msrp_listener, &delta);
        for (int i = 0; i < state->num_input_streams; i++) { 
            // if connected and time to send
            if (state->input_streams[i].connected && timespec_to_ms(&delta) > MSRP_LISTENER_CONN_INTERVAL_MSEC) {
                state->last_transmitted_msrp_listener = time_now;
                avb_send_msrp_listener(
                    state, 
                    mrp_attr_event_join_mt, 
                    msrp_listener_event_ready, 
                    false,
                    &state->input_streams[i].stream_id);
            // if idle and time to send
            } else if (timespec_to_ms(&delta) > MSRP_LISTENER_IDLE_INTERVAL_MSEC) {
                state->last_transmitted_msrp_listener = time_now;
                avb_send_msrp_listener(
                    state, 
                    mrp_attr_event_mt, 
                    msrp_listener_event_ready, 
                    false,
                    &state->input_streams[i].stream_id);
            }
        }
    }

    // if time to send leaveAll
    clock_timespec_subtract(&time_now, &state->last_transmitted_msrp_leaveall, &delta);
    if (timespec_to_ms(&delta) > MSRP_LEAVEALL_INTERVAL_MSEC) {
        state->last_transmitted_msrp_leaveall = time_now;
        avb_send_msrp_domain(state, mrp_attr_event_mt, true);
    }

    // Send Unsolicited notifications
    if (state->unsol_notif_enabled) {

        if (state->config.talker) {
            // Send get counters stream_output notification
            for (int i = 0; i < state->num_output_streams; i++) {
                // if the output stream is active
                if (octets_to_uint(state->output_streams[i].connection_count, 2) > 0) {
                    clock_timespec_subtract(&time_now, &state->last_transmitted_unsol_notif, &delta);
                    if (timespec_to_ms(&delta) > UNSOL_NOTIF_INTERVAL_MSEC) {
                        state->last_transmitted_unsol_notif = time_now;
                        avb_send_aecp_unsol_get_counters(state, aem_desc_type_stream_output, i);
                    }
                }
            }
        }
        if (state->config.listener) {
            // Send get counters stream_input notification
            for (int i = 0; i < state->num_input_streams; i++) {
                // if the input stream is active
                if (state->input_streams[i].connected) {
                    clock_timespec_subtract(&time_now, &state->last_transmitted_unsol_notif, &delta);
                    if (timespec_to_ms(&delta) > UNSOL_NOTIF_INTERVAL_MSEC) {
                        state->last_transmitted_unsol_notif = time_now;
                        avb_send_aecp_unsol_get_counters(state, aem_desc_type_stream_input, i);
                    }
                }
            }
        }
    }
    return OK;
} // avb_periodic_send

/* Determine received message type and process it */

static int avb_process_rx_message(
    avb_state_s *state,
    int protocol_idx,
    ssize_t length
) {
  // Check if the message length is valid
  if (length <= ETH_HEADER_LEN) {
    avbwarn("Ignoring invalid message, length only %d bytes", (int)length);
        return OK;
  }
  eth_addr_t src_addr;
  memcpy(&src_addr, &state->rxsrc[protocol_idx], ETH_ADDR_LEN);

  // String representation of source address
  char src_addr_str[ETH_ADDR_LEN * 3 + 1];
  octets_to_hex_string((uint8_t*)src_addr, ETH_ADDR_LEN, src_addr_str, ':');

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
          avbinfo("***** Got an AAF message from %s", src_addr_str);
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
      while (header.attr_type != msrp_attr_type_none && offset < sizeof(msrp_msgbuf_s)) {
        size_t attr_size = octets_to_uint(header.attr_list_len, 2) + 4; // 4 bytes for the header w/o vechead
        switch (header.attr_type) {
          case msrp_attr_type_domain:
            avb_process_msrp_domain(state, msg, offset, attr_size);
            break;
          case msrp_attr_type_talker_advertise:
            avb_process_msrp_talker(state, msg, offset, attr_size, false, &src_addr);
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

  struct stream_in_params_s *stream_in_params = (struct stream_in_params_s *)task_param;
  if (stream_in_params == NULL) {goto err;}

  uint8_t *stream_in_data = calloc(1, stream_in_params->buffer_size);
  if (!stream_in_data) {
    avberr("I2S: No memory for read data buffer");
    goto err;
  }
  int ret = 0;
  struct timespec ts;
  eth_addr_t src_addr;
  size_t bytes_read = 0;
  size_t bytes_write = 0;
  size_t timeout_ms = 1000;
  // Set up pollfd for AVTP interface
  struct pollfd poll_fd = {
    .events = POLLIN,
    .fd = stream_in_params->l2if
  };
  // Validate file descriptor
  if (poll_fd.fd < 0) {
    avberr("Invalid file descriptor for interface fd=%d (AVTP)", poll_fd.fd);
    goto err;
  }

  while (1) {
    memset(stream_in_data, 0, stream_in_params->buffer_size);

    /* Read data from stream input */

    /* Wait for a packet on the L2TAP interface */
    if (poll(&poll_fd, 1, stream_in_params->interval / 1000)) {
      /* Check for events on each L2TAP interface */
      if (poll_fd.revents) {
        /* Get a packet from the L2TAP interface */
        ret = avb_net_recv(stream_in_params->l2if, stream_in_data, stream_in_params->buffer_size, &ts, &src_addr);
        if (ret <= 0) {
          avberr("Failed to read from stream in");
          goto err;
        }
        /* Write sample data to earphone */
        aaf_pcm_message_s *aaf_msg = (aaf_pcm_message_s *)stream_in_data;
        if ((aaf_msg->subtype == avtp_subtype_aaf) 
        && (memcmp(aaf_msg->stream_id, stream_in_params->stream_id, UNIQUE_ID_LEN) == 0)) {
          memcpy(aaf_msg->stream_id, stream_in_params->stream_id, UNIQUE_ID_LEN);
          int ret = i2s_channel_write(stream_in_params->i2s_tx_handle, stream_in_data, stream_in_params->buffer_size, &bytes_write, timeout_ms);
          if (ret != ESP_OK) {
            avberr("I2S: write failed, %s", ret == ESP_ERR_TIMEOUT ? "timeout" : "unknown");
            goto err;
          }
          if (bytes_read != bytes_write) {
            avbwarn("I2S: %d bytes read but only %d bytes are written", bytes_read, bytes_write);
          }
        }
      }
    }    
  }
err:
  avberr("Stream in task stopped");
  vTaskDelete(NULL);
}

/* Start the AVB stream input task */
int avb_start_stream_in(avb_state_s *state, uint16_t index) {

  if (!state->input_streams[index].connected) {
    // Setup the stream input params
    struct stream_in_params_s *stream_in_params;
    stream_in_params = calloc(1, sizeof(struct stream_in_params_s));
    stream_in_params->i2s_tx_handle = state->i2s_tx_handle;
    stream_in_params->l2if = state->l2if[AVTP];
    memcpy(&stream_in_params->stream_id, state->input_streams[index].stream_id, UNIQUE_ID_LEN);
    stream_in_params->format = state->input_streams[index].stream_format.subtype;
    if (stream_in_params->format == 0x02) { // AAF
        stream_in_params->bit_depth = state->input_streams[index].stream_format.aaf_pcm.bit_depth;
        stream_in_params->channels = state->input_streams[index].stream_format.aaf_pcm.chan_per_frame;
        stream_in_params->sample_rate = state->input_streams[index].stream_format.aaf_pcm.sample_rate;
    } else { // assume IEC 61883-6 AM824
        stream_in_params->bit_depth = state->config.default_bits_per_sample; // ?
        stream_in_params->channels = state->input_streams[index].stream_format.am824.dbs; // ?
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
        buffer_size = samples_per_interval 
            * stream_in_params->channels 
            * (stream_in_params->bit_depth / 8);
    } else { // assume IEC 61883-6 AM824
        buffer_size = samples_per_interval 
            * stream_in_params->channels 
            * (stream_in_params->bit_depth / 8);
    }
    stream_in_params->buffer_size = buffer_size;
    if (!stream_in_params) {
      avberr("I2S: No memory for stream in params");
      return ERROR;
    }

    // Start the stream input task
    xTaskCreate(avb_stream_in_task, "AVB-IN", 6144, (void *)stream_in_params, 20, NULL);
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

    /* Initialize i2s interface to codec */
    if (avb_config_i2s(state) != ESP_OK) {
        avberr("I2S init failed");
        goto err;
    }

    /* Initialize codec */
    ESP_ERROR_CHECK(avb_config_codec(state));

    // Set up pollfds for each L2TAP interface
    struct pollfd pollfds[AVB_NUM_PROTOCOLS];
    for (int i = 0; i < AVB_NUM_PROTOCOLS; i++) {
        pollfds[i].events = POLLIN;
        pollfds[i].fd = state->l2if[i];
        // Validate file descriptor
        if (pollfds[i].fd < 0) {
        avberr("Invalid file descriptor for interface %d: fd=%d", i, pollfds[i].fd);
        continue;
        }
    }
    // allow receive on L2TAP interfaces
    state->l2tap_receive = true;
    //ESP_ERROR_CHECK(esp_eth_update_input_path(state->config.eth_handle, my_callback, NULL));

    bool first_time_receive_off = true;
    
    // Main AVB loop
    while (!state->stop) {

        if (state->l2tap_receive) {
            /* Wait for a message on any of the L2TAP interfaces */
            if (poll(pollfds, AVB_NUM_PROTOCOLS, AVB_POLL_INTERVAL_MS)) {
                /* Check for events on each L2TAP interface */
                for (int i = 0; i < AVB_NUM_PROTOCOLS; i++) {
                    if (pollfds[i].revents) {
                        /* Get a message from the L2TAP interface */
                        int ret = avb_net_recv(
                            state->l2if[i], 
                            &state->rxbuf[i], 
                            AVB_MAX_MSG_LEN, 
                            &state->rxtime[i], 
                            &state->rxsrc[i]);
                        if (ret > 0) {
                            /* ignore active streams as they are too fast */
                            // if (i == AVTP
                            // && (state->rxbuf[i].avtp.subtype == avtp_subtype_aaf
                            // || state->rxbuf[i].avtp.subtype == avtp_subtype_61883)) {
                            //     avbinfo("*");
                            // } 
                            //if (i == AVTP) avbinfo("*");
                            /* Process the received message */
                            avb_process_rx_message(state, i, ret);
                        }
                    }
                }
            }
        }
        else {
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

        // Get PTP status
        struct timespec time_now;
        struct timespec delta;
        clock_gettime(CLOCK_MONOTONIC, &time_now);
        clock_timespec_subtract(&time_now, &state->last_ptp_status_update, &delta);
        if (timespec_to_ms(&delta) > PTP_STATUS_UPDATE_INTERVAL_MSEC) {
            state->last_ptp_status_update = time_now;
            avb_update_ptp_status(state);
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
int avb_start(avb_config_s *config)
{
  if (!config->talker && !config->listener) {
    avberr("No talker or listener enabled");
    return ERROR;
  }
  if (s_state == NULL) {
    xTaskCreate(avb_task, "AVB", 12288,
              (void *)config, 21, NULL);
    return OK;
  }
  avberr("Another instance of AVB is already running");
  return ERROR;
}

/* Query status from a running AVB task */
int avb_status(avb_status_s *status)
{
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
  if (sem_timedwait(&donesem, &timeout) != 0)
    {
      req.done = NULL;
      req.dest = NULL;
      s_state->status_req = req;
      ret = -errno;
    }
  sem_destroy(&donesem);
  return ret;
}

/* Stop the AVB task */
int avb_stop()
{
  s_state->stop = true;
  return OK;
}

/* Get the codec handle */
void *avb_get_codec_handle() {
  return s_state->config.codec_handle;
}
