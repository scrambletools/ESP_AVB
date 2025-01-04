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
struct avb_state_s *s_state;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Initialize AVB state and create L2TAP FD - FIXME 3 FDs needed */
static int avb_initialize_state(struct avb_state_s *state,
                                const char *interface) {
  int ret = 0;

  // Initialize the network interface
  ret = avb_net_init(state, interface);
  if (ret < 0) {
    avberr("Failed to initialize network interface");
    return ERROR;
  }

  // Set entity id based on MAC address and model id
  memcpy(state->own_entity.summary.entity_id, state->internal_mac_addr, ETH_ADDR_LEN);
  uint64_t model_id = CONFIG_ESP_AVB_MODEL_ID;
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
  if (state->config.talker) {
    uint16_t talker_sources = 1;
    int_to_octets(&talker_sources, state->own_entity.summary.talker_stream_sources, 2);
    avb_talker_cap_s talker_caps;
    memset(&talker_caps, 0, sizeof(avb_talker_cap_s));
    talker_caps.implemented = 1;
    talker_caps.audio_source = 1;
    memcpy(&state->own_entity.summary.talker_capabilities, &talker_caps, sizeof(avb_talker_cap_s));
  }

  // Set listener sinks and capabilities
  if (state->config.listener) {
    uint16_t listener_sinks = 1;
    int_to_octets(&listener_sinks, state->own_entity.summary.listener_stream_sinks, 2);
    avb_listener_cap_s listener_caps;
    memset(&listener_caps, 0, sizeof(avb_listener_cap_s));
    listener_caps.implemented = 1;
    listener_caps.audio_sink = 1;
    memcpy(&state->own_entity.summary.listener_capabilities, &listener_caps, sizeof(avb_listener_cap_s));
  }

  // Set controller capabilities
  if (state->config.controller) {
    avb_controller_cap_s controller_caps;
    memset(&controller_caps, 0, sizeof(avb_controller_cap_s));
    controller_caps.implemented = 1;
    memcpy(&state->own_entity.summary.controller_capabilities, &controller_caps, sizeof(avb_controller_cap_s));
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
  size_t config_num = AVB_MAX_NUM_CONFIGS;
  int_to_octets(&config_num, state->own_entity.detail.configurations_count, 2);
  size_t config_index = DEFAULT_CONFIG_INDEX;
  int_to_octets(&config_index, state->own_entity.detail.current_configuration, 2);

  // Get latest PTP status
  struct ptpd_status_s ptp_status;
  // if valid PTP status
  if (ptpd_status(0, &ptp_status) == 0) {
    state->ptp_status = ptp_status;
  }

  // Set stop to false
  state->stop = false;

  // Set global state
  s_state = state;
  return OK;
}

/* Destroy l2ifs */
static int avb_destroy_state(struct avb_state_s *state) {

  for (int i = 0; i < AVB_NUM_PROTOCOLS; i++) {
    if (state->l2if[i] > 0) {
      close(state->l2if[i]);
      state->l2if[i] = -1;
    }
  }
  return OK;
}

/* Get latest PTP status */
static void avb_update_ptp_status(struct avb_state_s *state) {
  // if valid PTP status
  struct ptpd_status_s ptp_status;
  // if valid PTP status
  if (ptpd_status(0, &ptp_status) == 0) {
    memcpy(&state->ptp_status, &ptp_status, sizeof(struct ptpd_status_s));
  }
}

/* Send periodic messages */
static int avb_periodic_send(struct avb_state_s *state) {
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
    avb_send_mvrp_vlan_id(state);
  }

  // Send MSRP domain message
  clock_timespec_subtract(&time_now, &state->last_transmitted_msrp_domain, &delta);
  if (timespec_to_ms(&delta) > MSRP_DOMAIN_INTERVAL_MSEC) {
    state->last_transmitted_msrp_domain = time_now;
    avb_send_msrp_domain(state);
  }

  // Send MSRP talker and AVTP MAAP announce messages
  if (state->config.talker) {
    clock_timespec_subtract(&time_now, &state->last_transmitted_msrp_talker_adv, &delta);
    if (timespec_to_ms(&delta) > MSRP_TALKER_ADV_INTERVAL_MSEC) {
      state->last_transmitted_msrp_talker_adv = time_now;
      avb_send_msrp_talker(state, msrp_attr_event_join_in, false);
    }
    clock_timespec_subtract(&time_now, &state->last_transmitted_maap_announce, &delta);
    if (timespec_to_ms(&delta) > MAAP_ANNOUNCE_INTERVAL_MSEC) {
      state->last_transmitted_maap_announce = time_now;
      avb_send_maap_announce(state);
    }
  }

  // Send MSRP listener message
  if (state->config.listener) {
    clock_timespec_subtract(&time_now, &state->last_transmitted_msrp_listener_ready, &delta);
    if (timespec_to_ms(&delta) > MSRP_LISTENER_READY_INTERVAL_MSEC) {
      state->last_transmitted_msrp_listener_ready = time_now;
      avb_send_msrp_listener(state, msrp_attr_event_join_in, msrp_listener_event_ready);
    }
  }
  return OK;
}

/* Determine received message type and process it */
static int avb_process_rx_message(struct avb_state_s *state,
                                  int protocol_idx,
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
  octets_to_hex_string((uint8_t*)src_addr, ETH_ADDR_LEN, src_addr_str, ':');

  /* Route the message to the appropriate handler */
  switch (protocol_idx) {
    case AVTP: {
      avtp_msgbuf_u *msg = (avtp_msgbuf_u *)&state->rxbuf[protocol_idx].avtp;
      switch (msg->subtype) {
        case avtp_subtype_aaf:
          avbinfo("Got an AAF message from %s", src_addr_str);
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

    /* Read sample data from stream in */

    /* Wait for a message on the L2TAP interface */
    if (poll(&poll_fd, 1, stream_in_params->interval / 1000)) {
      /* Check for events on each L2TAP interface */
      if (poll_fd.revents) {
        /* Get a message from the L2TAP interface */
        ret = avb_net_recv(stream_in_params->l2if, stream_in_data, stream_in_params->buffer_size, &ts, &src_addr);
        if (ret <= 0) {
          avberr("Failed to read from stream in");
          goto err;
        }
      }
    }

    /* Write sample data to earphone */
    aaf_pcm_message_s *aaf_msg = (aaf_pcm_message_s *)stream_in_data;
    if (aaf_msg->stream_id == stream_in_params->stream_id) {
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
err:
  avberr("Stream in task stopped");
  vTaskDelete(NULL);
}

/* Start the AVB stream input task */
int avb_start_stream_in(struct avb_state_s *state, unique_id_t *stream_id) {

  // create a string representation of the stream id for error messages
  char stream_id_str[UNIQUE_ID_LEN * 3 + 1];
  octets_to_hex_string((uint8_t*)stream_id, UNIQUE_ID_LEN, stream_id_str, '-');

  // get the connection from the stream id
  int index = avb_find_connection_by_id(state, stream_id, avb_entity_type_listener);
  if (index < 0) {
    avberr("No connection found for stream %s", stream_id_str);
    return ERROR;
  }

  // check that the connection has a talker id
  if (memcmp(&state->connections[index].talker_id, &EMPTY_ID, UNIQUE_ID_LEN) == 0) {
    avberr("No talker ID for stream %s", stream_id_str);
    return ERROR;
  }

  // find the talker based on the talker id
  char talker_id_str[UNIQUE_ID_LEN * 3 + 1];
  octets_to_hex_string((uint8_t*)&state->connections[index].talker_id, UNIQUE_ID_LEN, talker_id_str, '-');
  int talker_idx = avb_find_entity_by_id(state, &state->connections[index].talker_id, avb_entity_type_talker);
  if (talker_idx < 0) {
    avberr("Talker %s not found for stream %s among %d talkers", talker_id_str, stream_id_str, state->num_talkers);
    return ERROR;
  }

  // if talker stream format is not set, then return error
  avb_talker_s talker = state->talkers[talker_idx];
  if (talker.stream.stream_format.bit_depth == 0) {
    avberr("Talker %s stream format not set", talker_id_str);
    return ERROR;
  }

  if (!state->connections[index].active) {
    // Setup the stream input params
    struct stream_in_params_s *stream_in_params;
    stream_in_params = calloc(1, sizeof(struct stream_in_params_s));
    stream_in_params->i2s_tx_handle = state->config.i2s_tx_handle;
    stream_in_params->l2if = state->l2if[AVTP];
    memcpy(&stream_in_params->stream_id, stream_id, UNIQUE_ID_LEN);
    stream_in_params->bit_depth = talker.stream.stream_format.bit_depth;
    stream_in_params->channels = talker.stream.stream_format.channels_per_frame;
    stream_in_params->sample_rate = talker.stream.stream_format.nsr;
    stream_in_params->format = talker.stream.stream_format.format;
    int samples_per_interval = 0;
    // Class A (1ms intervals)
    if (talker.info.priority == 3) { 
      samples_per_interval = (talker.stream.stream_format.nsr / 1000);
      stream_in_params->interval = 1000;
    // Class B (4ms intervals)
    } else { 
      samples_per_interval = (talker.stream.stream_format.nsr / 250);
      stream_in_params->interval = 4000;
    }
    int buffer_size = samples_per_interval \
        * talker.stream.stream_format.channels_per_frame \
        * (talker.stream.stream_format.bit_depth / 8);
    stream_in_params->buffer_size = buffer_size;
    if (!stream_in_params) {
      avberr("I2S: No memory for stream in params");
      return ERROR;
    }
    // Start the stream input task
    xTaskCreate(avb_stream_in_task, "AVB-IN", 6144,
              (void *)stream_in_params, 21, NULL);
    state->connections[index].active = true;
    return OK;
  }
  avberr("Another instance of AVB-IN is already running");
  return ERROR;
}

/* Process status information request */
static void avb_process_statusreq(struct avb_state_s *state) {
  struct avb_status_s *status;

  if (!state->status_req.dest) {
    return; /* No active request */
  }
  //
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

  // Get configuration from task_param
  struct avb_config_s *config = (struct avb_config_s *)task_param;

  const char *interface = "eth0"; // default interface
  struct avb_state_s *state;
  struct pollfd pollfds[AVB_NUM_PROTOCOLS];
  int ret = 0;
  state = calloc(1, sizeof(struct avb_state_s));
  memset(state, 0, sizeof(struct avb_state_s));
  avbinfo("AVB state size: %d bytes", sizeof(struct avb_state_s));

  // Get interface from config if provided
  if (task_param != NULL) {
    interface = config->interface;
    memcpy(&state->config, config, sizeof(struct avb_config_s));
  }

  // Initialize AVB state
  if (avb_initialize_state(state, interface) != OK) {
    avberr("Failed to initialize AVB state, stopping AVB task");
    avb_destroy_state(state);
    free(state);
    goto err;
  }

  // Set up pollfds for each L2TAP interface
  for (int i = 0; i < AVB_NUM_PROTOCOLS; i++) {
    pollfds[i].events = POLLIN;
    pollfds[i].fd = state->l2if[i];
    // Validate file descriptor
    if (pollfds[i].fd < 0) {
      avberr("Invalid file descriptor for interface %d: fd=%d", i, pollfds[i].fd);
      continue;
    }
  }

  // Main AVB loop
  while (!state->stop) {

    /* Wait for a message on any of the L2TAP interfaces */
    if (poll(pollfds, AVB_NUM_PROTOCOLS, AVB_POLL_INTERVAL_MS)) {
      /* Check for events on each L2TAP interface */
      for (int i = 0; i < AVB_NUM_PROTOCOLS; i++) {
        if (pollfds[i].revents) {
          /* Get a message from the L2TAP interface */
          ret = avb_net_recv(state->l2if[i], &state->rxbuf[i], AVB_MAX_MSG_LEN, &state->rxtime[i], &state->rxsrc[i]);
          if (ret > 0) {
            /* Process the received message */
            avb_process_rx_message(state, i, ret);
          }
        }
      }
    }

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
  } // while (!state->stop)

  avb_destroy_state(state);
  free(state);

err:
  s_state = NULL;
  vTaskDelete(NULL);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* @brief Start the AVB task and bind it to a specified interface
 * 
 * @param interface Name of the network interface to bind to, e.g. "eth0"
 * 
 * @return OK on success, ERROR on failure
 * 
 * @note Only one instance of AVB task can run at a time. Attempting to 
 *       start multiple instances will fail with an error. CONFIG_ESP_AVB_STACKSIZE
 */
int avb_start(struct avb_config_s *config)
{
  if (s_state == NULL) {
    xTaskCreate(avb_task, "AVB", 6144,
              (void *)config, 20, NULL);
    return OK;
  }
  avberr("Another instance of AVB is already running");
  return ERROR;
}

/* @brief Query status from a running AVB task
 * 
 * @param status Pointer to status storage structure
 * 
 * @return OK on success, negative errno on failure
 * 
 * @note Multiple threads with priority less than CONFIG_ESP_AVB_TASKPRIO
 *       can request status simultaneously. If higher priority threads 
 *       request status simultaneously, some of the requests may timeout.
 */
int avb_status(struct avb_status_s *status)
{
  int ret = 0;
  sem_t donesem;
  struct avb_statusreq_s req;
  struct timespec timeout;

  /* Fill in the status request */
  memset(status, 0, sizeof(struct avb_status_s));
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

/* @brief Stop the AVB task
 * 
 * @return OK on success, negative errno on failure
 **/
int avb_stop()
{
  s_state->stop = true;
  return OK;
}
