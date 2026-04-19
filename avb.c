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

#include "avb.h"
#include "esp_codec_dev.h"
#include "esp_timer.h"
#include <esp_task_wdt.h>
#include <nvs_flash.h>

/* Global state */
avb_state_s *s_state;

// logo.png
extern const char logo_png_start[] asm("_binary_logo_png_start");
extern const char logo_png_end[] asm("_binary_logo_png_end");

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
  entity_caps.class_b = false;
  entity_caps.aem_supported = true;
  entity_caps.aem_config_index_valid = true;
  entity_caps.aem_identify_control_index_valid = true;
  /* vendor_unique_supported signals the Milan MVU protocol. Only advertise
   * it when CONFIG_ESP_AVB_MILAN is enabled so non-Milan controllers (e.g.
   * macOS native AVDECC) don't run Milan-specific validation against us. */
#if CONFIG_ESP_AVB_MILAN
  entity_caps.vendor_unique_supported = true;
#else
  entity_caps.vendor_unique_supported = false;
#endif
  entity_caps.address_access_supported = true;
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

  // Set listener sinks and capabilities. When talker is enabled we still
  // advertise the CRF media clock input (stream index 1), so listener_sinks
  // = AVB_MAX_NUM_INPUT_STREAMS any time either role is active.
  if (config->listener || config->talker) {
    uint16_t listener_sinks = AVB_MAX_NUM_INPUT_STREAMS;
    int_to_octets(&listener_sinks,
                  state->own_entity.summary.listener_stream_sinks, 2);
    avb_listener_cap_s listener_caps;
    memset(&listener_caps, 0, sizeof(avb_listener_cap_s));
    listener_caps.implemented = true;
    listener_caps.audio_sink = 1;
    memcpy(&state->own_entity.summary.listener_capabilities, &listener_caps,
           sizeof(avb_listener_cap_s));
    if (config->listener)
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
  // avtp_stream_format_am824_s sf0 =
  //     AVB_DEFAULT_FORMAT_AM824(cip_sfc_sample_rate_44_1k);
  avtp_stream_format_am824_s sf0 =
      AVB_DEFAULT_FORMAT_AM824(cip_sfc_sample_rate_48k);
  // avtp_stream_format_am824_s sf2 =
  //     AVB_DEFAULT_FORMAT_AM824(cip_sfc_sample_rate_96k);
  uint8_t aaf_ch = 8; // 8 channels to match MOTU/Apple AVB expectations
  avtp_stream_format_aaf_pcm_s sf1 =
      AVB_DEFAULT_FORMAT_AAF(24, aaf_pcm_sample_rate_48k, aaf_ch, false);
  avtp_stream_format_aaf_pcm_s sf2 =
      AVB_DEFAULT_FORMAT_AAF(32, aaf_pcm_sample_rate_48k, aaf_ch, false);
  state->supported_formats[0].am824 = sf0;
  state->supported_formats[1].aaf_pcm = sf1;
  state->supported_formats[2].aaf_pcm = sf2;
  // state->supported_formats[2].am824 = sf2;
  // state->supported_formats[3].aaf_pcm = sf3;
  // state->supported_formats[4].aaf_pcm = sf4;
  // state->supported_formats[5].aaf_pcm = sf5;
  // state->supported_formats[6].aaf_pcm = sf6;
  state->num_supported_formats = 3;
  avtp_stream_format_aaf_pcm_s format = sf1;

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

  // Build input streams — stream 0 is AAF audio, stream 1 is the Milan v1.3
  // §7.2.2 CRF media clock input. The CRF input is required whenever we
  // advertise an AAF talker, so set up both input streams whenever either
  // talker or listener is enabled.
  if (state->config.listener || state->config.talker) {
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
    /* Stream 1 carries the Milan AVnu CRF format */
    uint8_t crf_bytes[8] = MILAN_AVNU_CRF_FORMAT_BYTES;
    memcpy(&state->input_streams[AVB_CRF_INPUT_INDEX].stream_format, crf_bytes,
           sizeof(crf_bytes));
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
      /* Stream dest addr will be set by MAAP after address acquisition */
      memset(&state->output_streams[i].stream_dest_addr, 0, ETH_ADDR_LEN);
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

  // Initialize MAAP for output stream multicast address acquisition
  if (config->talker) {
    avb_maap_init(state);
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

  // Send MVRP VLAN ID message — always JoinIn so the switch registers
  // VLAN membership on this port from boot. Required for MSRP Listener
  // registrations to be accepted by the switch.
  clock_timespec_subtract(&time_now, &state->last_transmitted_mvrp_vlan_id,
                          &delta);
  if (timespec_to_ms(&delta) > MVRP_VLAN_ID_INTERVAL_MSEC) {
    state->last_transmitted_mvrp_vlan_id = time_now;
    avb_send_mvrp_vlan_id(state, mrp_attr_event_join_in, false);
  }

  // Send MSRP domain message — use JoinIn when connected
  clock_timespec_subtract(&time_now, &state->last_transmitted_msrp_domain,
                          &delta);
  if (timespec_to_ms(&delta) > MSRP_DOMAIN_INTERVAL_MSEC) {
    state->last_transmitted_msrp_domain = time_now;
    avb_send_msrp_domain(state, mrp_attr_event_join_in, false);
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
    avb_maap_tick(state);
  }

  // Send MSRP listener message
  if (state->config.listener) {
    clock_timespec_subtract(&time_now, &state->last_transmitted_msrp_listener,
                            &delta);
    for (int i = 0; i < state->num_input_streams; i++) {
      if (state->input_streams[i].connected &&
          timespec_to_ms(&delta) > MSRP_LISTENER_CONN_INTERVAL_MSEC) {
        state->last_transmitted_msrp_listener = time_now;
        avb_send_msrp_listener(state, mrp_attr_event_join_in,
                               msrp_listener_event_ready, false,
                               &state->input_streams[i].stream_id);
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
      // Don't reset last_transmitted_msrp_listener here — let the periodic
      // path manage its own timer independently from LeaveAll.
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

  /* Attempt fast-connect for any listener stream with a saved binding
   * that hasn't reconnected yet. Self-throttled per-stream. */
  if (state->config.listener) {
    avb_periodic_fast_connect(state);
  }

  return OK;
} // avb_periodic_send

/* Determine received message type and process it */

static int avb_process_rx_message(avb_state_s *state, int protocol_idx,
                                  ssize_t length) {
  // Length is payload only (ETH header already stripped by EMAC dispatcher)
  if (length <= 0) {
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
        avb_process_msrp_listener(state, msg, offset, attr_size, &src_addr);
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
    /* VLAN stream data is handled directly by the EMAC RX dispatcher
     * via the registered stream handler — never reaches this function. */
  }
  return OK;
}

/* Stream-in code is in avtp.c (avb_start_stream_in, avb_stop_stream_in).
 * Diagnostics accessor declared there as avb_stream_in_get_ctx(). */

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

  // Load persistent data from NVS — must be after codec init so
  // persisted volume/gain override the codec defaults
  avb_persist_load(state);

  // Apply persisted codec values to hardware
  if (state->codec_enabled) {
    avb_codec_set_vol(state, state->ctrl_speaker_vol);
    avb_codec_set_mic_gain(state, state->ctrl_mic_gain);
  }

  // Main AVB loop — receive control frames from EMAC RX dispatcher queue,
  // process them, and handle periodic tasks. VLAN stream data is handled
  // directly by the registered stream handler in the EMAC RX task.
  while (!state->stop) {
    int protocol_idx;
    eth_addr_t src_addr;
    /* Use max(AVB_POLL_INTERVAL_MS, one tick) to guarantee yield.
     * With 100Hz ticks, pdMS_TO_TICKS(1) = 0 which busy-loops. */
    int ret = avb_net_recv_ctrl(state, &protocol_idx, &state->rxbuf[0],
                                AVB_MAX_MSG_LEN, &src_addr, portTICK_PERIOD_MS);
    if (ret > 0 && protocol_idx >= 0 && protocol_idx < AVB_NUM_PROTOCOLS) {
      /* Copy payload and source addr into the protocol-indexed slot so
       * avb_process_rx_message can access them at state->rxbuf[idx] */
      if (protocol_idx != 0) {
        memmove(&state->rxbuf[protocol_idx], &state->rxbuf[0], ret);
      }
      memcpy(&state->rxsrc[protocol_idx], &src_addr, ETH_ADDR_LEN);
      avb_process_rx_message(state, protocol_idx, ret);
    }

    // Get PTP status (needed for ADP and AECP responses, not just streaming)
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

    // Stream-in diagnostics — avb_stream_in_print_diag (avtp.c) prints
    // one-shot first-packet info, safe to call from main loop context.
    avb_stream_in_print_diag();

    // Media-clock PLL: measure, log, apply MCLK correction
    avb_pll_tick(state);

    // Process status requests
    avb_process_statusreq(state);
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
    xTaskCreate(avb_task, "AVB", 16384, (void *)config, 21, NULL);
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
  if (duration_ms == 0)
    duration_ms = 3000;

  xTaskCreatePinnedToCore(test_playback_task, "CODEC-TEST", 8192,
                          (void *)(uintptr_t)duration_ms,
                          configMAX_PRIORITIES - 2, NULL, 0);
  return ESP_OK;
}

/* Identify tone task — plays a 24-bit 1kHz sine wave through I2S TX
 * without reconfiguring the codec. Self-deleting. */
static void identify_tone_task(void *param) {
  avb_state_s *state = (avb_state_s *)param;
  if (!state || !state->i2s_tx_handle) {
    vTaskDelete(NULL);
    return;
  }

  /* 1kHz sine LUT, 48 samples/cycle at 48kHz, ~50% amplitude */
  static const int32_t sine48[48] = {
      0,    544665,  1079631, 1595279, 2082235, 2532439, 2938203, 3293268,
      3592842, 3833605, 4013711, 4132776, 4191840, 4193292, 4140750, 4039013,
      3893852, 3711777, 3499778, 3265042, 3014650, 2755321, 2493117, 2233135,
      1979243, 1734805, 1502529, 1284363, 1081490, 894262, 722287, 564457,
      419026, 283660, 155571, 31687, -91418, -217017, -348206, -488058,
      -639633, -805996, -990239, -1195498, -1425006, -1682125, -1970389, -2293550
  };

  int frames_per_ms = 48;
  uint8_t buf[48 * 6]; /* 1ms worth of 24-bit stereo */
  uint32_t duration_ms = 500;
  uint32_t phase = 0;

  avbinfo("Identify tone: %lums", duration_ms);

  int64_t end_time = esp_timer_get_time() + (int64_t)duration_ms * 1000;
  while (esp_timer_get_time() < end_time) {
    uint8_t *p = buf;
    for (int i = 0; i < frames_per_ms; i++) {
      int32_t val = sine48[phase % 48];
      /* 24-bit little-endian stereo: [LSB, MID, MSB, LSB, MID, MSB] */
      p[0] = val & 0xFF;
      p[1] = (val >> 8) & 0xFF;
      p[2] = (val >> 16) & 0xFF;
      p[3] = p[0]; p[4] = p[1]; p[5] = p[2];
      p += 6;
      phase++;
    }
    size_t bw = 0;
    i2s_channel_write(state->i2s_tx_handle, buf, sizeof(buf), &bw, 10);
  }

  avbinfo("Identify tone: done");
  vTaskDelete(NULL);
}

/* Play an identify tone — spawns a short-lived task */
void avb_identify_tone(avb_state_s *state, uint32_t duration_ms) {
  xTaskCreatePinnedToCore(identify_tone_task, "IDENTIFY", 4096, (void *)state,
                          configMAX_PRIORITIES - 2, NULL, 0);
}

/****************************************************************************
 * NVS Persistent Storage
 ****************************************************************************/

#define AVB_NVS_NAMESPACE "avb"
#define AVB_NVS_KEY "persist"

/* Helper: true if any byte in an 8-byte array is non-zero.
 * Used to distinguish "never saved" (all zero) from a valid persisted
 * stream_format, since the AM824 format has subtype byte 0x00. */
static inline bool any_nonzero8(const uint8_t *b) {
  return (b[0] | b[1] | b[2] | b[3] | b[4] | b[5] | b[6] | b[7]) != 0;
}

/* Populate the persist struct from current state */
static void avb_persist_gather(avb_state_s *state) {
  avb_persistent_data_s *p = &state->persist;
  memset(p, 0, sizeof(*p));
  p->version = AVB_PERSIST_VERSION;

  /* Descriptor names */
  int n = AVB_NAME_COUNT < AVB_PERSIST_MAX_NAMES ? AVB_NAME_COUNT
                                                 : AVB_PERSIST_MAX_NAMES;
  memcpy(p->descriptor_names, state->descriptor_names, n * 64);

  /* Per-input-stream state (connection + format + streaming_wait) */
  int n_in = state->num_input_streams < AVB_PERSIST_MAX_INPUT_STREAMS
                 ? state->num_input_streams
                 : AVB_PERSIST_MAX_INPUT_STREAMS;
  for (int i = 0; i < n_in; i++) {
    avb_persist_input_stream_s *dst = &p->input_streams[i];
    const avb_listener_stream_s *src = &state->input_streams[i];
    memcpy(dst->talker_id, src->talker_id, 8);
    memcpy(dst->talker_uid, src->talker_uid, 2);
    memcpy(dst->controller_id, src->controller_id, 8);
    memcpy(dst->stream_format, &src->stream_format, 8);
    dst->connected = src->connected ? 1 : 0;
    dst->streaming_wait = src->stream_flags.streaming_wait ? 1 : 0;
  }

  /* Per-output-stream state (format; presentation offset is placeholder) */
  int n_out = state->num_output_streams < AVB_PERSIST_MAX_OUTPUT_STREAMS
                  ? state->num_output_streams
                  : AVB_PERSIST_MAX_OUTPUT_STREAMS;
  for (int i = 0; i < n_out; i++) {
    avb_persist_output_stream_s *dst = &p->output_streams[i];
    const avb_talker_stream_s *src = &state->output_streams[i];
    memcpy(dst->stream_format, &src->stream_format, 8);
    dst->presentation_time_offset_ns = 0; /* TODO: wire once talker exposes it */
  }

  /* Controls */
  p->speaker_vol_db = state->ctrl_speaker_vol;
  p->mic_gain_db = state->ctrl_mic_gain;
  p->active_clock_source_index = state->media_clock.active_clock_source_index;
  p->audio_unit_sample_rate_hz = 0; /* TODO: wire when sample-rate policy lands */
}

/* Apply loaded persist data to current state */
static void avb_persist_apply(avb_state_s *state) {
  avb_persistent_data_s *p = &state->persist;

  /* Descriptor names */
  int n = AVB_NAME_COUNT < AVB_PERSIST_MAX_NAMES ? AVB_NAME_COUNT
                                                 : AVB_PERSIST_MAX_NAMES;
  memcpy(state->descriptor_names, p->descriptor_names, n * 64);

  /* Copy entity name and group name into their canonical locations too,
   * since the entity descriptor builder reads from there */
  if (state->descriptor_names[AVB_NAME_ENTITY][0])
    memcpy(state->own_entity.detail.entity_name,
           state->descriptor_names[AVB_NAME_ENTITY], 64);
  if (state->descriptor_names[AVB_NAME_GROUP][0])
    memcpy(state->own_entity.detail.group_name,
           state->descriptor_names[AVB_NAME_GROUP], 64);
  if (state->descriptor_names[AVB_NAME_AVB_INTERFACE][0])
    memcpy(state->avb_interface.object_name,
           state->descriptor_names[AVB_NAME_AVB_INTERFACE], 64);

  /* Per-input-stream state */
  int n_in = AVB_MAX_NUM_INPUT_STREAMS < AVB_PERSIST_MAX_INPUT_STREAMS
                 ? AVB_MAX_NUM_INPUT_STREAMS
                 : AVB_PERSIST_MAX_INPUT_STREAMS;
  for (int i = 0; i < n_in; i++) {
    const avb_persist_input_stream_s *src = &p->input_streams[i];
    avb_listener_stream_s *dst = &state->input_streams[i];
    if (any_nonzero8(src->stream_format))
      memcpy(&dst->stream_format, src->stream_format, 8);
    /* Restore the binding identity only. Do NOT restore `connected` —
     * stream_id, stream_dest_addr, and vlan_id are derived on reconnect
     * (not persisted), so setting connected=true here would make
     * GET_RX_STATE report "connected=1 with zero stream_id/dest", which
     * both violates Milan §5.5.3.6.16 (fields must be zero when not
     * settled) and triggers a Hive enumeration fatal. Fast-connect
     * fires on any stream with a non-zero talker_id and, on success,
     * avb_connect_listener atomically sets connected=true together
     * with stream_id / stream_dest_addr. */
    if (any_nonzero8(src->talker_id)) {
      memcpy(dst->talker_id, src->talker_id, 8);
      memcpy(dst->talker_uid, src->talker_uid, 2);
      memcpy(dst->controller_id, src->controller_id, 8);
      dst->stream_flags.streaming_wait = src->streaming_wait ? 1 : 0;
      avbinfo("NVS: restored binding for stream_input %d (pending fast-connect)",
              i);
    }
  }

  /* Per-output-stream state */
  int n_out = AVB_MAX_NUM_OUTPUT_STREAMS < AVB_PERSIST_MAX_OUTPUT_STREAMS
                  ? AVB_MAX_NUM_OUTPUT_STREAMS
                  : AVB_PERSIST_MAX_OUTPUT_STREAMS;
  for (int i = 0; i < n_out; i++) {
    const avb_persist_output_stream_s *src = &p->output_streams[i];
    avb_talker_stream_s *dst = &state->output_streams[i];
    if (any_nonzero8(src->stream_format))
      memcpy(&dst->stream_format, src->stream_format, 8);
    /* src->presentation_time_offset_ns ignored for now — placeholder */
  }

  /* Controls — apply if non-zero (default struct is zeroed) */
  if (p->speaker_vol_db != 0.0f || p->mic_gain_db != 0.0f) {
    state->ctrl_speaker_vol = p->speaker_vol_db;
    state->ctrl_mic_gain = p->mic_gain_db;
  }
  /* active_clock_source_index: 0 is a valid value (INTERNAL/gPTP), so we
   * always restore it — but only if the blob looks non-trivial. A freshly
   * zeroed persist struct would leave this at 0, which happens to be the
   * compile-time default anyway, so unconditional restore is safe. */
  state->media_clock.active_clock_source_index = p->active_clock_source_index;
  /* p->audio_unit_sample_rate_hz ignored for now — placeholder */
}

/* Load persistent data from NVS */
esp_err_t avb_persist_load(avb_state_s *state) {
  /* Ensure NVS is initialized */
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
      err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    err = nvs_flash_init();
  }
  if (err != ESP_OK) {
    avberr("NVS: flash init failed: %d", err);
    return err;
  }

  bool need_reinit = false; /* true -> overwrite flash with a fresh v2 blob */

  nvs_handle_t handle;
  err = nvs_open(AVB_NVS_NAMESPACE, NVS_READONLY, &handle);
  if (err != ESP_OK) {
    avbinfo("NVS: no saved data (namespace not found) — initializing");
    need_reinit = true;
  } else {
    /* Zero the whole struct first so any fields that aren't present in
     * an older (shorter) stored blob stay zero after nvs_get_blob. */
    memset(&state->persist, 0, sizeof(state->persist));
    size_t stored_size = sizeof(avb_persistent_data_s);
    err = nvs_get_blob(handle, AVB_NVS_KEY, &state->persist, &stored_size);
    nvs_close(handle);

    if (err != ESP_OK) {
      avbinfo("NVS: no saved data (key not found) — initializing");
      need_reinit = true;
    } else if (state->persist.version < 2 ||
               state->persist.version > AVB_PERSIST_VERSION ||
               stored_size > sizeof(avb_persistent_data_s)) {
      /* Acceptance policy:
       *  - version < 2: legacy blob with config-derived array sizes.
       *  - version > current: written by newer firmware.
       *  - stored_size > sizeof(current): struct shrank/reshaped.
       * Forward-compat for stored_size < sizeof(current) is handled by
       * the append-only rule + memset above. */
      avbwarn("NVS: incompatible blob (size=%d ver=%d) — discarding and reinitializing",
              (int)stored_size, state->persist.version);
      memset(&state->persist, 0, sizeof(avb_persistent_data_s));
      need_reinit = true;
    } else {
      avb_persist_apply(state);
      avbinfo("NVS: loaded persistent data (%d bytes, version %d)",
              (int)stored_size, state->persist.version);
    }
  }

  /* Replace missing/stale blobs with a fresh v2 snapshot of current
   * state so the warning doesn't re-fire on every subsequent boot.
   * apply() guards against clobbering runtime defaults with zeros,
   * so it's safe to save here even if state is mostly default. */
  if (need_reinit) {
    avb_persist_save(state);
  }

  return err;
}

/* Save persistent data to NVS */
esp_err_t avb_persist_save(avb_state_s *state) {
  avb_persist_gather(state);

  nvs_handle_t handle;
  esp_err_t err = nvs_open(AVB_NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    avberr("NVS: failed to open for writing: %d", err);
    return err;
  }

  err = nvs_set_blob(handle, AVB_NVS_KEY, &state->persist,
                     sizeof(avb_persistent_data_s));
  if (err != ESP_OK) {
    avberr("NVS: failed to write: %d", err);
    nvs_close(handle);
    return err;
  }

  err = nvs_commit(handle);
  nvs_close(handle);
  avbinfo("NVS: saved persistent data (%d bytes)",
          (int)sizeof(avb_persistent_data_s));
  return err;
}
