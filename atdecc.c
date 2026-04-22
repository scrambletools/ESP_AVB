/*
 * Copyright 2024-2026 Scramble Tools
 * License: MIT
 *
 * ESP_AVB Component
 *
 * This component provides an implementation of an AVB talker and listener.
 *
 * This file provides the required features of the ATDECC protocol.
 */

#include "avb.h"
#include <stddef.h> /* offsetof */

/* Send ADP entity available message */
int avb_send_adp_entity_available(avb_state_s *state) {
  adp_message_s msg;
  struct timespec ts;
  int ret;
  size_t body_size = 56; // ADP Entity Available message body length
  memset(&msg, 0, sizeof(msg));

  // Populate the message
  msg.header.subtype = avtp_subtype_adp;
  msg.header.msg_type = adp_msg_type_entity_available;
  msg.header.version = 0;
  msg.header.status_valtime = 10; // valid time: 20 seconds (Milan v1.3)
  msg.header.control_data_len = body_size;
  memcpy(&msg.entity, &state->own_entity.summary, sizeof(avb_entity_summary_s));
  memcpy(msg.gptp_gm_id, state->ptp_status.clock_source_info.gm_id, 8);
  msg.gptp_domain_num = 0;
  uint64_t association_id = CONFIG_ESP_AVB_ASSOCIATION_ID;
  int_to_octets(&association_id, msg.association_id, 8);

  uint16_t msg_len = 4 + body_size + 8; // header + body
  ret = avb_net_send(state, ethertype_avtp, &msg, msg_len, &ts);
  if (ret < 0) {
    avberr("send ADP Entity Available failed: %d", errno);
  }
  // Increment available index
  uint32_t index = octets_to_uint(state->own_entity.summary.available_index, 4);
  index++;
  int_to_octets(&index, state->own_entity.summary.available_index, 4);
  return ret;
}

/* Send AECP command controller available message */
int avb_send_aecp_cmd_controller_available(avb_state_s *state,
                                           unique_id_t *target_id) {
  // not implemented
  return OK;
}

/* Send AECP command get stream info message */
int avb_send_aecp_cmd_get_stream_info(avb_state_s *state,
                                      unique_id_t *target_id) {
  aecp_get_stream_info_s msg;
  struct timespec ts;
  int ret;
  size_t body_size = 24; // AEC Get Stream Info message body length
  memset(&msg, 0, sizeof(msg));

  // Populate the message
  msg.common.header.subtype = avtp_subtype_aecp;
  msg.common.header.msg_type = aecp_msg_type_aem_command;
  msg.common.header.status_valtime = 8; // valid time: 16 seconds
  msg.common.header.control_data_len = body_size;
  memcpy(msg.common.target_entity_id, target_id, UNIQUE_ID_LEN);
  memcpy(msg.common.controller_entity_id, state->own_entity.summary.entity_id,
         UNIQUE_ID_LEN);
  size_t command_type = aecp_cmd_code_get_stream_info;
  int_to_octets(&command_type, &msg.aem.command_type, 2);
  int descriptor_type = aem_desc_type_stream_output;
  int_to_octets(&descriptor_type, msg.descriptor_type, 2);

  uint16_t msg_len = 4 + body_size; // header + body
  ret = avb_net_send(state, ethertype_avtp, &msg, msg_len, &ts);
  if (ret < 0) {
    avberr("send AECP Get Stream Info failed: %d", errno);
  }
  avbinfo("sent AECP Get Stream Info");
  return ret;
}

/* Send AECP response get stream info message */
int avb_send_aecp_rsp_get_stream_info(avb_state_s *state,
                                      aecp_get_stream_info_s *msg,
                                      eth_addr_t *dest_addr) {
  int ret = OK;
  struct timespec ts;
  uint16_t index = octets_to_uint(msg->descriptor_index, 2);
  uint16_t descriptor_type = octets_to_uint(msg->descriptor_type, 2);

  // set if input or output
  bool is_output = descriptor_type == aem_desc_type_stream_output;

  // check if the index is out of range
  if ((is_output && index >= state->num_output_streams) ||
      (!is_output && index >= state->num_input_streams)) {
    avberr("stream descriptor index out of range: %d", index);
    ret = ERROR;
  }

  // create a response message
  aecp_get_stream_info_rsp_s response = {0};
  memcpy(&response, msg, sizeof(aecp_get_stream_info_s));
  response.common.header.msg_type = aecp_msg_type_aem_response;

  // populate the response message from the appropriate stream
  static const uint8_t zero_id[UNIQUE_ID_LEN] = {0};
  static const eth_addr_t zero_mac = {0};
  static const uint8_t zero_lat[4] = {0};

  if (is_output) {
    // output stream info
    avb_talker_stream_s *s = &state->output_streams[index];
    aem_stream_info_flags_s flags = s->stream_info_flags;
    flags.stream_id_valid = memcmp(s->stream_id, zero_id, UNIQUE_ID_LEN) != 0;
    flags.stream_format_valid = 1;
    flags.stream_dest_mac_valid =
        memcmp(s->stream_dest_addr, zero_mac, ETH_ADDR_LEN) != 0;
    flags.msrp_acc_lat_valid =
        memcmp(s->msrp_accumulated_latency, zero_lat, 4) != 0;
    flags.msrp_failure_valid = s->msrp_failure_code[0] != 0;
    flags.connected = octets_to_uint(s->connection_count, 2) > 0;
    memcpy(&response.stream.flags, &flags, sizeof(aem_stream_info_flags_s));
    memcpy(&response.stream.stream_format, &s->stream_format,
           sizeof(avtp_stream_format_s));
    memcpy(&response.stream.stream_id, &s->stream_id, UNIQUE_ID_LEN);
    memcpy(&response.stream.msrp_accumulated_latency,
           &s->msrp_accumulated_latency, 4);
    memcpy(&response.stream.dest_addr, &s->stream_dest_addr,
           sizeof(eth_addr_t));
    memcpy(&response.stream.msrp_failure_code, &s->msrp_failure_code, 2);
    memcpy(&response.vlan_id, &s->vlan_id, 2);
  } else {
    // input stream info
    avb_listener_stream_s *s = &state->input_streams[index];
    aem_stream_info_flags_s flags = s->stream_info_flags;
    flags.stream_format_valid = 1;
    flags.stream_dest_mac_valid =
        memcmp(s->stream_dest_addr, zero_mac, ETH_ADDR_LEN) != 0;
    flags.msrp_acc_lat_valid =
        memcmp(s->msrp_accumulated_latency, zero_lat, 4) != 0;
    flags.msrp_failure_valid = s->msrp_failure_code[0] != 0;
    flags.connected = s->connected;
    memcpy(&response.stream.flags, &flags, sizeof(aem_stream_info_flags_s));
    memcpy(&response.stream.stream_format, &s->stream_format,
           sizeof(avtp_stream_format_s));
    memcpy(&response.stream.stream_id, &s->stream_id, UNIQUE_ID_LEN);
    memcpy(&response.stream.msrp_accumulated_latency,
           &s->msrp_accumulated_latency, 4);
    memcpy(&response.stream.dest_addr, &s->stream_dest_addr,
           sizeof(eth_addr_t));
    memcpy(&response.stream.msrp_failure_code, &s->msrp_failure_code, 2);
    memcpy(&response.vlan_id, &s->vlan_id, 2);
  }

  // calc control data length
  uint16_t control_data_len =
      sizeof(aecp_get_stream_info_rsp_s) - AVTP_CDL_PREAMBLE_LEN;

  // set the control data length
  response.common.header.control_data_len_h = (control_data_len >> 8) & 0xFF;
  response.common.header.control_data_len = control_data_len & 0xFF;

  // send the response message
  uint16_t msg_len = sizeof(aecp_get_stream_info_rsp_s);
  ret = avb_net_send_to(state, ethertype_avtp, &response, msg_len, &ts,
                        dest_addr);
  if (ret < 0) {
    avberr("send AECP get stream info response failed: %d", errno);
  }
  return ret;
}

/* Send AECP response set stream format message */
int avb_send_aecp_rsp_set_stream_format(avb_state_s *state,
                                        aecp_stream_format_s *msg,
                                        eth_addr_t *dest_addr) {
  int ret = OK;
  struct timespec ts;

  // set the message type to response
  msg->common.header.msg_type = aecp_msg_type_aem_response;

  // set the control data length
  uint16_t control_data_len =
      sizeof(aecp_stream_format_s) - AVTP_CDL_PREAMBLE_LEN;
  msg->common.header.control_data_len_h = (control_data_len >> 8) & 0xFF;
  msg->common.header.control_data_len = control_data_len & 0xFF;

  // send the response
  uint16_t msg_len = sizeof(aecp_stream_format_s);
  ret = avb_net_send_to(state, ethertype_avtp, msg, msg_len, &ts, dest_addr);
  if (ret < 0) {
    avberr("send AECP Set Stream Format response failed: %d", errno);
  }
  return ret;
}

/* Send AECP unsolicited notification get stream info response */
// sends to all regitered notification recipients
int avb_send_aecp_unsol_get_stream_info(avb_state_s *state, uint16_t index,
                                        bool is_output) {
  // not implemented
  return OK;
}

/* Send AECP command get stream info message */
int avb_send_aecp_cmd_get_counters(avb_state_s *state, unique_id_t *target_id) {
  // not implemented
  return OK;
}

/* Send AECP response get counters message */
// may be sent as an unsolicited notification
int avb_send_aecp_rsp_get_counters(avb_state_s *state, aecp_get_counters_s *msg,
                                   eth_addr_t *dest_addr) {
  // not implemented
  return OK;
}

/* Send AECP unsolicited notification get stream info response */
// sends to all regitered notification recipients
int avb_send_aecp_unsol_get_counters(avb_state_s *state,
                                     aem_desc_type_t descriptor_type,
                                     uint16_t index) {
  // not implemented
  return OK;
}

/* Send AECP response get descriptor for entity message */
int avb_send_aecp_rsp_read_descr_entity(avb_state_s *state,
                                        aecp_read_descriptor_rsp_s *msg,
                                        eth_addr_t *dest_addr) {
  int ret = OK;
  struct timespec ts;
  uint16_t control_data_len = sizeof(aecp_read_descriptor_rsp_s) -
                              sizeof(atdecc_header_s) - AEM_MAX_DESC_LEN;

  // create a descriptor
  aem_entity_desc_s descriptor;
  memcpy(&descriptor, &state->own_entity, sizeof(aem_entity_desc_s));
  descriptor.detail.vendor_name_ref[1] = 12; // strings desc 1, string_4
  descriptor.detail.model_name_ref[1] = 13;  // strings desc 1, string_5

  // insert the descriptor data
  memcpy(msg->descriptor_data, &state->own_entity, sizeof(aem_entity_desc_s));
  /* descriptor_type and descriptor_index (4 bytes) are already counted
   * in the initial control_data_len via sizeof(aecp_read_descriptor_rsp_s);
   * only add the descriptor body here. */
  control_data_len += sizeof(aem_entity_desc_s);

  // set the control data length
  msg->common.header.control_data_len_h = (control_data_len >> 8) & 0xFF;
  msg->common.header.control_data_len = control_data_len & 0xFF;

  // send the response message
  uint16_t msg_len =
      sizeof(atdecc_header_s) + sizeof(unique_id_t) + control_data_len;
  ret = avb_net_send_to(state, ethertype_avtp, msg, msg_len, &ts, dest_addr);
  if (ret < 0) {
    avberr("send AECP Read Descriptor response failed: %d", errno);
  }
  return ret;
}

/* Send AECP response get descriptor for configuration message */
int avb_send_aecp_rsp_read_descr_configuration(avb_state_s *state,
                                               aecp_read_descriptor_rsp_s *msg,
                                               eth_addr_t *dest_addr) {
  int ret = OK;
  struct timespec ts;
  uint16_t control_data_len = sizeof(aecp_read_descriptor_rsp_s) -
                              sizeof(atdecc_header_s) - AEM_MAX_DESC_LEN;

  // create a configuration descriptor
  aem_config_desc_s config_desc = {0};

  // give it a name
  // 3bits for base_strings offset, 3bits for index in strings desc
  uint16_t localized_description = 0;
  int_to_octets(&localized_description, config_desc.localized_description, 2);

  // These are the top level descriptors that will be listed in the descriptor
  // counts Eventually, the entire entity model should probably be stored in the
  // state
  uint16_t descriptor_counts_count = 8; // assume at least talker or listener
  if (state->config.listener && state->config.talker) {
    descriptor_counts_count++; // if both talker and listener, then one more
  }
  uint16_t descriptors[descriptor_counts_count];
  int i = 0;
  descriptors[i++] = aem_desc_type_audio_unit;
  if (state->config.listener) {
    descriptors[i++] = aem_desc_type_stream_input;
  }
  if (state->config.talker) {
    descriptors[i++] = aem_desc_type_stream_output;
  }
  descriptors[i++] = aem_desc_type_avb_interface;
  descriptors[i++] = aem_desc_type_clock_source;
  descriptors[i++] = aem_desc_type_memory_object;
  descriptors[i++] = aem_desc_type_locale;
  descriptors[i++] = aem_desc_type_control;
  descriptors[i++] = aem_desc_type_clock_domain;

  // build the descriptor
  for (int i = 0; i < descriptor_counts_count; i++) {
    aem_config_desc_count_s desc_count = {0};
    int_to_octets(&descriptors[i], desc_count.descriptor_type, 2);
    size_t count;
    switch (descriptors[i]) {
    case aem_desc_type_control:
      count = AEM_NUM_CONTROLS;
      break;
    case aem_desc_type_stream_input:
      /* Stream input 0: AAF audio; stream input 1: CRF media clock */
      count = state->num_input_streams;
      break;
    case aem_desc_type_stream_output:
      count = state->num_output_streams;
      break;
    case aem_desc_type_clock_source:
      /* Clock source 0: gPTP (INTERNAL); clock source 1: CRF stream input */
      count = 2;
      break;
    default:
      count = AEM_MAX_DESC_COUNT;
      break;
    }
    int_to_octets(&count, desc_count.count, 2);
    memcpy(&config_desc.descriptor_counts[i], &desc_count,
           sizeof(aem_config_desc_count_s));
  }

  // insert the descriptor data into the response message
  int_to_octets(&descriptor_counts_count, config_desc.descriptor_counts_count,
                2);
  uint16_t offset = 74; // As defined in section 7.2.2
  int_to_octets(&offset, config_desc.descriptor_counts_offset, 2);
  memcpy(msg->descriptor_data, &config_desc, sizeof(aem_config_desc_s));
  /* descriptor_type and descriptor_index (4 bytes) are already counted
   * in the initial control_data_len; only add the descriptor body, minus
   * the unused slots in the descriptor_counts array. */
  control_data_len +=
      sizeof(aem_config_desc_s) -
      (4 * (AEM_MAX_NUM_DESC - descriptor_counts_count));

  // set the control data length
  msg->common.header.control_data_len_h = (control_data_len >> 8) & 0xFF;
  msg->common.header.control_data_len = control_data_len & 0xFF;

  // send the response message
  uint16_t msg_len =
      sizeof(atdecc_header_s) + sizeof(unique_id_t) + control_data_len;
  ret = avb_net_send_to(state, ethertype_avtp, msg, msg_len, &ts, dest_addr);
  if (ret < 0) {
    avberr("send AECP Read Descriptor response failed: %d", errno);
  }
  return ret;
}

/* Send AECP response get descriptor for audio unit message */
int avb_send_aecp_rsp_read_descr_audio_unit(avb_state_s *state,
                                            aecp_read_descriptor_rsp_s *msg,
                                            eth_addr_t *dest_addr) {
  int ret = OK;
  struct timespec ts;
  uint16_t control_data_len = sizeof(aecp_read_descriptor_rsp_s) -
                              sizeof(atdecc_header_s) - AEM_MAX_DESC_LEN;

  // data for the audio unit descriptor
  int localized_description = 1;
  /* STREAM_PORT counts — only audio streams have a port. The Milan CRF
   * media clock input (stream_input index 1) feeds the clock domain, not an
   * audio map, so it has no STREAM_PORT_INPUT. Hence the port counts stay
   * at 1 even though num_input_streams = 2. */
  int num_input_ports = state->config.listener ? 1 : 0;
  int num_output_ports = state->num_output_streams;
  int sampling_rate = state->config.default_sample_rate;
  int offset = 144; // 144 for this version of AEM
  int sampling_rates_count = state->config.supported_sample_rates.num_rates;
  uint32_t sampling_rates[sampling_rates_count];
  for (int i = 0; i < sampling_rates_count; i++) {
    sampling_rates[i] = state->config.supported_sample_rates.sample_rates[i];
  }

  // create an audio unit descriptor
  aem_audio_unit_desc_s descriptor = {0};

  // populate the audio unit descriptor
  int_to_octets(&localized_description, descriptor.localized_description, 2);
  if (state->config.listener) {
    int_to_octets(&num_input_ports, descriptor.num_stream_input_ports, 2);
  }
  if (state->config.talker) {
    int_to_octets(&num_output_ports, descriptor.num_stream_output_ports, 2);
  }
  int_to_octets(&sampling_rate, descriptor.current_sampling_rate, 4);
  int_to_octets(&offset, descriptor.sampling_rate_offset, 2);
  int_to_octets(&sampling_rates_count, descriptor.sampling_rates_count, 2);
  for (int i = 0; i < sampling_rates_count; i++) {
    int_to_octets(&sampling_rates[i], (uint8_t *)&descriptor.sampling_rates[i],
                  4);
  }

  // insert the descriptor into the response message
  memcpy(msg->descriptor_data, &descriptor, sizeof(aem_audio_unit_desc_s));
  /* descriptor_type and descriptor_index (4 bytes) are already counted
   * in the initial control_data_len; only add the descriptor body, minus
   * the unused slots in the sampling_rates array. */
  control_data_len +=
      sizeof(aem_audio_unit_desc_s) -
      (sizeof(aem_sample_rate_t) *
       (AEM_MAX_NUM_SAMPLE_RATES - sampling_rates_count));

  // set the control data length
  msg->common.header.control_data_len_h = (control_data_len >> 8) & 0xFF;
  msg->common.header.control_data_len = control_data_len & 0xFF;

  // send the response message
  uint16_t msg_len =
      sizeof(atdecc_header_s) + sizeof(unique_id_t) + control_data_len;
  ret = avb_net_send_to(state, ethertype_avtp, msg, msg_len, &ts, dest_addr);
  if (ret < 0) {
    avberr("send AECP Read Descriptor response failed: %d", errno);
  }
  return ret;
}

/* Send AECP response get descriptor for stream input or output message */
// TODO: change to use the input_streams and output_streams arrays from the
// state also move the supported formats to the state
int avb_send_aecp_rsp_read_descr_stream(avb_state_s *state,
                                        aecp_read_descriptor_rsp_s *msg,
                                        eth_addr_t *dest_addr, bool is_output) {
  int ret = OK;
  struct timespec ts;
  uint16_t index = octets_to_uint(msg->descriptor_index, 2);

  // data for the stream input descriptor
  uint16_t localized_description = 4; // see below for strings
  avtp_stream_format_s current_format =
      state->input_streams[index].stream_format;
  if (is_output) {
    localized_description = 5;
    current_format = state->output_streams[index].stream_format;
  }
  aem_stream_flags_s stream_flags = {0};
  stream_flags.class_a = true;
  stream_flags.class_b = true;

  // print the current format
  // char current_format_str[200];
  // octets_to_binary_string((uint8_t *)&current_format, sizeof(current_format),
  // current_format_str); avbinfo("current format: %s", current_format_str);

  /* formats_offset is an offset from the start of the descriptor (which per
   * IEEE 1722.1 begins at the descriptor_type field, 4 bytes before the
   * object_name[] field where aem_stream_desc_s begins in memory). So the
   * on-wire value is offsetof within the struct plus 4. */
  int formats_offset = (int)offsetof(aem_stream_desc_s, formats) + 4;

  /* Milan v1.3 §5.3.3.4: a stream supporting CRF must not also support AAF.
   * The CRF input (index 1) advertises only the Milan AVnu CRF format; all
   * other streams advertise the shared AAF/AM824 supported_formats list. */
  bool is_crf_input =
      !is_output && index == AVB_CRF_INPUT_INDEX;
  avtp_stream_format_s crf_format = {0};
  uint8_t crf_bytes[8] = MILAN_AVNU_CRF_FORMAT_BYTES;
  memcpy(&crf_format, crf_bytes, sizeof(crf_bytes));

  const avtp_stream_format_s *formats_list;
  int number_of_formats;
  if (is_crf_input) {
    formats_list = &crf_format;
    number_of_formats = 1;
    current_format = crf_format;
  } else {
    formats_list = state->supported_formats;
    number_of_formats = state->num_supported_formats;
  }

  int buffer_length = 8; // 8ns ingress buffer
  int redundant_offset = formats_offset + 8 * number_of_formats;

  // create a stream input descriptor
  aem_stream_desc_s descriptor;
  memset(&descriptor, 0, sizeof(aem_stream_desc_s));

  // populate the stream descriptor — copy persisted name if set
  if (is_output && index < AVB_MAX_NUM_OUTPUT_STREAMS)
    memcpy(descriptor.object_name,
           state->descriptor_names[AVB_NAME_STREAM_OUTPUT_0 + index], 64);
  else if (!is_output && index < AVB_MAX_NUM_INPUT_STREAMS)
    memcpy(descriptor.object_name,
           state->descriptor_names[AVB_NAME_STREAM_INPUT_0 + index], 64);
  int_to_octets(&localized_description, descriptor.localized_description, 2);
  int_to_octets(&stream_flags, (uint8_t *)&descriptor.stream_flags, 2);
  memcpy(&descriptor.current_format, &current_format,
         sizeof(avtp_stream_format_s));
  int_to_octets(&formats_offset, descriptor.formats_offset, 2);
  int_to_octets(&number_of_formats, descriptor.number_of_formats, 2);
  int_to_octets(&buffer_length, descriptor.buffer_length, 4);
  int_to_octets(&redundant_offset, descriptor.redundant_offset, 2);
  memcpy(&descriptor.formats, formats_list,
         sizeof(avtp_stream_format_s) * number_of_formats);

  // insert the descriptor into the response message
  memcpy(msg->descriptor_data, &descriptor, sizeof(aem_stream_desc_s));

  // calc control data length
  uint16_t control_data_len =
      AECP_DESC_PREAMBLE_LEN + sizeof(aem_stream_desc_s) -
      (AEM_MAX_NUM_FORMATS - number_of_formats) *
          sizeof(avtp_stream_format_s); // resize for actual number of formats

  // set the control data length
  msg->common.header.control_data_len_h = (control_data_len >> 8) & 0xFF;
  msg->common.header.control_data_len = control_data_len & 0xFF;

  // send the response message
  uint16_t msg_len =
      sizeof(atdecc_header_s) + sizeof(unique_id_t) + control_data_len;
  ret = avb_net_send_to(state, ethertype_avtp, msg, msg_len, &ts, dest_addr);
  if (ret < 0) {
    avberr("send AECP Read Descriptor response failed: %d", errno);
  }
  return ret;
}

/* Update AVB interface descriptor with current PTP status */
void avb_update_avb_interface_from_ptp(avb_state_s *state) {
  memcpy(state->avb_interface.clock_identity,
         state->ptp_status.own_identity_info.id, sizeof(unique_id_t));
  state->avb_interface.priority1 =
      state->ptp_status.clock_source_info.priority1;
  state->avb_interface.clock_class =
      state->ptp_status.clock_source_info.clockclass;
  uint16_t oslv = state->ptp_status.clock_source_info.variance;
  int_to_octets(&oslv, state->avb_interface.offset_scaled_log_variance, 2);
  state->avb_interface.clock_accuracy =
      state->ptp_status.clock_source_info.accuracy;
  state->avb_interface.priority2 =
      state->ptp_status.clock_source_info.priority2;
}

/* Send AECP response get descriptor for avb interface message */
int avb_send_aecp_rsp_read_descr_avb_interface(avb_state_s *state,
                                               aecp_read_descriptor_rsp_s *msg,
                                               eth_addr_t *dest_addr) {
  int ret = OK;
  struct timespec ts;

  // copy the avb interface descriptor from state
  aem_avb_interface_desc_s descriptor = state->avb_interface;

  // set localized description string reference
  uint16_t localized_description = 6;
  int_to_octets(&localized_description, descriptor.localized_description, 2);

  // insert the descriptor into the response message
  memcpy(msg->descriptor_data, &descriptor, sizeof(aem_avb_interface_desc_s));

  // calc control data length
  uint16_t control_data_len =
      AECP_DESC_PREAMBLE_LEN + sizeof(aem_avb_interface_desc_s);

  // set the control data length
  msg->common.header.control_data_len_h = (control_data_len >> 8) & 0xFF;
  msg->common.header.control_data_len = control_data_len & 0xFF;

  // send the response message
  uint16_t msg_len =
      sizeof(atdecc_header_s) + sizeof(unique_id_t) + control_data_len;
  ret = avb_net_send_to(state, ethertype_avtp, msg, msg_len, &ts, dest_addr);
  if (ret < 0) {
    avberr("send AECP Read Descriptor response failed: %d", errno);
  }
  return ret;
}

/* Send AECP response get descriptor for clock source message */
int avb_send_aecp_rsp_read_descr_clock_source(avb_state_s *state,
                                              aecp_read_descriptor_rsp_s *msg,
                                              eth_addr_t *dest_addr) {
  int ret = OK;
  struct timespec ts;
  uint16_t index = octets_to_uint(msg->descriptor_index, 2);

  // create a clock source descriptor
  aem_clock_source_desc_s descriptor;
  memset(&descriptor, 0, sizeof(aem_clock_source_desc_s));

  if (index == 1) {
    /* Clock source 1: CRF media clock input stream (Milan v1.3 §7.2.2).
     * Sourced from STREAM_INPUT at AVB_CRF_INPUT_INDEX. */
    int localized_description = 12; /* strings desc 1, string_4 (CRF) */
    uint16_t source_type = aem_clock_source_type_input_stream;
    uint16_t location_type = aem_desc_type_stream_input;
    uint16_t location_index = AVB_CRF_INPUT_INDEX;
    int_to_octets(&localized_description, descriptor.localized_description, 2);
    int_to_octets(&source_type, descriptor.clock_source_type, 2);
    int_to_octets(&location_type, &descriptor.clock_source_location_type, 2);
    int_to_octets(&location_index, &descriptor.clock_source_location_index, 2);
    /* clock_source_id stays zero for stream-derived source */
  } else {
    /* Clock source 0: gPTP (INTERNAL), sourced from AVB interface */
    int localized_description = 11; /* strings desc 1, string_3 */
    uint16_t location_type = aem_desc_type_audio_unit;
    int_to_octets(&localized_description, descriptor.localized_description, 2);
    memcpy(&descriptor.clock_source_id,
           state->ptp_status.clock_source_info.gm_id, sizeof(unique_id_t));
    int_to_octets(&location_type, &descriptor.clock_source_location_type, 2);
  }

  // insert the descriptor into the response message
  memcpy(msg->descriptor_data, &descriptor, sizeof(aem_clock_source_desc_s));

  // calc control data length
  uint16_t control_data_len =
      AECP_DESC_PREAMBLE_LEN + sizeof(aem_clock_source_desc_s);

  // set the control data length
  msg->common.header.control_data_len_h = (control_data_len >> 8) & 0xFF;
  msg->common.header.control_data_len = control_data_len & 0xFF;

  // send the response message
  uint16_t msg_len =
      sizeof(atdecc_header_s) + sizeof(unique_id_t) + control_data_len;
  ret = avb_net_send_to(state, ethertype_avtp, msg, msg_len, &ts, dest_addr);
  if (ret < 0) {
    avberr("send AECP Read Descriptor response failed: %d", errno);
  }
  return ret;
}

/* Send AECP response get descriptor for memory object message */
int avb_send_aecp_rsp_read_descr_memory_obj(avb_state_s *state,
                                            aecp_read_descriptor_rsp_s *msg,
                                            eth_addr_t *dest_addr) {
  int ret = OK;
  struct timespec ts;

  // data for the stream input descriptor
  int localized_description = 8; // strings desc 1, string_0
  uint16_t object_type = aem_memory_obj_type_png_entity;

  // create a stream input descriptor
  aem_memory_object_desc_s descriptor;
  memset(&descriptor, 0, sizeof(aem_memory_object_desc_s));

  // populate the descriptor (only non-zero fields)
  int_to_octets(&localized_description, descriptor.localized_description, 2);
  int_to_octets(&object_type, &descriptor.memory_object_type, 2);
  int_to_octets(&state->logo_start, &descriptor.start_address[4], 4);
  int_to_octets(&state->logo_length, &descriptor.length[4], 4);
  int_to_octets(&state->logo_length, &descriptor.maximum_length[4], 4);
  uint32_t max_segment = 400; /* safe limit for single AECP frame */
  int_to_octets(&max_segment, &descriptor.maximum_segment_length[4], 4);

  // insert the descriptor into the response message
  memcpy(msg->descriptor_data, &descriptor, sizeof(aem_memory_object_desc_s));

  // calc control data length
  uint16_t control_data_len =
      AECP_DESC_PREAMBLE_LEN + sizeof(aem_memory_object_desc_s);

  // set the control data length
  msg->common.header.control_data_len_h = (control_data_len >> 8) & 0xFF;
  msg->common.header.control_data_len = control_data_len & 0xFF;

  // send the response message
  uint16_t msg_len =
      sizeof(atdecc_header_s) + sizeof(unique_id_t) + control_data_len;
  ret = avb_net_send_to(state, ethertype_avtp, msg, msg_len, &ts, dest_addr);
  if (ret < 0) {
    avberr("send AECP Read Descriptor response failed: %d", errno);
  }
  return ret;
}

/* Send AECP response get descriptor for locale message */
int avb_send_aecp_rsp_read_descr_locale(avb_state_s *state,
                                        aecp_read_descriptor_rsp_s *msg,
                                        eth_addr_t *dest_addr) {
  int ret = OK;
  struct timespec ts;

  // data for the stream input descriptor
  size_t locale_identifier_len = 2;
  char *locale_identifier = "en";
  uint16_t number_of_strings = 3;

  // create a stream input descriptor
  aem_locale_desc_s descriptor;
  memset(&descriptor, 0, sizeof(aem_locale_desc_s));

  // populate the descriptor (only non-zero fields)
  memcpy(&descriptor.locale_identifier, locale_identifier,
         locale_identifier_len);
  int_to_octets(&number_of_strings, &descriptor.number_of_strings, 2);

  // insert the descriptor into the response message
  memcpy(msg->descriptor_data, &descriptor, sizeof(aem_locale_desc_s));

  // calc control data length
  uint16_t control_data_len =
      AECP_DESC_PREAMBLE_LEN + sizeof(aem_locale_desc_s);

  // set the control data length
  msg->common.header.control_data_len_h = (control_data_len >> 8) & 0xFF;
  msg->common.header.control_data_len = control_data_len & 0xFF;

  // send the response message
  uint16_t msg_len =
      sizeof(atdecc_header_s) + sizeof(unique_id_t) + control_data_len;
  ret = avb_net_send_to(state, ethertype_avtp, msg, msg_len, &ts, dest_addr);
  if (ret < 0) {
    avberr("send AECP Read Descriptor response failed: %d", errno);
  }
  return ret;
}

/* Send AECP response get descriptor for strings message */
int avb_send_aecp_rsp_read_descr_strings(avb_state_s *state,
                                         aecp_read_descriptor_rsp_s *msg,
                                         eth_addr_t *dest_addr) {
  int ret = OK;
  struct timespec ts;

  // data for the stream input descriptor
  char *string_0 = ""; // Config Name (addr 0)
  if (state->config.talker && state->config.listener) {
    string_0 = "Talker & Listener";
  } else if (state->config.talker) {
    string_0 = "Talker";
  } else if (state->config.listener) {
    string_0 = "Listener";
  }
  char *string_1 = "ESP-AVB";        // Audio Unit Name (addr 1)
  char *string_2 = "Mono Audio In";  // Stream Port Input Cluster Name (addr 2)
  char *string_3 = "Mono Audio Out"; // Stream Port Output Cluster Name (addr 3)
  char *string_4 = "Audio Stream In";  // Stream Input Name (addr 4)
  char *string_5 = "Audio Stream Out"; // Stream Output Name (addr 5)
  char *string_6 = "Ethernet";         // AVB Interface Name (addr 6)
  if (msg->descriptor_index[1] == 1) {
    string_0 = "Logo";           // Memory Object Name (addr 8)
    string_1 = "Identify";       // Control Name (addr 9)
    string_2 = "Clock Domain";   // Clock Domain Name (addr 10)
    string_3 = "Internal Clock"; // Clock Source Name (addr 11)
    string_4 = "Vendor Name";    // Vendor Name (addr 12)
    string_5 = "AVB Endpoint";   // Model Name (addr 13)
    string_6 = "Speaker Volume"; // (addr 14)
  } else if (msg->descriptor_index[1] == 2) {
    string_0 = "Mic Gain"; // (addr 16)
    string_1 = "";
    string_2 = "";
    string_3 = "";
    string_4 = "";
    string_5 = "";
    string_6 = "";
  }

  // create a stream input descriptor
  aem_strings_desc_s descriptor;
  memset(&descriptor, 0, sizeof(aem_strings_desc_s));

  // populate the descriptor (only non-zero fields)
  memcpy(&descriptor.string_0, string_0, strlen(string_0));
  memcpy(&descriptor.string_1, string_1, strlen(string_1));
  memcpy(&descriptor.string_2, string_2, strlen(string_2));
  memcpy(&descriptor.string_3, string_3, strlen(string_3));
  memcpy(&descriptor.string_4, string_4, strlen(string_4));
  memcpy(&descriptor.string_5, string_5, strlen(string_5));
  memcpy(&descriptor.string_6, string_6, strlen(string_6));

  // insert the descriptor into the response message
  memcpy(msg->descriptor_data, &descriptor, sizeof(aem_strings_desc_s));

  // calc control data length
  uint16_t control_data_len =
      AECP_DESC_PREAMBLE_LEN + sizeof(aem_strings_desc_s);

  // set the control data length
  msg->common.header.control_data_len_h = (control_data_len >> 8) & 0xFF;
  msg->common.header.control_data_len = control_data_len & 0xFF;

  // send the response message
  uint16_t msg_len =
      sizeof(atdecc_header_s) + sizeof(unique_id_t) + control_data_len;
  ret = avb_net_send_to(state, ethertype_avtp, msg, msg_len, &ts, dest_addr);
  if (ret < 0) {
    avberr("send AECP Read Descriptor response failed: %d", errno);
  }
  return ret;
}

/* Send AECP response get descriptor for stream port input message */
int avb_send_aecp_rsp_read_descr_stream_port(avb_state_s *state,
                                             aecp_read_descriptor_rsp_s *msg,
                                             eth_addr_t *dest_addr,
                                             bool is_output) {
  int ret = OK;
  struct timespec ts;

  // data for the stream input descriptor
  uint16_t num_clusters = 1;
  uint16_t num_maps = 1;

  // create a stream input descriptor
  aem_stream_port_desc_s descriptor;
  memset(&descriptor, 0, sizeof(aem_stream_port_desc_s));

  // populate the descriptor (only non-zero fields)
  int_to_octets(&num_clusters, &descriptor.number_of_clusters, 2);
  int_to_octets(&num_maps, &descriptor.number_of_maps, 2);
  if (is_output) { // bases are different for output port
    descriptor.base_cluster[1] = 1;
    descriptor.base_map[1] = 1;
  }

  // insert the descriptor into the response message
  memcpy(msg->descriptor_data, &descriptor, sizeof(aem_stream_port_desc_s));

  // calc control data length
  uint16_t control_data_len =
      AECP_DESC_PREAMBLE_LEN + sizeof(aem_stream_port_desc_s);

  // set the control data length
  msg->common.header.control_data_len_h = (control_data_len >> 8) & 0xFF;
  msg->common.header.control_data_len = control_data_len & 0xFF;

  // send the response message
  uint16_t msg_len =
      sizeof(atdecc_header_s) + sizeof(unique_id_t) + control_data_len;
  ret = avb_net_send_to(state, ethertype_avtp, msg, msg_len, &ts, dest_addr);
  if (ret < 0) {
    avberr("send AECP Read Descriptor response failed: %d", errno);
  }
  return ret;
}

/* Send AECP response get descriptor for audio cluster message */
int avb_send_aecp_rsp_read_descr_audio_cluster(avb_state_s *state,
                                               aecp_read_descriptor_rsp_s *msg,
                                               eth_addr_t *dest_addr) {
  int ret = OK;
  struct timespec ts;

  // data for the stream input descriptor
  int localized_description = 2; // strings desc 0, string_1
  aem_desc_type_t signal_type = aem_desc_type_invalid;
  uint16_t num_channels = state->config.num_channels_input;
  if (msg->descriptor_index[1] == 1) { // index 1 used for output port
    localized_description = 3;         // strings desc 0, string_2
    signal_type = aem_desc_type_audio_unit;
    num_channels = state->config.num_channels_output;
  }
  // TODO: may need to change depending on 61883 vs aaf
  aem_audio_cluster_format_t cluster_format = aem_audio_cluster_format_mbla;

  // create a stream input descriptor
  aem_audio_cluster_desc_s descriptor;
  memset(&descriptor, 0, sizeof(aem_audio_cluster_desc_s));

  // populate the descriptor (only non-zero fields)
  int_to_octets(&localized_description, descriptor.localized_description, 2);
  int_to_octets(&cluster_format, descriptor.format, 2);
  int_to_octets(&num_channels, descriptor.channel_count, 2);
  int_to_octets(&signal_type, descriptor.signal_type, 2);

  // insert the descriptor into the response message
  memcpy(msg->descriptor_data, &descriptor, sizeof(aem_audio_cluster_desc_s));

  // calc control data length
  uint16_t control_data_len =
      AECP_DESC_PREAMBLE_LEN + sizeof(aem_audio_cluster_desc_s);

  // set the control data length
  msg->common.header.control_data_len_h = (control_data_len >> 8) & 0xFF;
  msg->common.header.control_data_len = control_data_len & 0xFF;

  // send the response message
  uint16_t msg_len =
      sizeof(atdecc_header_s) + sizeof(unique_id_t) + control_data_len;
  ret = avb_net_send_to(state, ethertype_avtp, msg, msg_len, &ts, dest_addr);
  if (ret < 0) {
    avberr("send AECP Read Descriptor response failed: %d", errno);
  }
  return ret;
}

/* Send AECP response get descriptor for audio map message */
int avb_send_aecp_rsp_read_descr_audio_map(avb_state_s *state,
                                           aecp_read_descriptor_rsp_s *msg,
                                           eth_addr_t *dest_addr) {
  int ret = OK;
  struct timespec ts;

  // data for the stream input descriptor
  uint16_t mappings_offset = 8; // Required by spec
  uint16_t num_mappings = state->config.num_channels_input;
  if (msg->descriptor_index[1] == 1) { // audio map index 1 used for output port
    num_mappings = state->config.num_channels_output;
  }
  aem_audio_mapping_s mappings[num_mappings];
  for (uint16_t i = 0; i < num_mappings; i++) {
    aem_audio_mapping_s mapping = {0};
    int_to_octets(&i, mapping.mapping_stream_channel, 2);
    int_to_octets(&i, mapping.mapping_cluster_channel, 2);
    mappings[i] = mapping;
  }

  // create a stream input descriptor
  aem_audio_map_desc_s descriptor = {0};

  // populate the descriptor (only non-zero fields)
  int_to_octets(&mappings_offset, &descriptor.mappings_offset, 2);
  int_to_octets(&num_mappings, &descriptor.number_of_mappings, 2);
  memcpy(descriptor.mappings, mappings,
         num_mappings * sizeof(aem_audio_mapping_s));

  // insert the descriptor into the response message
  memcpy(msg->descriptor_data, &descriptor, sizeof(aem_audio_map_desc_s));

  // calc control data length
  uint16_t control_data_len =
      AECP_DESC_PREAMBLE_LEN + sizeof(aem_audio_map_desc_s);

  // set the control data length
  msg->common.header.control_data_len_h = (control_data_len >> 8) & 0xFF;
  msg->common.header.control_data_len = control_data_len & 0xFF;

  // send the response message
  uint16_t msg_len =
      sizeof(atdecc_header_s) + sizeof(unique_id_t) + control_data_len;
  ret = avb_net_send_to(state, ethertype_avtp, msg, msg_len, &ts, dest_addr);
  if (ret < 0) {
    avberr("send AECP Read Descriptor response failed: %d", errno);
  }
  return ret;
}

/* Send AECP response get descriptor for control message
 * Supports: 0=IDENTIFY, 1=Speaker Volume (listener), 2=Mic Gain (talker)
 */
int avb_send_aecp_rsp_read_descr_control(avb_state_s *state,
                                         aecp_read_descriptor_rsp_s *msg,
                                         eth_addr_t *dest_addr) {
  int ret = OK;
  struct timespec ts;
  uint16_t index = octets_to_uint(msg->descriptor_index, 2);

  aem_control_desc_s descriptor;
  memset(&descriptor, 0, sizeof(aem_control_desc_s));

  uint16_t values_offset = 104; // Required by spec
  uint16_t num_values = 1;
  uint16_t signal_type = aem_desc_type_invalid;
  int_to_octets(&values_offset, &descriptor.values_offset, 2);
  int_to_octets(&num_values, &descriptor.number_of_values, 2);
  int_to_octets(&signal_type, &descriptor.signal_type, 2);

  switch (index) {
  case 0: { /* IDENTIFY */
    int localized_description = 9;
    uint16_t control_value_type = 1; // CONTROL_LINEAR_UINT8
    uint8_t control_type[8] = {0x90, 0xe0, 0xf0, 0x00,
                               0x00, 0x00, 0x00, 0x01};
    aem_identify_control_value_s control_values = {
        .values = {0, 255, 255, 0, 0},
        .units = {0},
        .string_ref = {0xff, 0xff},
    };
    memcpy(descriptor.object_name, state->descriptor_names[AVB_NAME_CONTROL_0], 64);
    int_to_octets(&localized_description, descriptor.localized_description, 2);
    int_to_octets(&control_value_type, descriptor.control_value_type, 2);
    memcpy(descriptor.control_type, control_type, 8);
    memcpy(descriptor.value_details, &control_values,
           sizeof(aem_identify_control_value_s));
    break;
  }
  case 1: { /* Speaker Volume (LEVEL control type) */
    if (!state->config.listener) {
      avbwarn("Control index 1 (volume) not available: listener not enabled");
      return OK;
    }
    int localized_desc_vol = 14; /* strings desc 1, string_6 */
    uint16_t control_value_type = 2; // CONTROL_LINEAR_INT16
    uint8_t control_type[8] = {0x90, 0xe0, 0xf0, 0x00,
                               0x00, 0x00, 0x00, 0x03}; // LEVEL
    memcpy(descriptor.object_name, state->descriptor_names[AVB_NAME_CONTROL_1], 64);
    int_to_octets(&localized_desc_vol, descriptor.localized_description, 2);
    int_to_octets(&control_value_type, descriptor.control_value_type, 2);
    memcpy(descriptor.control_type, control_type, 8);
    /* value_details: min(2) max(2) step(2) default(2) current(2) unit(2)
     * string(2) = 14 bytes per IEEE 1722.1 Table 7-39 */
    /* value_details order per §7.3.6.2: min, max, step, default, current,
     * unit, string_ref — each field is 2 bytes for CONTROL_LINEAR_INT16 */
    codec_control_range_s *r = &state->codec_ranges;
    int16_t current = (int16_t)(state->ctrl_speaker_vol * 10.0f);
    uint8_t *v = descriptor.value_details;
    int_to_octets(&r->vol_min_tenth_db, v + 0, 2);
    int_to_octets(&r->vol_max_tenth_db, v + 2, 2);
    int_to_octets(&r->vol_step_tenth_db, v + 4, 2);
    int_to_octets(&r->vol_default_tenth_db, v + 6, 2);
    int_to_octets(&current, v + 8, 2);
    /* units: multiplier=-1 (0xFF), code=0xB0 (DB) */
    v[10] = 0xFF; /* multiplier */
    v[11] = 0xB0; /* code: DB */
    v[12] = 0xFF; /* string_ref high (not set) */
    v[13] = 0xFF; /* string_ref low */
    break;
  }
  case 2: { /* Mic Gain (GAIN control type) */
    if (!state->config.talker) {
      avbwarn("Control index 2 (mic gain) not available: talker not enabled");
      return OK;
    }
    int localized_desc_gain = 16; /* strings desc 2, string_0 */
    uint16_t control_value_type = 2; // CONTROL_LINEAR_INT16
    uint8_t control_type[8] = {0x90, 0xe0, 0xf0, 0x00,
                               0x00, 0x00, 0x00, 0x04}; // GAIN
    memcpy(descriptor.object_name, state->descriptor_names[AVB_NAME_CONTROL_2], 64);
    int_to_octets(&localized_desc_gain, descriptor.localized_description, 2);
    int_to_octets(&control_value_type, descriptor.control_value_type, 2);
    memcpy(descriptor.control_type, control_type, 8);
    codec_control_range_s *r = &state->codec_ranges;
    int16_t current = (int16_t)(state->ctrl_mic_gain * 10.0f);
    uint8_t *v = descriptor.value_details;
    int_to_octets(&r->gain_min_tenth_db, v + 0, 2);
    int_to_octets(&r->gain_max_tenth_db, v + 2, 2);
    int_to_octets(&r->gain_step_tenth_db, v + 4, 2);
    int_to_octets(&r->gain_default_tenth_db, v + 6, 2);
    int_to_octets(&current, v + 8, 2);
    v[10] = 0xFF;
    v[11] = 0xB0;
    v[12] = 0xFF;
    v[13] = 0xFF;
    break;
  }
  default:
    avbwarn("Unsupported control descriptor index: %d", index);
    return OK;
  }

  memcpy(msg->descriptor_data, &descriptor, sizeof(aem_control_desc_s));

  uint16_t control_data_len =
      AECP_DESC_PREAMBLE_LEN + sizeof(aem_control_desc_s);
  msg->common.header.control_data_len_h = (control_data_len >> 8) & 0xFF;
  msg->common.header.control_data_len = control_data_len & 0xFF;

  uint16_t msg_len =
      sizeof(atdecc_header_s) + sizeof(unique_id_t) + control_data_len;
  ret = avb_net_send_to(state, ethertype_avtp, msg, msg_len, &ts, dest_addr);
  if (ret < 0) {
    avberr("send AECP Read Descriptor response failed: %d", errno);
  }
  return ret;
}

/* Process AECP SET_CONTROL command (IEEE 1722.1 §7.4.25) */
int avb_process_aecp_cmd_set_control(avb_state_s *state, aecp_message_u *msg,
                                     eth_addr_t *src_addr) {
  struct timespec ts;
  aecp_aem_short_s *cmd = (aecp_aem_short_s *)msg;
  uint16_t desc_index = octets_to_uint(cmd->descriptor_index, 2);
  uint8_t *values = (uint8_t *)cmd + sizeof(aecp_aem_short_s);
  uint8_t status = aecp_status_success;

  switch (desc_index) {
  case 0: { /* IDENTIFY */
    uint8_t val = values[0];
    state->ctrl_identify = val;
    if (val != 0) {
      avb_identify_tone(state, 500);
    }
    avbinfo("SET_CONTROL: IDENTIFY = %d", val);
    break;
  }
  case 1: { /* Speaker Volume */
    if (!state->config.listener) {
      status = aecp_status_not_supported;
      break;
    }
    int16_t raw = (int16_t)((values[0] << 8) | values[1]);
    float vol = raw / 10.0f;
    /* Clamp to codec range */
    float min_vol = state->codec_ranges.vol_min_tenth_db / 10.0f;
    float max_vol = state->codec_ranges.vol_max_tenth_db / 10.0f;
    if (vol < min_vol) vol = min_vol;
    if (vol > max_vol) vol = max_vol;
    state->ctrl_speaker_vol = vol;
    avb_codec_set_vol(state, vol);
    avbinfo("SET_CONTROL: Speaker Volume = %.1f dB", vol);
    avb_persist_request_save(state);
    break;
  }
  case 2: { /* Mic Gain */
    if (!state->config.talker) {
      status = aecp_status_not_supported;
      break;
    }
    int16_t raw = (int16_t)((values[0] << 8) | values[1]);
    float gain = raw / 10.0f;
    float min_gain = state->codec_ranges.gain_min_tenth_db / 10.0f;
    float max_gain = state->codec_ranges.gain_max_tenth_db / 10.0f;
    if (gain < min_gain) gain = min_gain;
    if (gain > max_gain) gain = max_gain;
    state->ctrl_mic_gain = gain;
    avb_codec_set_mic_gain(state, gain);
    avbinfo("SET_CONTROL: Mic Gain = %.1f dB", gain);
    avb_persist_request_save(state);
    break;
  }
  default:
    status = aecp_status_no_such_descriptor;
    break;
  }

  /* Build response: echo command with msg_type=response, set status */
  msg->header.msg_type = aecp_msg_type_aem_response;
  msg->header.status_valtime = status;

  /* Write current value back into response */
  if (status == aecp_status_success) {
    switch (desc_index) {
    case 0:
      values[0] = state->ctrl_identify;
      break;
    case 1: {
      int16_t cur = (int16_t)(state->ctrl_speaker_vol * 10.0f);
      values[0] = (cur >> 8) & 0xFF;
      values[1] = cur & 0xFF;
      break;
    }
    case 2: {
      int16_t cur = (int16_t)(state->ctrl_mic_gain * 10.0f);
      values[0] = (cur >> 8) & 0xFF;
      values[1] = cur & 0xFF;
      break;
    }
    }
  }

  /* Compute exact response length from value size, not from the incoming
   * command length — some controllers send extra data (full value_details)
   * in the command that must not be echoed back. */
  uint16_t values_len = (desc_index == 0) ? 1 : 2;
  uint16_t control_data_len =
      sizeof(aecp_common_s) + sizeof(aecp_common_aem_s) + 4 + values_len -
      AVTP_CDL_PREAMBLE_LEN;
  msg->header.control_data_len_h = (control_data_len >> 8) & 0xFF;
  msg->header.control_data_len = control_data_len & 0xFF;
  uint16_t msg_len =
      sizeof(atdecc_header_s) + sizeof(unique_id_t) + control_data_len;
  int ret =
      avb_net_send_to(state, ethertype_avtp, msg, msg_len, &ts, src_addr);
  if (ret < 0) {
    avberr("send AECP SET_CONTROL response failed: %d", errno);
  }
  return ret;
}

/* Process AECP GET_CONTROL command (IEEE 1722.1 §7.4.26) */
int avb_process_aecp_cmd_get_control(avb_state_s *state, aecp_message_u *msg,
                                     eth_addr_t *src_addr) {
  struct timespec ts;
  aecp_aem_short_s *cmd = (aecp_aem_short_s *)msg;
  uint16_t desc_index = octets_to_uint(cmd->descriptor_index, 2);
  uint8_t *values = (uint8_t *)cmd + sizeof(aecp_aem_short_s);
  uint8_t status = aecp_status_success;

  switch (desc_index) {
  case 0: /* IDENTIFY */
    values[0] = state->ctrl_identify;
    break;
  case 1: { /* Speaker Volume */
    if (!state->config.listener) {
      status = aecp_status_not_supported;
      break;
    }
    int16_t cur = (int16_t)(state->ctrl_speaker_vol * 10.0f);
    values[0] = (cur >> 8) & 0xFF;
    values[1] = cur & 0xFF;
    break;
  }
  case 2: { /* Mic Gain */
    if (!state->config.talker) {
      status = aecp_status_not_supported;
      break;
    }
    int16_t cur = (int16_t)(state->ctrl_mic_gain * 10.0f);
    values[0] = (cur >> 8) & 0xFF;
    values[1] = cur & 0xFF;
    break;
  }
  default:
    status = aecp_status_no_such_descriptor;
    break;
  }

  msg->header.msg_type = aecp_msg_type_aem_response;
  msg->header.status_valtime = status;

  /* Set control_data_len to include the values field */
  uint16_t values_len = (desc_index == 0) ? 1 : 2;
  uint16_t control_data_len =
      sizeof(aecp_common_s) + sizeof(aecp_common_aem_s) + 4 + values_len -
      AVTP_CDL_PREAMBLE_LEN;
  msg->header.control_data_len_h = (control_data_len >> 8) & 0xFF;
  msg->header.control_data_len = control_data_len & 0xFF;

  uint16_t msg_len =
      sizeof(atdecc_header_s) + sizeof(unique_id_t) + control_data_len;
  int ret =
      avb_net_send_to(state, ethertype_avtp, msg, msg_len, &ts, src_addr);
  if (ret < 0) {
    avberr("send AECP GET_CONTROL response failed: %d", errno);
  }
  return ret;
}

/* Send AECP response get descriptor for clock domain message */
int avb_send_aecp_rsp_read_descr_clock_domain(avb_state_s *state,
                                              aecp_read_descriptor_rsp_s *msg,
                                              eth_addr_t *dest_addr) {
  int ret = OK;
  struct timespec ts;

  // data for the clock domain descriptor
  int localized_description = 10; // strings desc 1, string_2
  /* clock_sources_offset is from the start of the descriptor (the
   * descriptor_type field, 4 bytes before the struct body). Add 4 to the
   * struct-relative offsetof to get the on-wire value. */
  uint16_t clock_sources_offset =
      (uint16_t)offsetof(aem_clock_domain_desc_s, clock_sources) + 4;
  /* Two clock sources: [0]=gPTP (INTERNAL), [1]=CRF input stream.
   * Milan v1.3 §7.2.2 requires the CRF source to be selectable for
   * each clock domain that serves an AAF talker. */
  uint16_t clock_sources_count = 2;
  aem_clock_source_t clock_sources[2] = {{0, 0}, {0, 1}};
  /* Currently-selected source — reflects any prior SET_CLOCK_SOURCE */
  uint16_t clock_source_index = state->media_clock.active_clock_source_index;

  // create a clock domain descriptor
  aem_clock_domain_desc_s descriptor;
  memset(&descriptor, 0, sizeof(aem_clock_domain_desc_s));

  // populate the descriptor (only non-zero fields)
  int_to_octets(&localized_description, descriptor.localized_description, 2);
  int_to_octets(&clock_source_index, descriptor.clock_source_index, 2);
  int_to_octets(&clock_sources_offset, &descriptor.clock_sources_offset, 2);
  int_to_octets(&clock_sources_count, &descriptor.clock_sources_count, 2);
  memcpy(descriptor.clock_sources, clock_sources,
         sizeof(aem_clock_source_t) * clock_sources_count);

  // insert the descriptor into the response message
  memcpy(msg->descriptor_data, &descriptor, sizeof(aem_clock_domain_desc_s));

  // calc control data length — trim to the actual clock_sources_count
  uint16_t control_data_len =
      AECP_DESC_PREAMBLE_LEN + sizeof(aem_clock_domain_desc_s) -
      (AEM_MAX_NUM_CLOCK_SOURCES - clock_sources_count) *
          sizeof(aem_clock_source_t);

  // set the control data length
  msg->common.header.control_data_len_h = (control_data_len >> 8) & 0xFF;
  msg->common.header.control_data_len = control_data_len & 0xFF;

  // send the response message
  uint16_t msg_len =
      sizeof(atdecc_header_s) + sizeof(unique_id_t) + control_data_len;
  ret = avb_net_send_to(state, ethertype_avtp, msg, msg_len, &ts, dest_addr);
  if (ret < 0) {
    avberr("send AECP Read Descriptor response failed: %d", errno);
  }
  return ret;
}

/* Process received ATDECC ADP message */
int avb_process_adp(avb_state_s *state, adp_message_s *msg,
                    eth_addr_t *src_addr) {

  /* Process ADP Entity Available message */
  if (msg->header.msg_type == adp_msg_type_entity_available) {

    // If the entity is an audio talker, then remember it
    if (msg->entity.talker_capabilities.implemented &&
        msg->entity.talker_capabilities.audio_source) {

      // if the talker is not already known, add it to the list
      int index =
          avb_find_entity_by_addr(state, src_addr, avb_entity_type_talker);
      if (index == NOT_FOUND) {
        // create talker struct
        avb_talker_s talker;
        memset(&talker, 0, sizeof(avb_talker_s));
        memcpy(talker.entity_id, msg->entity.entity_id, UNIQUE_ID_LEN);
        memcpy(talker.model_id, msg->entity.model_id, UNIQUE_ID_LEN);
        memcpy(talker.mac_addr, src_addr, ETH_ADDR_LEN);

        // if talker list is not full, add the talker to the list
        if (state->num_talkers < AVB_MAX_NUM_TALKERS) {
          state->talkers[state->num_talkers] = talker;
          state->num_talkers++;
        }
        // if talker list is full, then replace the oldest talker
        else {
          memmove(&state->talkers[0], &state->talkers[1],
                  (state->num_talkers - 1) * sizeof(avb_talker_s));
          state->talkers[state->num_talkers - 1] = talker;
        }
      }
      // if it is already known, then update the entity id and model id if
      // missing or changed
      else {
        if (memcmp(state->talkers[index].entity_id, msg->entity.entity_id,
                   UNIQUE_ID_LEN) != 0) {
          memcpy(&state->talkers[index].entity_id, msg->entity.entity_id,
                 UNIQUE_ID_LEN);
        }
        if (memcmp(state->talkers[index].model_id, msg->entity.model_id,
                   UNIQUE_ID_LEN) != 0) {
          memcpy(&state->talkers[index].model_id, msg->entity.model_id,
                 UNIQUE_ID_LEN);
        }
      }
    }
    // If the entity is an audio listener, then remember it
    if (msg->entity.listener_capabilities.implemented &&
        msg->entity.listener_capabilities.audio_sink) {

      // if the listener is not already known, add it to the list
      int index =
          avb_find_entity_by_addr(state, src_addr, avb_entity_type_listener);
      if (index == NOT_FOUND) {

        // create listener struct
        avb_listener_s listener;
        memset(&listener, 0, sizeof(avb_listener_s));
        memcpy(listener.entity_id, msg->entity.entity_id, UNIQUE_ID_LEN);
        memcpy(listener.model_id, msg->entity.model_id, UNIQUE_ID_LEN);
        memcpy(listener.mac_addr, src_addr, ETH_ADDR_LEN);

        // if listener list is not full, add the listener to the list
        if (state->num_listeners < AVB_MAX_NUM_LISTENERS) {
          state->listeners[state->num_listeners] = listener;
          state->num_listeners++;
        }
        // if listener list is full, then replace the oldest listener
        else {
          memmove(&state->listeners[0], &state->listeners[1],
                  (state->num_listeners - 1) * sizeof(avb_listener_s));
          state->listeners[state->num_listeners - 1] = listener;
        }
      }
      // if it is already known, then update the entity id and model id if
      // missing or changed
      else {
        if (memcmp(state->listeners[index].entity_id, msg->entity.entity_id,
                   UNIQUE_ID_LEN) != 0) {
          memcpy(&state->listeners[index].entity_id, msg->entity.entity_id,
                 UNIQUE_ID_LEN);
        }
        if (memcmp(state->listeners[index].model_id, msg->entity.model_id,
                   UNIQUE_ID_LEN) != 0) {
          memcpy(&state->listeners[index].model_id, msg->entity.model_id,
                 UNIQUE_ID_LEN);
        }
      }
    }
    // If the entity is a controller, then remember it
    if (msg->entity.controller_capabilities.implemented) {

      // if the controller is not already known, add it to the list
      int index =
          avb_find_entity_by_addr(state, src_addr, avb_entity_type_controller);
      if (index == NOT_FOUND) {

        // create controller struct
        avb_controller_s controller;
        memset(&controller, 0, sizeof(avb_controller_s));
        memcpy(controller.entity_id, msg->entity.entity_id, UNIQUE_ID_LEN);
        memcpy(controller.model_id, msg->entity.model_id, UNIQUE_ID_LEN);
        memcpy(controller.mac_addr, src_addr, ETH_ADDR_LEN);

        // if controller list is not full, add the controller to the list
        if (state->num_controllers < AVB_MAX_NUM_CONTROLLERS) {
          state->controllers[state->num_controllers] = controller;
          state->num_controllers++;
        }
        /* if controller list is full, then replace the oldest controller */
        else {
          memmove(&state->controllers[0], &state->controllers[1],
                  (state->num_controllers - 1) * sizeof(avb_controller_s));
          state->controllers[state->num_controllers - 1] = controller;
        }
      }
      // if it is already known, then update the entity id and model id if
      // missing or changed
      else {
        if (memcmp(state->controllers[index].entity_id, msg->entity.entity_id,
                   UNIQUE_ID_LEN) != 0) {
          memcpy(&state->controllers[index].entity_id, msg->entity.entity_id,
                 UNIQUE_ID_LEN);
        }
        if (memcmp(state->controllers[index].model_id, msg->entity.model_id,
                   UNIQUE_ID_LEN) != 0) {
          memcpy(&state->controllers[index].model_id, msg->entity.model_id,
                 UNIQUE_ID_LEN);
        }
      }
    }
  }
  return OK;
}

/* Process received ATDECC AECP message */
/* Resolve a descriptor's object_name pointer from type/index/name_index.
 * Returns NULL if the descriptor type is not supported for naming. */
static uint8_t *avb_resolve_name_ptr(avb_state_s *state, uint16_t desc_type,
                                     uint16_t desc_index,
                                     uint16_t name_index) {
  switch (desc_type) {
  case aem_desc_type_entity:
    if (name_index == 0)
      return (uint8_t *)state->descriptor_names[AVB_NAME_ENTITY];
    if (name_index == 1)
      return (uint8_t *)state->descriptor_names[AVB_NAME_GROUP];
    return NULL;
  case aem_desc_type_avb_interface:
    return (name_index == 0)
               ? (uint8_t *)state->descriptor_names[AVB_NAME_AVB_INTERFACE]
               : NULL;
  case aem_desc_type_stream_input:
    if (desc_index < AVB_MAX_NUM_INPUT_STREAMS && name_index == 0)
      return (uint8_t *)state->descriptor_names[AVB_NAME_STREAM_INPUT_0 +
                                                desc_index];
    return NULL;
  case aem_desc_type_stream_output:
    if (desc_index < AVB_MAX_NUM_OUTPUT_STREAMS && name_index == 0)
      return (uint8_t *)state->descriptor_names[AVB_NAME_STREAM_OUTPUT_0 +
                                                desc_index];
    return NULL;
  case aem_desc_type_control:
    if (desc_index < AEM_NUM_CONTROLS && name_index == 0)
      return (uint8_t *)state->descriptor_names[AVB_NAME_CONTROL_0 +
                                                desc_index];
    return NULL;
  default:
    return NULL;
  }
}

/* Process AECP SET_NAME command (IEEE 1722.1-2021 §7.4.17)
 * Format after AEM common: descriptor_type(2) descriptor_index(2)
 *   name_index(2) configuration_index(2) name(64) */
int avb_process_aecp_cmd_set_name(avb_state_s *state, aecp_message_u *msg,
                                  eth_addr_t *src_addr) {
  struct timespec ts;
  uint8_t *data = msg->raw + sizeof(aecp_common_s) + sizeof(aecp_common_aem_s);
  uint16_t desc_type = (data[0] << 8) | data[1];
  uint16_t desc_index = (data[2] << 8) | data[3];
  uint16_t name_index = (data[4] << 8) | data[5];
  uint8_t *name = data + 8; /* 64-byte name field at offset 8 */
  uint8_t status = aecp_status_success;

  uint8_t *target = avb_resolve_name_ptr(state, desc_type, desc_index,
                                         name_index);
  if (target) {
    memcpy(target, name, 64);
    /* Sync to canonical locations for descriptors that read from elsewhere */
    if (desc_type == aem_desc_type_entity && name_index == 0)
      memcpy(state->own_entity.detail.entity_name, target, 64);
    else if (desc_type == aem_desc_type_entity && name_index == 1)
      memcpy(state->own_entity.detail.group_name, target, 64);
    else if (desc_type == aem_desc_type_avb_interface)
      memcpy(state->avb_interface.object_name, target, 64);
    avbinfo("SET_NAME: type=0x%04x idx=%d name='%.64s'",
            desc_type, desc_index, (char *)target);
    avb_persist_request_save(state);
  } else {
    status = aecp_status_not_implemented;
  }

  /* Build response */
  msg->header.msg_type = aecp_msg_type_aem_response;
  msg->header.status_valtime = status;
  if (target) {
    memcpy(name, target, 64); /* echo current value */
  }

  uint16_t control_data_len =
      (msg->header.control_data_len_h << 8) | msg->header.control_data_len;
  uint16_t msg_len =
      sizeof(atdecc_header_s) + sizeof(unique_id_t) + control_data_len;
  int ret =
      avb_net_send_to(state, ethertype_avtp, msg, msg_len, &ts, src_addr);
  if (ret < 0) {
    avberr("send AECP SET_NAME response failed: %d", errno);
  }
  return ret;
}

/* Process AECP GET_NAME command (IEEE 1722.1-2021 §7.4.18)
 * Command format: descriptor_type(2) descriptor_index(2)
 *   name_index(2) configuration_index(2)
 * Response format: same as SET_NAME (adds 64-byte name) */
int avb_process_aecp_cmd_get_name(avb_state_s *state, aecp_message_u *msg,
                                  eth_addr_t *src_addr) {
  struct timespec ts;
  uint8_t *data = msg->raw + sizeof(aecp_common_s) + sizeof(aecp_common_aem_s);
  uint16_t desc_type = (data[0] << 8) | data[1];
  uint16_t desc_index = (data[2] << 8) | data[3];
  uint16_t name_index = (data[4] << 8) | data[5];
  uint8_t *name = data + 8;
  uint8_t status = aecp_status_success;

  uint8_t *target = avb_resolve_name_ptr(state, desc_type, desc_index,
                                         name_index);
  if (target) {
    memcpy(name, target, 64);
  } else {
    status = aecp_status_not_implemented;
    memset(name, 0, 64);
  }

  msg->header.msg_type = aecp_msg_type_aem_response;
  msg->header.status_valtime = status;

  /* Update control_data_len to include the 64-byte name field */
  uint16_t control_data_len =
      sizeof(aecp_common_s) + sizeof(aecp_common_aem_s) + 8 + 64 -
      AVTP_CDL_PREAMBLE_LEN;
  msg->header.control_data_len_h = (control_data_len >> 8) & 0xFF;
  msg->header.control_data_len = control_data_len & 0xFF;

  uint16_t msg_len =
      sizeof(atdecc_header_s) + sizeof(unique_id_t) + control_data_len;
  int ret =
      avb_net_send_to(state, ethertype_avtp, msg, msg_len, &ts, src_addr);
  if (ret < 0) {
    avberr("send AECP GET_NAME response failed: %d", errno);
  }
  return ret;
}

/* Process AECP ADDRESS_ACCESS command (IEEE 1722.1-2021 §9.4)
 * Handles READ requests for memory objects (entity icon PNG). */
int avb_process_aecp_addr_access(avb_state_s *state, aecp_message_u *msg,
                                 eth_addr_t *src_addr) {
  struct timespec ts;
  aecp_addr_access_s *cmd = &msg->addr_access;
  uint16_t tlv_count = (cmd->tlv_count[0] << 8) | cmd->tlv_count[1];
  uint8_t status = 0; /* SUCCESS */

  /* Base address of the logo in memory */
  uintptr_t logo_base = (uintptr_t)state->logo_start;
  uint32_t logo_len = state->logo_length;

  /* Process each TLV */
  uint8_t *tlv = cmd->tlv_data;
  for (int i = 0; i < tlv_count && i < 4; i++) {
    /* TLV format: mode(4bits) + length(12bits) [2 bytes], address [8 bytes],
     * memory_data [length bytes] */
    uint8_t mode = (tlv[0] >> 4) & 0x0F;
    uint16_t length = ((tlv[0] & 0x0F) << 8) | tlv[1];
    uint64_t address = 0;
    for (int b = 0; b < 8; b++) {
      address = (address << 8) | tlv[2 + b];
    }
    uint8_t *memory_data = tlv + 10;

    if (mode == 0) { /* READ */
      /* Check if address falls within the logo memory region */
      if (address < logo_base) {
        status = 2; /* ADDRESS_TOO_LOW */
        break;
      }
      uint64_t offset = address - logo_base;
      if (offset + length > logo_len) {
        status = 3; /* ADDRESS_TOO_HIGH */
        /* Clamp to available data */
        if (offset < logo_len) {
          length = logo_len - offset;
          tlv[0] = (mode << 4) | ((length >> 8) & 0x0F);
          tlv[1] = length & 0xFF;
        } else {
          break;
        }
      }
      /* Copy requested bytes into response */
      memcpy(memory_data, state->logo_start + offset, length);
      status = 0;
    } else {
      status = 1; /* NOT_IMPLEMENTED (WRITE/EXECUTE not supported) */
      break;
    }

    /* Advance to next TLV */
    tlv += 10 + length;
  }

  /* Build response */
  msg->header.msg_type = aecp_msg_type_addr_access_response;
  msg->header.status_valtime = status;

  uint16_t control_data_len =
      (msg->header.control_data_len_h << 8) | msg->header.control_data_len;
  uint16_t msg_len =
      sizeof(atdecc_header_s) + sizeof(unique_id_t) + control_data_len;
  int ret =
      avb_net_send_to(state, ethertype_avtp, msg, msg_len, &ts, src_addr);
  if (ret < 0) {
    avberr("send AECP ADDRESS_ACCESS response failed: %d", errno);
  }
  return ret;
}

/* ---- Milan Vendor Unique (MVU) command handlers ---- */

/* Send an MVU response. The caller passes a pointer to a struct whose first
 * member is aecp_mvu_common_s (which starts with aecp_common_s.header).
 * Sets msg_type, status, control_data_length, then sends. */
static int avb_send_mvu_response(avb_state_s *state, void *msg,
                                 eth_addr_t *src_addr, uint16_t msg_len,
                                 uint8_t status) {
  struct timespec ts;
  aecp_mvu_common_s *mvu = (aecp_mvu_common_s *)msg;
  mvu->common.header.msg_type = aecp_msg_type_vendor_unique_response;
  mvu->common.header.status_valtime = status;
  uint16_t cdl = msg_len - AVTP_CDL_PREAMBLE_LEN;
  mvu->common.header.control_data_len_h = (cdl >> 8) & 0x07;
  mvu->common.header.control_data_len = cdl & 0xFF;
  mvu->u = 0; /* direct response, not notification */
  int ret = avb_net_send_to(state, ethertype_avtp, msg, msg_len, &ts, src_addr);
  if (ret < 0)
    avberr("send MVU response failed: %d", errno);
  return ret;
}

/* GET_MILAN_INFO (MVU 0x0000) — returns Milan protocol info.
 *
 * Hive uses the success/failure of this command to decide whether an entity
 * is Milan-compatible. When CONFIG_ESP_AVB_MILAN is disabled we reflect the
 * command with NOT_IMPLEMENTED so Hive treats us as non-Milan and skips
 * Milan-specific validation — useful for interop with pure IEEE 1722.1
 * controllers/talkers that refuse to connect to Milan-advertised devices. */
int avb_process_aecp_cmd_mvu_get_milan_info(avb_state_s *state,
                                            aecp_message_u *msg,
                                            eth_addr_t *src_addr) {
#if !CONFIG_ESP_AVB_MILAN
  /* Reflect command with NOT_IMPLEMENTED — same length as incoming */
  uint16_t cdl = (msg->header.control_data_len_h << 8) |
                 msg->header.control_data_len;
  uint16_t msg_len = AVTP_CDL_PREAMBLE_LEN + cdl;
  return avb_send_mvu_response(state, msg, src_addr, msg_len,
                               aecp_status_not_implemented);
#else
  aecp_mvu_get_milan_info_rsp_s rsp;
  memset(&rsp, 0, sizeof(rsp));
  memcpy(&rsp.mvu, msg, sizeof(aecp_mvu_common_s));

  uint32_t protocol_version = 1;
  int_to_octets(&protocol_version, rsp.protocol_version, 4);
  uint32_t features = 0x04; /* bit 29: MVU_BINDING */
  int_to_octets(&features, rsp.features_flags, 4);
  rsp.specification_version[0] = 1;
  rsp.specification_version[1] = 3;

  return avb_send_mvu_response(state, &rsp, src_addr, sizeof(rsp),
                               aecp_status_success);
#endif
}

/* GET_SYSTEM_UNIQUE_ID (MVU 0x0002) — returns system-wide unique ID */
int avb_process_aecp_cmd_mvu_get_system_unique_id(avb_state_s *state,
                                                  aecp_message_u *msg,
                                                  eth_addr_t *src_addr) {
  aecp_mvu_system_unique_id_s rsp;
  memset(&rsp, 0, sizeof(rsp));
  memcpy(&rsp.mvu, msg, sizeof(aecp_mvu_common_s));
  /* number: 0 (not assigned), name: empty string (defaults) */

  return avb_send_mvu_response(state, &rsp, src_addr, sizeof(rsp),
                               aecp_status_success);
}

/* BIND_STREAM / UNBIND_STREAM (MVU 0x0005 / 0x0006)
 * Binds/unbinds a listener stream input to a talker.
 * BIND layout after aecp_mvu_s: flags(2) descriptor_type(2) descriptor_index(2)
 *   talker_entity_id(8) talker_stream_index(2) reserved(2)
 * UNBIND layout after aecp_mvu_s: reserved(2) descriptor_type(2)
 *   descriptor_index(2) */
int avb_process_aecp_cmd_mvu_bind_stream(avb_state_s *state,
                                         aecp_message_u *msg,
                                         eth_addr_t *src_addr, bool unbind) {
  if (unbind) {
    aecp_mvu_unbind_stream_s *cmd = (aecp_mvu_unbind_stream_s *)msg;
    uint16_t desc_type = octets_to_uint(cmd->descriptor_type, 2);
    uint16_t desc_index = octets_to_uint(cmd->descriptor_index, 2);

    if (desc_type != aem_desc_type_stream_input ||
        desc_index >= state->num_input_streams) {
      return avb_send_mvu_response(state, cmd, src_addr, sizeof(*cmd),
                                   aecp_status_no_such_descriptor);
    }

    avb_listener_stream_s *stream = &state->input_streams[desc_index];
    if (stream->connected) {
      avb_stop_stream_in(state, desc_index);
      stream->connected = false;
      memset(stream->talker_id, 0, UNIQUE_ID_LEN);
    }
    /* Clear saved binding params too — Milan §5.5.3.6.20 step 3. */
    memset(stream->talker_uid, 0, 2);
    memset(stream->controller_id, 0, UNIQUE_ID_LEN);
    stream->stream_flags.streaming_wait = 0;
    avb_persist_request_save(state);
    avbinfo("MVU: unbind stream input %d", desc_index);
    return avb_send_mvu_response(state, cmd, src_addr, sizeof(*cmd),
                                 aecp_status_success);
  }

  aecp_mvu_bind_stream_s *cmd = (aecp_mvu_bind_stream_s *)msg;
  uint16_t desc_type = octets_to_uint(cmd->descriptor_type, 2);
  uint16_t desc_index = octets_to_uint(cmd->descriptor_index, 2);

  if (desc_type != aem_desc_type_stream_input ||
      desc_index >= state->num_input_streams) {
    return avb_send_mvu_response(state, cmd, src_addr, sizeof(*cmd),
                                 aecp_status_no_such_descriptor);
  }

  avb_listener_stream_s *stream = &state->input_streams[desc_index];
  memcpy(stream->talker_id, cmd->talker_entity_id, UNIQUE_ID_LEN);
  memcpy(stream->talker_uid, cmd->talker_stream_index, 2);
  memcpy(stream->controller_id, msg->common.controller_entity_id,
         UNIQUE_ID_LEN);
  uint16_t flags = octets_to_uint(cmd->flags, 2);
  /* Milan v1.3 §5.4.4.6: STREAMING_WAIT is bit 15 (MSB) of the flags field */
  stream->stream_flags.streaming_wait = (flags & 0x8000) ? 1 : 0;

  /* Milan §5.5.3.6.17 step 5: save binding params to NVM BEFORE probing,
   * so a reboot mid-probe still reconnects on next boot. */
  avb_persist_request_save(state);

  /* Send ACMP CONNECT_TX_COMMAND to the talker to get stream info */
  acmp_message_s acmp_cmd = {0};
  acmp_cmd.header.subtype = avtp_subtype_acmp;
  acmp_cmd.header.msg_type = acmp_msg_type_connect_tx_command;
  memcpy(acmp_cmd.talker_entity_id, stream->talker_id, UNIQUE_ID_LEN);
  memcpy(acmp_cmd.talker_uid, stream->talker_uid, 2);
  memcpy(acmp_cmd.listener_entity_id, state->own_entity.summary.entity_id,
         UNIQUE_ID_LEN);
  uint16_t listener_uid = desc_index;
  int_to_octets(&listener_uid, acmp_cmd.listener_uid, 2);
  memcpy(acmp_cmd.controller_entity_id, msg->common.controller_entity_id,
         UNIQUE_ID_LEN);

  avb_send_acmp_command(state, acmp_msg_type_connect_tx_command, &acmp_cmd,
                        false);
  stream->pending_connection = true;
  avbinfo("MVU: bind stream input %d to talker", desc_index);
  return avb_send_mvu_response(state, cmd, src_addr, sizeof(*cmd),
                               aecp_status_success);
}

/* GET_MEDIA_CLOCK_REFERENCE_INFO (MVU 0x0004)
 * Layout after aecp_mvu_s: clock_domain_index(2)
 * Response adds: flags(2) default_mcr_prio(1) user_mcr_prio(1)
 *   media_clock_domain_name(64) */
int avb_process_aecp_cmd_mvu_get_media_clock_ref_info(avb_state_s *state,
                                                      aecp_message_u *msg,
                                                      eth_addr_t *src_addr) {
  aecp_mvu_media_clock_ref_info_s *cmd =
      (aecp_mvu_media_clock_ref_info_s *)msg;

  aecp_mvu_media_clock_ref_info_s rsp;
  memset(&rsp, 0, sizeof(rsp));
  memcpy(&rsp.mvu, &cmd->mvu, sizeof(aecp_mvu_common_s));
  memcpy(rsp.clock_domain_index, cmd->clock_domain_index, 2);
  rsp.flags = 0x03; /* bits 7+6: user_mcr_prio and domain_name valid */
  rsp.default_mcr_prio = 192; /* Stageboxes/audio interfaces */
  rsp.user_mcr_prio = 192;
  strncpy((char *)rsp.media_clock_domain_name, "DEFAULT",
          sizeof(rsp.media_clock_domain_name));

  return avb_send_mvu_response(state, &rsp, src_addr, sizeof(rsp),
                               aecp_status_success);
}

/* GET_STREAM_INPUT_INFO_EX (MVU 0x0007)
 * Command layout after aecp_mvu_s: reserved(2) descriptor_type(2)
 *   descriptor_index(2)
 * Response adds: talker_entity_id(8) talker_unique_id(2)
 *   probing_acmp_status(1) reserved(1) */
int avb_process_aecp_cmd_mvu_get_stream_input_info_ex(avb_state_s *state,
                                                      aecp_message_u *msg,
                                                      eth_addr_t *src_addr) {
  aecp_mvu_get_stream_input_info_ex_cmd_s *cmd =
      (aecp_mvu_get_stream_input_info_ex_cmd_s *)msg;
  uint16_t desc_index = octets_to_uint(cmd->descriptor_index, 2);

  aecp_mvu_get_stream_input_info_ex_rsp_s rsp;
  memset(&rsp, 0, sizeof(rsp));
  memcpy(&rsp.mvu, &cmd->mvu, sizeof(aecp_mvu_common_s));
  memcpy(rsp.descriptor_type, cmd->descriptor_type, 2);
  memcpy(rsp.descriptor_index, cmd->descriptor_index, 2);

  if (desc_index >= state->num_input_streams) {
    return avb_send_mvu_response(state, &rsp, src_addr, sizeof(rsp),
                                 aecp_status_no_such_descriptor);
  }

  avb_listener_stream_s *stream = &state->input_streams[desc_index];
  memcpy(rsp.talker_entity_id, stream->talker_id, UNIQUE_ID_LEN);
  memcpy(rsp.talker_unique_id, stream->talker_uid, 2);
  /* probing_status (Milan Table 5.22): 0=Disabled, 3=Completed.
   * acmp_status mirrors IEEE 1722.1 ACMP status (0=SUCCESS). */
  uint8_t probing = stream->connected ? 3 : 0;
  uint8_t acmp = 0;
  rsp.probing_acmp_status = ((probing << 5) & 0xe0) | (acmp & 0x1f);

  return avb_send_mvu_response(state, &rsp, src_addr, sizeof(rsp),
                               aecp_status_success);
}

int avb_process_aecp(avb_state_s *state, aecp_message_u *msg,
                     eth_addr_t *src_addr) {

  /* Drop commands not addressed to us. Responses must always pass through
   * so the inflight tracker can match them. */
  uint8_t msg_type = msg->header.msg_type;
  bool is_command = (msg_type == aecp_msg_type_aem_command ||
                     msg_type == aecp_msg_type_addr_access_command ||
                     msg_type == aecp_msg_type_vendor_unique_command);
  if (is_command &&
      memcmp(msg->common.target_entity_id,
             state->own_entity.summary.entity_id, UNIQUE_ID_LEN) != 0) {
    return OK;
  }

  /* Process ADDRESS_ACCESS command (separate message type from AEM) */
  if (msg->header.msg_type == aecp_msg_type_addr_access_command) {
    return avb_process_aecp_addr_access(state, msg, src_addr);
  }

  /* Process Milan Vendor Unique command */
  if (msg->header.msg_type == aecp_msg_type_vendor_unique_command) {
    /* Validate MVU protocol ID */
    aecp_mvu_common_s *mvu = (aecp_mvu_common_s *)msg;
    uint8_t expected_pid[] = MVU_PROTOCOL_ID;
    if (memcmp(mvu->protocol_id, expected_pid, 6) != 0) {
      return OK; /* Not Milan MVU — ignore */
    }
    uint16_t mvu_cmd = (mvu->command_type_h << 8) | mvu->command_type;
    switch (mvu_cmd) {
    case mvu_cmd_get_milan_info:
      return avb_process_aecp_cmd_mvu_get_milan_info(state, msg, src_addr);
    case mvu_cmd_bind_stream:
      return avb_process_aecp_cmd_mvu_bind_stream(state, msg, src_addr, false);
    case mvu_cmd_unbind_stream:
      return avb_process_aecp_cmd_mvu_bind_stream(state, msg, src_addr, true);
    case mvu_cmd_get_system_unique_id:
      return avb_process_aecp_cmd_mvu_get_system_unique_id(state, msg, src_addr);
    case mvu_cmd_get_media_clock_ref_info:
      return avb_process_aecp_cmd_mvu_get_media_clock_ref_info(state, msg,
                                                               src_addr);
    case mvu_cmd_get_stream_input_info_ex:
      return avb_process_aecp_cmd_mvu_get_stream_input_info_ex(state, msg,
                                                               src_addr);
    default:
      /* Milan v1.3 §5.4.3: unsupported MVU commands must be reflected back
       * with NOT_IMPLEMENTED status rather than silently dropped */
      avbinfo("MVU: unsupported command 0x%04x, returning NOT_IMPLEMENTED",
              mvu_cmd);
      uint16_t cdl = (mvu->common.header.control_data_len_h << 8) |
                     mvu->common.header.control_data_len;
      uint16_t msg_len = AVTP_CDL_PREAMBLE_LEN + cdl;
      return avb_send_mvu_response(state, msg, src_addr, msg_len,
                                   aecp_status_not_implemented);
    }
  }

  /* Process AECP command */
  if (msg->header.msg_type == aecp_msg_type_aem_command) {
    switch (msg->basic.aem.command_type) {
    case aecp_cmd_code_register_unsol_notif:
      return avb_process_aecp_cmd_register_unsol_notif(state, msg, src_addr);
      break;
    case aecp_cmd_code_deregister_unsol_notif:
      return avb_process_aecp_cmd_deregister_unsol_notif(state, msg, src_addr);
      break;
    case aecp_cmd_code_acquire_entity:
      return avb_process_aecp_cmd_acquire_entity(state, msg, src_addr);
      break;
    case aecp_cmd_code_lock_entity:
      return avb_process_aecp_cmd_lock_entity(state, msg, src_addr);
      break;
    case aecp_cmd_code_entity_available:
      return avb_process_aecp_cmd_entity_available(state, msg, src_addr);
      break;
    case aecp_cmd_code_get_configuration:
      return avb_process_aecp_cmd_get_configuration(state, msg, src_addr);
      break;
    case aecp_cmd_code_read_descriptor:
      return avb_process_aecp_cmd_read_descriptor(state, msg, src_addr);
      break;
    case aecp_cmd_code_set_stream_format:
      return avb_process_aecp_cmd_set_stream_format(state, msg, src_addr);
      break;
    case aecp_cmd_code_get_stream_format:
      return avb_process_aecp_cmd_get_stream_format(state, msg, src_addr);
      break;
    case aecp_cmd_code_get_clock_source:
      return avb_process_aecp_cmd_get_clock_source(state, msg, src_addr);
      break;
    case aecp_cmd_code_set_clock_source:
      return avb_process_aecp_cmd_set_clock_source(state, msg, src_addr);
      break;
    case aecp_cmd_code_get_stream_info:
      return avb_process_aecp_cmd_get_stream_info(state, msg, src_addr);
      break;
    case aecp_cmd_code_get_avb_info:
      return avb_process_aecp_cmd_get_avb_info(state, msg, src_addr);
      break;
    case aecp_cmd_code_get_as_path:
      return avb_process_aecp_cmd_get_as_path(state, msg, src_addr);
      break;
    case aecp_cmd_code_get_counters:
      return avb_process_aecp_cmd_get_counters(state, msg, src_addr);
      break;
    case aecp_cmd_code_set_max_transit_time:
    case aecp_cmd_code_get_max_transit_time:
      return avb_process_aecp_cmd_get_max_transit_time(state, msg, src_addr);
      break;
    case aecp_cmd_code_set_name:
      return avb_process_aecp_cmd_set_name(state, msg, src_addr);
      break;
    case aecp_cmd_code_get_name:
      return avb_process_aecp_cmd_get_name(state, msg, src_addr);
      break;
    case aecp_cmd_code_set_control:
      return avb_process_aecp_cmd_set_control(state, msg, src_addr);
      break;
    case aecp_cmd_code_get_control:
      return avb_process_aecp_cmd_get_control(state, msg, src_addr);
      break;
    default: {
      /* IEEE 1722.1-2021 §9.2.1.3.1.4: unhandled AEM commands must be reflected
       * back with NOT_IMPLEMENTED status rather than silently dropped */
      avbinfo("AECP: unhandled AEM command 0x%04x, returning NOT_IMPLEMENTED",
              msg->basic.aem.command_type);
      struct timespec ts;
      uint16_t cdl = (msg->header.control_data_len_h << 8) |
                     msg->header.control_data_len;
      uint16_t msg_len = AVTP_CDL_PREAMBLE_LEN + cdl;
      msg->header.msg_type = aecp_msg_type_aem_response;
      msg->header.status_valtime = aecp_status_not_implemented;
      return avb_net_send_to(state, ethertype_avtp, msg, msg_len, &ts, src_addr);
    }
    }
  }
  /* Process AECP response */
  else {
    switch (msg->basic.aem.command_type) {
    case aecp_cmd_code_register_unsol_notif:
      return avb_process_aecp_rsp_register_unsol_notif(state, msg);
      break;
    case aecp_cmd_code_deregister_unsol_notif:
      return avb_process_aecp_rsp_deregister_unsol_notif(state, msg);
      break;
    case aecp_cmd_code_entity_available:
      return avb_process_aecp_rsp_entity_available(state, msg);
      break;
    case aecp_cmd_code_controller_available:
      return avb_process_aecp_rsp_controller_available(state, msg);
      break;
    case aecp_cmd_code_get_stream_info:
      return avb_process_aecp_rsp_get_stream_info(state, msg);
      break;
    case aecp_cmd_code_get_counters:
      return avb_process_aecp_rsp_get_counters(state, msg);
      break;
    default:
      return OK;
    }
  }
}

/* Process AECP command register unsolicited notification.
 * Response body is target + controller + seq + u+cmd_type + flags = 24
 * bytes, so cdl = 24 and the on-wire frame is 28 bytes (4-byte AVTP
 * common header + cdl). Previously this handler sent 44 bytes from a
 * 28-byte stack buffer (leaking 16 bytes of stack memory) and echoed
 * whatever cdl the command carried — which some Hive versions reject
 * because the declared cdl then doesn't match the frame's effective
 * body length. */
int avb_process_aecp_cmd_register_unsol_notif(avb_state_s *state,
                                              aecp_message_u *msg,
                                              eth_addr_t *src_addr) {
  int ret = OK;
  struct timespec ts;

  aecp_register_unsol_notif_s response;
  memset(&response, 0, sizeof(response));
  memcpy(&response, msg, sizeof(aecp_register_unsol_notif_s));
  response.common.header.msg_type = aecp_msg_type_aem_response;

  uint16_t cdl = sizeof(aecp_register_unsol_notif_s) - AVTP_CDL_PREAMBLE_LEN;
  response.common.header.control_data_len_h = (cdl >> 8) & 0x07;
  response.common.header.control_data_len = cdl & 0xFF;

  uint16_t msg_len = sizeof(aecp_register_unsol_notif_s);
  ret =
      avb_net_send_to(state, ethertype_avtp, &response, msg_len, &ts, src_addr);
  if (ret < 0) {
    avberr("send AECP register unsolicited notification response failed: %d",
           errno);
  } else {
    state->unsol_notif_enabled = true;
  }
  return ret;
}

/* Process AECP command deregister unsolicited notification. Same
 * size/cdl hygiene as the register handler above. */
int avb_process_aecp_cmd_deregister_unsol_notif(avb_state_s *state,
                                                aecp_message_u *msg,
                                                eth_addr_t *src_addr) {
  int ret = OK;
  struct timespec ts;

  aecp_register_unsol_notif_s response;
  memset(&response, 0, sizeof(response));
  memcpy(&response, msg, sizeof(aecp_register_unsol_notif_s));
  response.common.header.msg_type = aecp_msg_type_aem_response;

  uint16_t cdl = sizeof(aecp_register_unsol_notif_s) - AVTP_CDL_PREAMBLE_LEN;
  response.common.header.control_data_len_h = (cdl >> 8) & 0x07;
  response.common.header.control_data_len = cdl & 0xFF;

  uint16_t msg_len = sizeof(aecp_register_unsol_notif_s);
  ret =
      avb_net_send_to(state, ethertype_avtp, &response, msg_len, &ts, src_addr);
  if (ret < 0) {
    avberr("send AECP deregister unsolicited notification response failed: %d",
           errno);
  } else {
    state->unsol_notif_enabled = false;
  }
  return ret;
}

/* Process AECP command acquire entity */
int avb_process_aecp_cmd_acquire_entity(avb_state_s *state, aecp_message_u *msg,
                                        eth_addr_t *src_addr) {
  int ret;
  struct timespec ts;

  // create a response, copy the cmd message data, change the msg type and set
  // owner id
  aecp_acquire_entity_s response = {0};
  memcpy(&response, msg, sizeof(aecp_acquire_entity_s));
  response.common.header.msg_type = aecp_msg_type_aem_response;
  if (!msg->acquire_entity.release) {
    memcpy(response.owner_id, msg->common.controller_entity_id, UNIQUE_ID_LEN);
  }

  // send the response message
  uint16_t msg_len = sizeof(atdecc_header_s) + sizeof(aecp_acquire_entity_s);
  ret =
      avb_net_send_to(state, ethertype_avtp, &response, msg_len, &ts, src_addr);

  if (ret < 0) {
    avberr("send AECP Acquire Entity response failed: %d", errno);
  } else {
    if (msg->acquire_entity.release) {
      state->acquired = false;
    } else {
      state->acquired = true;
      memcpy(state->acquired_by, msg->common.controller_entity_id,
             UNIQUE_ID_LEN);
      state->last_acquired.tv_sec = ts.tv_sec;
      state->last_acquired.tv_usec = (suseconds_t)(ts.tv_nsec / 1000);
    }
  }
  return ret;
}

/* Process AECP command lock entity */
int avb_process_aecp_cmd_lock_entity(avb_state_s *state, aecp_message_u *msg,
                                     eth_addr_t *src_addr) {
  int ret;
  struct timespec ts;

  // create a response, copy the cmd message data, change the msg type and set
  // locked id
  aecp_lock_entity_s response = {0};
  memcpy(&response, msg, sizeof(aecp_lock_entity_s));
  response.common.header.msg_type = aecp_msg_type_aem_response;
  if (!msg->lock_entity.unlock) {
    memcpy(response.locked_id, msg->common.controller_entity_id, UNIQUE_ID_LEN);
  }

  // send the response message
  uint16_t msg_len = sizeof(atdecc_header_s) + sizeof(aecp_lock_entity_s);
  ret =
      avb_net_send_to(state, ethertype_avtp, &response, msg_len, &ts, src_addr);
  if (ret < 0) {
    avberr("send AECP Lock Entity response failed: %d", errno);
  } else {
    if (msg->lock_entity.unlock) {
      state->locked = false;
    } else {
      state->locked = true;
      memcpy(state->locked_by, msg->common.controller_entity_id, UNIQUE_ID_LEN);
      state->last_locked.tv_sec = ts.tv_sec;
      state->last_locked.tv_usec = (suseconds_t)(ts.tv_nsec / 1000);
    }
  }
  return ret;
}

/* Process AECP command entity available */
int avb_process_aecp_cmd_entity_available(avb_state_s *state,
                                          aecp_message_u *msg,
                                          eth_addr_t *src_addr) {
  int ret;
  struct timespec ts;

  // create a response, copy the cmd message data and change the msg type
  aecp_entity_available_rsp_s response;
  memset(&response, 0, sizeof(aecp_entity_available_rsp_s));
  memcpy(&response.common, &msg->common, sizeof(aecp_common_s));
  memcpy(&response.aem, &msg->basic.aem, sizeof(aecp_common_aem_s));
  response.common.header.msg_type = aecp_msg_type_aem_response;

  // send the response message
  uint16_t msg_len =
      sizeof(atdecc_header_s) + sizeof(aecp_entity_available_rsp_s);
  ret =
      avb_net_send_to(state, ethertype_avtp, &response, msg_len, &ts, src_addr);
  if (ret < 0) {
    avberr("send AECP Entity Available response failed: %d", errno);
  }
  return ret;
}

/* Process AECP command get configuration */
int avb_process_aecp_cmd_get_configuration(avb_state_s *state,
                                           aecp_message_u *msg,
                                           eth_addr_t *src_addr) {
  int ret;

  struct timespec ts;

  // create a response, copy the cmd message data and change the msg type, add
  // config index
  aecp_get_configuration_rsp_s response;
  memset(&response, 0, sizeof(aecp_get_configuration_rsp_s));
  memcpy(&response.common, &msg->common, sizeof(aecp_common_s));
  memcpy(&response.aem, &msg->basic.aem, sizeof(aecp_common_aem_s));
  response.common.header.msg_type = aecp_msg_type_aem_response;
  size_t config_index = DEFAULT_CONFIG_INDEX;
  int_to_octets(&config_index, response.configuration_index, 2);

  // send the response message
  uint16_t msg_len = sizeof(atdecc_header_s) + sizeof(aecp_lock_entity_s);
  ret =
      avb_net_send_to(state, ethertype_avtp, &response, msg_len, &ts, src_addr);
  if (ret < 0) {
    avberr("send AECP Get Configuration response failed: %d", errno);
  }
  return ret;
}

/* Process AECP command read descriptor */
int avb_process_aecp_cmd_read_descriptor(avb_state_s *state,
                                         aecp_message_u *msg,
                                         eth_addr_t *src_addr) {
  int ret = OK;

  // create a response, copy the cmd message data and change the msg type
  aecp_read_descriptor_rsp_s *response = NULL;
  response = (aecp_read_descriptor_rsp_s *)calloc(
      1, sizeof(aecp_read_descriptor_rsp_s));
  memcpy(response, msg, sizeof(aecp_read_descriptor_s));
  response->common.header.msg_type = aecp_msg_type_aem_response;

  // check if the descriptor type is supported
  switch (octets_to_uint(msg->read_descriptor.descriptor_type, 2)) {
  case aem_desc_type_entity:
    avb_send_aecp_rsp_read_descr_entity(state, response, src_addr);
    break;
  case aem_desc_type_configuration:
    avb_send_aecp_rsp_read_descr_configuration(state, response, src_addr);
    break;
  case aem_desc_type_audio_unit:
    avb_send_aecp_rsp_read_descr_audio_unit(state, response, src_addr);
    break;
  case aem_desc_type_stream_input:
    avb_send_aecp_rsp_read_descr_stream(state, response, src_addr, false);
    break;
  case aem_desc_type_stream_output:
    avb_send_aecp_rsp_read_descr_stream(state, response, src_addr, true);
    break;
  case aem_desc_type_avb_interface:
    avb_send_aecp_rsp_read_descr_avb_interface(state, response, src_addr);
    break;
  case aem_desc_type_clock_source:
    avb_send_aecp_rsp_read_descr_clock_source(state, response, src_addr);
    break;
  case aem_desc_type_memory_object:
    avb_send_aecp_rsp_read_descr_memory_obj(state, response, src_addr);
    break;
  case aem_desc_type_locale:
    avb_send_aecp_rsp_read_descr_locale(state, response, src_addr);
    break;
  case aem_desc_type_strings:
    avb_send_aecp_rsp_read_descr_strings(state, response, src_addr);
    break;
  case aem_desc_type_stream_port_input:
    avb_send_aecp_rsp_read_descr_stream_port(state, response, src_addr, false);
    break;
  case aem_desc_type_stream_port_output:
    avb_send_aecp_rsp_read_descr_stream_port(state, response, src_addr, true);
    break;
  case aem_desc_type_audio_cluster:
    avb_send_aecp_rsp_read_descr_audio_cluster(state, response, src_addr);
    break;
  case aem_desc_type_audio_map:
    avb_send_aecp_rsp_read_descr_audio_map(state, response, src_addr);
    break;
  case aem_desc_type_control:
    avb_send_aecp_rsp_read_descr_control(state, response, src_addr);
    break;
  case aem_desc_type_clock_domain:
    avb_send_aecp_rsp_read_descr_clock_domain(state, response, src_addr);
    break;
  default:
    char desc_type_str[7];
    octets_to_hex_string(msg->read_descriptor.descriptor_type, 2, desc_type_str,
                         '-');
    avbinfo("Ignoring AECP Read Descriptor for unsupported descriptor type %s",
            desc_type_str);
    ret = ERROR;
  }
  free(response);
  return ret;
}

/* Process AECP command set stream format */
int avb_process_aecp_cmd_set_stream_format(avb_state_s *state,
                                           aecp_message_u *msg,
                                           eth_addr_t *src_addr) {
  // validate descriptor type is stream input or output
  uint16_t descriptor_type =
      octets_to_uint(msg->stream_format.descriptor_type, 2);
  if (descriptor_type != aem_desc_type_stream_input &&
      descriptor_type != aem_desc_type_stream_output) {
    avberr("AECP Set Stream Format: unsupported descriptor type: %d",
           descriptor_type);
    return ERROR;
  }

  // validate descriptor index is in range
  uint16_t index = octets_to_uint(msg->stream_format.descriptor_index, 2);
  bool is_output = descriptor_type == aem_desc_type_stream_output;
  if ((is_output && index >= state->num_output_streams) ||
      (!is_output && index >= state->num_input_streams)) {
    avberr("AECP Set Stream Format: descriptor index out of range: %d", index);
    return ERROR;
  }

  // check requested format against supported formats. The CRF input
  // (stream_input index 1) only accepts the Milan AVnu CRF format per
  // Milan v1.3 §5.3.3.4 (AAF and CRF mutually exclusive in one stream).
  avtp_stream_format_s *requested = &msg->stream_format.stream_format;
  bool is_crf_input = !is_output && index == AVB_CRF_INPUT_INDEX;
  bool format_supported = false;
  if (is_crf_input) {
    uint8_t crf_bytes[8] = MILAN_AVNU_CRF_FORMAT_BYTES;
    if (memcmp(requested, crf_bytes, sizeof(crf_bytes)) == 0)
      format_supported = true;
  } else {
    for (size_t i = 0; i < state->num_supported_formats; i++) {
      if (memcmp(requested, &state->supported_formats[i],
                 sizeof(avtp_stream_format_s)) == 0) {
        format_supported = true;
        break;
      }
    }
  }

  if (!format_supported) {
    avberr("AECP Set Stream Format: unsupported format requested");
    return ERROR;
  }

  // update the stream format in state
  if (is_output) {
    memcpy(&state->output_streams[index].stream_format, requested,
           sizeof(avtp_stream_format_s));
  } else {
    memcpy(&state->input_streams[index].stream_format, requested,
           sizeof(avtp_stream_format_s));
  }
  avb_persist_request_save(state);

  // send the response
  return avb_send_aecp_rsp_set_stream_format(state, &msg->stream_format,
                                             src_addr);
}

/* Process AECP command get stream format */
int avb_process_aecp_cmd_get_stream_format(avb_state_s *state,
                                           aecp_message_u *msg,
                                           eth_addr_t *src_addr) {

  uint16_t descriptor_type =
      octets_to_uint(msg->stream_format.descriptor_type, 2);
  if (descriptor_type != aem_desc_type_stream_input &&
      descriptor_type != aem_desc_type_stream_output) {
    return ERROR;
  }

  uint16_t index = octets_to_uint(msg->stream_format.descriptor_index, 2);
  bool is_output = descriptor_type == aem_desc_type_stream_output;
  if ((is_output && index >= state->num_output_streams) ||
      (!is_output && index >= state->num_input_streams)) {
    return ERROR;
  }

  // fill in the current stream format
  avtp_stream_format_s *fmt = is_output
                                  ? &state->output_streams[index].stream_format
                                  : &state->input_streams[index].stream_format;
  memcpy(&msg->stream_format.stream_format, fmt, sizeof(avtp_stream_format_s));

  return avb_send_aecp_rsp_set_stream_format(state, &msg->stream_format,
                                             src_addr);
}

/* Process AECP command get clock source (IEEE 1722.1 §7.4.12) */
int avb_process_aecp_cmd_get_clock_source(avb_state_s *state,
                                          aecp_message_u *msg,
                                          eth_addr_t *src_addr) {

  struct timespec ts;
  aecp_aem_short_s *cmd = (aecp_aem_short_s *)msg;

  /* Response: descriptor_type + descriptor_index + clock_source_index (2 bytes).
   * Echo the currently-selected source the SET command committed, so
   * controllers can read-after-write to confirm the change. */
  uint8_t *clock_source_index = (uint8_t *)cmd + sizeof(aecp_aem_short_s);
  uint16_t active = state->media_clock.active_clock_source_index;
  int_to_octets(&active, clock_source_index, 2);

  cmd->common.header.msg_type = aecp_msg_type_aem_response;
  uint16_t control_data_len =
      sizeof(aecp_aem_short_s) + 2 - AVTP_CDL_PREAMBLE_LEN;
  cmd->common.header.control_data_len_h = (control_data_len >> 8) & 0xFF;
  cmd->common.header.control_data_len = control_data_len & 0xFF;
  uint16_t msg_len = sizeof(aecp_aem_short_s) + 2;

  int ret = avb_net_send_to(state, ethertype_avtp, cmd, msg_len, &ts, src_addr);
  if (ret < 0) {
    avberr("send AECP GET_CLOCK_SOURCE response failed: %d", errno);
  }
  return ret;
}

/* Process AECP command set clock source (IEEE 1722.1 §7.4.24, Milan
 * §5.4.4). Command body after the AEM header is:
 *   descriptor_type (2)       — must be CLOCK_DOMAIN (0x0024)
 *   descriptor_index (2)      — must be 0 (we have one clock domain)
 *   clock_source_index (2)    — which of our advertised sources to make active
 *   reserved (2)
 * The response mirrors the command; the status field signals success or
 * the specific validation failure. Once committed, the PLL (avbpll.c)
 * picks up the new active_clock_source_index on its next tick. */
int avb_process_aecp_cmd_set_clock_source(avb_state_s *state,
                                          aecp_message_u *msg,
                                          eth_addr_t *src_addr) {

  struct timespec ts;
  aecp_aem_short_s *cmd = (aecp_aem_short_s *)msg;

  uint16_t descriptor_type = octets_to_uint(cmd->descriptor_type, 2);
  uint16_t descriptor_index = octets_to_uint(cmd->descriptor_index, 2);
  uint8_t *csi_ptr = (uint8_t *)cmd + sizeof(aecp_aem_short_s);
  uint16_t requested_index = (uint16_t)octets_to_uint(csi_ptr, 2);

  aecp_status_t status = aecp_status_success;
  if (descriptor_type != aem_desc_type_clock_domain || descriptor_index != 0) {
    status = aecp_status_no_such_descriptor;
  } else if (requested_index > 1) {
    /* We only advertise clock sources 0 (INTERNAL/gPTP) and 1 (CRF). */
    status = aecp_status_bad_arguments;
  } else {
    state->media_clock.active_clock_source_index = requested_index;
    avbinfo("Clock source changed to %u (%s)", requested_index,
            requested_index == 0 ? "INTERNAL/gPTP" : "CRF stream input");
  }

  cmd->common.header.msg_type = aecp_msg_type_aem_response;
  cmd->common.header.status_valtime = (uint8_t)status;
  uint16_t control_data_len =
      sizeof(aecp_aem_short_s) + 2 - AVTP_CDL_PREAMBLE_LEN;
  cmd->common.header.control_data_len_h = (control_data_len >> 8) & 0xFF;
  cmd->common.header.control_data_len = control_data_len & 0xFF;
  uint16_t msg_len = sizeof(aecp_aem_short_s) + 2;

  int ret = avb_net_send_to(state, ethertype_avtp, cmd, msg_len, &ts, src_addr);
  if (ret < 0) {
    avberr("send AECP SET_CLOCK_SOURCE response failed: %d", errno);
  }
  return ret;
}

/* Process AECP command get max transit time (IEEE 1722.1 §7.4.78) */
int avb_process_aecp_cmd_get_max_transit_time(avb_state_s *state,
                                              aecp_message_u *msg,
                                              eth_addr_t *src_addr) {

  struct timespec ts;
  /* Echo command back as response, preserving payload.
   * For SET: accept the value. For GET: append max_transit_time. */
  uint16_t incoming_cdl =
      (msg->header.control_data_len_h << 8) | msg->header.control_data_len;
  msg->header.msg_type = aecp_msg_type_aem_response;

  bool is_get = (msg->basic.aem.command_type == aecp_cmd_code_get_max_transit_time);
  if (is_get) {
    /* GET response: append max_transit_time (uint64 ns) to command payload */
    aecp_max_transit_time_s *rsp = (aecp_max_transit_time_s *)msg;
    uint64_t transit_ns = 2000000; /* Class A: 2 ms */
    int_to_octets(&transit_ns, rsp->max_transit_time, 8);
    uint16_t cdl = sizeof(aecp_max_transit_time_s) - AVTP_CDL_PREAMBLE_LEN;
    msg->header.control_data_len_h = (cdl >> 8) & 0x07;
    msg->header.control_data_len = cdl & 0xFF;
    incoming_cdl = cdl;
  }
  uint16_t msg_len =
      sizeof(atdecc_header_s) + sizeof(unique_id_t) + incoming_cdl;

  int ret = avb_net_send_to(state, ethertype_avtp, msg, msg_len, &ts, src_addr);
  if (ret < 0) {
    avberr("send AECP MAX_TRANSIT_TIME response failed: %d", errno);
  }
  return ret;
}

/* Process AECP command get stream info */
int avb_process_aecp_cmd_get_stream_info(avb_state_s *state,
                                         aecp_message_u *msg,
                                         eth_addr_t *src_addr) {

  // validate descriptor type is stream input or output
  uint16_t descriptor_type =
      octets_to_uint(msg->get_stream_info.descriptor_type, 2);
  if (descriptor_type != aem_desc_type_stream_input &&
      descriptor_type != aem_desc_type_stream_output) {
    avberr("AECP Get Stream Info: unsupported descriptor type: %d",
           descriptor_type);
    return ERROR;
  }

  // validate descriptor index is in range
  uint16_t index = octets_to_uint(msg->get_stream_info.descriptor_index, 2);
  bool is_output = descriptor_type == aem_desc_type_stream_output;
  if ((is_output && index >= state->num_output_streams) ||
      (!is_output && index >= state->num_input_streams)) {
    avberr("AECP Get Stream Info: descriptor index out of range: %d", index);
    return ERROR;
  }

  // send the response
  return avb_send_aecp_rsp_get_stream_info(state, &msg->get_stream_info,
                                           src_addr);
}

/* Send AECP response get AVB info */
int avb_send_aecp_rsp_get_avb_info(avb_state_s *state, aecp_get_avb_info_s *msg,
                                   eth_addr_t *dest_addr) {
  int ret;
  struct timespec ts;

  // create a response message
  aecp_get_avb_info_rsp_s response = {0};
  memcpy(&response, msg, sizeof(aecp_get_avb_info_s));
  response.common.header.msg_type = aecp_msg_type_aem_response;

  // populate gPTP grandmaster ID from PTP status
  if (state->ptp_status.clock_source_valid) {
    memcpy(&response.gptp_grandmaster_id,
           state->ptp_status.clock_source_info.gm_id, UNIQUE_ID_LEN);
  }

  // populate propagation delay from PTP peer delay measurement
  uint32_t prop_delay = (uint32_t)state->ptp_status.peer_delay_ns;
  int_to_octets(&prop_delay, response.propagation_delay, 4);

  response.gptp_domain_number = state->avb_interface.domain_number;
  response.flags.as_capable = state->ptp_status.clock_source_valid;
  response.flags.gptp_enabled = state->avb_interface.flags.gptp_supported;
  response.flags.srp_enabled = state->avb_interface.flags.srp_supported;

  // populate MSRP mappings from state
  int_to_octets(&state->msrp_mappings_count, response.msrp_mappings_count, 2);
  memcpy(response.msrp_mappings, state->msrp_mappings,
         state->msrp_mappings_count * sizeof(aem_msrp_mapping_s));

  // calc control data length
  uint16_t control_data_len =
      sizeof(aecp_get_avb_info_rsp_s) - AVTP_CDL_PREAMBLE_LEN;

  // set the control data length
  response.common.header.control_data_len_h = (control_data_len >> 8) & 0xFF;
  response.common.header.control_data_len = control_data_len & 0xFF;

  // send the response message
  uint16_t msg_len = sizeof(aecp_get_avb_info_rsp_s);
  ret = avb_net_send_to(state, ethertype_avtp, &response, msg_len, &ts,
                        dest_addr);
  if (ret < 0) {
    avberr("send AECP Get AVB Info response failed: %d", errno);
  }
  return ret;
}

/* Process AECP command get AVB info */
int avb_process_aecp_cmd_get_avb_info(avb_state_s *state, aecp_message_u *msg,
                                      eth_addr_t *src_addr) {

  // validate descriptor type is avb_interface
  uint16_t descriptor_type =
      octets_to_uint(msg->get_avb_info.descriptor_type, 2);
  if (descriptor_type != aem_desc_type_avb_interface) {
    avberr("AECP Get AVB Info: unsupported descriptor type: %d",
           descriptor_type);
    return ERROR;
  }

  // validate descriptor index (only one interface supported)
  uint16_t index = octets_to_uint(msg->get_avb_info.descriptor_index, 2);
  if (index != 0) {
    avberr("AECP Get AVB Info: descriptor index out of range: %d", index);
    return ERROR;
  }

  // send the response
  return avb_send_aecp_rsp_get_avb_info(state, &msg->get_avb_info, src_addr);
}

/* Send AECP response get AS path */
int avb_send_aecp_rsp_get_as_path(avb_state_s *state, aecp_get_as_path_s *msg,
                                  eth_addr_t *dest_addr) {
  int ret;
  struct timespec ts;

  // create a response message
  aecp_get_as_path_rsp_s response = {0};
  memcpy(&response, msg, sizeof(aecp_get_as_path_s));
  response.common.header.msg_type = aecp_msg_type_aem_response;

  // build path sequence: grandmaster -> [intermediate clocks] -> this entity
  uint16_t count = 0;
  if (state->ptp_status.clock_source_valid) {
    // add grandmaster clock identity
    memcpy(&response.path_sequence[count],
           state->ptp_status.clock_source_info.gm_id, UNIQUE_ID_LEN);
    count++;
    // if the selected source is not the grandmaster (i.e. an intermediate
    // boundary clock like the AVB switch), add it to the path
    if (memcmp(state->ptp_status.clock_source_info.id,
               state->ptp_status.clock_source_info.gm_id,
               UNIQUE_ID_LEN) != 0) {
      memcpy(&response.path_sequence[count],
             state->ptp_status.clock_source_info.id, UNIQUE_ID_LEN);
      count++;
    }
  }
  // always end with this entity's own clock identity
  memcpy(&response.path_sequence[count],
         state->ptp_status.own_identity_info.id, UNIQUE_ID_LEN);
  count++;
  int_to_octets(&count, response.count, 2);

  // calc control data length (variable based on path count)
  uint16_t control_data_len = sizeof(aecp_common_s) - AVTP_CDL_PREAMBLE_LEN +
                              sizeof(aecp_common_aem_s) + 4 + 2 +
                              (count * UNIQUE_ID_LEN);

  // set the control data length
  response.common.header.control_data_len_h = (control_data_len >> 8) & 0xFF;
  response.common.header.control_data_len = control_data_len & 0xFF;

  // send the response message (only the populated portion)
  uint16_t msg_len = AVTP_CDL_PREAMBLE_LEN + control_data_len;
  ret = avb_net_send_to(state, ethertype_avtp, &response, msg_len, &ts,
                        dest_addr);
  if (ret < 0) {
    avberr("send AECP Get AS Path response failed: %d", errno);
  }
  return ret;
}

/* Process AECP command get AS path */
int avb_process_aecp_cmd_get_as_path(avb_state_s *state, aecp_message_u *msg,
                                     eth_addr_t *src_addr) {

  // validate descriptor index (only one interface supported)
  uint16_t index = octets_to_uint(msg->get_as_path.descriptor_index, 2);
  if (index != 0) {
    avberr("AECP Get AS Path: descriptor index out of range: %d", index);
    return ERROR;
  }

  // send the response
  return avb_send_aecp_rsp_get_as_path(state, &msg->get_as_path, src_addr);
}

/* Process AECP command get counters */
// counters can be for entity, stream input, stream output, avb interface or
// clock domain
int avb_process_aecp_cmd_get_counters(avb_state_s *state, aecp_message_u *msg,
                                      eth_addr_t *src_addr) {
  int ret;
  struct timespec ts;
  uint16_t control_data_len =
      sizeof(aecp_get_counters_rsp_s) - sizeof(atdecc_header_s);


  // create a response, copy the cmd message data and change the msg type
  aecp_get_counters_rsp_s response;
  memset(&response, 0, sizeof(aecp_get_counters_rsp_s));
  memcpy(&response, msg, sizeof(aecp_get_counters_rsp_s));
  response.common.header.msg_type = aecp_msg_type_aem_response;

  // check if the descriptor type is supported
  switch (octets_to_uint(msg->get_counters.descriptor_type, 2)) {
  case aem_desc_type_entity:
    // create entity counters valid flags
    aem_entity_counters_val_s entity_counters_val;
    memset(&entity_counters_val, 0, sizeof(aem_entity_counters_val_s));
    // create entity counters block
    aem_entity_counters_s entity_counters;
    memset(&entity_counters, 0, sizeof(aem_entity_counters_s));
    /* no entity specific counters */
    break;
  case aem_desc_type_stream_input:
    avb_get_stream_in_counters(&response.counters_valid.stream_in_counters_val,
                               &response.counters_block.stream_in_counters);
    break;
  case aem_desc_type_stream_output: {
    /* Set valid flags for Milan mandatory counters (Table 5.14) */
    aem_stream_out_counters_val_s *out_valid =
        &response.counters_valid.stream_out_counters_val;
    out_valid->stream_start = true;
    out_valid->stream_stop = true;
    out_valid->media_reset = true;
    out_valid->ts_uncertain = true;
    out_valid->frames_tx = true;
    /* Counter values are zero-initialized from memset above */
    break;
  }
  case aem_desc_type_avb_interface: {
    /* Set valid flags for Milan mandatory counters (Table 5.10) */
    aem_avb_interface_counters_val_s *iface_valid =
        &response.counters_valid.avb_interface_counters_val;
    iface_valid->link_up = true;
    iface_valid->link_down = true;
    iface_valid->gptp_gm_changed = true;
    break;
  }
  case aem_desc_type_clock_domain: {
    /* Set valid flags for Milan mandatory counters (Table 5.12) */
    aem_clock_domain_counters_val_s *clock_valid =
        &response.counters_valid.clock_domain_counters_val;
    clock_valid->locked = true;
    clock_valid->unlocked = true;
    break;
  }
  default:
    char desc_type_str[7];
    octets_to_hex_string(msg->get_counters.descriptor_type, 2, desc_type_str,
                         '-');
    avberr("Ignoring AECP Get Counters for unsupported descriptor type %s",
           desc_type_str);
    return ERROR;
  }

  // set the control data length
  response.common.header.control_data_len_h = (control_data_len >> 8) & 0xFF;
  response.common.header.control_data_len = control_data_len & 0xFF;

  // send the response message
  uint16_t msg_len = sizeof(atdecc_header_s) + control_data_len;
  ret =
      avb_net_send_to(state, ethertype_avtp, &response, msg_len, &ts, src_addr);
  if (ret < 0) {
    avberr("send AECP Get Counters response failed: %d", errno);
  }
  return ret;
}

/* Process AECP response register unsol notif */
int avb_process_aecp_rsp_register_unsol_notif(avb_state_s *state,
                                              aecp_message_u *msg) {
  return OK;
}

/* Process AECP response deregister unsol notif */
int avb_process_aecp_rsp_deregister_unsol_notif(avb_state_s *state,
                                                aecp_message_u *msg) {
  return OK;
}

/* Process AECP response entity available */
int avb_process_aecp_rsp_entity_available(avb_state_s *state,
                                          aecp_message_u *msg) {
  return OK;
}

/* Process AECP response controller available */
int avb_process_aecp_rsp_controller_available(avb_state_s *state,
                                              aecp_message_u *msg) {
  return OK;
}

/* Process AECP response get stream info
 * this may be sent as an unsolicited notification
 */
int avb_process_aecp_rsp_get_stream_info(avb_state_s *state,
                                         aecp_message_u *msg) {

  // find the talker and update the talker info
  int index = avb_find_entity_by_id(state, &msg->common.target_entity_id,
                                    avb_entity_type_talker);
  if (index == NOT_FOUND) {
    avberr("Ignoring AECP Get Stream Info response for unknown talker");
    return OK;
  }
  // update the talker stream info
  else {
    memcpy(&state->talkers[index].stream, &msg->get_stream_info_rsp.stream,
           sizeof(aem_stream_summary_s));
  }
  return OK;
}

/* Process AECP response get counters
 * this may be sent by the talker as an unsolicited notification
 */
int avb_process_aecp_rsp_get_counters(avb_state_s *state, aecp_message_u *msg) {
  return OK;
}

/* Process received ATDECC ACMP message */
int avb_process_acmp(avb_state_s *state, acmp_message_s *msg) {

  switch (msg->header.msg_type) {
  case acmp_msg_type_connect_rx_command:
    if (memcmp(msg->listener_entity_id, state->own_entity.summary.entity_id,
               UNIQUE_ID_LEN) != 0) {
      avbinfo("Ignoring ACMP Connect RX Command for different listener");
      break;
    }
    avb_process_acmp_connect_rx_command(state, msg, false);
    break;
  case acmp_msg_type_disconnect_rx_command:
    if (memcmp(msg->listener_entity_id, state->own_entity.summary.entity_id,
               UNIQUE_ID_LEN) != 0) {
      avbinfo("Ignoring ACMP Disconnect RX Command for different listener");
      break;
    }
    avb_process_acmp_connect_rx_command(state, msg, true);
    break;
  case acmp_msg_type_connect_tx_command:
    if (memcmp(msg->talker_entity_id, state->own_entity.summary.entity_id,
               UNIQUE_ID_LEN) != 0) {
      avbinfo("Ignoring ACMP Connect TX Command for different talker");
      break;
    }
    avb_process_acmp_connect_tx_command(state, msg, false);
    break;
  case acmp_msg_type_connect_tx_response:
    if (memcmp(msg->listener_entity_id, state->own_entity.summary.entity_id,
               UNIQUE_ID_LEN) != 0) {
      avbinfo("Ignoring ACMP Connect TX Response for different listener");
      break;
    }
    avb_process_acmp_connect_tx_response(state, msg, false);
    break;
  case acmp_msg_type_disconnect_tx_command:
    if (memcmp(msg->talker_entity_id, state->own_entity.summary.entity_id,
               UNIQUE_ID_LEN) != 0) {
      avbinfo("Ignoring ACMP Disconnect TX Command for different talker");
      break;
    }
    avb_process_acmp_connect_tx_command(state, msg, true);
    break;
  case acmp_msg_type_disconnect_tx_response:
    if (memcmp(msg->listener_entity_id, state->own_entity.summary.entity_id,
               UNIQUE_ID_LEN) != 0) {
      avbinfo("Ignoring ACMP Disconnect TX Response for different listener");
      break;
    }
    avb_process_acmp_connect_tx_response(state, msg, true);
    break;
  case acmp_msg_type_get_rx_state_command:
    if (memcmp(msg->listener_entity_id, state->own_entity.summary.entity_id,
               UNIQUE_ID_LEN) != 0) {
      avbinfo("Ignoring ACMP Get RX State Command for different listener");
      break;
    }
    avb_process_acmp_get_rx_state_command(state, msg, true);
    break;
  case acmp_msg_type_get_tx_state_command:
    if (memcmp(msg->talker_entity_id, state->own_entity.summary.entity_id,
               UNIQUE_ID_LEN) != 0) {
      avbinfo("Ignoring ACMP Get TX State Command for different talker");
      break;
    }
    avb_process_acmp_get_tx_state_command(state, msg, true);
    break;
  case acmp_msg_type_get_tx_connection_command:
    if (memcmp(msg->talker_entity_id, state->own_entity.summary.entity_id,
               UNIQUE_ID_LEN) != 0) {
      avbinfo("Ignoring ACMP Get TX Connection Command for different talker");
      break;
    }
    avb_process_acmp_get_tx_connection_command(state, msg, true);
    break;
  default:
    avbinfo("Ignoring %s (unsupported)",
            get_acmp_message_type_name(msg->header.msg_type));
    break;
  }
  return OK;
}

/* Send an ACMP command message */
int avb_send_acmp_command(avb_state_s *state, acmp_msg_type_t msg_type,
                          acmp_message_s *command, bool retried) {
  int ret = OK;
  struct timespec ts;
  int control_data_len = 84; // IEEE 1722.1-2021 sets it at 84 bytes

  // Set the message type
  command->header.msg_type = msg_type;

  // increment the sequence ID
  state->acmp_seq_id++;
  int_to_octets(&state->acmp_seq_id, &command->seq_id, 2);

  // send the message
  uint16_t msg_len = control_data_len + 12; // 12 bytes for the header
  ret = avb_net_send(state, ethertype_avtp, command, msg_len, &ts);
  if (ret < 0) {
    avberr("Failed to send %s (error: %d)",
           get_acmp_message_type_name(msg_type), errno);
  }

  // add the command to the inflight commands list
  avb_add_inflight_command(state, (atdecc_command_u *)command, false);

  avbinfo("Sent %s", get_acmp_message_type_name(msg_type));
  return ret;
}

/* Send an ACMP response message */
int avb_send_acmp_response(avb_state_s *state, acmp_msg_type_t msg_type,
                           acmp_message_s *response, acmp_status_t status) {
  int ret = OK;
  struct timespec ts;
  int control_data_len = 84; // IEEE 1722.1-2021 sets it at 84 bytes

  // Set the message type and status
  // sequence ID should be already set
  response->header.msg_type = msg_type;
  response->header.status_valtime = status;

  // as the response is mostly a copy of the command we will
  // keep the control data length the same as the command;
  // if the status is success, then we use the new control data length
  if (status == acmp_status_success) {
    response->header.control_data_len = control_data_len;
  }

  // send the message
  uint16_t msg_len =
      response->header.control_data_len + 12; // 12 bytes for the header
  ret = avb_net_send(state, ethertype_avtp, response, msg_len, &ts);
  if (ret < 0) {
    avberr("Failed to send %s (error: %d)",
           get_acmp_message_type_name(msg_type), errno);
  }
  avbinfo("Sent %s", get_acmp_message_type_name(msg_type));
  return ret;
}

/* Process ACMP connect/disconnect rx command */
int avb_process_acmp_connect_rx_command(avb_state_s *state, acmp_message_s *msg,
                                        bool disconnect) {
  int ret = OK;
  uint16_t index = octets_to_uint(msg->listener_uid, 2);
  acmp_status_t status = acmp_status_success;
  acmp_msg_type_t rx_response_type = disconnect
                                         ? acmp_msg_type_disconnect_rx_response
                                         : acmp_msg_type_connect_rx_response;
  acmp_msg_type_t tx_command_type = disconnect
                                        ? acmp_msg_type_disconnect_tx_command
                                        : acmp_msg_type_connect_tx_command;

  // check if listener is valid
  uint16_t listener_uid = octets_to_uint(msg->listener_uid, 2);
  if (!avb_valid_talker_listener_uid(state, listener_uid,
                                     avb_entity_type_listener)) {
    avb_send_acmp_response(state, rx_response_type, msg,
                           acmp_status_listener_unknown_id);
    return ERROR;
  }

  // check if locked or acquired by another controller
  if (avb_acquired_or_locked_by_other(state, &msg->controller_entity_id)) {
    avberr("ConnRX cmd: Locked or acquired by another controller");
    avb_send_acmp_response(state, rx_response_type, msg,
                           acmp_status_controller_not_authorized);
    return ERROR;
  }

  // check if listener is connected to another talker in case of connect rx
  // check if listener is not connected to the same talker in case of disconnect
  // rx
  bool same_talker = disconnect ? true : false;
  if (!avb_listener_is_connected(state, msg, same_talker)) {
    if (disconnect) {
      // listener is not connected to the talker in case of disconnect rx
      avberr("DisconnRX cmd: Listener is not connected to the talker");
      avb_send_acmp_response(state, rx_response_type, msg,
                             acmp_status_not_connected);
      return ERROR;
    }
  } else if (!disconnect) {
    // listener is connected to the another talker in case of connect rx
    avberr("ConnRX cmd: Listener is already connected to another talker");
    avb_send_acmp_response(state, rx_response_type, msg,
                           acmp_status_listener_exclusive);
    return ERROR;
  } else {
    // disconnect the listener
    status = avb_disconnect_listener(state, msg);
    if (status != acmp_status_success) {
      avberr("Failed to disconnect listener");
      avb_send_acmp_response(state, rx_response_type, msg, status);
      return ERROR;
    }
  }

  // add the inbound command to the inflight commands list
  ret = avb_add_inflight_command(state, (atdecc_command_u *)msg, true);
  if (ret == ERROR) {
    avberr("Failed to add inflight command");
    avb_send_acmp_response(state, rx_response_type, msg,
                           acmp_status_listener_misbehaving);
    return ERROR;
  }

  // create a new command for the connect/disconnect tx command
  acmp_message_s new_command = {0};
  memcpy(&new_command, msg, sizeof(acmp_message_s));

  // send connect/disconnect tx command to talker
  ret = avb_send_acmp_command(state, tx_command_type, &new_command, false);
  if (ret) {
    memcpy(&state->input_streams[index].talker_id, &msg->talker_entity_id,
           UNIQUE_ID_LEN);
    memcpy(&state->input_streams[index].talker_uid, &msg->talker_uid, 2);
    state->input_streams[index].pending_connection = true;
  }
  return ret;
}

/* Process ACMP connect tx command */
int avb_process_acmp_connect_tx_command(avb_state_s *state, acmp_message_s *msg,
                                        bool disconnect) {
  int ret = OK;
  acmp_msg_type_t tx_response_type = disconnect
                                         ? acmp_msg_type_disconnect_tx_response
                                         : acmp_msg_type_connect_tx_response;

  // validate talker UID refers to one of our output streams
  uint16_t talker_uid = octets_to_uint(msg->talker_uid, 2);
  if (!avb_valid_talker_listener_uid(state, talker_uid,
                                     avb_entity_type_talker)) {
    avb_send_acmp_response(state, tx_response_type, msg,
                           acmp_status_talker_unknown_id);
    return ERROR;
  }

  // check if locked or acquired by another controller
  if (avb_acquired_or_locked_by_other(state, &msg->controller_entity_id)) {
    avberr("ConnTX cmd: Locked or acquired by another controller");
    avb_send_acmp_response(state, tx_response_type, msg,
                           acmp_status_controller_not_authorized);
    return ERROR;
  }
  if (disconnect) {
    /* Disconnect: stop streaming as fallback if MSRP listener leave
     * doesn't arrive. MSRP handler will also stop on listener leave. */
    if (state->output_streams[talker_uid].streaming) {
      avb_stop_stream_out(state, talker_uid);
    }
  }
  /* Streaming is now driven by MSRP Listener Ready, not ACMP.
   * Respond with stream info so the listener can do MSRP registration. */
  memcpy(msg->stream_id, state->output_streams[talker_uid].stream_id,
         UNIQUE_ID_LEN);
  memcpy(msg->stream_dest_addr,
         state->output_streams[talker_uid].stream_dest_addr, ETH_ADDR_LEN);
  avb_send_acmp_response(state, tx_response_type, msg, acmp_status_success);
  return ret;
}

/* Process ACMP connect tx response */
int avb_process_acmp_connect_tx_response(avb_state_s *state,
                                         acmp_message_s *response,
                                         bool disconnect) {
  int ret = OK;
  acmp_status_t status = response->header.status_valtime;
  uint16_t listener_uid =
      octets_to_uint(response->listener_uid, 2); // for the conn tx command
  acmp_msg_type_t rx_reponse_type = disconnect
                                        ? acmp_msg_type_disconnect_rx_response
                                        : acmp_msg_type_connect_rx_response;

  // check if listener is valid
  if (!avb_valid_talker_listener_uid(state, listener_uid,
                                     avb_entity_type_listener)) {
    if (disconnect) {
      // don't send a disconnect rx response if the listener is unknown
      return ERROR;
    }
    status = acmp_status_listener_unknown_id;
  }

  // create a response message for the controller
  acmp_message_s new_response = {0};
  memcpy(&new_response, response, sizeof(acmp_message_s));

  if (!disconnect) {
    // if succcessful connect tx response then connect the listener
    if (response->header.status_valtime == acmp_status_success) {
      status = avb_connect_listener(state, response);
      avbinfo("ConnTX rsp: listener_uid=%d, connect_status=%d", listener_uid, status);
      if (status == acmp_status_success) {
        int stream_ret = avb_start_stream_in(state, listener_uid);
        avbinfo("avb_start_stream_in returned %d", stream_ret);
      }
    } else {
      avbwarn("ConnTX rsp: talker returned status %d", response->header.status_valtime);
    }
    // update input stream
    state->input_streams[listener_uid].pending_connection = false;
  }

  // put the sequence ID from the inflight command into the new response
  response->header.msg_type = rx_reponse_type;
  int index = avb_find_inflight_command_by_data(
      state, (atdecc_command_u *)response, true);
  if (index != NOT_FOUND) {
    int_to_octets(&state->inflight_commands[index].acmp_seq_id,
                  &new_response.seq_id, 2);
  }

  // other stuff TODO
  // cancelTimeout(msg);

  // remove the inflight command for the connect tx command
  avb_remove_inflight_command(state, octets_to_uint(response->seq_id, 2),
                              false);

  // remove the inflight command for the original connect rx command
  if (index != NOT_FOUND) {
    avb_remove_inflight_command(
        state, state->inflight_commands[index].acmp_seq_id, true);
  }

  /* Only send a CONNECT_RX_RESPONSE back to a controller if this
   * TX response was actually triggered by a controller's CONNECT_RX.
   * Fast-connect probes (self-issued at boot) have no matching
   * inflight RX — sending would emit a spurious response with a
   * stale seq_id that no controller asked for. */
  if (index != NOT_FOUND) {
    ret = avb_send_acmp_response(state, rx_reponse_type, &new_response, status);
  }

  if (status == acmp_status_success) {
    /* Stream-in now handled by the unified EMAC RX dispatcher via
     * avb_net_set_stream_rx_handler — no EMAC callback change needed here. */
  }
  return ret;
}

/* Process ACMP get rx state command */
int avb_process_acmp_get_rx_state_command(avb_state_s *state,
                                          acmp_message_s *msg, bool rx) {
  uint16_t listener_uid = octets_to_uint(msg->listener_uid, 2);

  // validate listener UID
  if (!avb_valid_talker_listener_uid(state, listener_uid,
                                     avb_entity_type_listener)) {
    avb_send_acmp_response(state, acmp_msg_type_get_rx_state_response, msg,
                           acmp_status_listener_unknown_id);
    return ERROR;
  }

  // build response from current listener stream state
  acmp_message_s response = {0};
  memcpy(&response, msg, sizeof(acmp_message_s));
  avb_listener_stream_s *stream = &state->input_streams[listener_uid];

  // populate talker info and stream fields from the listener's stored state
  memcpy(&response.talker_entity_id, &stream->talker_id, UNIQUE_ID_LEN);
  memcpy(&response.talker_uid, &stream->talker_uid, 2);
  memcpy(&response.stream_id, &stream->stream_id, UNIQUE_ID_LEN);
  memcpy(&response.stream_dest_addr, &stream->stream_dest_addr,
         sizeof(eth_addr_t));
  memcpy(&response.stream_vlan_id, &stream->vlan_id, 2);

  // set connection count: 1 if connected, 0 otherwise
  uint16_t count = stream->connected ? 1 : 0;
  int_to_octets(&count, response.connection_count, 2);

  return avb_send_acmp_response(state, acmp_msg_type_get_rx_state_response,
                                &response, acmp_status_success);
}

/* Process ACMP get tx state command */
int avb_process_acmp_get_tx_state_command(avb_state_s *state,
                                          acmp_message_s *msg, bool tx) {
  uint16_t talker_uid = octets_to_uint(msg->talker_uid, 2);

  // validate talker UID
  if (!avb_valid_talker_listener_uid(state, talker_uid,
                                     avb_entity_type_talker)) {
    avb_send_acmp_response(state, acmp_msg_type_get_tx_state_response, msg,
                           acmp_status_talker_unknown_id);
    return ERROR;
  }

  // build response from current talker stream state
  acmp_message_s response = {0};
  memcpy(&response, msg, sizeof(acmp_message_s));
  avb_talker_stream_s *stream = &state->output_streams[talker_uid];

  // populate stream fields from the talker's stored state
  memcpy(&response.stream_id, &stream->stream_id, UNIQUE_ID_LEN);
  memcpy(&response.stream_dest_addr, &stream->stream_dest_addr,
         sizeof(eth_addr_t));
  memcpy(&response.stream_vlan_id, &stream->vlan_id, 2);
  memcpy(&response.connection_count, &stream->connection_count, 2);

  return avb_send_acmp_response(state, acmp_msg_type_get_tx_state_response,
                                &response, acmp_status_success);
}

/* Process ACMP get tx connection command — not supported (Milan v1.3 5.5.4.4) */
int avb_process_acmp_get_tx_connection_command(avb_state_s *state,
                                               acmp_message_s *msg, bool tx) {
  return avb_send_acmp_response(state, acmp_msg_type_get_tx_connection_response,
                                msg, acmp_status_not_supported);
}

/* Find an entity in the known list of talkers, listeners, or controllers */
int avb_find_entity_by_id(avb_state_s *state, unique_id_t *entity_id,
                          avb_entity_type_t entity_type) {
  switch (entity_type) {
  case avb_entity_type_talker:
    for (int i = 0; i < state->num_talkers; i++) {
      if (memcmp(state->talkers[i].entity_id, entity_id, UNIQUE_ID_LEN) == 0) {
        return i;
      }
    }
    break;
  case avb_entity_type_listener:
    for (int i = 0; i < state->num_listeners; i++) {
      if (memcmp(state->listeners[i].entity_id, entity_id, UNIQUE_ID_LEN) ==
          0) {
        return i;
      }
    }
    break;
  case avb_entity_type_controller:
    for (int i = 0; i < state->num_controllers; i++) {
      if (memcmp(state->controllers[i].entity_id, entity_id, UNIQUE_ID_LEN) ==
          0) {
        return i;
      }
    }
    break;
  default:
    return NOT_FOUND;
  }
  return NOT_FOUND;
}

/* Find an entity in the known list of talkers, listeners, or controllers */
int avb_find_entity_by_addr(avb_state_s *state, eth_addr_t *entity_addr,
                            avb_entity_type_t entity_type) {
  switch (entity_type) {
  case avb_entity_type_talker:
    for (int i = 0; i < state->num_talkers; i++) {
      if (memcmp(state->talkers[i].mac_addr, entity_addr, ETH_ADDR_LEN) == 0) {
        return i;
      }
    }
    break;
  case avb_entity_type_listener:
    for (int i = 0; i < state->num_listeners; i++) {
      if (memcmp(state->listeners[i].mac_addr, entity_addr, ETH_ADDR_LEN) ==
          0) {
        return i;
      }
    }
    break;
  case avb_entity_type_controller:
    for (int i = 0; i < state->num_controllers; i++) {
      if (memcmp(state->controllers[i].mac_addr, entity_addr, ETH_ADDR_LEN) ==
          0) {
        return i;
      }
    }
    break;
  default:
    return NOT_FOUND;
  }
  return NOT_FOUND;
}

// Get the name of the ADP message type
const char *get_adp_message_type_name(adp_msg_type_t message_type) {
  switch (message_type) {
  case adp_msg_type_entity_available:
    return "ADP Entity Available";
  case adp_msg_type_entity_departing:
    return "ADP Entity Departing";
  case adp_msg_type_entity_discover:
    return "ADP Entity Discover";
  default:
    return "Unknown";
  }
}

// Get the name of the AECP command code
const char *get_aecp_command_code_name(aecp_cmd_code_t command_code) {
  switch (command_code) {
  case aecp_cmd_code_acquire_entity:
    return "AECP Acquire Entity";
  case aecp_cmd_code_lock_entity:
    return "AECP Lock Entity";
  case aecp_cmd_code_entity_available:
    return "AECP Entity Available";
  case aecp_cmd_code_controller_available:
    return "AECP Controller Available";
  case aecp_cmd_code_read_descriptor:
    return "AECP Read Descriptor";
  default:
    return "Unknown";
  }
}

// Get the name of the ACMP message type
const char *get_acmp_message_type_name(acmp_msg_type_t message_type) {
  switch (message_type) {
  case acmp_msg_type_connect_tx_command:
    return "ACMP Connect TX Command";
  case acmp_msg_type_connect_tx_response:
    return "ACMP Connect TX Response";
  case acmp_msg_type_disconnect_tx_command:
    return "ACMP Disconnect TX Command";
  case acmp_msg_type_disconnect_tx_response:
    return "ACMP Disconnect TX Response";
  case acmp_msg_type_get_tx_state_command:
    return "ACMP Get TX State Command";
  case acmp_msg_type_get_tx_state_response:
    return "ACMP Get TX State Response";
  case acmp_msg_type_connect_rx_command:
    return "ACMP Connect RX Command";
  case acmp_msg_type_connect_rx_response:
    return "ACMP Connect RX Response";
  case acmp_msg_type_disconnect_rx_command:
    return "ACMP Disconnect RX Command";
  case acmp_msg_type_disconnect_rx_response:
    return "ACMP Disconnect RX Response";
  case acmp_msg_type_get_rx_state_command:
    return "ACMP Get RX State Command";
  case acmp_msg_type_get_rx_state_response:
    return "ACMP Get RX State Response";
  default:
    return "Unknown";
  }
}

/* Check if the talker or listener unique ID is valid */
bool avb_valid_talker_listener_uid(avb_state_s *state, uint16_t uid,
                                   avb_entity_type_t entity_type) {
  /* uid here is the local listener_uid / talker_uid from ACMP
   * (i.e. our own STREAM_INPUT / STREAM_OUTPUT descriptor index).
   * num_listeners / num_talkers count remotely-discovered entities,
   * not our own streams — wrong array. Use num_input/output_streams. */
  switch (entity_type) {
  case avb_entity_type_listener:
    return uid < state->num_input_streams;
  case avb_entity_type_talker:
    return uid < state->num_output_streams;
  default:
    return false;
  }
}

/* Check if the entity is acquired or locked by another entity */
bool avb_acquired_or_locked_by_other(avb_state_s *state,
                                     unique_id_t *entity_id) {
  if (!state->locked && !state->acquired) {
    return false;
  }
  if (memcmp(state->acquired_by, entity_id, UNIQUE_ID_LEN) != 0) {
    return true;
  }
  if (memcmp(state->locked_by, entity_id, UNIQUE_ID_LEN) != 0) {
    return true;
  }
  return false;
}

/* Check if the listener is connected to another talker or same talker */
bool avb_listener_is_connected(avb_state_s *state, acmp_message_s *msg,
                               bool same_talker) {
  // walk through all input streams
  for (int i = 0; i < state->num_input_streams; i++) {
    // check if the stream is connected or pending connection
    if (state->input_streams[i].connected ||
        state->input_streams[i].pending_connection) {
      if (same_talker) {
        // check if the talker is the same
        if (memcmp(state->input_streams[i].talker_id, msg->talker_entity_id,
                   UNIQUE_ID_LEN) == 0 &&
            memcmp(state->input_streams[i].talker_uid, msg->talker_uid, 2) ==
                0) {
          return true;
        }
      } else {
        // check if the talker is different
        if (memcmp(state->input_streams[i].talker_id, msg->talker_entity_id,
                   UNIQUE_ID_LEN) != 0 &&
            memcmp(state->input_streams[i].talker_uid, msg->talker_uid, 2) !=
                0) {
          return true;
        }
      }
    }
  }
  return false;
}

/* add an inflight command to the inflight commands list */
// return the index of the command, if the list is full then return -1
int avb_add_inflight_command(avb_state_s *state, atdecc_command_u *command,
                             bool inbound) {
  if (state->num_inflight_commands >= AVB_MAX_NUM_INFLIGHT_COMMANDS) {
    avberr("Cannot add command: Inflight commands list is full.");
    return ERROR;
  }

  // check if the command is already in the list
  atdecc_command_u match_command = {0};
  memcpy(&match_command, command, sizeof(atdecc_command_u));
  match_command.header.msg_type = command->header.msg_type + 1;
  int index = avb_find_inflight_command_by_data(state, &match_command, inbound);
  if (index != NOT_FOUND) {
    avbwarn("Command already in list at index %d", index);
    return index;
  }

  // set the index to the next available slot
  index = state->num_inflight_commands;
  int timeout_ms = 250; // default timeout amount

  // copy the sequence ID from the command and adjust the timeout_ms for ACMP
  // commands
  if (command->header.subtype == avtp_subtype_acmp) {
    state->inflight_commands[index].acmp_seq_id =
        octets_to_uint(command->acmp.seq_id, 2);
    timeout_ms = avb_get_acmp_timeout_ms(
        command->header.msg_type); // ACMP timeouts are more specific
  } else {
    state->inflight_commands[index].aecp_seq_id =
        octets_to_uint(command->aecp.common.seq_id, 2);
  }

  // set the timeout
  struct timeval timeout;
  gettimeofday(&timeout, NULL);
  timeval_add_ms(&timeout, timeout_ms);
  memcpy(&state->inflight_commands[index].timeout, &timeout,
         sizeof(struct timeval));

  // set the inbound flag
  state->inflight_commands[index].inbound = inbound;

  // copy the command data to the inflight command
  memcpy(&state->inflight_commands[index].command, command,
         sizeof(atdecc_command_u));

  // set retried to false and increment the number of inflight commands
  state->inflight_commands[index].retried = false;
  state->num_inflight_commands++;
  return index;
}

/* find an inflight command by the sequence ID */
int avb_find_inflight_command(avb_state_s *state, uint16_t seq_id,
                              bool inbound) {
  for (int i = 0; i < state->num_inflight_commands; i++) {
    if (state->inflight_commands[i].command.header.subtype ==
        avtp_subtype_acmp) {
      if (state->inflight_commands[i].acmp_seq_id == seq_id &&
          state->inflight_commands[i].inbound == inbound) {
        return i;
      }
    } else {
      if (state->inflight_commands[i].aecp_seq_id == seq_id &&
          state->inflight_commands[i].inbound == inbound) {
        return i;
      }
    }
  }
  return NOT_FOUND;
}

/* find an inflight commands list by the matching command data */
int avb_find_inflight_command_by_data(avb_state_s *state,
                                      atdecc_command_u *data, bool inbound) {

  for (int i = 0; i < state->num_inflight_commands; i++) {
    if (state->inflight_commands[i].inbound == inbound &&
        state->inflight_commands[i].command.header.msg_type + 1 ==
            data->header.msg_type) {
      if (data->header.subtype == avtp_subtype_acmp) {
        if (memcmp(state->inflight_commands[i].command.acmp.talker_uid,
                   data->acmp.talker_uid, 2) == 0 &&
            memcmp(state->inflight_commands[i].command.acmp.listener_uid,
                   data->acmp.listener_uid, 2) == 0 &&
            memcmp(state->inflight_commands[i].command.acmp.talker_entity_id,
                   data->acmp.talker_entity_id, UNIQUE_ID_LEN) == 0 &&
            memcmp(state->inflight_commands[i].command.acmp.listener_entity_id,
                   data->acmp.listener_entity_id, UNIQUE_ID_LEN) == 0 &&
            memcmp(
                state->inflight_commands[i].command.acmp.controller_entity_id,
                data->acmp.controller_entity_id, UNIQUE_ID_LEN) == 0) {
          return i;
        }
      } else {
        if (memcmp(state->inflight_commands[i]
                       .command.aecp.common.target_entity_id,
                   data->aecp.common.target_entity_id, UNIQUE_ID_LEN) == 0 &&
            memcmp(state->inflight_commands[i]
                       .command.aecp.common.controller_entity_id,
                   data->aecp.common.controller_entity_id,
                   UNIQUE_ID_LEN) == 0 &&
            state->inflight_commands[i].command.aecp.aem.command_type ==
                data->aecp.aem.command_type) {
          return i;
        }
      }
    }
  }
  return NOT_FOUND;
}

/* remove an inflight command from the inflight commands list */
// shift any elements after the index to the left
void avb_remove_inflight_command(avb_state_s *state, uint16_t seq_id,
                                 bool inbound) {
  int index = avb_find_inflight_command(state, seq_id, inbound);
  if (index == NOT_FOUND) {
    avberr("Cannot remove: Inflight command with seq ID %d not found.", seq_id);
    return;
  }
  for (int i = index; i < state->num_inflight_commands - 1; i++) {
    state->inflight_commands[i] = state->inflight_commands[i + 1];
  }
  state->num_inflight_commands--;
}

/* Connect a listener to a stream */
acmp_status_t avb_connect_listener(avb_state_s *state,
                                   acmp_message_s *response) {
  acmp_status_t status = acmp_status_success;
  uint16_t index = octets_to_uint(response->listener_uid, 2);
  avb_listener_stream_s *stream = &state->input_streams[index];

  // Copy identity + stream parameters from the talker's response.
  // Works for both plain 1722.1 CONNECT_TX_RESPONSE and Milan
  // PROBE_TX_RESPONSE (same wire format).
  memcpy(stream->talker_id, response->talker_entity_id, UNIQUE_ID_LEN);
  memcpy(stream->talker_uid, response->talker_uid, 2);
  memcpy(stream->controller_id, response->controller_entity_id, UNIQUE_ID_LEN);
  memcpy(stream->stream_id, response->stream_id, UNIQUE_ID_LEN);
  memcpy(stream->stream_dest_addr, response->stream_dest_addr, ETH_ADDR_LEN);
  /* Some talkers send stream_vlan_id=0 meaning "use configured default".
   * Milan §5.5.3.6.16 requires a non-zero vlan_id whenever the listener
   * is connected — Hive flags a zero here as a fatal enumeration error
   * on the next GET_RX_STATE. Fall back to the configured Class A VID
   * (Milan default = 2) in that case. */
  if (octets_to_uint(response->stream_vlan_id, 2) == 0) {
    uint16_t default_vid = CONFIG_ESP_AVB_STREAM_VLAN_ID;
    int_to_octets(&default_vid, stream->vlan_id, 2);
  } else {
    memcpy(stream->vlan_id, response->stream_vlan_id, 2);
  }

  // Class from ACMP flags per IEEE 1722.1-2021 §8.2.1.16 Table 8-4:
  // bit 15 = CLASS_B. Milan talkers always set this to 0 (class is
  // derived from stream_format instead), which is correct since Milan
  // uses Class A by default — so reading the bit works for both.
  uint16_t flags = octets_to_uint(response->flags, 2);
  stream->stream_info_flags.class_b = (flags & 0x8000) ? 1 : 0;

  stream->connected = true;

  // send SRP listener ready command
  int ret = avb_send_msrp_listener(state, mrp_attr_event_new,
                                   msrp_listener_event_ready, false,
                                   &state->input_streams[index].stream_id);
  if (ret < 0) {
    return acmp_status_listener_misbehaving;
  }

  /* Persist the new connection state to NVS so it survives reboots
   * (Milan §5.5.3.6.17 saved-state requirement). */
  avb_persist_request_save(state);

  return status;
}

acmp_status_t avb_disconnect_listener(avb_state_s *state,
                                      acmp_message_s *response) {
  acmp_status_t status = acmp_status_success;
  uint16_t index = octets_to_uint(response->listener_uid, 2);

  // Stop stream-in handler if active
  avb_stop_stream_in(state, index);

  // Reset connection state but preserve stream format and vlan_id
  avtp_stream_format_s saved_format = state->input_streams[index].stream_format;
  uint8_t saved_vlan[2];
  memcpy(saved_vlan, state->input_streams[index].vlan_id, 2);
  memset(&state->input_streams[index], 0, sizeof(avb_listener_stream_s));
  memcpy(&state->input_streams[index].stream_format, &saved_format,
         sizeof(avtp_stream_format_s));
  memcpy(state->input_streams[index].vlan_id, saved_vlan, 2);

  // send SRP listener deregistration command
  int ret = avb_send_msrp_listener(state, mrp_attr_event_lv,
                                   msrp_listener_event_ready, false,
                                   &state->input_streams[index].stream_id);
  if (ret < 0) {
    return acmp_status_listener_misbehaving;
  }

  /* Persist the cleared state so we don't auto-reconnect on next boot. */
  avb_persist_request_save(state);

  return status;
}

/* Issue a fast-connect CONNECT_TX_COMMAND (Milan PROBE_TX_COMMAND) for
 * one listener stream that has a saved talker binding but is not yet
 * connected. Called periodically from the main loop — per Milan
 * §5.5.3.6.17 / TMR_RETRY the listener retries on a 4 s cadence while
 * waiting for the talker to reappear on the network.
 *
 * Returns true if a command was sent, false otherwise (no binding,
 * already connected, talker not yet in ADP discovery table, or
 * attempt too recent). */
static bool avb_fast_connect_listener(avb_state_s *state, uint16_t index) {
  avb_listener_stream_s *stream = &state->input_streams[index];

  /* Gate: must have a saved binding and not already be connected/probing. */
  unique_id_t zero_id = {0};
  if (memcmp(stream->talker_id, zero_id, UNIQUE_ID_LEN) == 0)
    return false;
  if (stream->connected || stream->pending_connection)
    return false;

  /* Milan requires the talker to be ADP-discovered before probing. */
  int talker_idx =
      avb_find_entity_by_id(state, &stream->talker_id, avb_entity_type_talker);
  if (talker_idx == NOT_FOUND)
    return false;

  /* Build CONNECT_TX_COMMAND — same shape as BIND_STREAM's probe. */
  acmp_message_s cmd = {0};
  cmd.header.subtype = avtp_subtype_acmp;
  cmd.header.msg_type = acmp_msg_type_connect_tx_command;
  memcpy(cmd.talker_entity_id, stream->talker_id, UNIQUE_ID_LEN);
  memcpy(cmd.talker_uid, stream->talker_uid, 2);
  memcpy(cmd.listener_entity_id, state->own_entity.summary.entity_id,
         UNIQUE_ID_LEN);
  int_to_octets(&index, cmd.listener_uid, 2);
  memcpy(cmd.controller_entity_id, stream->controller_id, UNIQUE_ID_LEN);

  if (avb_send_acmp_command(state, acmp_msg_type_connect_tx_command, &cmd,
                            false) < 0) {
    return false;
  }

  stream->pending_connection = true;
  avbinfo("ACMP: fast-connect probe for stream_input %d", index);
  return true;
}

/* Periodically attempt fast-connect for any listener stream with a
 * saved binding that isn't currently connected. Throttled per-stream
 * by AVB_FAST_CONNECT_RETRY_MSEC. Called from avb_periodic_send. */
void avb_periodic_fast_connect(avb_state_s *state) {
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);

  for (uint16_t i = 0; i < state->num_input_streams; i++) {
    avb_listener_stream_s *stream = &state->input_streams[i];

    /* Cheap early-out: no saved binding or already connected. */
    if (stream->connected || stream->pending_connection)
      continue;
    unique_id_t zero_id = {0};
    if (memcmp(stream->talker_id, zero_id, UNIQUE_ID_LEN) == 0)
      continue;

    /* Throttle retries. zero-initialized timestamp on boot means the
     * first attempt fires immediately. */
    struct timespec delta;
    clock_timespec_subtract(&now, &stream->last_fast_connect_attempt, &delta);
    if (stream->last_fast_connect_attempt.tv_sec != 0 &&
        timespec_to_ms(&delta) < AVB_FAST_CONNECT_RETRY_MSEC)
      continue;

    stream->last_fast_connect_attempt = now;
    avb_fast_connect_listener(state, i);
  }
}

/* Start a stream */
acmp_status_t avb_connect_talker(avb_state_s *state, acmp_message_s *response) {
  acmp_status_t status = acmp_status_success;
  uint16_t index = octets_to_uint(response->talker_uid, 2);

  // create a stream ID from the internal MAC address and stream index
  // unique_id_t stream_id;
  // stream_id_from_mac(&state->internal_mac_addr, stream_id, index);

  // if output stream not already started then
  // create a stream ID, send SRP talker registration
  return status;
}

/* Get the timeout for an ACMP message type */
int avb_get_acmp_timeout_ms(acmp_msg_type_t msg_type) {
  switch (msg_type) {
  case acmp_msg_type_connect_tx_command:
    return acmp_timeout_connect_tx;
  case acmp_msg_type_disconnect_tx_command:
    return acmp_timeout_disconnect_tx;
  case acmp_msg_type_get_tx_state_command:
    return acmp_timeout_get_tx_state;
  case acmp_msg_type_connect_rx_command:
    return acmp_timeout_connect_rx;
  case acmp_msg_type_disconnect_rx_command:
    return acmp_timeout_disconnect_rx;
  case acmp_msg_type_get_rx_state_command:
    return acmp_timeout_get_rx_state;
  case acmp_msg_type_get_tx_connection_command:
    return acmp_timeout_get_tx_connection;
  default:
    return 250;
  }
}
