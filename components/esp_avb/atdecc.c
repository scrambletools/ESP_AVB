/*
 * Copyright 2024 Scramble Tools
 * License: MIT
 *
 * ESP_AVB Component
 *
 * This component provides an implementation of an AVB talker and listener.
 * 
 * This file provides the required features of the ATDECC protocol.
 */

#include "avb.h"

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
  msg.header.status_valtime = 8; // valid time: 16 seconds
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
  if (index < 9999) {  
    index++;
  }
  else {
    index = 0;
  }
  int_to_octets(&index, state->own_entity.summary.available_index, 4);
  return ret;
}

/* Send AECP command controller available message */
int avb_send_aecp_cmd_controller_available(
    avb_state_s *state, 
    unique_id_t *target_id
) {
  // not implemented
  return OK;
}

/* Send AECP command get stream info message */
int avb_send_aecp_cmd_get_stream_info(
    avb_state_s *state, 
    unique_id_t *target_id
) {
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
  memcpy(msg.common.controller_entity_id, state->own_entity.summary.entity_id, UNIQUE_ID_LEN);
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
int avb_send_aecp_rsp_get_stream_info(
    avb_state_s *state, 
    aecp_get_stream_info_s *msg,
    eth_addr_t *dest_addr
) {
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

    // populate the response message
    memcpy(&response.stream.flags, &state->input_streams[index].stream_info_flags, sizeof(aem_stream_info_flags_s));
    memcpy(&response.stream.stream_format, &state->input_streams[index].stream_format, sizeof(avtp_stream_format_s));
    memcpy(&response.stream.stream_id, &state->input_streams[index].stream_id, UNIQUE_ID_LEN);
    memcpy(&response.stream.msrp_accumulated_latency, &state->input_streams[index].msrp_accumulated_latency, 4);
    memcpy(&response.stream.dest_addr, &state->input_streams[index].stream_dest_addr, sizeof(eth_addr_t));
    memcpy(&response.stream.msrp_failure_code, &state->input_streams[index].msrp_failure_code, 2);
    memcpy(&response.vlan_id, &state->input_streams[index].vlan_id, 2);
    if (is_output) {
        memcpy(&response.stream.flags, &state->output_streams[index].stream_info_flags, sizeof(aem_stream_info_flags_s));
        memcpy(&response.stream.stream_format, &state->output_streams[index].stream_format, sizeof(avtp_stream_format_s));
        memcpy(&response.stream.stream_id, &state->output_streams[index].stream_id, UNIQUE_ID_LEN);
        memcpy(&response.stream.msrp_accumulated_latency, &state->output_streams[index].msrp_accumulated_latency, 4);
        memcpy(&response.stream.dest_addr, &state->output_streams[index].stream_dest_addr, sizeof(eth_addr_t));
        memcpy(&response.stream.msrp_failure_code, &state->output_streams[index].msrp_failure_code, 2);
        memcpy(&response.vlan_id, &state->output_streams[index].vlan_id, 2);
    }

    // calc control data length
    uint16_t control_data_len = sizeof(aecp_get_stream_info_rsp_s) - AVTP_CDL_PREAMBLE_LEN;

    // set the control data length
    msg->common.header.control_data_len_h = (control_data_len >> 8) & 0xFF;
    msg->common.header.control_data_len = control_data_len & 0xFF;
    
    // send the response message
    uint16_t msg_len = sizeof(aecp_get_stream_info_rsp_s);
    ret = avb_net_send_to(state, ethertype_avtp, msg, msg_len, &ts, dest_addr);
    if (ret < 0) {
        avberr("send AECP get stream info response failed: %d", errno);
    }
    return ret;
}

/* Send AECP unsolicited notification get stream info response */
// sends to all regitered notification recipients
int avb_send_aecp_unsol_get_stream_info(
    avb_state_s *state,
    uint16_t index,
    bool is_output 
) {
  // not implemented
  return OK;
}

/* Send AECP command get stream info message */
int avb_send_aecp_cmd_get_counters(
    avb_state_s *state, 
    unique_id_t *target_id
) {
  // not implemented
  return OK;
}

/* Send AECP response get counters message */
// may be sent as an unsolicited notification
int avb_send_aecp_rsp_get_counters(
    avb_state_s *state,
    aecp_get_counters_s *msg,
    eth_addr_t *dest_addr
) {
  // not implemented
  return OK;
}

/* Send AECP unsolicited notification get stream info response */
// sends to all regitered notification recipients
int avb_send_aecp_unsol_get_counters(
    avb_state_s *state,
    aem_desc_type_t descriptor_type,
    uint16_t index
) {
  // not implemented
  return OK;
}

/* Send AECP response get descriptor for entity message */
int avb_send_aecp_rsp_read_descr_entity(
    avb_state_s *state,
    aecp_read_descriptor_rsp_s *msg,
    eth_addr_t *dest_addr
) {
    int ret = OK;
    struct timespec ts;
    uint16_t control_data_len = sizeof(aecp_read_descriptor_rsp_s) 
        - sizeof(atdecc_header_s) - AEM_MAX_DESC_LEN;

    // create a descriptor
    aem_entity_desc_s descriptor;
    memcpy(&descriptor, &state->own_entity, sizeof(aem_entity_desc_s));
    descriptor.detail.vendor_name_ref[1] = 12; // strings desc 1, string_4
    descriptor.detail.model_name_ref[1] = 13; // strings desc 1, string_5

    // insert the descriptor data
    memcpy(msg->descriptor_data, &state->own_entity, sizeof(aem_entity_desc_s));
    control_data_len += sizeof(aem_entity_desc_s) + 4;  // 4 bytes for type and index

    // set the control data length
    msg->common.header.control_data_len_h = (control_data_len >> 8) & 0xFF;
    msg->common.header.control_data_len = control_data_len & 0xFF;
    
    // send the response message
    uint16_t msg_len = sizeof(atdecc_header_s) + sizeof(unique_id_t) + control_data_len;
    ret = avb_net_send_to(state, ethertype_avtp, msg, msg_len, &ts, dest_addr);
    if (ret < 0) {
        avberr("send AECP Read Descriptor response failed: %d", errno);
    }
    return ret;
}

/* Send AECP response get descriptor for configuration message */
int avb_send_aecp_rsp_read_descr_configuration(
    avb_state_s *state,
    aecp_read_descriptor_rsp_s *msg,
    eth_addr_t *dest_addr
) {
    int ret = OK;
    struct timespec ts;
    uint16_t control_data_len = sizeof(aecp_read_descriptor_rsp_s) 
        - sizeof(atdecc_header_s) - AEM_MAX_DESC_LEN;

    // create a configuration descriptor
    aem_config_desc_s config_desc = {0};

    // give it a name
    // 3bits for base_strings offset, 3bits for index in strings desc
    uint16_t localized_description = 0;
    int_to_octets(&localized_description, config_desc.localized_description, 2);
    
    // These are the top level descriptors that will be listed in the descriptor counts
    // Eventually, the entire entity model should probably be stored in the state
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
        size_t count = AEM_MAX_DESC_COUNT;
        int_to_octets(&count, desc_count.count, 2);
        memcpy(&config_desc.descriptor_counts[i], &desc_count, sizeof(aem_config_desc_count_s));
    }
    
    // insert the descriptor data into the response message
    int_to_octets(&descriptor_counts_count, config_desc.descriptor_counts_count, 2);
    uint16_t offset = 74; // As defined in section 7.2.2
    int_to_octets(&offset, config_desc.descriptor_counts_offset, 2);
    memcpy(msg->descriptor_data, &config_desc, sizeof(aem_config_desc_s));
    control_data_len += sizeof(aem_config_desc_s) + 4  // 4 bytes for type and index
        - (4 * (AEM_MAX_NUM_DESC - descriptor_counts_count)); // adjust for the descriptor counts
    
    // set the control data length
    msg->common.header.control_data_len_h = (control_data_len >> 8) & 0xFF;
    msg->common.header.control_data_len = control_data_len & 0xFF;
    
    // send the response message
    uint16_t msg_len = sizeof(atdecc_header_s) + sizeof(unique_id_t) + control_data_len;
    ret = avb_net_send_to(state, ethertype_avtp, msg, msg_len, &ts, dest_addr);
    if (ret < 0) {
        avberr("send AECP Read Descriptor response failed: %d", errno);
    }
    return ret;
}

/* Send AECP response get descriptor for audio unit message */
int avb_send_aecp_rsp_read_descr_audio_unit(
    avb_state_s *state,
    aecp_read_descriptor_rsp_s *msg,
    eth_addr_t *dest_addr
) {
    int ret = OK;
    struct timespec ts;
    uint16_t control_data_len = sizeof(aecp_read_descriptor_rsp_s) 
        - sizeof(atdecc_header_s) - AEM_MAX_DESC_LEN;

    // data for the audio unit descriptor
    int localized_description = 1;
    int num_input_ports = state->num_input_streams;
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
        int_to_octets(&sampling_rates[i], (uint8_t *)&descriptor.sampling_rates[i], 4);
    }

    // insert the descriptor into the response message
    memcpy(msg->descriptor_data, &descriptor, sizeof(aem_audio_unit_desc_s));
    control_data_len += sizeof(aem_audio_unit_desc_s) + 4; // 4 bytes for type and index
    
    // set the control data length
    msg->common.header.control_data_len_h = (control_data_len >> 8) & 0xFF;
    msg->common.header.control_data_len = control_data_len & 0xFF;
    
    // send the response message
    uint16_t msg_len = sizeof(atdecc_header_s) + sizeof(unique_id_t) + control_data_len;
    ret = avb_net_send_to(state, ethertype_avtp, msg, msg_len, &ts, dest_addr);
    if (ret < 0) {
        avberr("send AECP Read Descriptor response failed: %d", errno);
    }
    return ret;
}

/* Send AECP response get descriptor for stream input or output message */
// TODO: change to use the input_streams and output_streams arrays from the state
// also move the supported formats to the state
int avb_send_aecp_rsp_read_descr_stream(
    avb_state_s *state,
    aecp_read_descriptor_rsp_s *msg,
    eth_addr_t *dest_addr,
    bool is_output
) {
    int ret = OK;
    struct timespec ts;
    uint16_t index = octets_to_uint(msg->descriptor_index, 2);

    // data for the stream input descriptor
    uint16_t localized_description = 4;  // see below for strings
    avtp_stream_format_s current_format = state->input_streams[index].stream_format;
    if (is_output) {
        localized_description = 5;
        current_format = state->output_streams[index].stream_format;
    }
    aem_stream_flags_s stream_flags = {0};
    stream_flags.class_a = true;
    stream_flags.class_b = true;
    
    // print the current format
    // char current_format_str[200];
    // octets_to_binary_string((uint8_t *)&current_format, sizeof(current_format), current_format_str);
    // avbinfo("current format: %s", current_format_str);
    
    int formats_offset = 138;  // 138 for 2021 version of AEM
    int number_of_formats = 6;
    avtp_stream_format_s formats[number_of_formats];
    avtp_stream_format_am824_s f0 = AVB_DEFAULT_FORMAT(cip_sfc_sample_rate_44_1k);
    avtp_stream_format_am824_s f1 = AVB_DEFAULT_FORMAT(cip_sfc_sample_rate_48k);
    avtp_stream_format_am824_s f2 = AVB_DEFAULT_FORMAT(cip_sfc_sample_rate_96k);
    avtp_stream_format_aaf_pcm_s f3 = AVB_DEFAULT_FORMAT_AAF(16, aaf_pcm_sample_rate_44_1k);
    avtp_stream_format_aaf_pcm_s f4 = AVB_DEFAULT_FORMAT_AAF(24, aaf_pcm_sample_rate_48k);
    avtp_stream_format_aaf_pcm_s f5 = AVB_DEFAULT_FORMAT_AAF(24, aaf_pcm_sample_rate_96k);
    formats[0].am824 = f0;
    formats[1].am824 = f1;
    formats[2].am824 = f2;
    formats[3].aaf_pcm = f3;
    formats[4].aaf_pcm = f4;
    formats[5].aaf_pcm = f5;
    int buffer_length = 8; // 8ns ingress buffer
    int redundant_offset = formats_offset + 8 * number_of_formats;

    // create a stream input descriptor
    aem_stream_desc_s descriptor;
    memset(&descriptor, 0, sizeof(aem_stream_desc_s));

    // populate the stream input descriptor
    int_to_octets(&localized_description, descriptor.localized_description, 2);
    int_to_octets(&stream_flags, (uint8_t *)&descriptor.stream_flags, 2);
    memcpy(&descriptor.current_format, &current_format, sizeof(avtp_stream_format_s));
    int_to_octets(&formats_offset, descriptor.formats_offset, 2);
    int_to_octets(&number_of_formats, descriptor.number_of_formats, 2);
    int_to_octets(&buffer_length, descriptor.buffer_length, 4);
    int_to_octets(&redundant_offset, descriptor.redundant_offset, 2); 
    memcpy(&descriptor.formats, formats, sizeof(avtp_stream_format_s) * number_of_formats);

    // insert the descriptor into the response message
    memcpy(msg->descriptor_data, &descriptor, sizeof(aem_stream_desc_s));
    
    // calc control data length
    uint16_t control_data_len = AECP_DESC_PREAMBLE_LEN - AVTP_CDL_PREAMBLE_LEN // the part before the descriptor
        + sizeof(aem_stream_desc_s) + 4  // add the stream descriptor size including the type and index
        - (AEM_MAX_NUM_FORMATS - number_of_formats) * sizeof(avtp_stream_format_s) // resize for actual number of formats
        + 8; // for funzies

    // set the control data length
    msg->common.header.control_data_len_h = (control_data_len >> 8) & 0xFF;
    msg->common.header.control_data_len = control_data_len & 0xFF;
    
    // send the response message
    uint16_t msg_len = sizeof(atdecc_header_s) + sizeof(unique_id_t) + control_data_len;
    ret = avb_net_send_to(state, ethertype_avtp, msg, msg_len, &ts, dest_addr);
    if (ret < 0) {
        avberr("send AECP Read Descriptor response failed: %d", errno);
    }
    return ret;
}

/* Send AECP response get descriptor for avb interface message */
int avb_send_aecp_rsp_read_descr_avb_interface(
    avb_state_s *state,
    aecp_read_descriptor_rsp_s *msg,
    eth_addr_t *dest_addr
) {
    int ret = OK;
    struct timespec ts;

    // data for the stream input descriptor
    uint16_t localized_description = 6;  // see below for strings
    eth_addr_t mac_addr;
    memcpy(&mac_addr, state->internal_mac_addr, sizeof(eth_addr_t));
    aem_avb_interface_flags_s flags;
    memset(&flags, 0, sizeof(aem_avb_interface_flags_s));

    flags.gptp_gm_supported = true;
    flags.gptp_supported = true;
    flags.srp_supported = true;
    unique_id_t clock_id;
    memcpy(&clock_id, state->ptp_status.clock_source_info.gm_id, sizeof(unique_id_t));
    uint8_t priority1 = state->ptp_status.clock_source_info.priority1;
    uint8_t clock_class = state->ptp_status.clock_source_info.clockclass;
    uint16_t offset_scaled_log_variance = state->ptp_status.clock_source_info.variance; // ?
    uint8_t clock_accuracy = state->ptp_status.clock_source_info.accuracy;
    uint8_t priority2 = state->ptp_status.clock_source_info.priority2;
    uint8_t log_sync_interval = 0xfd; // ?
    uint8_t log_announce_interval = 0xfd; // ?
    uint8_t log_pdelay_interval = 0xfd; // ?

    // create an avb interface descriptor
    aem_avb_interface_desc_s descriptor;
    memset(&descriptor, 0, sizeof(aem_avb_interface_desc_s));

    // populate the descriptor (only non-zero fields)
    int_to_octets(&localized_description, descriptor.localized_description, 2);
    memcpy(&descriptor.mac_address, &mac_addr, sizeof(eth_addr_t));
    memcpy(&descriptor.interface_flags, &flags, sizeof(aem_avb_interface_flags_s));
    memcpy(&descriptor.clock_identity, &clock_id, sizeof(unique_id_t));
    descriptor.priority1 = priority1;
    descriptor.clock_class = clock_class;
    int_to_octets(&offset_scaled_log_variance, descriptor.offset_scaled_log_variance, 2);
    descriptor.clock_accuracy = clock_accuracy;
    descriptor.priority2 = priority2;
    descriptor.log_sync_interval = log_sync_interval;
    descriptor.log_announce_interval = log_announce_interval;
    descriptor.log_pdelay_interval = log_pdelay_interval;

    // insert the descriptor into the response message
    memcpy(msg->descriptor_data, &descriptor, sizeof(aem_avb_interface_desc_s));
    
    // calc control data length
    uint16_t control_data_len = AECP_DESC_PREAMBLE_LEN - AVTP_CDL_PREAMBLE_LEN // the part before the descriptor
        + sizeof(aem_avb_interface_desc_s) + 4;  // add the stream descriptor size including the type and index

    // set the control data length
    msg->common.header.control_data_len_h = (control_data_len >> 8) & 0xFF;
    msg->common.header.control_data_len = control_data_len & 0xFF;
    
    // send the response message
    uint16_t msg_len = sizeof(atdecc_header_s) + sizeof(unique_id_t) + control_data_len;
    ret = avb_net_send_to(state, ethertype_avtp, msg, msg_len, &ts, dest_addr);
    if (ret < 0) {
        avberr("send AECP Read Descriptor response failed: %d", errno);
    }
    return ret;
}

/* Send AECP response get descriptor for clock source message */
int avb_send_aecp_rsp_read_descr_clock_source(
    avb_state_s *state,
    aecp_read_descriptor_rsp_s *msg,
    eth_addr_t *dest_addr
) {
    int ret = OK;
    struct timespec ts;

    // data for the stream input descriptor
    int localized_description = 11; // strings desc 1, string_3
    uint16_t location_type = aem_desc_type_audio_unit;

    // create a stream input descriptor
    aem_clock_source_desc_s descriptor;
    memset(&descriptor, 0, sizeof(aem_clock_source_desc_s));

    // populate the descriptor (only non-zero fields)
    int_to_octets(&localized_description, descriptor.localized_description, 2);
    memcpy(&descriptor.clock_source_id, state->ptp_status.clock_source_info.gm_id, sizeof(unique_id_t));
    int_to_octets(&location_type, &descriptor.clock_source_location_type, 2);

    // insert the descriptor into the response message
    memcpy(msg->descriptor_data, &descriptor, sizeof(aem_clock_source_desc_s));
    
    // calc control data length
    uint16_t control_data_len = AECP_DESC_PREAMBLE_LEN - AVTP_CDL_PREAMBLE_LEN // the part before the descriptor
        + sizeof(aem_clock_source_desc_s) + 4;  // add the stream descriptor size including the type and index

    // set the control data length
    msg->common.header.control_data_len_h = (control_data_len >> 8) & 0xFF;
    msg->common.header.control_data_len = control_data_len & 0xFF;
    
    // send the response message
    uint16_t msg_len = sizeof(atdecc_header_s) + sizeof(unique_id_t) + control_data_len;
    ret = avb_net_send_to(state, ethertype_avtp, msg, msg_len, &ts, dest_addr);
    if (ret < 0) {
        avberr("send AECP Read Descriptor response failed: %d", errno);
    }
    return ret;
}

/* Send AECP response get descriptor for memory object message */
int avb_send_aecp_rsp_read_descr_memory_obj(
    avb_state_s *state,
    aecp_read_descriptor_rsp_s *msg,
    eth_addr_t *dest_addr
) {
    int ret = OK;
    struct timespec ts;

    // data for the stream input descriptor
    int localized_description = 8;  // strings desc 1, string_0
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

    // insert the descriptor into the response message
    memcpy(msg->descriptor_data, &descriptor, sizeof(aem_memory_object_desc_s));
    
    // calc control data length
    uint16_t control_data_len = AECP_DESC_PREAMBLE_LEN - AVTP_CDL_PREAMBLE_LEN // the part before the descriptor
        + sizeof(aem_memory_object_desc_s) + 4;  // add the stream descriptor size including the type and index

    // set the control data length
    msg->common.header.control_data_len_h = (control_data_len >> 8) & 0xFF;
    msg->common.header.control_data_len = control_data_len & 0xFF;
    
    // send the response message
    uint16_t msg_len = sizeof(atdecc_header_s) + sizeof(unique_id_t) + control_data_len;
    ret = avb_net_send_to(state, ethertype_avtp, msg, msg_len, &ts, dest_addr);
    if (ret < 0) {
        avberr("send AECP Read Descriptor response failed: %d", errno);
    }
    return ret;
}

/* Send AECP response get descriptor for locale message */
int avb_send_aecp_rsp_read_descr_locale(
    avb_state_s *state,
    aecp_read_descriptor_rsp_s *msg,
    eth_addr_t *dest_addr
) {
    int ret = OK;
    struct timespec ts;

    // data for the stream input descriptor
    size_t locale_identifier_len = 2;
    char *locale_identifier = "en";
    uint16_t number_of_strings = 2;

    // create a stream input descriptor
    aem_locale_desc_s descriptor;
    memset(&descriptor, 0, sizeof(aem_locale_desc_s));

    // populate the descriptor (only non-zero fields)
    memcpy(&descriptor.locale_identifier, locale_identifier, locale_identifier_len);
    int_to_octets(&number_of_strings, &descriptor.number_of_strings, 2);

    // insert the descriptor into the response message
    memcpy(msg->descriptor_data, &descriptor, sizeof(aem_locale_desc_s));

    // calc control data length
    uint16_t control_data_len = AECP_DESC_PREAMBLE_LEN - AVTP_CDL_PREAMBLE_LEN // the part before the descriptor
        + sizeof(aem_locale_desc_s) + 4;  // add the stream descriptor size including the type and index

    // set the control data length
    msg->common.header.control_data_len_h = (control_data_len >> 8) & 0xFF;
    msg->common.header.control_data_len = control_data_len & 0xFF;
    
    // send the response message
    uint16_t msg_len = sizeof(atdecc_header_s) + sizeof(unique_id_t) + control_data_len;
    ret = avb_net_send_to(state, ethertype_avtp, msg, msg_len, &ts, dest_addr);
    if (ret < 0) {
        avberr("send AECP Read Descriptor response failed: %d", errno);
    }
    return ret;
}

/* Send AECP response get descriptor for strings message */
int avb_send_aecp_rsp_read_descr_strings(
    avb_state_s *state,
    aecp_read_descriptor_rsp_s *msg,
    eth_addr_t *dest_addr
) {
    int ret = OK;
    struct timespec ts;

    // data for the stream input descriptor
    char *string_0 = "";  // Config Name (addr 0)
    if (state->config.talker && state->config.listener) {
        string_0 = "Talker & Listener";
    } else if (state->config.talker) {
        string_0 = "Talker";
    } else if (state->config.listener) {
        string_0 = "Listener";
    }
    char *string_1 = "ESP-AVB";          // Audio Unit Name (addr 1)
    char *string_2 = "Mono Audio In";    // Stream Port Input Cluster Name (addr 2)
    char *string_3 = "Mono Audio Out";   // Stream Port Output Cluster Name (addr 3)
    char *string_4 = "Audio Stream In";  // Stream Input Name (addr 4)
    char *string_5 = "Audio Stream Out"; // Stream Output Name (addr 5)
    char *string_6 = "Ethernet";         // AVB Interface Name (addr 6)
    if (msg->descriptor_index[1] == 1) {
        string_0 = "Logo";           // Memory Object Name (addr 8)
        string_1 = "Indentify";      // Control Name (addr 9)
        string_2 = "Clock Domain";   // Clock Domain Name (addr 10)
        string_3 = "Internal Clock"; // Clock Source Name (addr 11)
        string_4 = "Vendor Name";    // Vendor Name (addr 12)
        string_5 = "AVB Endpoint";   // Model Name (addr 13)
        string_6 = "";               // (addr 14)
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
    uint16_t control_data_len = AECP_DESC_PREAMBLE_LEN - AVTP_CDL_PREAMBLE_LEN // the part before the descriptor
        + sizeof(aem_strings_desc_s) + 4;  // add the stream descriptor size including the type and index

    // set the control data length
    msg->common.header.control_data_len_h = (control_data_len >> 8) & 0xFF;
    msg->common.header.control_data_len = control_data_len & 0xFF;
    
    // send the response message
    uint16_t msg_len = sizeof(atdecc_header_s) + sizeof(unique_id_t) + control_data_len;
    ret = avb_net_send_to(state, ethertype_avtp, msg, msg_len, &ts, dest_addr);
    if (ret < 0) {
        avberr("send AECP Read Descriptor response failed: %d", errno);
    }
    return ret;
}

/* Send AECP response get descriptor for stream port input message */
int avb_send_aecp_rsp_read_descr_stream_port(
    avb_state_s *state,
    aecp_read_descriptor_rsp_s *msg,
    eth_addr_t *dest_addr,
    bool is_output
) {
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
    uint16_t control_data_len = AECP_DESC_PREAMBLE_LEN - AVTP_CDL_PREAMBLE_LEN // the part before the descriptor
        + sizeof(aem_stream_port_desc_s) + 4;  // add the stream descriptor size including the type and index

    // set the control data length
    msg->common.header.control_data_len_h = (control_data_len >> 8) & 0xFF;
    msg->common.header.control_data_len = control_data_len & 0xFF;
    
    // send the response message
    uint16_t msg_len = sizeof(atdecc_header_s) + sizeof(unique_id_t) + control_data_len;
    ret = avb_net_send_to(state, ethertype_avtp, msg, msg_len, &ts, dest_addr);
    if (ret < 0) {
        avberr("send AECP Read Descriptor response failed: %d", errno);
    }
    return ret;
}

/* Send AECP response get descriptor for audio cluster message */
int avb_send_aecp_rsp_read_descr_audio_cluster(
    avb_state_s *state,
    aecp_read_descriptor_rsp_s *msg,
    eth_addr_t *dest_addr
) {
    int ret = OK;
    struct timespec ts;

    // data for the stream input descriptor
    int localized_description = 2; // strings desc 0, string_1
    aem_desc_type_t signal_type = aem_desc_type_invalid;
    uint16_t num_channels = state->config.num_channels_input;
    if (msg->descriptor_index[1] == 1) { // index 1 used for output port
        localized_description = 3; // strings desc 0, string_2
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
    uint16_t control_data_len = AECP_DESC_PREAMBLE_LEN - AVTP_CDL_PREAMBLE_LEN // the part before the descriptor
        + sizeof(aem_audio_cluster_desc_s) + 4;  // add the stream descriptor size including the type and index

    // set the control data length
    msg->common.header.control_data_len_h = (control_data_len >> 8) & 0xFF;
    msg->common.header.control_data_len = control_data_len & 0xFF;
    
    // send the response message
    uint16_t msg_len = sizeof(atdecc_header_s) + sizeof(unique_id_t) + control_data_len;
    ret = avb_net_send_to(state, ethertype_avtp, msg, msg_len, &ts, dest_addr);
    if (ret < 0) {
        avberr("send AECP Read Descriptor response failed: %d", errno);
    }
    return ret;
}

/* Send AECP response get descriptor for audio map message */
int avb_send_aecp_rsp_read_descr_audio_map(
    avb_state_s *state,
    aecp_read_descriptor_rsp_s *msg,
    eth_addr_t *dest_addr
) {
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
    memcpy(descriptor.mappings, mappings, num_mappings * sizeof(aem_audio_mapping_s));

    // insert the descriptor into the response message
    memcpy(msg->descriptor_data, &descriptor, sizeof(aem_audio_map_desc_s));

    // calc control data length
    uint16_t control_data_len = AECP_DESC_PREAMBLE_LEN - AVTP_CDL_PREAMBLE_LEN // the part before the descriptor
        + sizeof(aem_audio_map_desc_s) + 4;  // add the stream descriptor size including the type and index

    // set the control data length
    msg->common.header.control_data_len_h = (control_data_len >> 8) & 0xFF;
    msg->common.header.control_data_len = control_data_len & 0xFF;
    
    // send the response message
    uint16_t msg_len = sizeof(atdecc_header_s) + sizeof(unique_id_t) + control_data_len;
    ret = avb_net_send_to(state, ethertype_avtp, msg, msg_len, &ts, dest_addr);
    if (ret < 0) {
        avberr("send AECP Read Descriptor response failed: %d", errno);
    }
    return ret;
}

/* Send AECP response get descriptor for control message
 * currently only supporting identify control
 */
int avb_send_aecp_rsp_read_descr_control(
    avb_state_s *state,
    aecp_read_descriptor_rsp_s *msg,
    eth_addr_t *dest_addr
) {
    int ret = OK;
    struct timespec ts;

    // data for the stream input descriptor
    int localized_description = 9;  // strings desc 1, string_1
    uint16_t control_value_type = 1; // CONTROL_LINEAR_UINT8
    uint8_t control_type[8] = {0x90, 0xe0, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x01}; // INDENTIFY
    uint16_t values_offset = 104; // Required by spec
    uint16_t num_values = 1;
    uint16_t signal_type = aem_desc_type_invalid; // spec requires this to be invalid
    // milan compliant identify control values
    aem_identify_control_value_s control_values = {
        .values = {0, 255, 255, 0, 0},
        .units = {0},
        .string_ref = {0xff, 0xff}, // not set
    };

    // create a stream input descriptor
    aem_control_desc_s descriptor;
    memset(&descriptor, 0, sizeof(aem_control_desc_s));

    // populate the descriptor (only non-zero fields)
    int_to_octets(&localized_description, descriptor.localized_description, 2);
    int_to_octets(&control_value_type, descriptor.control_value_type, 2);
    memcpy(descriptor.control_type, control_type, 8);
    int_to_octets(&values_offset, &descriptor.values_offset, 2);
    int_to_octets(&num_values, &descriptor.number_of_values, 2);
    int_to_octets(&signal_type, &descriptor.signal_type, 2);
    memcpy(descriptor.value_details, &control_values, sizeof(aem_identify_control_value_s));

    // insert the descriptor into the response message
    memcpy(msg->descriptor_data, &descriptor, sizeof(aem_control_desc_s));
    
    // calc control data length
    uint16_t control_data_len = AECP_DESC_PREAMBLE_LEN - AVTP_CDL_PREAMBLE_LEN // the part before the descriptor
        + sizeof(aem_control_desc_s) + 4;  // add the stream descriptor size including the type and index

    // set the control data length
    msg->common.header.control_data_len_h = (control_data_len >> 8) & 0xFF;
    msg->common.header.control_data_len = control_data_len & 0xFF;
    
    // send the response message
    uint16_t msg_len = sizeof(atdecc_header_s) + sizeof(unique_id_t) + control_data_len;
    ret = avb_net_send_to(state, ethertype_avtp, msg, msg_len, &ts, dest_addr);
    if (ret < 0) {
        avberr("send AECP Read Descriptor response failed: %d", errno);
    }
    return ret;
}

/* Send AECP response get descriptor for clock domain message */
int avb_send_aecp_rsp_read_descr_clock_domain(
    avb_state_s *state,
    aecp_read_descriptor_rsp_s *msg,
    eth_addr_t *dest_addr
) {
    int ret = OK;
    struct timespec ts;

    // data for the stream input descriptor
    int localized_description = 10;  // strings desc 1, string_2
    uint16_t clock_sources_offset = 76; // Required by spec
    uint16_t clock_sources_count = 1;
    aem_clock_source_t clock_sources[1] = {0}; // only one source with id 0

    // create a stream input descriptor
    aem_clock_domain_desc_s descriptor;
    memset(&descriptor, 0, sizeof(aem_clock_domain_desc_s));

    // populate the descriptor (only non-zero fields)
    int_to_octets(&localized_description, descriptor.localized_description, 2);
    int_to_octets(&clock_sources_offset, &descriptor.clock_sources_offset, 2);
    int_to_octets(&clock_sources_count, &descriptor.clock_sources_count, 2);
    memcpy(descriptor.clock_sources, clock_sources, sizeof(aem_clock_source_t));

    // insert the descriptor into the response message
    memcpy(msg->descriptor_data, &descriptor, sizeof(aem_clock_domain_desc_s));
    
    // calc control data length
    uint16_t control_data_len = AECP_DESC_PREAMBLE_LEN - AVTP_CDL_PREAMBLE_LEN // the part before the descriptor
        + sizeof(aem_clock_domain_desc_s) + 4;  // add the stream descriptor size including the type and index

    // set the control data length
    msg->common.header.control_data_len_h = (control_data_len >> 8) & 0xFF;
    msg->common.header.control_data_len = control_data_len & 0xFF;
    
    // send the response message
    uint16_t msg_len = sizeof(atdecc_header_s) + sizeof(unique_id_t) + control_data_len;
    ret = avb_net_send_to(state, ethertype_avtp, msg, msg_len, &ts, dest_addr);
    if (ret < 0) {
        avberr("send AECP Read Descriptor response failed: %d", errno);
    }
    return ret;
}

/* Process received ATDECC ADP message */
int avb_process_adp(avb_state_s *state, adp_message_s *msg, eth_addr_t *src_addr) {
  
  /* Process ADP Entity Available message */
  if (msg->header.msg_type == adp_msg_type_entity_available) {
  
    // If the entity is an audio talker, then remember it
    if (msg->entity.talker_capabilities.implemented && msg->entity.talker_capabilities.audio_source) {
      
      // if the talker is not already known, add it to the list
      int index = avb_find_entity_by_addr(state, src_addr, avb_entity_type_talker);
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
          memmove(&state->talkers[0], &state->talkers[1], (state->num_talkers - 1) * sizeof(avb_talker_s));
          state->talkers[state->num_talkers - 1] = talker;
        }
      }
      // if it is already known, then update the entity id and model id if missing or changed
      else {
        if (memcmp(state->talkers[index].entity_id, msg->entity.entity_id, UNIQUE_ID_LEN) != 0) {
          memcpy(&state->talkers[index].entity_id, msg->entity.entity_id, UNIQUE_ID_LEN);
        }
        if (memcmp(state->talkers[index].model_id, msg->entity.model_id, UNIQUE_ID_LEN) != 0) {
          memcpy(&state->talkers[index].model_id, msg->entity.model_id, UNIQUE_ID_LEN);
        }
      }
    }
    // If the entity is an audio listener, then remember it
    if (msg->entity.listener_capabilities.implemented && msg->entity.listener_capabilities.audio_sink) {
      
      // if the listener is not already known, add it to the list
      int index = avb_find_entity_by_addr(state, src_addr, avb_entity_type_listener);
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
          memmove(&state->listeners[0], &state->listeners[1], (state->num_listeners - 1) * sizeof(avb_listener_s));
          state->listeners[state->num_listeners - 1] = listener;
        }
      }
      // if it is already known, then update the entity id and model id if missing or changed
      else {
        if (memcmp(state->listeners[index].entity_id, msg->entity.entity_id, UNIQUE_ID_LEN) != 0) {
          memcpy(&state->listeners[index].entity_id, msg->entity.entity_id, UNIQUE_ID_LEN);
        }
        if (memcmp(state->listeners[index].model_id, msg->entity.model_id, UNIQUE_ID_LEN) != 0) {
          memcpy(&state->listeners[index].model_id, msg->entity.model_id, UNIQUE_ID_LEN);
        }
      }
    }
    // If the entity is a controller, then remember it
    if (msg->entity.controller_capabilities.implemented) {
      
      // if the controller is not already known, add it to the list
      int index = avb_find_entity_by_addr(state, src_addr, avb_entity_type_controller);
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
          memmove(&state->controllers[0], &state->controllers[1], (state->num_controllers - 1) * sizeof(avb_controller_s));
          state->controllers[state->num_controllers - 1] = controller;
        }
      }
      // if it is already known, then update the entity id and model id if missing or changed
      else {
        if (memcmp(state->controllers[index].entity_id, msg->entity.entity_id, UNIQUE_ID_LEN) != 0) {
          memcpy(&state->controllers[index].entity_id, msg->entity.entity_id, UNIQUE_ID_LEN);
        }
        if (memcmp(state->controllers[index].model_id, msg->entity.model_id, UNIQUE_ID_LEN) != 0) {
          memcpy(&state->controllers[index].model_id, msg->entity.model_id, UNIQUE_ID_LEN);
        }
      }
    } 
  }
  return OK;
}

/* Process received ATDECC AECP message */
int avb_process_aecp(avb_state_s *state, aecp_message_u *msg, eth_addr_t *src_addr) {
  
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
      case aecp_cmd_code_get_stream_info:
        return avb_process_aecp_cmd_get_stream_info(state, msg, src_addr);
        break;
      case aecp_cmd_code_get_counters:
        return avb_process_aecp_cmd_get_counters(state, msg, src_addr);
        break;
      default:
        return OK;
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

/* Process AECP command register unsolicited notification */
int avb_process_aecp_cmd_register_unsol_notif(avb_state_s *state, 
                                        aecp_message_u *msg, 
                                        eth_addr_t *src_addr) {
  int ret = OK;
  struct timespec ts;

// create a response, copy the cmd message data and change the msg type and status
  aecp_register_unsol_notif_s response;
  memcpy(&response, msg, sizeof(aecp_register_unsol_notif_s));
  response.common.header.msg_type = aecp_msg_type_aem_response;

// send the response message  
  uint16_t msg_len = sizeof(atdecc_header_s) + sizeof(aecp_acquire_entity_s);
  ret = avb_net_send_to(state, ethertype_avtp, &response, msg_len, &ts, src_addr);
  if (ret < 0) {
    avberr("send AECP register unsolicited notification response failed: %d", errno);
  } else {
    state->unsol_notif_enabled = true;
  }
  return ret;
}

/* Process AECP command deregister unsolicited notification */
int avb_process_aecp_cmd_deregister_unsol_notif(
    avb_state_s *state, 
    aecp_message_u *msg, 
    eth_addr_t *src_addr
) {
  int ret = OK;
  struct timespec ts;

// create a response, copy the cmd message data and change the msg type and status
  aecp_register_unsol_notif_s response;
  memcpy(&response, msg, sizeof(aecp_register_unsol_notif_s));
  response.common.header.msg_type = aecp_msg_type_aem_response;

// send the response message  
  uint16_t msg_len = sizeof(atdecc_header_s) + sizeof(aecp_acquire_entity_s);
  ret = avb_net_send_to(state, ethertype_avtp, &response, msg_len, &ts, src_addr);
  if (ret < 0) {
    avberr("send AECP deregister unsolicited notification response failed: %d", errno);
  } else {
    state->unsol_notif_enabled = false;
  }
  return ret;
}

/* Process AECP command acquire entity */
int avb_process_aecp_cmd_acquire_entity(
    avb_state_s *state, 
    aecp_message_u *msg, 
    eth_addr_t *src_addr
) {
  int ret;
  struct timespec ts;

  // check if the target entity id is the own entity id
  if (memcmp(msg->common.target_entity_id, state->own_entity.summary.entity_id, UNIQUE_ID_LEN) != 0) {
    avbinfo("Ignoring AECP Acquire Entity for different entity");
    return OK;
  }

  // create a response, copy the cmd message data, change the msg type and set owner id
  aecp_acquire_entity_s response = {0};
  memcpy(&response, msg, sizeof(aecp_acquire_entity_s));
  response.common.header.msg_type = aecp_msg_type_aem_response;
  if (!msg->acquire_entity.release) {
    memcpy(response.owner_id, msg->common.controller_entity_id, UNIQUE_ID_LEN);
  }

  // send the response message  
  uint16_t msg_len = sizeof(atdecc_header_s) + sizeof(aecp_acquire_entity_s);
  ret = avb_net_send_to(state, ethertype_avtp, &response, msg_len, &ts, src_addr);

  if (ret < 0) {
    avberr("send AECP Acquire Entity response failed: %d", errno);
  } else {
    if (msg->acquire_entity.release) {
        state->acquired = false;
    } else {
        state->acquired = true;
        memcpy(state->acquired_by, msg->common.controller_entity_id, UNIQUE_ID_LEN);
        state->last_acquired.tv_sec = ts.tv_sec;
        state->last_acquired.tv_usec = (suseconds_t)(ts.tv_nsec / 1000);
    }
  }
  return ret;
}

/* Process AECP command lock entity */
int avb_process_aecp_cmd_lock_entity(
    avb_state_s *state, 
    aecp_message_u *msg, 
    eth_addr_t *src_addr
) {
  int ret;
  struct timespec ts;

  // check if the target entity id is the own entity id
  if (memcmp(msg->common.target_entity_id, state->own_entity.summary.entity_id, UNIQUE_ID_LEN) != 0) {
    avbinfo("Ignoring AECP Lock Entity for different entity");
    return OK;
  }

  // create a response, copy the cmd message data, change the msg type and set locked id
  aecp_lock_entity_s response = {0};
  memcpy(&response, msg, sizeof(aecp_lock_entity_s));
  response.common.header.msg_type = aecp_msg_type_aem_response;
  if (!msg->lock_entity.unlock) {
    memcpy(response.locked_id, msg->common.controller_entity_id, UNIQUE_ID_LEN);
  }

  // send the response message  
  uint16_t msg_len = sizeof(atdecc_header_s) + sizeof(aecp_lock_entity_s);
  ret = avb_net_send_to(state, ethertype_avtp, &response, msg_len, &ts, src_addr);
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

  // check if the target entity id is the own entity id
  if (memcmp(msg->common.target_entity_id, state->own_entity.summary.entity_id, UNIQUE_ID_LEN) != 0) {
    avbinfo("Ignoring AECP Entity Available for different entity");
    return OK;
  }

  // create a response, copy the cmd message data and change the msg type
  aecp_entity_available_rsp_s response;
  memset(&response, 0, sizeof(aecp_entity_available_rsp_s));
  memcpy(&response.common, &msg->common, sizeof(aecp_common_s));
  memcpy(&response.aem, &msg->basic.aem, sizeof(aecp_common_aem_s));
  response.common.header.msg_type = aecp_msg_type_aem_response;

  // send the response message
  uint16_t msg_len = sizeof(atdecc_header_s) + sizeof(aecp_entity_available_rsp_s);
  ret = avb_net_send_to(state, ethertype_avtp, &response, msg_len, &ts, src_addr);
  if (ret < 0) {
    avberr("send AECP Entity Available response failed: %d", errno);
  }
  return ret;
}

/* Process AECP command get configuration */
int avb_process_aecp_cmd_get_configuration(
    avb_state_s *state, 
    aecp_message_u *msg, 
    eth_addr_t *src_addr
) {
  int ret;
  struct timespec ts;

  // check if the target entity id is the own entity id
  if (memcmp(msg->common.target_entity_id, state->own_entity.summary.entity_id, UNIQUE_ID_LEN) != 0) {
    avbinfo("Ignoring AECP Get Configuration for different entity");
    return OK;
  }

  // create a response, copy the cmd message data and change the msg type, add config index
  aecp_get_configuration_rsp_s response;
  memset(&response, 0, sizeof(aecp_get_configuration_rsp_s));
  memcpy(&response.common, &msg->common, sizeof(aecp_common_s));
  memcpy(&response.aem, &msg->basic.aem, sizeof(aecp_common_aem_s));
  response.common.header.msg_type = aecp_msg_type_aem_response;
  size_t config_index = DEFAULT_CONFIG_INDEX;
  int_to_octets(&config_index, response.configuration_index, 2);

  // send the response message  
  uint16_t msg_len = sizeof(atdecc_header_s) + sizeof(aecp_lock_entity_s);
  ret = avb_net_send_to(state, ethertype_avtp, &response, msg_len, &ts, src_addr);
  if (ret < 0) {
    avberr("send AECP Get Configuration response failed: %d", errno);
  }
  return ret;
}

/* Process AECP command read descriptor */
int avb_process_aecp_cmd_read_descriptor(
    avb_state_s *state, 
    aecp_message_u *msg, 
    eth_addr_t *src_addr
) {
  int ret = OK;

  // check if the target entity id is the own entity id
  if (memcmp(msg->common.target_entity_id, state->own_entity.summary.entity_id, UNIQUE_ID_LEN) != 0) {
    avbinfo("Ignoring AECP Read Descriptor for different entity");
    return OK;
  }

  // create a response, copy the cmd message data and change the msg type
  aecp_read_descriptor_rsp_s *response = NULL;
  response = (aecp_read_descriptor_rsp_s *)calloc(1, sizeof(aecp_read_descriptor_rsp_s));
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
      octets_to_hex_string(msg->read_descriptor.descriptor_type, 2, desc_type_str, '-');
      avbinfo("Ignoring AECP Read Descriptor for unsupported descriptor type %s", desc_type_str);
      ret = ERROR;
  }
  free(response);
  return ret;
}

/* Process AECP command get stream info */
int avb_process_aecp_cmd_get_stream_info(avb_state_s *state, 
                                          aecp_message_u *msg, 
                                          eth_addr_t *src_addr) {
  // not implemented
  return OK;
}

/* Process AECP command get counters */
// counters can be for entity, stream input, stream output, avb interface or clock domain
int avb_process_aecp_cmd_get_counters(avb_state_s *state, 
                                        aecp_message_u *msg, 
                                        eth_addr_t *src_addr) {
  int ret;
  struct timespec ts;
  uint16_t control_data_len = sizeof(aecp_get_counters_rsp_s) - sizeof(atdecc_header_s);

  // check if the target entity id is the own entity id
  if (memcmp(msg->common.target_entity_id, state->own_entity.summary.entity_id, UNIQUE_ID_LEN) != 0) {
    avbinfo("Ignoring AECP Get Counters for different entity");
    return OK;
  }

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
      // create stream input counters valid flags
      aem_stream_in_counters_val_s stream_in_counters_val;
      memset(&stream_in_counters_val, 0, sizeof(aem_stream_in_counters_val_s));
      // create stream input counters block
      aem_stream_in_counters_s stream_in_counters;
      memset(&stream_in_counters, 0, sizeof(aem_stream_in_counters_s));
      // TBD put in counters
      break;
    case aem_desc_type_stream_output:
      // create stream output counters valid flags
      aem_stream_out_counters_val_s stream_out_counters_val;
      memset(&stream_out_counters_val, 0, sizeof(aem_stream_out_counters_val_s));
      // create stream output counters block
      aem_stream_out_counters_s stream_out_counters;
      memset(&stream_out_counters, 0, sizeof(aem_stream_out_counters_s));
      // TBD put in counters
      break;
    case aem_desc_type_avb_interface:
      // create avb interface counters valid flags
      aem_avb_interface_counters_val_s avb_interface_counters_val;
      memset(&avb_interface_counters_val, 0, sizeof(aem_avb_interface_counters_val_s));
      // create avb interface counters block
      aem_avb_interface_counters_s avb_interface_counters;
      memset(&avb_interface_counters, 0, sizeof(aem_avb_interface_counters_s));
      // TBD put in counters
      break;
    case aem_desc_type_clock_domain:
      // create clock domain counters valid flags
      aem_clock_domain_counters_val_s clock_domain_counters_val;
      memset(&clock_domain_counters_val, 0, sizeof(aem_clock_domain_counters_val_s));
      // create clock domain counters block
      aem_clock_domain_counters_s clock_domain_counters;
      memset(&clock_domain_counters, 0, sizeof(aem_clock_domain_counters_s));
      // TBD put in counters
      break;
    default:
      char desc_type_str[7];
      octets_to_hex_string(msg->get_counters.descriptor_type, 2, desc_type_str, '-');
      avberr("Ignoring AECP Get Counters for unsupported descriptor type %s", desc_type_str);
      return ERROR;
  }

  // set the control data length
  response.common.header.control_data_len_h = (control_data_len >> 8) & 0xFF;
  response.common.header.control_data_len = control_data_len & 0xFF;

  // send the response message  
  uint16_t msg_len = sizeof(atdecc_header_s) + control_data_len;
  ret = avb_net_send_to(state, ethertype_avtp, &response, msg_len, &ts, src_addr);
  if (ret < 0) {
    avberr("send AECP Get Counters response failed: %d", errno);
  }
  return ret;
}

/* Process AECP response register unsol notif */
int avb_process_aecp_rsp_register_unsol_notif(avb_state_s *state, aecp_message_u *msg) {
  return OK;
}

/* Process AECP response deregister unsol notif */
int avb_process_aecp_rsp_deregister_unsol_notif(avb_state_s *state, aecp_message_u *msg) {
  return OK;
}

/* Process AECP response entity available */
int avb_process_aecp_rsp_entity_available(avb_state_s *state, aecp_message_u *msg) {
  return OK;
}

/* Process AECP response controller available */
int avb_process_aecp_rsp_controller_available(avb_state_s *state, aecp_message_u *msg) {
  return OK;
}

/* Process AECP response get stream info 
 * this may be sent as an unsolicited notification
*/
int avb_process_aecp_rsp_get_stream_info(avb_state_s *state, aecp_message_u *msg) {
  
  // find the talker and update the talker info
  int index = avb_find_entity_by_id(state, &msg->common.target_entity_id, avb_entity_type_talker);
  if (index == NOT_FOUND) {
    avberr("Ignoring AECP Get Stream Info response for unknown talker");
    return OK;
  }
  // update the talker stream info
  else {
    memcpy(&state->talkers[index].stream, &msg->get_stream_info_rsp.stream, sizeof(aem_stream_summary_s));
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
      if (memcmp(msg->listener_entity_id, state->own_entity.summary.entity_id, UNIQUE_ID_LEN) != 0) {
        avbinfo("Ignoring ACMP Connect RX Command for different listener");
        break;
      }
      avb_process_acmp_connect_rx_command(state, msg, false);
      break;
    case acmp_msg_type_disconnect_rx_command:
      if (memcmp(msg->listener_entity_id, state->own_entity.summary.entity_id, UNIQUE_ID_LEN) != 0) {
        avbinfo("Ignoring ACMP Disconnect RX Command for different listener");
        break;
      }
      avb_process_acmp_connect_rx_command(state, msg, true);
      break;
    case acmp_msg_type_connect_tx_command:
      if (memcmp(msg->talker_entity_id, state->own_entity.summary.entity_id, UNIQUE_ID_LEN) != 0) {
        avbinfo("Ignoring ACMP Connect TX Command for different talker");
        break;
      }
      avb_process_acmp_connect_tx_command(state, msg, false);
      break;
    case acmp_msg_type_connect_tx_response:
      if (memcmp(msg->listener_entity_id, state->own_entity.summary.entity_id, UNIQUE_ID_LEN) != 0) {
        avbinfo("Ignoring ACMP Connect TX Response for different listener");
        break;
      }
      avb_process_acmp_connect_tx_response(state, msg, false);
      break;
    case acmp_msg_type_disconnect_tx_command:
      if (memcmp(msg->talker_entity_id, state->own_entity.summary.entity_id, UNIQUE_ID_LEN) != 0) {
        avbinfo("Ignoring ACMP Disconnect TX Command for different talker");
        break;
      }
      avb_process_acmp_connect_tx_command(state, msg, true);
      break;
    case acmp_msg_type_disconnect_tx_response:
      if (memcmp(msg->listener_entity_id, state->own_entity.summary.entity_id, UNIQUE_ID_LEN) != 0) {
        avbinfo("Ignoring ACMP Disconnect TX Response for different listener");
        break;
      }
      avb_process_acmp_connect_tx_response(state, msg, true);
      break;
    default:
      avbinfo("Ignoring %s (unsupported)", get_acmp_message_type_name(msg->header.msg_type));
      break;
  }
  return OK;
}

/* Send an ACMP command message */
int avb_send_acmp_command(
    avb_state_s *state,
    acmp_msg_type_t msg_type, 
    acmp_message_s *command, 
    bool retried
) {
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
      avberr("Failed to send %s (error: %d)", get_acmp_message_type_name(msg_type), errno);
  }
  
  // add the command to the inflight commands list
  avb_add_inflight_command(state, (atdecc_command_u *)command, false);

  avbinfo("Sent %s", get_acmp_message_type_name(msg_type));
  return ret;
}

/* Send an ACMP response message */
int avb_send_acmp_response(
    avb_state_s *state, 
    acmp_msg_type_t msg_type, 
    acmp_message_s *response, 
    acmp_status_t status
) {
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
  uint16_t msg_len = response->header.control_data_len + 12; // 12 bytes for the header
  ret = avb_net_send(state, ethertype_avtp, response, msg_len, &ts);
  if (ret < 0) {
      avberr("Failed to send %s (error: %d)", get_acmp_message_type_name(msg_type), errno);
  }
  avbinfo("Sent %s", get_acmp_message_type_name(msg_type));
  return ret;
}

/* Process ACMP connect/disconnect rx command */
int avb_process_acmp_connect_rx_command(
    avb_state_s *state, 
    acmp_message_s *msg,
    bool disconnect
) {
    int ret = OK;
    uint16_t index = octets_to_uint(msg->listener_uid, 2);
    acmp_status_t status = acmp_status_success;
    acmp_msg_type_t rx_response_type = disconnect ? acmp_msg_type_disconnect_rx_response : acmp_msg_type_connect_rx_response;
    acmp_msg_type_t tx_command_type = disconnect ? acmp_msg_type_disconnect_tx_command : acmp_msg_type_connect_tx_command;

    // check if listener is valid
    uint16_t listener_uid = octets_to_uint(msg->listener_uid, 2);
    if (!avb_valid_talker_listener_uid(state, listener_uid, avb_entity_type_listener)) {
        avb_send_acmp_response(state, rx_response_type, msg, acmp_status_listener_unknown_id);
        return ERROR;
    }

    // check if locked or acquired by another controller
    if (avb_acquired_or_locked_by_other(state, &msg->controller_entity_id)) {
        avberr("ConnRX cmd: Locked or acquired by another controller");
        avb_send_acmp_response(state, rx_response_type, msg, acmp_status_controller_not_authorized);
        return ERROR;
    }

    // check if listener is connected to another talker in case of connect rx
    // check if listener is not connected to the same talker in case of disconnect rx
    bool same_talker = disconnect ? true : false;
    if (!avb_listener_is_connected(state, msg, same_talker)) {
        if (disconnect) {
            // listener is not connected to the talker in case of disconnect rx
            avberr("DisconnRX cmd: Listener is not connected to the talker");
            avb_send_acmp_response(state, rx_response_type, msg, acmp_status_not_connected);
            return ERROR;
        }
    } else if (!disconnect) {
        // listener is connected to the another talker in case of connect rx
        avberr("ConnRX cmd: Listener is already connected to another talker");
        avb_send_acmp_response(state, rx_response_type, msg, acmp_status_listener_exclusive);
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
        avb_send_acmp_response(state, rx_response_type, msg, acmp_status_listener_misbehaving);
        return ERROR;
    }

    // create a new command for the connect/disconnect tx command
    acmp_message_s new_command = {0};
    memcpy(&new_command, msg, sizeof(acmp_message_s));

    // send connect/disconnect tx command to talker
    ret = avb_send_acmp_command(state, tx_command_type, &new_command, false);
    if (ret) {
        memcpy(&state->input_streams[index].talker_id, &msg->talker_entity_id, UNIQUE_ID_LEN);
        memcpy(&state->input_streams[index].talker_uid, &msg->talker_uid, 2);
        state->input_streams[index].pending_connection = true;
    }
    return ret;
}

/* Process ACMP connect tx command */
int avb_process_acmp_connect_tx_command(
    avb_state_s *state, 
    acmp_message_s *msg,
    bool disconnect
) {
  // not implemented
  return OK;
}

/* Process ACMP connect tx response */
int avb_process_acmp_connect_tx_response(
    avb_state_s *state, 
    acmp_message_s *response,
    bool disconnect
) {
    int ret = OK;
    acmp_status_t status = response->header.status_valtime;
    uint16_t listener_uid = octets_to_uint(response->listener_uid, 2); // for the conn tx command
    acmp_msg_type_t rx_reponse_type = disconnect ? acmp_msg_type_disconnect_rx_response : acmp_msg_type_connect_rx_response;

    // check if listener is valid
    if (!avb_valid_talker_listener_uid(state, listener_uid, avb_entity_type_listener)) {
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
        }
        // update input stream
        state->input_streams[listener_uid].pending_connection = false;
    }

    // put the sequence ID from the inflight command into the new response
    response->header.msg_type = rx_reponse_type;
    int index = avb_find_inflight_command_by_data(state, (atdecc_command_u *)response, true);
    if (index == NOT_FOUND) {
        avberr("Inflight command not found.");
    }
    else {
        int_to_octets(&state->inflight_commands[index].acmp_seq_id, &new_response.seq_id, 2);
    }

    // other stuff TODO
    // cancelTimeout(msg);

    // remove the inflight command for the connect tx command
    avb_remove_inflight_command(state, octets_to_uint(response->seq_id, 2), false);

    // remove the inflight command for the original connect rx command
    if (index != NOT_FOUND) {
        avb_remove_inflight_command(state, state->inflight_commands[index].acmp_seq_id, true);
    }

    // send the response to the controller
    ret = avb_send_acmp_response(state, rx_reponse_type, &new_response, status);

    if (status == acmp_status_success) {
        // stop the l2tap interface
        //esp_err_t err = esp_l2tap_stop(state->config.eth_handle);
        //state->l2tap_receive = false;
        // Register the stack input function with the Ethernet driver
        ESP_ERROR_CHECK(esp_eth_update_input_path(state->config.eth_handle, my_callback, NULL));
    }
    return ret;
}

/* Find an entity in the known list of talkers, listeners, or controllers */
  int avb_find_entity_by_id(avb_state_s *state, 
                    unique_id_t *entity_id, 
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
        if (memcmp(state->listeners[i].entity_id, entity_id, UNIQUE_ID_LEN) == 0) {
          return i;
        }
      }
      break;
    case avb_entity_type_controller:
      for (int i = 0; i < state->num_controllers; i++) {
        if (memcmp(state->controllers[i].entity_id, entity_id, UNIQUE_ID_LEN) == 0) {
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
  int avb_find_entity_by_addr(avb_state_s *state, 
                    eth_addr_t *entity_addr, 
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
        if (memcmp(state->listeners[i].mac_addr, entity_addr, ETH_ADDR_LEN) == 0) {
          return i;
        }
      }
      break;
    case avb_entity_type_controller:
      for (int i = 0; i < state->num_controllers; i++) {
        if (memcmp(state->controllers[i].mac_addr, entity_addr, ETH_ADDR_LEN) == 0) {
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
const char* get_adp_message_type_name(adp_msg_type_t message_type) {
    switch (message_type) {
        case adp_msg_type_entity_available: return "ADP Entity Available";
        case adp_msg_type_entity_departing: return "ADP Entity Departing";
        case adp_msg_type_entity_discover: return "ADP Entity Discover";
        default: return "Unknown";
    }
}

// Get the name of the AECP command code
const char* get_aecp_command_code_name(aecp_cmd_code_t command_code) {
    switch (command_code) {
        case aecp_cmd_code_acquire_entity: return "AECP Acquire Entity";
        case aecp_cmd_code_lock_entity: return "AECP Lock Entity";
        case aecp_cmd_code_entity_available: return "AECP Entity Available";
        case aecp_cmd_code_controller_available: return "AECP Controller Available";
        case aecp_cmd_code_read_descriptor: return "AECP Read Descriptor";
        default: return "Unknown";
    }
}

// Get the name of the ACMP message type
const char* get_acmp_message_type_name(acmp_msg_type_t message_type) {
    switch (message_type) {
        case acmp_msg_type_connect_tx_command: return "ACMP Connect TX Command";
        case acmp_msg_type_connect_tx_response: return "ACMP Connect TX Response";
        case acmp_msg_type_disconnect_tx_command: return "ACMP Disconnect TX Command";
        case acmp_msg_type_disconnect_tx_response: return "ACMP Disconnect TX Response";
        case acmp_msg_type_get_tx_state_command: return "ACMP Get TX State Command";
        case acmp_msg_type_get_tx_state_response: return "ACMP Get TX State Response";
        case acmp_msg_type_connect_rx_command: return "ACMP Connect RX Command";
        case acmp_msg_type_connect_rx_response: return "ACMP Connect RX Response";
        case acmp_msg_type_disconnect_rx_command: return "ACMP Disconnect RX Command";
        case acmp_msg_type_disconnect_rx_response: return "ACMP Disconnect RX Response";
        case acmp_msg_type_get_rx_state_command: return "ACMP Get RX State Command";
        case acmp_msg_type_get_rx_state_response: return "ACMP Get RX State Response";
        default: return "Unknown";
    }
}

/* Check if the talker or listener unique ID is valid */
bool avb_valid_talker_listener_uid(avb_state_s *state, uint16_t uid, avb_entity_type_t entity_type) {
    switch (entity_type) {
        case avb_entity_type_listener:
            if (uid < state->num_listeners) {
                return true;
            }
            break;
        case avb_entity_type_talker:
            if (uid < state->num_talkers) {
                return true;
            }
            break;
        default:
            return false;
    }
    return false;
}

/* Check if the entity is acquired or locked by another entity */
bool avb_acquired_or_locked_by_other(avb_state_s *state, unique_id_t *entity_id) {
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
bool avb_listener_is_connected(avb_state_s *state, acmp_message_s *msg, bool same_talker) {
    // walk through all input streams
    for (int i = 0; i < state->num_input_streams; i++) {
        // check if the stream is connected or pending connection
        if (state->input_streams[i].connected || state->input_streams[i].pending_connection) {
            if (same_talker) {
                // check if the talker is the same
                if (memcmp(state->input_streams[i].talker_id, msg->talker_entity_id, UNIQUE_ID_LEN) == 0
                && memcmp(state->input_streams[i].talker_uid, msg->talker_uid, 2) == 0) {
                    return true;
                }
            }
            else {
                // check if the talker is different
                if (memcmp(state->input_streams[i].talker_id, msg->talker_entity_id, UNIQUE_ID_LEN) != 0
                && memcmp(state->input_streams[i].talker_uid, msg->talker_uid, 2) != 0) {
                    return true;
                }
            }
        }
    }
    return false;
}

/* add an inflight command to the inflight commands list */
// return the index of the command, if the list is full then return -1
int avb_add_inflight_command(avb_state_s *state, atdecc_command_u *command, bool inbound) {
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

  // copy the sequence ID from the command and adjust the timeout_ms for ACMP commands
  if (command->header.subtype == avtp_subtype_acmp) {
    state->inflight_commands[index].acmp_seq_id = octets_to_uint(command->acmp.seq_id, 2);
    timeout_ms = avb_get_acmp_timeout_ms(command->header.msg_type); // ACMP timeouts are more specific
  } else {
    state->inflight_commands[index].aecp_seq_id = octets_to_uint(command->aecp.common.seq_id, 2);
  }
  
  // set the timeout
  struct timeval timeout;
  gettimeofday(&timeout, NULL);
  timeval_add_ms(&timeout, timeout_ms);
  memcpy(&state->inflight_commands[index].timeout, &timeout, sizeof(struct timeval));

  // set the inbound flag
  state->inflight_commands[index].inbound = inbound;

  // copy the command data to the inflight command
  memcpy(&state->inflight_commands[index].command, command, sizeof(atdecc_command_u));

  // set retried to false and increment the number of inflight commands
  state->inflight_commands[index].retried = false;
  state->num_inflight_commands++;
  return index;
}

/* find an inflight command by the sequence ID */
int avb_find_inflight_command(avb_state_s *state, uint16_t seq_id, bool inbound) {
    for (int i = 0; i < state->num_inflight_commands; i++) {
        if (state->inflight_commands[i].command.header.subtype == avtp_subtype_acmp) {
            if (state->inflight_commands[i].acmp_seq_id == seq_id 
            && state->inflight_commands[i].inbound == inbound) {
                return i;
            }
        }
        else {
            if (state->inflight_commands[i].aecp_seq_id == seq_id 
            && state->inflight_commands[i].inbound == inbound) {
                return i;
            }
        }
    }
    return NOT_FOUND;
}

/* find an inflight commands list by the matching command data */
int avb_find_inflight_command_by_data(avb_state_s *state, atdecc_command_u *data, bool inbound) {

    for (int i = 0; i < state->num_inflight_commands; i++) {
        if (state->inflight_commands[i].inbound == inbound
        && state->inflight_commands[i].command.header.msg_type + 1 == data->header.msg_type) {
            if (data->header.subtype == avtp_subtype_acmp) {
                if (memcmp(state->inflight_commands[i].command.acmp.talker_uid, data->acmp.talker_uid, 2) == 0
                && memcmp(state->inflight_commands[i].command.acmp.listener_uid, data->acmp.listener_uid, 2) == 0
                && memcmp(state->inflight_commands[i].command.acmp.talker_entity_id, data->acmp.talker_entity_id, UNIQUE_ID_LEN) == 0
                && memcmp(state->inflight_commands[i].command.acmp.listener_entity_id, data->acmp.listener_entity_id, UNIQUE_ID_LEN) == 0
                && memcmp(state->inflight_commands[i].command.acmp.controller_entity_id, data->acmp.controller_entity_id, UNIQUE_ID_LEN) == 0) {
                    return i;
                }
            }
            else {
                if (memcmp(state->inflight_commands[i].command.aecp.common.target_entity_id, data->aecp.common.target_entity_id, UNIQUE_ID_LEN) == 0
                && memcmp(state->inflight_commands[i].command.aecp.common.controller_entity_id, data->aecp.common.controller_entity_id, UNIQUE_ID_LEN) == 0
                && state->inflight_commands[i].command.aecp.aem.command_type == data->aecp.aem.command_type) {
                    return i;
                }
            }
        }
    }
    return NOT_FOUND;
}

/* remove an inflight command from the inflight commands list */
// shift any elements after the index to the left
void avb_remove_inflight_command(avb_state_s *state, uint16_t seq_id, bool inbound) {
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
acmp_status_t avb_connect_listener(
    avb_state_s *state, 
    acmp_message_s *response
) {
    acmp_status_t status = acmp_status_success;
    uint16_t index = octets_to_uint(response->listener_uid, 2);

    // update the input stream
    memcpy(&state->input_streams[index].talker_id, response->talker_entity_id, UNIQUE_ID_LEN);
    memcpy(&state->input_streams[index].talker_uid, response->talker_uid, 2);
    memcpy(&state->input_streams[index].controller_id, response->controller_entity_id, UNIQUE_ID_LEN);
    memcpy(&state->input_streams[index].stream_id, response->stream_id, UNIQUE_ID_LEN);
    memcpy(&state->input_streams[index].stream_dest_addr, response->stream_dest_addr, ETH_ADDR_LEN);
    state->input_streams[index].connected = true;

    // send SRP listener ready command
    int ret = avb_send_msrp_listener(
        state, 
        mrp_attr_event_new, 
        msrp_listener_event_ready,
        false,
        &state->input_streams[index].stream_id);
    if (ret < 0) {
        return acmp_status_listener_misbehaving;
    }
    
    return status;
}

acmp_status_t avb_disconnect_listener(
    avb_state_s *state, 
    acmp_message_s *response
) {
    acmp_status_t status = acmp_status_success;
    uint16_t index = octets_to_uint(response->listener_uid, 2);

    // reset the input stream data
    memset(&state->input_streams[index], 0, sizeof(avb_listener_stream_s));
    uint16_t vlan_id = DEFAULT_VLAN_ID;
    avtp_stream_format_am824_s format = AVB_DEFAULT_FORMAT(cip_sfc_sample_rate_48k);
    memcpy(&state->input_streams[index].stream_format, &format, sizeof(avtp_stream_format_s));
    int_to_octets(&vlan_id, state->input_streams[index].vlan_id, 2);

    // send SRP listener deregistration command
    int ret = avb_send_msrp_listener(
        state, 
        mrp_attr_event_lv, 
        msrp_listener_event_ready,
        false,
        &state->input_streams[index].stream_id);
    if (ret < 0) {
        return acmp_status_listener_misbehaving;
    }
    return status;
}

/* Start a stream */
acmp_status_t avb_connect_talker(
    avb_state_s *state, 
    acmp_message_s *response
) {
    acmp_status_t status = acmp_status_success;
    uint16_t index = octets_to_uint(response->talker_uid, 2);

    // create a stream ID from the internal MAC address and stream index
    //unique_id_t stream_id;
    //stream_id_from_mac(&state->internal_mac_addr, stream_id, index);

    // if output stream not already started then 
    // create a stream ID, send SRP talker registration
    return status;
}

/* Get the timeout for an ACMP message type */
int avb_get_acmp_timeout_ms(acmp_msg_type_t msg_type) {
    switch (msg_type) {
        case acmp_msg_type_connect_tx_command: return acmp_timeout_connect_tx;
        case acmp_msg_type_disconnect_tx_command: return acmp_timeout_disconnect_tx;
        case acmp_msg_type_get_tx_state_command: return acmp_timeout_get_tx_state;
        case acmp_msg_type_connect_rx_command: return acmp_timeout_connect_rx;
        case acmp_msg_type_disconnect_rx_command: return acmp_timeout_disconnect_rx;
        case acmp_msg_type_get_rx_state_command: return acmp_timeout_get_rx_state;
        case acmp_msg_type_connection_command: return acmp_timeout_connection;
        default: return 250;
    }
}
