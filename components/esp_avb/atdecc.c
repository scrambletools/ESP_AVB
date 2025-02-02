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
int avb_send_adp_entity_available(struct avb_state_s *state) {
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
int avb_send_aecp_cmd_controller_available(struct avb_state_s *state, unique_id_t *target_id) {
  // not implemented
  return OK;
}

/* Send AECP command get stream info message */
int avb_send_aecp_cmd_get_stream_info(struct avb_state_s *state, unique_id_t *target_id) {
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
  int_to_octets(&command_type, &msg.common.command_type, 2);
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


/* Send AECP command get counters message */
int avb_send_aecp_cmd_get_counters(struct avb_state_s *state, unique_id_t *target_id) {
  // not implemented
  return OK;
}

/* Send AECP response get stream info message */
int avb_send_aecp_rsp_get_stream_info(struct avb_state_s *state, unique_id_t *target_id) {
  // not implemented
  return OK;
}

/* Send AECP response get counters message */
int avb_send_aecp_rsp_get_counters(struct avb_state_s *state, unique_id_t *target_id) {
  // not implemented
  return OK;
}

/* Send AECP response get descriptor for entity message */
int avb_send_aecp_rsp_get_descr_entity(struct avb_state_s *state, unique_id_t *target_id) {
  // not implemented
  return OK;
}

/* Send AECP response get descriptor for configuration message */
int avb_send_aecp_rsp_get_descr_configuration(struct avb_state_s *state, unique_id_t *target_id) {
  // not implemented
  return OK;
}

/* Send AECP response get descriptor for audio unit message */
int avb_send_aecp_rsp_get_descr_audio_unit(struct avb_state_s *state, unique_id_t *target_id) {
  // not implemented
  return OK;
}

/* Send AECP response get descriptor for stream input message */
int avb_send_aecp_rsp_get_descr_stream_input(struct avb_state_s *state, unique_id_t *target_id) {
  // not implemented
  return OK;
}

/* Send AECP response get descriptor for stream output message */
int avb_send_aecp_rsp_get_descr_stream_output(struct avb_state_s *state, unique_id_t *target_id) {
  // not implemented
  return OK;
}

/* Send AECP response get descriptor for avb interface message */
int avb_send_aecp_rsp_get_descr_avb_interface(struct avb_state_s *state, unique_id_t *target_id) {
  // not implemented
  return OK;
}

/* Send AECP response get descriptor for clock source message */
int avb_send_aecp_rsp_get_descr_clock_source(struct avb_state_s *state, unique_id_t *target_id) {
  // not implemented
  return OK;
}

/* Send AECP response get descriptor for locale message */
int avb_send_aecp_rsp_get_descr_locale(struct avb_state_s *state, unique_id_t *target_id) {
  // not implemented
  return OK;
}

/* Send AECP response get descriptor for clock domain message */
int avb_send_aecp_rsp_get_descr_clock_domain(struct avb_state_s *state, unique_id_t *target_id) {
  // not implemented
  return OK;
}

/* Send ACMP connect rx command (acting as controller) */
int avb_send_acmp_connect_rx_command(struct avb_state_s *state, 
                                     unique_id_t *talker_id,
                                     unique_id_t *listener_id) {
  // not implemented
  return OK;
}

/* Send ACMP connect tx command (acting as listener) */
int avb_send_acmp_connect_tx_command(struct avb_state_s *state, 
                                     unique_id_t *controller_id, 
                                     unique_id_t *talker_id) {
  acmp_message_s msg;
  struct timespec ts;
  int ret;
  size_t body_size = 84; // ACMP message body length (IEEE 1722.1-2021 section 8.2.1)
  memset(&msg, 0, sizeof(msg));

  // Populate the message
  msg.header.subtype = avtp_subtype_acmp;
  msg.header.msg_type = acmp_msg_type_connect_tx_command;
  msg.header.control_data_len = body_size;
  memcpy(msg.controller_entity_id, controller_id, UNIQUE_ID_LEN);
  memcpy(msg.talker_entity_id, talker_id, UNIQUE_ID_LEN);
  memcpy(msg.listener_entity_id, state->own_entity.summary.entity_id, UNIQUE_ID_LEN);

  // Get the talker uid (listener uid will be 0)
  eth_addr_t talker_addr;
  int index = avb_find_entity_by_id(state, talker_id, avb_entity_type_talker);
  memcpy(msg.talker_uid, state->talkers[index].talker_uid, 2);
  memcpy(talker_addr, state->talkers[index].mac_addr, ETH_ADDR_LEN);

  // send the message 
  uint16_t msg_len = 4 + body_size; // header + body
  ret = avb_net_send(state, ethertype_avtp, &msg, msg_len, &ts);
  if (ret < 0) {
    avberr("send ACMP Connect Tx Command failed: %d", errno);
  }
  avbinfo("sent ACMP Connect Tx Command");
  return ret;
}

/* Send ACMP disconnect rx command (acting as controller) */
int avb_send_acmp_disconnect_rx_command(struct avb_state_s *state, avb_connection_s *connection) {
  // not implemented
  return OK;
}

/* Send ACMP disconnect tx command (acting as listener) */
int avb_send_acmp_disconnect_tx_command(struct avb_state_s *state, avb_connection_s *connection) {
  // not implemented
  return OK;
}

/* Send ACMP connect rx response */
int avb_send_acmp_connect_rx_response(struct avb_state_s *state, avb_connection_s *connection) {
  
  // take the command and update the stream id, dest addr and connection count
  // TBD
  
  acmp_message_s msg;
  struct timespec ts;
  int ret;
  size_t body_size = 84; // ACMP message body length (IEEE 1722.1-2021 section 8.2.1)
  memset(&msg, 0, sizeof(msg));

  // Populate the message
  msg.header.subtype = avtp_subtype_acmp;
  msg.header.msg_type = acmp_msg_type_connect_rx_response;
  msg.header.status_valtime = 0; // status: success
  msg.header.control_data_len = body_size;
  memcpy(msg.controller_entity_id, connection->controller_id, UNIQUE_ID_LEN);
  memcpy(msg.talker_entity_id, connection->talker_id, UNIQUE_ID_LEN);
  memcpy(msg.listener_entity_id, connection->listener_id, UNIQUE_ID_LEN);

  // get the talker and listener uids
  int index = avb_find_entity_by_id(state, &connection->talker_id, avb_entity_type_talker);
  memcpy(msg.talker_uid, state->talkers[index].talker_uid, 2);
  index = avb_find_entity_by_id(state, &connection->listener_id, avb_entity_type_listener);
  memcpy(msg.listener_uid, state->listeners[index].listener_uid, 2); 

  // send the message 
  uint16_t msg_len = 4 + body_size; // header + body
  ret = avb_net_send(state, ethertype_avtp, &msg, msg_len, &ts);
  if (ret < 0) {
      avberr("send ACMP Connect Rx Response failed: %d", errno);
  }
  avbinfo("sent ACMP Connect Rx Response");
  return ret;
}

/* Send ACMP connect tx response (acting as talker) */
int avb_send_acmp_connect_tx_response(struct avb_state_s *state, avb_connection_s *connection) {
  acmp_message_s msg;
  struct timespec ts;
  int ret;
  size_t body_size = 84; // ACMP message body length (IEEE 1722.1-2021 section 8.2.1)
  memset(&msg, 0, sizeof(msg));

  // Populate the message
  msg.header.subtype = avtp_subtype_acmp;
  msg.header.msg_type = acmp_msg_type_connect_tx_response;
  msg.header.status_valtime = 0; // status: success
  msg.header.control_data_len = body_size;
  memcpy(msg.controller_entity_id, connection->controller_id, UNIQUE_ID_LEN);
  memcpy(msg.talker_entity_id, connection->talker_id, UNIQUE_ID_LEN);
  memcpy(msg.listener_entity_id, connection->listener_id, UNIQUE_ID_LEN);

  // get the talker and listener uids
  int index = avb_find_entity_by_id(state, &connection->talker_id, avb_entity_type_talker);
  memcpy(msg.talker_uid, state->talkers[index].talker_uid, 2);
  index = avb_find_entity_by_id(state, &connection->listener_id, avb_entity_type_listener);
  memcpy(msg.listener_uid, state->listeners[index].listener_uid, 2);

  // send the message 
  uint16_t msg_len = 4 + body_size; // header + body
  ret = avb_net_send(state, ethertype_avtp, &msg, msg_len, &ts);
  if (ret < 0) {
      avberr("send ACMP Connect Tx Response failed: %d", errno);
    }
  avbinfo("sent ACMP Connect Tx Response");
  return ret;
}

/* Send ACMP disconnect rx response */
int avb_send_acmp_disconnect_rx_response(struct avb_state_s *state, avb_connection_s *connection) {
  // not implemented
  return OK;
}

/* Send ACMP disconnect tx response */
int avb_send_acmp_disconnect_tx_response(struct avb_state_s *state, avb_connection_s *connection) {
  // not implemented
  return OK;
}

/* Process received ATDECC ADP message */
int avb_process_adp(struct avb_state_s *state, adp_message_s *msg, eth_addr_t *src_addr) {
  
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
int avb_process_aecp(struct avb_state_s *state, aecp_message_u *msg, eth_addr_t *src_addr) {
  
  /* Process AECP command */
  if (msg->header.msg_type == aecp_msg_type_aem_command) {
    switch (msg->common.command_type) {
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
    switch (msg->common.command_type) {
      case aecp_cmd_code_register_unsol_notif:
        return avb_process_aecp_rsp_register_unsol_notif(state, msg);
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

/* Process AECP command acquire entity */
int avb_process_aecp_cmd_acquire_entity(struct avb_state_s *state, 
                                        aecp_message_u *msg, 
                                        eth_addr_t *src_addr) {
  int ret;
  struct timespec ts;

  // check if the target entity id is the own entity id
  if (memcmp(msg->common.target_entity_id, state->own_entity.summary.entity_id, UNIQUE_ID_LEN) != 0) {
    avbinfo("Ignoring AECP Acquire Entity for different entity");
    return OK;
  }

  // create a response, copy the cmd message data and change the msg type and status
  aecp_acquire_entity_s response;
  memset(&response, 0, sizeof(aecp_acquire_entity_s));
  memcpy(&response.common, &msg->common, sizeof(aem_common_s));
  response.common.header.msg_type = aecp_msg_type_aem_response;
  response.common.header.status_valtime = aecp_status_not_supported; // status: not supported

  // send the response message  
  uint16_t msg_len = sizeof(atdecc_header_s) + sizeof(aecp_acquire_entity_s);
  ret = avb_net_send_to(state, ethertype_avtp, &response, msg_len, &ts, src_addr);
  if (ret < 0) {
    avberr("send AECP Acquire Entity response failed: %d", errno);
  }
  return ret;
}

/* Process AECP command lock entity */
int avb_process_aecp_cmd_lock_entity(struct avb_state_s *state, 
                                      aecp_message_u *msg, 
                                      eth_addr_t *src_addr) {
  int ret;
  struct timespec ts;

  // check if the target entity id is the own entity id
  if (memcmp(msg->common.target_entity_id, state->own_entity.summary.entity_id, UNIQUE_ID_LEN) != 0) {
    avbinfo("Ignoring AECP Lock Entity for different entity");
    return OK;
  }

  // create a response, copy the cmd message data and change the msg type and status
  aecp_lock_entity_s response;
  memset(&response, 0, sizeof(aecp_lock_entity_s));
  memcpy(&response.common, &msg->common, sizeof(aem_common_s));
  response.common.header.msg_type = aecp_msg_type_aem_response;
  response.common.header.status_valtime = aecp_status_not_supported; // status: not supported

  // send the response message  
  uint16_t msg_len = sizeof(atdecc_header_s) + sizeof(aecp_lock_entity_s);
  ret = avb_net_send_to(state, ethertype_avtp, &response, msg_len, &ts, src_addr);
  if (ret < 0) {
    avberr("send AECP Lock Entity response failed: %d", errno);
  }
  return ret;
}

/* Process AECP command entity available */
int avb_process_aecp_cmd_entity_available(struct avb_state_s *state, 
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
  memcpy(&response.common, &msg->common, sizeof(aem_common_s));
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
int avb_process_aecp_cmd_get_configuration(struct avb_state_s *state, 
                                            aecp_message_u *msg, 
                                            eth_addr_t *src_addr) {
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
  memcpy(&response.common, &msg->common, sizeof(aem_common_s));
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
int avb_process_aecp_cmd_read_descriptor(struct avb_state_s *state, 
                                          aecp_message_u *msg, 
                                          eth_addr_t *src_addr) {
  int ret;
  struct timespec ts;
  uint16_t control_data_len = sizeof(aecp_read_descriptor_rsp_s) - sizeof(atdecc_header_s) - AEM_MAX_DESC_LEN;
  
  // check if the target entity id is the own entity id
  if (memcmp(msg->common.target_entity_id, state->own_entity.summary.entity_id, UNIQUE_ID_LEN) != 0) {
    avbinfo("Ignoring AECP Read Descriptor for different entity");
    return OK;
  }

  // create a response, copy the cmd message data and change the msg type
  aecp_read_descriptor_rsp_s response;
  memset(&response, 0, sizeof(aecp_read_descriptor_rsp_s));
  memcpy(&response, msg, sizeof(aecp_read_descriptor_rsp_s));
  response.common.header.msg_type = aecp_msg_type_aem_response;

  // check if the descriptor type is supported
  switch (octets_to_uint(msg->read_descriptor.descriptor_type, 2)) {
    case aem_desc_type_entity:
      memcpy(response.descriptor_data, &state->own_entity, sizeof(aem_entity_desc_s));
      control_data_len += sizeof(aem_entity_desc_s);
      break;
    case aem_desc_type_configuration:
      
      // create a configuration descriptor
      aem_config_desc_s config_desc;
      memset(&config_desc, 0, sizeof(aem_config_desc_s));
      memcpy(config_desc.descriptor_type, msg->read_descriptor.descriptor_type, 2);
      memcpy(config_desc.descriptor_index, msg->read_descriptor.descriptor_index, 2);
      // These are the top level descriptors that will be listed in the descriptor counts
      uint16_t descriptors[] = {
        aem_desc_type_audio_unit,
        aem_desc_type_stream_input,
        aem_desc_type_stream_output,
        aem_desc_type_avb_interface,
        aem_desc_type_clock_source,
        aem_desc_type_memory_object,
        aem_desc_type_locale,
        aem_desc_type_strings,
        aem_desc_type_control,
        aem_desc_type_clock_domain
      };
      uint16_t descriptor_counts_count = sizeof(descriptors) / sizeof(descriptors[0]);
      int_to_octets(&descriptor_counts_count, config_desc.descriptor_counts_count, 2);
      uint16_t offset = 74; // As defined in section 7.2.2
      int_to_octets(&offset, config_desc.descriptor_counts_offset, 2);
      
      // add the configuration descriptors
      for (int i = 0; i < descriptor_counts_count; i++) {
        aem_config_desc_count_s desc_count;
        memset(&desc_count, 0, sizeof(aem_config_desc_count_s));
        int_to_octets(&AEM_CONFIG_DESCRIPTORS[i], desc_count.descriptor_type, 2);
        size_t count = AEM_CONFIG_MAX_DESC_COUNT;
        int_to_octets(&count, desc_count.count, 2);
        memcpy(&config_desc.descriptor_counts[i], &desc_count, 4);
      }
      memcpy(response.descriptor_data, &config_desc, sizeof(aem_config_desc_s));
      control_data_len += sizeof(aem_config_desc_s) 
                       - (4 * (AEM_CONFIG_MAX_NUM_DESC - sizeof(AEM_CONFIG_DESCRIPTORS))); // adjust for the descriptor counts
      break;
    case aem_desc_type_audio_unit:
      response.common.header.status_valtime = aecp_status_not_implemented; // status: not implemented
      break;
    case aem_desc_type_stream_input:
      response.common.header.status_valtime = aecp_status_not_implemented; // status: not implemented
      break;
    case aem_desc_type_stream_output:
      response.common.header.status_valtime = aecp_status_not_implemented; // status: not implemented
      break;
    case aem_desc_type_avb_interface:
      response.common.header.status_valtime = aecp_status_not_implemented; // status: not implemented
      break;
    case aem_desc_type_clock_source:
      response.common.header.status_valtime = aecp_status_not_implemented; // status: not implemented
      break;
    case aem_desc_type_memory_object:
      response.common.header.status_valtime = aecp_status_not_implemented; // status: not implemented
      break;
    case aem_desc_type_locale:
      response.common.header.status_valtime = aecp_status_not_implemented; // status: not implemented
      break;
    case aem_desc_type_strings:
      response.common.header.status_valtime = aecp_status_not_implemented; // status: not implemented
      break;
    case aem_desc_type_stream_port_input:
      response.common.header.status_valtime = aecp_status_not_implemented; // status: not implemented
      break;
    case aem_desc_type_stream_port_output:
      response.common.header.status_valtime = aecp_status_not_implemented; // status: not implemented
      break;
    case aem_desc_type_audio_cluster:
      response.common.header.status_valtime = aecp_status_not_implemented; // status: not implemented
      break;
    case aem_desc_type_audio_map:
      response.common.header.status_valtime = aecp_status_not_implemented; // status: not implemented
      break;
    case aem_desc_type_control:
      response.common.header.status_valtime = aecp_status_not_implemented; // status: not implemented
      break;
    case aem_desc_type_clock_domain:
      response.common.header.status_valtime = aecp_status_not_implemented; // status: not implemented
      break;
    default:
      char desc_type_str[7];
      octets_to_hex_string(msg->read_descriptor.descriptor_type, 2, desc_type_str, '-');
      avbinfo("Ignoring AECP Read Descriptor for unsupported descriptor type %s", desc_type_str);
      return OK;
  }
  // set the control data length
  response.common.header.control_data_len_h = (control_data_len >> 8) & 0xFF;
  response.common.header.control_data_len = control_data_len & 0xFF;
  
  // send the response message
  uint16_t msg_len = sizeof(atdecc_header_s) + control_data_len;
  ret = avb_net_send_to(state, ethertype_avtp, &response, msg_len, &ts, src_addr);
  if (ret < 0) {
    avberr("send AECP Read Descriptor response failed: %d", errno);
  }
  return ret;
}

/* Process AECP command get stream info */
int avb_process_aecp_cmd_get_stream_info(struct avb_state_s *state, 
                                          aecp_message_u *msg, 
                                          eth_addr_t *src_addr) {
  // not implemented
  return OK;
}

/* Process AECP command get counters */
// counters can be for entity, stream input, stream output, avb interface or clock domain
int avb_process_aecp_cmd_get_counters(struct avb_state_s *state, 
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
int avb_process_aecp_rsp_register_unsol_notif(struct avb_state_s *state, aecp_message_u *ms) {
  return OK;
}

/* Process AECP response entity available */
int avb_process_aecp_rsp_entity_available(struct avb_state_s *state, aecp_message_u *msg) {
  return OK;
}

/* Process AECP response controller available */
int avb_process_aecp_rsp_controller_available(struct avb_state_s *state, aecp_message_u *msg) {
  return OK;
}

/* Process AECP response get stream info 
 * this may be sent by the talker as an unsolicited notification
*/
int avb_process_aecp_rsp_get_stream_info(struct avb_state_s *state, aecp_message_u *msg) {
  
  // find the talker and update the talker info
  int index = avb_find_entity_by_id(state, &msg->common.target_entity_id, avb_entity_type_talker);
  if (index == NOT_FOUND) {
    avberr("Ignoring AECP Get Stream Info response for unknown talker");
    return OK;
  }
  // update the talker stream info
  else {
    memcpy(&state->talkers[index].stream, &msg->get_set_stream_info.stream, sizeof(aem_stream_summary_s));
    
    // if the connection is active, then send connect rx response to controller
    int index = avb_find_connection_by_id(state, &msg->common.target_entity_id, avb_entity_type_listener);
    if (index != NOT_FOUND && state->connections[index].active) {
      avb_send_acmp_connect_rx_response(state, &state->connections[index]);
    }
  }
  return OK;
}

/* Process AECP response get counters 
 * this may be sent by the talker as an unsolicited notification
*/
int avb_process_aecp_rsp_get_counters(struct avb_state_s *state, aecp_message_u *msg) {
  return OK;
}

/* Process received ATDECC ACMP message */
int avb_process_acmp(struct avb_state_s *state, acmp_message_s *msg) {

  switch (msg->header.msg_type) {
    case acmp_msg_type_connect_rx_command:
      if (memcmp(msg->listener_entity_id, state->own_entity.summary.entity_id, UNIQUE_ID_LEN) != 0) {
        avbinfo("Ignoring ACMP Connect RX Command for different listener");
        break;
      }
      avb_process_acmp_connect_rx_command(state, msg);
      break;
    case acmp_msg_type_connect_rx_response:
      if (memcmp(msg->controller_entity_id, state->own_entity.summary.entity_id, UNIQUE_ID_LEN) != 0) {
        avbinfo("Ignoring ACMP Connect RX Response for different controller");
        break;
      }
      avb_process_acmp_connect_rx_response(state, msg);
      break;
    case acmp_msg_type_disconnect_rx_command:
      if (memcmp(msg->listener_entity_id, state->own_entity.summary.entity_id, UNIQUE_ID_LEN) != 0) {
        avbinfo("Ignoring ACMP Disconnect RX Command for different listener");
        break;
      }
      avb_process_acmp_disconnect_rx_command(state, msg);
      break;
    case acmp_msg_type_disconnect_rx_response:
      if (memcmp(msg->controller_entity_id, state->own_entity.summary.entity_id, UNIQUE_ID_LEN) != 0) {
        avbinfo("Ignoring ACMP Disconnect RX Response for different controller");
        break;
      }
      avb_process_acmp_disconnect_rx_response(state, msg);
      break;
    case acmp_msg_type_connect_tx_command:
      if (memcmp(msg->talker_entity_id, state->own_entity.summary.entity_id, UNIQUE_ID_LEN) != 0) {
        avbinfo("Ignoring ACMP Connect TX Command for different talker");
        break;
      }
      avb_process_acmp_connect_tx_command(state, msg);
      break;
    case acmp_msg_type_connect_tx_response:
      if (memcmp(msg->listener_entity_id, state->own_entity.summary.entity_id, UNIQUE_ID_LEN) != 0) {
        avbinfo("Ignoring ACMP Connect TX Response for different listener");
        break;
      }
      avb_process_acmp_connect_tx_response(state, msg);
      break;
    case acmp_msg_type_disconnect_tx_command:
      if (memcmp(msg->talker_entity_id, state->own_entity.summary.entity_id, UNIQUE_ID_LEN) != 0) {
        avbinfo("Ignoring ACMP Disconnect TX Command for different talker");
        break;
      }
      avb_process_acmp_disconnect_tx_command(state, msg);
      break;
    case acmp_msg_type_disconnect_tx_response:
      if (memcmp(msg->listener_entity_id, state->own_entity.summary.entity_id, UNIQUE_ID_LEN) != 0) {
        avbinfo("Ignoring ACMP Disconnect TX Response for different listener");
        break;
      }
      avb_process_acmp_disconnect_tx_response(state, msg);
      break;
    default:
      avbinfo("Ignoring unsupported ACMP message type: %d", msg->header.msg_type);
      break;
  }
  return OK;
}

/* Process ACMP connect rx command */
int avb_process_acmp_connect_rx_command(struct avb_state_s *state, acmp_message_s *msg) {

  int ret;
  struct timespec ts;
  size_t body_size = 56; // ACMP Message body length

  // find talker address from talker entity id
  eth_addr_t dest_addr;
  int index = avb_find_entity_by_id(state, &msg->talker_entity_id, true);
  memcpy(dest_addr, state->talkers[index].mac_addr, ETH_ADDR_LEN);

  // send connect tx cmd to talker
  acmp_message_s connect_tx_cmd;
  memcpy(&connect_tx_cmd, msg, sizeof(acmp_message_s));
  uint16_t msg_len = 4 + body_size; // header + body
  ret = avb_net_send(state, ethertype_avtp, &connect_tx_cmd, msg_len, &ts);
  if (ret < 0) {
    avberr("send ACMP Connect TX Command failed: %d", errno);
    return ret;
  }

  // send get stream info to talker, TBD
  avb_send_aecp_cmd_get_stream_info(state, &msg->talker_entity_id);

  // when receive stream info, send connect rx response to controller
  return OK;
}

/* Process ACMP connect rx response */
int avb_process_acmp_connect_rx_response(struct avb_state_s *state, acmp_message_s *msg) {
  // not implemented
  return OK;
}

/* Process ACMP disconnect rx command */
int avb_process_acmp_disconnect_rx_command(struct avb_state_s *state, acmp_message_s *msg) {
  // not implemented
  return OK;
}

/* Process ACMP disconnect rx response */
int avb_process_acmp_disconnect_rx_response(struct avb_state_s *state, acmp_message_s *msg) {
  // not implemented
  return OK;
}

/* Process ACMP connect tx command */
int avb_process_acmp_connect_tx_command(struct avb_state_s *state, acmp_message_s *msg) {
  // not implemented
  return OK;
}

/* Process ACMP connect tx response */
int avb_process_acmp_connect_tx_response(struct avb_state_s *state, acmp_message_s *msg) {
  
  // check if the listener is me
  char listener_id_str[UNIQUE_ID_LEN * 3 + 1];
  octets_to_hex_string((uint8_t*)&msg->listener_entity_id, UNIQUE_ID_LEN, listener_id_str, '-');
  if (memcmp(&msg->listener_entity_id, state->own_entity.summary.entity_id, UNIQUE_ID_LEN) != 0) {
    avberr("ConnTX resp: Listener %s is not me", listener_id_str);
    return ERROR;
  }

  // check if the talker is known
  char talker_id_str[UNIQUE_ID_LEN * 3 + 1];
  octets_to_hex_string((uint8_t*)&msg->talker_entity_id, UNIQUE_ID_LEN, talker_id_str, '-');
  int talker_idx = avb_find_entity_by_id(state, &msg->talker_entity_id, avb_entity_type_talker);
  if (talker_idx == NOT_FOUND) {
    avbwarn("ConnTX resp: Talker %s not found among %d talkers", talker_id_str, state->num_talkers);
    return ERROR;
  }

  // check if the talker adv info is set
  if (memcmp(&state->talkers[talker_idx].info, &EMPTY_ID, UNIQUE_ID_LEN) == 0) {
    avbwarn("ConnTX resp: Talker %s info not yet known", talker_id_str);
    return ERROR;
  }

  // if the connection is not yet known, add it to the list
  int index = avb_find_connection_by_id(state, &msg->stream_id, avb_entity_type_listener);
  if (index == NOT_FOUND) {

    // create a new connection
    avb_connection_s connection;
    memset(&connection, 0, sizeof(avb_connection_s));
    memcpy(&connection.talker_info, &state->talkers[talker_idx].info, sizeof(talker_adv_info_s));
    memcpy(&connection.stream, &state->talkers[talker_idx].stream, sizeof(aem_stream_summary_s));
    memcpy(&connection.stream.stream_id, &msg->stream_id, UNIQUE_ID_LEN); // in case stream summary is empty
    memcpy(&connection.talker_id, &msg->talker_entity_id, UNIQUE_ID_LEN);
    memcpy(&connection.listener_id, &msg->listener_entity_id, UNIQUE_ID_LEN);
    memcpy(&connection.controller_id, &msg->controller_entity_id, UNIQUE_ID_LEN);
    memcpy(&connection.dest_addr, &msg->stream_dest_addr, ETH_ADDR_LEN);
    memcpy(&connection.vlan_id, &msg->stream_vlan_id, 2);

    // if the connection list is not full, add the connection
    if (state->num_connections < AVB_MAX_NUM_CONNECTIONS) {
      state->connections[state->num_connections] = connection;
      state->num_connections++;
    }
    // if the connection list is full, replace the oldest connection
    else {
      memmove(&state->connections[0], &state->connections[1], (state->num_connections - 1) * sizeof(avb_connection_s));
      state->connections[state->num_connections - 1] = connection;
    }
  }
  return OK;
}

/* Process ACMP disconnect tx command */
int avb_process_acmp_disconnect_tx_command(struct avb_state_s *state, acmp_message_s *msg) {
  // not implemented
  return OK;
}

/* Process ACMP disconnect tx response */
int avb_process_acmp_disconnect_tx_response(struct avb_state_s *state, acmp_message_s *msg) {
  // not implemented
  return OK;
}

/* Find an entity in the known list of talkers, listeners, or controllers */
  int avb_find_entity_by_id(struct avb_state_s *state, 
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
  int avb_find_entity_by_addr(struct avb_state_s *state, 
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

/* Find a connection in the known list of connections by stream id and role this device plays */
int avb_find_connection_by_id(struct avb_state_s *state, 
                              unique_id_t *stream_id, 
                              avb_entity_type_t role) {
  switch (role) { 
    case avb_entity_type_talker:
      for (int i = 0; i < state->num_connections; i++) {
        if (memcmp(state->connections[i].stream.stream_id, stream_id, UNIQUE_ID_LEN) == 0 
        && memcmp(state->connections[i].talker_id, state->own_entity.summary.entity_id, UNIQUE_ID_LEN) == 0) {
          return i;
        }
      }
      break;
    case avb_entity_type_listener:
      for (int i = 0; i < state->num_connections; i++) {
        if (memcmp(state->connections[i].stream.stream_id, stream_id, UNIQUE_ID_LEN) == 0 
        && memcmp(state->connections[i].listener_id, state->own_entity.summary.entity_id, UNIQUE_ID_LEN) == 0) {
          return i;
        }
      }
      break;
    case avb_entity_type_controller:
      for (int i = 0; i < state->num_connections; i++) {
        if (memcmp(state->connections[i].stream.stream_id, stream_id, UNIQUE_ID_LEN) == 0 
        && memcmp(state->connections[i].controller_id, state->own_entity.summary.entity_id, UNIQUE_ID_LEN) == 0) {
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
