/*
 * Copyright 2024 Scramble Tools
 * License: MIT
 *
 * ESP_AVB Component
 *
 * This component provides an implementation of an AVB talker and listener.
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
  msg.subtype = avtp_subtype_adp;
  msg.msg_type = adp_msg_type_entity_available;
  msg.version = 0;
  msg.valid_time = 8; // 16 seconds
  msg.control_data_len = body_size;
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
  else {
      avbinfo("Sent ADP Entity Available message");
    }
  return ret;
}

/* Process received ATDECC ADP message */
int avb_process_adp(struct avb_state_s *state, adp_message_s *msg, eth_addr_t *entity_addr) {
  
  if (msg->subtype == adp_msg_type_entity_available) {
    
    // If the entity is an audio talker, then remember it
    if (msg->entity.talker_capabilities.implemented && msg->entity.talker_capabilities.audio_source) {
      avb_add_entity(state, &msg->entity, entity_addr, true);
    }
    // If the entity is an audio listener, then remember it
    else if (msg->entity.listener_capabilities.implemented && msg->entity.listener_capabilities.audio_sink) {
      avb_add_entity(state, &msg->entity, entity_addr, false);
    }

    // Add the entity to the 
  }
  return OK;
}

/* Process received ATDECC AECP message */
int avb_process_aecp(struct avb_state_s *state, aecp_message_s *msg) {
  
  if (msg->msg_type == aecp_msg_type_aem_command) {
    switch (msg->command_type) {
      case aecp_cmd_code_acquire_entity:
        return avb_process_aecp_cmd_acquire_entity(state, msg);
      case aecp_cmd_code_lock_entity:
        return avb_process_aecp_cmd_lock_entity(state, msg);
        break;
      case aecp_cmd_code_entity_available:
        return avb_process_aecp_cmd_entity_available(state, msg);
        break;
      case aecp_cmd_code_get_configuration:
        return avb_process_aecp_cmd_get_configuration(state, msg);
        break;
      case aecp_cmd_code_read_descriptor:
        return avb_process_aecp_cmd_read_descriptor(state, msg);
        break;
      case aecp_cmd_code_get_stream_info:
        return avb_process_aecp_cmd_get_stream_info(state, msg);
        break;
      case aecp_cmd_code_get_counters:
        return avb_process_aecp_cmd_get_counters(state, msg);
        break;
      default:
        return OK;
    }
  }
  else { /* AECP responses */
    switch (msg->command_type) {
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
int avb_process_aecp_cmd_acquire_entity(struct avb_state_s *state, aecp_message_s *msg) {
  return OK;
}

/* Process AECP command lock entity */
int avb_process_aecp_cmd_lock_entity(struct avb_state_s *state, aecp_message_s *msg) {
  return OK;
}

/* Process AECP command entity available */
int avb_process_aecp_cmd_entity_available(struct avb_state_s *state, aecp_message_s *msg) {
  return OK;
}

/* Process AECP command get configuration */
int avb_process_aecp_cmd_get_configuration(struct avb_state_s *state, aecp_message_s *msg) {
  return OK;
}

/* Process AECP command read descriptor */
int avb_process_aecp_cmd_read_descriptor(struct avb_state_s *state, aecp_message_s *msg) {
  return OK;
}

/* Process AECP command get stream info */
int avb_process_aecp_cmd_get_stream_info(struct avb_state_s *state, aecp_message_s *msg) {
  return OK;
}

/* Process AECP command get counters */
int avb_process_aecp_cmd_get_counters(struct avb_state_s *state, aecp_message_s *msg) {
  return OK;
}

/* Process AECP response register unsol notif */
int avb_process_aecp_rsp_register_unsol_notif(struct avb_state_s *state, aecp_message_s *msg) {
  return OK;
}

/* Process AECP response entity available */
int avb_process_aecp_rsp_entity_available(struct avb_state_s *state, aecp_message_s *msg) {
  return OK;
}

/* Process AECP response controller available */
int avb_process_aecp_rsp_controller_available(struct avb_state_s *state, aecp_message_s *msg) {
  return OK;
}

/* Process AECP response get stream info */
int avb_process_aecp_rsp_get_stream_info(struct avb_state_s *state, aecp_message_s *msg) {
  return OK;
}

/* Process AECP response get counters */
int avb_process_aecp_rsp_get_counters(struct avb_state_s *state, aecp_message_s *msg) {
  return OK;
}

/* Process received ATDECC ACMP message */
int avb_process_acmp(struct avb_state_s *state, acmp_message_s *msg) {

  if (msg->subtype == acmp_msg_type_connect_rx_command) {

    int ret;
    struct timespec ts;
    size_t body_size = 56; // ACMP Message body length
    // Check if the listener entity ID matches the own entity ID
    if (memcmp(msg->listener_entity_id, state->own_entity.summary.entity_id, 8) != 0) {
      avbinfo("Ignoring ACMP Connect RX Command for different listener");
      return OK;
    }

    // find talker address from talker entity id
    eth_addr_t dest_addr;
    int index = avb_find_entity(state, &msg->talker_entity_id, true);
    memcpy(dest_addr, state->talkers[index].mac_addr, ETH_ADDR_LEN);

    // send connect tx cmd to talker
    acmp_message_s connect_tx_cmd;
    memcpy(&connect_tx_cmd, msg, sizeof(acmp_message_s));
    uint16_t msg_len = 4 + body_size; // header + body
    ret = avb_net_send_to(state, ethertype_avtp, &connect_tx_cmd, msg_len, &ts, &dest_addr);
    if (ret < 0) {
      avberr("send ADP Entity Available failed: %d", errno);
      return ret;
    }

    // send get stream info to talker


    // when receive stream info, send connect rx response to controller

  }
  return OK;
}

/* Process ACMP connect rx command */
int avb_process_acmp_connect_rx_command(struct avb_state_s *state, acmp_message_s *msg) {
  return OK;
}

/* Process ACMP connect rx response */
int avb_process_acmp_connect_rx_response(struct avb_state_s *state, acmp_message_s *msg) {
  return OK;
}

/* Process ACMP disconnect rx command */
int avb_process_acmp_disconnect_rx_command(struct avb_state_s *state, acmp_message_s *msg) {
  return OK;
}

/* Process ACMP disconnect rx response */
int avb_process_acmp_disconnect_rx_response(struct avb_state_s *state, acmp_message_s *msg) {
  return OK;
}

/* Process ACMP connect tx command */
int avb_process_acmp_connect_tx_command(struct avb_state_s *state, acmp_message_s *msg) {
  return OK;
}

/* Process ACMP connect tx response */
int avb_process_acmp_connect_tx_response(struct avb_state_s *state, acmp_message_s *msg) {
  return OK;
}

/* Process ACMP disconnect tx command */
int avb_process_acmp_disconnect_tx_command(struct avb_state_s *state, acmp_message_s *msg) {
  return OK;
}

/* Process ACMP disconnect tx response */
int avb_process_acmp_disconnect_tx_response(struct avb_state_s *state, acmp_message_s *msg) {
  return OK;
}

/* Add an entity to the known list of talkers or listeners;
 * if the list is full, replaces oldest item; returns the index of the entity
*/
int avb_add_entity(struct avb_state_s *state, 
                    avb_entity_summary_s *entity, 
                    eth_addr_t *entity_addr, 
                    bool is_talker) {
  int index = NOT_FOUND;
  if (is_talker) {
    /* if the talker is not already known, add it to the list */
    index = avb_find_entity(state, &entity->entity_id, true);
    if (index == NOT_FOUND) {
      avb_talker_s talker;
      memset(&talker, 0, sizeof(avb_talker_s));
      memcpy(talker.entity_id, entity->entity_id, UNIQUE_ID_LEN);
      memcpy(talker.model_id, entity->entity_id, UNIQUE_ID_LEN);
      memcpy(talker.mac_addr, entity_addr, ETH_ADDR_LEN);
      /* if talker list is not full, add the talker to the list */
      size_t num_talkers = sizeof(state->talkers) / sizeof(state->talkers[0]);
      if (num_talkers < AVB_NUM_TALKERS) {
        state->talkers[num_talkers] = talker;
        return num_talkers;
      }
      /* if talker list is full, then replace the oldest talker */
      else {
        memmove(&state->talkers[0], &state->talkers[1], (num_talkers - 1) * sizeof(avb_talker_s));
        state->talkers[num_talkers - 1] = talker;
        return num_talkers - 1;
      }
    }
  }
  else {
    /* if the listener is not already known, add it to the list */
    int index = avb_find_entity(state, &entity->entity_id, false);
    if (index == NOT_FOUND) {
      avb_listener_s listener;
      memset(&listener, 0, sizeof(avb_listener_s));
      memcpy(listener.entity_id, entity->entity_id, UNIQUE_ID_LEN);
      memcpy(listener.mac_addr, entity_addr, ETH_ADDR_LEN);
      /* if listener list is not full, add the listener to the list */
      size_t num_listeners = sizeof(state->listeners) / sizeof(state->listeners[0]);
      if (num_listeners < AVB_NUM_LISTENERS) {
        state->listeners[num_listeners] = listener;
        return num_listeners;
      }
      /* if listener list is full, then replace the oldest listener */
      else {
        memmove(&state->listeners[0], &state->listeners[1], (num_listeners - 1) * sizeof(avb_listener_s));
        state->listeners[num_listeners - 1] = listener;
        return num_listeners - 1;
      }
    }
  }
  return index;
}

/* Find an entity in the known list of talkers or listeners */
int avb_find_entity(struct avb_state_s *state, unique_id_t *entity_id, bool is_talker) {
  if (is_talker) {
    for (int i = 0; i < AVB_NUM_TALKERS; i++) {
      if (memcmp(state->talkers[i].entity_id, entity_id, UNIQUE_ID_LEN) == 0) {
        return i;
      }
    }
  }
  else {
    for (int i = 0; i < AVB_NUM_LISTENERS; i++) {
      if (memcmp(state->listeners[i].entity_id, entity_id, UNIQUE_ID_LEN) == 0) {
        return i;
      }
    }
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
