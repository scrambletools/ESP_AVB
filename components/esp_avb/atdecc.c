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
  memset(&msg, 0, sizeof(msg));

  // Populate the message
  msg.subtype = avtp_subtype_adp;
  msg.h_ver_msgtype = 0x00;
  uint8_t validtime_controldatalen[2] = {0x40, 0x38};
  memcpy(msg.validtime_controldatalen, validtime_controldatalen, 2);
  memcpy(&msg.entity, &state->own_entity.summary, sizeof(avb_entity_summary_s));
  memcpy(msg.gptp_gm_id, state->ptp_status.clock_source_info.gm_id, 8);
  msg.gptp_domain_num = 0;
  memcpy(msg.association_id, &EMPTY_ID, 8);

  size_t body = 56;
  uint16_t msg_len = 4 + body + 8; // header + body
  
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
int avb_process_adp(struct avb_state_s *state, adp_message_s *msg) {
  // TODO: Implement processing
  return OK;
}

/* Process received ATDECC AECP message */
int avb_process_aecp(struct avb_state_s *state, aecp_message_s *msg) {
  // TODO: Implement processing
  return OK;
}

/* Process received ATDECC ACMP message */
int avb_process_acmp(struct avb_state_s *state, acmp_message_s *msg) {
  // TODO: Implement processing
  return OK;
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
