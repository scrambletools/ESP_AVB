#ifndef ESP_AVB_SIMPLE_TALKER_INCLUDE_ATDECC_H_
#define ESP_AVB_SIMPLE_TALKER_INCLUDE_ATDECC_H_

#include <stdlib.h>
#include <esp_log.h>
#include "avtp.h"
#include "utils.h"
#include "config.h"

#define ATDECC_TEMPLATE_MAX_SIZE 500

// ADP message types in enumerated order (1722.1 Clause 6.2)
typedef enum {
	adp_message_type_entity_available,
	adp_message_type_entity_departing,
	adp_message_type_entity_discover
} adp_message_type_t;

// AECP message types (subset) in enumerated order (1722.1 Clause 9)
typedef enum {
	aecp_message_type_aem_command,
	aecp_message_type_aem_response
} aecp_message_type_t;

// AECP command codes (subset) and their values (1722.1 Clause 7.4)
typedef enum {
	aecp_command_code_acquire_entity = 0x0000,
	aecp_command_code_lock_entity = 0x0001,
	aecp_command_code_entity_available = 0x0002,
	aecp_command_code_controller_available = 0x0003,
	aecp_command_code_read_descriptor = 0x0004
} aecp_command_code_t;

// ACMP message types (subset) in enumerated order (1722.1 Clause 8.2)
typedef enum {
	acmp_message_type_connect_tx_command,
	acmp_message_type_connect_tx_response,
	acmp_message_type_disconnect_tx_command,
  acmp_message_type_disconnect_tx_response,
	acmp_message_type_get_tx_state_command,
	acmp_message_type_get_tx_state_response,
	acmp_message_type_connect_rx_command,
  acmp_message_type_connect_rx_response,
  acmp_message_type_disconnect_rx_command,
	acmp_message_type_disconnect_rx_response,
	acmp_message_type_get_rx_state_command,
	acmp_message_type_get_rx_state_response
} acmp_message_type_t;

// Functions
void append_adpdu(adp_message_type_t type, eth_frame_t *frame);
void append_aecpdu(aecp_message_type_t type, eth_frame_t *frame);
void append_acmpdu(acmp_message_type_t type, eth_frame_t *frame);
avb_frame_type_t detect_atdecc_frame_type(eth_frame_t *frame, ssize_t size);
void print_atdecc_frame(avb_frame_type_t type, eth_frame_t *frame, int format);

#endif /* ESP_AVB_SIMPLE_TALKER_INCLUDE_ATDECC_H_ */