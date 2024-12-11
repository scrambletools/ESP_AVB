#ifndef ESP_AVB_SIMPLE_TALKER_INCLUDE_GPTP_H_
#define ESP_AVB_SIMPLE_TALKER_INCLUDE_GPTP_H_

#include <stdlib.h>
#include <esp_log.h>
#include "utils.h"
#include "config.h"

// gPTP message types and their values
typedef enum {
	gptp_message_type_sync = 0x0,
	gptp_message_type_pdelay_request = 0x2,
	gptp_message_type_pdelay_response = 0x3,
	gptp_message_type_follow_up = 0x8,
	gptp_message_type_pdelay_response_follow_up = 0xa,
	gptp_message_type_announce = 0xb
} gptp_message_type_t;

// gPTP state information structure
typedef struct {
	bool gm_is_present; // a gm is present (local or remote)
	bool gm_is_local; // local host is currently gm
	uint64_t mean_pdelay; // (in nsec) calculated from pdelay list
	uint64_t mean_offset; // (in nsec) calculated from sync list
	double mean_rate_ratio; // calculated from sync list
	double mean_freq_change; // (in ppm) calculated from sync list
} gptp_state_info_t;

// Grandmaster (GM) information structure
typedef struct {
	uint8_t priority_1;
    uint8_t clock_class;
    uint8_t clock_accuracy;
    uint16_t clock_variance;
	uint8_t priority_2;
	uint64_t clock_id;
	uint16_t steps_removed;
	uint8_t time_source;
	uint16_t port_id;
	uint16_t announce_sequence_id; // from most recent announce message
	uint16_t announce_period; // from most recent announce message
	struct timeval timestamp_announce[CONFIG_ANNOUNCE_LIST_SIZE];
	uint64_t path_trace_ids[CONFIG_PATH_TRACE_LIMIT];
} gptp_gm_info_t;

// Peer delay information structure
typedef struct {
	uint16_t sequence_id; // from most recent pdelay request sent from local
	uint16_t period; // from most recent pdelay request sent from local
    struct timeval timestamp_request_transmission; // from local
	struct timeval timestamp_request_receipt; // by peer
    struct timeval timestamp_response_transmission; // from peer
	struct timeval timestamp_response_receipt; // by local
	uint64_t calculated_pdelay; // in nsec; ((req_r - req_t) + (res_r - res_t)) / 2
} gptp_pdelay_info_t;

// Sync information structure
typedef struct {
	uint16_t sequence_id; // from most recent sync message
	uint16_t period; // from most recent sync message
    struct timeval timestamp_sync_transmission; // from master sync
	struct timeval timestamp_sync_receipt; // by local
	double scaled_rate_offset; // from most recent sync message
	double scaled_freq_change; // from most recent sync message
	uint64_t calculated_offset; // in nsec; ((sync_r - sync_t) - (req_r - req_t)) / 2
} gptp_sync_info_t;

// Functions
gptp_state_info_t* get_gptp_state(void);
gptp_gm_info_t* get_current_gm(void);
gptp_pdelay_info_t* get_current_pdelay(void);
gptp_sync_info_t* get_current_sync(void);
gptp_gm_info_t* get_gm_list(void);
gptp_pdelay_info_t* get_pdelay_list(void);
gptp_sync_info_t* get_sync_list(void);
esp_err_t add_to_gm_list(gptp_gm_info_t *gm_to_add);
esp_err_t add_to_pdelay_list(gptp_pdelay_info_t *pdelay_to_add);
esp_err_t add_to_sync_list(gptp_sync_info_t *sync_to_add);
esp_err_t remove_from_gm_list(gptp_gm_info_t *gm_to_remove);
esp_err_t remove_from_pdelay_list(gptp_pdelay_info_t *pdelay_to_remove);
esp_err_t remove_from_sync_list(gptp_sync_info_t *sync_to_remove);
int find_gm_index(uint64_t clock_id);
int find_pdelay_index(uint16_t sequence_id);
int find_sync_index(uint16_t sequence_id);
bool evaluate_bmca(gptp_gm_info_t *gm_a, gptp_gm_info_t *gm_b);
uint64_t calculate_pdelay(gptp_pdelay_info_t *pdelay_info);
uint64_t calculate_offset(gptp_sync_info_t *sync_info, gptp_pdelay_info_t *pdelay_info);
void append_gptpdu(avb_frame_type_t type, eth_frame_t *frame);
avb_frame_type_t detect_gptp_frame_type(eth_frame_t *frame, ssize_t size);
void print_gptp_frame(avb_frame_type_t type, eth_frame_t *frame, int format);

#endif /* ESP_AVB_SIMPLE_TALKER_INCLUDE_GPTP_H_ */