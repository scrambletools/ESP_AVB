#ifndef ESP_AVB_SIMPLE_TALKER_INCLUDE_UTILS_H_
#define ESP_AVB_SIMPLE_TALKER_INCLUDE_UTILS_H_

#include <esp_log.h>
#include <esp_eth.h>
#include <lwip/prot/ethernet.h> // Ethernet header
#include <lwip/prot/ieee.h>
#include <arpa/inet.h> // ntohs
#include <string.h>
#include <math.h>

#define AVTP_MAX_PAYLOAD_LENGTH 1486
#define PRINT_SUMMARY 0
#define PRINT_VERBOSE 1

// bitswap helpers
#define bitswap_64(x)   ((uint64_t)                 \
     ((((x) & 0xff00000000000000ull) >> 56) |       \
      (((x) & 0x00ff000000000000ull) >> 40) |       \
      (((x) & 0x0000ff0000000000ull) >> 24) |       \
      (((x) & 0x000000ff00000000ull) >>  8) |       \
      (((x) & 0x00000000ff000000ull) <<  8) |       \
      (((x) & 0x0000000000ff0000ull) << 24) |       \
      (((x) & 0x000000000000ff00ull) << 40) |       \
      (((x) & 0x00000000000000ffull) << 56)))

#define bitswap_32(x)    ((uint32_t)                \
    ((((x) & 0xff000000) >> 24) |                   \
     (((x) & 0x00ff0000) >>  8) |                   \
     (((x) & 0x0000ff00) <<  8) |                   \
     (((x) & 0x000000ff) << 24)))

#define bitswap_16(x)   ((uint16_t)                 \
    ((((x) & 0xff00) >> 8) |                        \
     (((x) & 0x00ff) << 8)))

// Commonly used mac addresses
static const uint8_t bcast_mac_addr[6] = { 0x91, 0xe0, 0xf0, 0x01, 0x00, 0x00 }; // adp
static const uint8_t maap_mcast_mac_addr[6] = { 0x91, 0xe0, 0xf0, 0x00, 0xff, 0x00 }; // maap
static const uint8_t spantree_mac_addr[6] = { 0x01, 0x80, 0xc2, 0x00, 0x00, 0x21 }; // 
static const uint8_t lldp_mcast_mac_addr[6] = { 0x01, 0x80, 0xc2, 0x00, 0x00, 0x0e }; // msrp,mvrp

// Ethertypes
typedef enum {
	ethertype_msrp = 0x22ea,
	ethertype_avtp = 0x22f0,
	ethertype_mvrp = 0x88f5,
	ethertype_gptp = 0x88f7
} ethertype_t;

// Useful when doing things like printing frames
typedef enum {
	avb_frame_gptp_announce,
	avb_frame_gptp_sync,
	avb_frame_gptp_follow_up,
	avb_frame_gptp_pdelay_request,
	avb_frame_gptp_pdelay_response,
	avb_frame_gptp_pdelay_response_follow_up,
	avb_frame_adp_entity_available,
	avb_frame_adp_entity_departing,
	avb_frame_adp_entity_discover,
	avb_frame_aecp_command_acquire_entity,
	avb_frame_aecp_response_acquire_entity,
	avb_frame_aecp_command_lock_entity,
	avb_frame_aecp_response_lock_entity,
	avb_frame_aecp_command_entity_available,
	avb_frame_aecp_response_entity_available,
	avb_frame_aecp_command_controller_available,
	avb_frame_aecp_response_controller_available,
	avb_frame_aecp_command_read_descriptor, // currently not distinguishing between descriptor types
	avb_frame_aecp_response_read_entity, // responses are specific to descriptor type; currently only entity supported
	avb_frame_acmp_connect_tx_command,
	avb_frame_acmp_connect_tx_response,
	avb_frame_acmp_disconnect_tx_command,
  avb_frame_acmp_disconnect_tx_response,
	avb_frame_acmp_get_tx_state_command,
	avb_frame_acmp_get_tx_state_response,
	avb_frame_acmp_connect_rx_command,
  avb_frame_acmp_connect_rx_response,
  avb_frame_acmp_disconnect_rx_command,
	avb_frame_acmp_disconnect_rx_response,
	avb_frame_acmp_get_rx_state_command,
	avb_frame_acmp_get_rx_state_response,
	avb_frame_avtp_stream,
	avb_frame_maap_announce,
	avb_frame_msrp_talker_advertise,
	avb_frame_msrp_talker_failed,
	avb_frame_msrp_listener,
	avb_frame_msrp_domain,
	avb_frame_mvrp_vlan_identifier,
	avb_frame_unknown
} avb_frame_type_t;

// Basic ethernet frame
typedef struct {
    struct eth_hdr header;
    uint8_t payload[AVTP_MAX_PAYLOAD_LENGTH];
		ssize_t payload_size;
		avb_frame_type_t frame_type;
		struct timeval time_sent;
		struct timeval time_received;
} eth_frame_t;

// Functions
const char* get_ethertype_name(ethertype_t ethertype);
const char* get_frame_type_name(avb_frame_type_t type);
uint64_t octets_to_uint(const uint8_t *buffer, size_t size);
void octets_to_timeval(const uint8_t *buffer, struct timeval *tv);
void reverse_octets(uint8_t *buffer, size_t size);
void octets_to_binary_string(const uint8_t *buffer, size_t size, char *bit_string);
void int_to_octets(void *value, uint8_t *buffer, size_t size);
void int_to_binary_string(uint64_t value, int num_bits, char *bit_string, bool reverse_order);
void int_to_binary_string_64(uint64_t value, char *bit_string);
void int_to_binary_string_32(uint32_t value, char *bit_string);
void int_to_binary_string_16(uint16_t value, char *bit_string);
uint64_t reverse_endianness(uint64_t value, int num_bits);
uint64_t reverse_endianness_64(uint64_t value);
uint32_t reverse_endianness_32(uint32_t value);
uint16_t reverse_endianness_16(uint16_t value);
char* mac_address_to_string(uint8_t *address);
void timeval_to_octets(struct timeval *tv, uint8_t *buffer_sec, uint8_t *buffer_nsec);
void timeval_add(struct timeval *result, struct timeval *a, struct timeval *b);
void timeval_subtract(struct timeval *result, struct timeval *a, struct timeval *b);
struct timeval timeval_divide_by_int(struct timeval tv, int divisor);
int compare_timeval(struct timeval t1, struct timeval t2);
esp_err_t add_to_list_front(void *item_to_add, void *list, size_t item_size, size_t list_size);
int8_t msec_to_log_period(uint16_t msec_period);
uint16_t log_period_to_msec(int8_t log_period);
double scaled_to_ppm(int32_t scaled_value);
int32_t ppm_to_scaled(double ppm_value);

#endif /* ESP_AVB_SIMPLE_TALKER_INCLUDE_UTILS_H_ */