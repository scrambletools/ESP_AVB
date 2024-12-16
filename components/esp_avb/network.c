#include "avb.h"

// Ethernet driver handle
static esp_eth_handle_t eth_handle = NULL;

// Define logging tag
static const char *TAG = "NET";

// Managed mac addresses
static uint8_t local_eth_mac_addr[ETH_ADDR_LEN] = { 0 };
static uint8_t local_wifi_mac_addr[ETH_ADDR_LEN] = { 0 };
static uint8_t controller_mac_addr[ETH_ADDR_LEN] = { 0 };
static uint8_t listener_mac_addr[ETH_ADDR_LEN] = { 0 }; // remote device
static uint8_t talker_mac_addr[ETH_ADDR_LEN] = { 0 }; // remote device

// Set the Ethernet driver handle
// void set_eth_handle(esp_eth_handle_t handle) {
//     eth_handle = handle;

//     // Set the local Ethernet mac address
//     esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, local_eth_mac_addr);
// }

void avb_create_eth_frame(uint8_t *eth_frame, 
                                eth_addr_t *dest_addr, 
                                struct avb_state_s *state, 
                                ethertype_t ethertype, 
                                void *msg, 
                                uint16_t msg_len) {
  struct eth_hdr eth_hdr = {
    .type = htons(ethertype)
  };
  memcpy(&eth_hdr.dest.addr, dest_addr, ETH_ADDR_LEN);
  memcpy(&eth_hdr.src.addr, state->internal_mac_addr, ETH_ADDR_LEN);
  memcpy(eth_frame, &eth_hdr, sizeof(eth_hdr));
  memcpy(eth_frame + sizeof(eth_hdr), msg, msg_len);
}

int avb_net_send(struct avb_state_s *state, int l2if, void *msg, uint16_t msg_len, struct timespec *ts)
{
  uint8_t eth_frame[msg_len + ETH_HEADER_LEN];
  eth_addr_t *dest_addr = &BCAST_MAC_ADDR;
  ethertype_t ethertype = ethertype_avtp;
  avb_create_eth_frame(eth_frame, dest_addr, state, ethertype, msg, msg_len);

  // wrap "Info Records Buffer" into union to ensure proper alignment of data (this is typically needed when
  // accessing double word variables or structs containing double word variables)
  union {
      uint8_t info_recs_buff[L2TAP_IREC_SPACE(sizeof(struct timespec))];
      l2tap_irec_hdr_t align;
  } u;

  l2tap_extended_buff_t msg_ext_buff;

  msg_ext_buff.info_recs_len = sizeof(u.info_recs_buff);
  msg_ext_buff.info_recs_buff = u.info_recs_buff;
  msg_ext_buff.buff = eth_frame;
  msg_ext_buff.buff_len = sizeof(eth_frame);

  l2tap_irec_hdr_t *ts_info = L2TAP_IREC_FIRST(&msg_ext_buff);
  ts_info->len = L2TAP_IREC_LEN(sizeof(struct timespec));
  ts_info->type = L2TAP_IREC_TIME_STAMP;

  int ret = write(l2if, &msg_ext_buff, 0);

  // check if write was successful, ts exists and ts_info is valid
  if (ret > 0 && ts && ts_info->type == L2TAP_IREC_TIME_STAMP)
    {
      *ts = *(struct timespec *)ts_info->data;
      ESP_LOGD("avb_net_send", "ts is %lld.%09ld", (long long)ts->tv_sec, ts->tv_nsec);
    }
  return ret;
}

int avb_net_recv(struct avb_state_s *state, int l2if, void *msg, uint16_t msg_len, struct timespec *ts)
{
  uint8_t eth_frame[msg_len + ETH_HEADER_LEN];

  // wrap "Info Records Buffer" into union to ensure proper alignment of data (this is typically needed when
  // accessing double word variables or structs containing double word variables)
  union {
      uint8_t info_recs_buff[L2TAP_IREC_SPACE(sizeof(struct timespec))];
      l2tap_irec_hdr_t align;
  } u;
  l2tap_extended_buff_t msg_ext_buff;

  msg_ext_buff.info_recs_len = sizeof(u.info_recs_buff);
  msg_ext_buff.info_recs_buff = u.info_recs_buff;
  msg_ext_buff.buff = eth_frame;
  msg_ext_buff.buff_len = sizeof(eth_frame);

  l2tap_irec_hdr_t *ts_info = L2TAP_IREC_FIRST(&msg_ext_buff);
  ts_info->len = L2TAP_IREC_LEN(sizeof(struct timespec));
  ts_info->type = L2TAP_IREC_TIME_STAMP;

  int ret = read(l2if, &msg_ext_buff, 0);

  // check if read was successful, ts exists and ts_info is valid
  if (ret > 0 && ts && ts_info->type == L2TAP_IREC_TIME_STAMP) {
    *ts = *(struct timespec *)ts_info->data;
    ESP_LOGD("avb_net_recv", "ts is %lld.%09ld", (long long)ts->tv_sec, ts->tv_nsec);
  }

  memcpy(msg, &eth_frame[ETH_HEADER_LEN], ret);
  return ret;
}

// Print out the frame (format: 0 for short (default), 1 for long format)
void print_frame_of_type(avb_frame_type_t type, eth_frame_t *frame, int format) {
    if (frame->payload_size < 2) {
        ESP_LOGI(TAG, "Can't print frame, payload is too small: %d", frame->payload_size);
    }
    else {
        switch (type) {
            case avb_frame_adp_entity_available ... avb_frame_acmp_get_rx_state_response:
                print_atdecc_frame(type, frame, format);
                break;
            case avb_frame_avtp_stream ... avb_frame_mvrp_vlan_identifier:
                print_avtp_frame(type, frame, format);
                break;
            default:
                ESP_LOGI(TAG, "Can't print frame of unknown frame type: 0x%x", type);
        }
    }
}

// Print out the frame; get type from frame
void print_frame(eth_frame_t *frame, int format) {
    print_frame_of_type(frame->frame_type, frame, format);
}
