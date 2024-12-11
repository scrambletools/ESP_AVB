/*
 * Copyright 2024 Scramble Tools
 * License: MIT
 *
 * ESP_AVB Component
 *
 * This component provides an implementation of an AVB talker and listener.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <sys/l2if.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <esp_eth_driver.h>
#include <esp_vfs_l2tap.h>
#include <semaphore.h>
#include <esp_log.h>
#include <esp_err.h>
#include <lwip/prot/ethernet.h> // Ethernet headers
#include <esp_eth_time.h>
#include <ptpd.h>
#include "esp_avb.h" // External API
#include "avb.h" // Internal API

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP_PTP

/* Ethertypes */
#define ETHERTYPE_AVTP 0x22f0
#define ETHERTYPE_MSRP 0x22ea
#define ETHERTYPE_MVRP 0x88f5

/* AVTP Message types */
#define AVTP_MSGTYPE_ANNOUNCE 0x00
#define AVTP_MSGTYPE_SYNC 0x01

/* MSRP Message types */
#define MSRP_MSGTYPE_SYNC 0x00
#define MSRP_MSGTYPE_ANNOUNCE 0x01
#define MSRP_MSGTYPE_FOLLOW_UP 0x02

/* MVRP Message types */
#define MVRP_MSGTYPE_SYNC 0x00
#define MVRP_MSGTYPE_ANNOUNCE 0x01
#define MVRP_MSGTYPE_FOLLOW_UP 0x02

/* Error and OK definitions */

#define ERROR ESP_FAIL
#define OK ESP_OK

#define UNUSED (void)

#define MSEC_PER_SEC 1000
#define NSEC_PER_USEC 1000
#define NSEC_PER_MSEC 1000000ll
#define NSEC_PER_SEC 1000000000ll

#define clock_timespec_subtract(ts1, ts2, ts3) timespecsub(ts1, ts2, ts3)
#define clock_timespec_add(ts1, ts2, ts3) timespecadd(ts1, ts2, ts3)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Carrier structure for querying AVB status */

struct avb_statusreq_s
{
  sem_t *done;
  struct avb_status_s *dest;
};

/* Main AVB state storage */

struct avb_state_s
{
  /* Request for AVB task to stop or report status */

  bool stop;
  struct avb_statusreq_s status_req;

  uint8_t internal_mac_addr[ETH_ADDR_LEN];
  int avb_l2if;

  /* Our own entity */
  struct avb_entity_s own_entity;

  /* Latest received packet and its timestamp (CLOCK_REALTIME) */
  struct timespec rxtime;
  uint8_t rxbuf[500];
  avtp_msgbuf avtp_rxbuf;
  struct timespec msrp_rxtime;
  msrp_msgbuf msrp_rxbuf;
  struct timespec mvrp_rxtime;
  mvrp_msgbuf mvrp_rxbuf;
};

#define AVB_POLL_INTERVAL_MS 1000

/* AVB debug messages are enabled by either CONFIG_DEBUG_NET_INFO
 * or separately by CONFIG_ESP_AVB_DEBUG. This simplifies
 * debugging without having excessive amount of logging from net.
 */

static const char *TAG = "avb";
#define avbinfo(format, ...) ESP_LOGI(TAG, format, ##__VA_ARGS__)
#define avbwarn(format, ...) ESP_LOGW(TAG, format, ##__VA_ARGS__)
#define avberr(format, ...) ESP_LOGE(TAG, format, ##__VA_ARGS__)

static struct avb_state_s *s_state;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void avb_create_eth_frame(struct avb_state_s *state, uint8_t *eth_frame, void *msg, uint16_t msg_len)
{
  struct eth_hdr eth_hdr = {
    .type = htons(ETHERTYPE_AVTP) // TODO: Change to MSRP or MVRP
  };
  memcpy(&eth_hdr.dest.addr, LLDP_MULTICAST_ADDR, ETH_ADDR_LEN);
  memcpy(&eth_hdr.src.addr, state->internal_mac_addr, ETH_ADDR_LEN);
  memcpy(eth_frame, &eth_hdr, sizeof(eth_hdr));
  memcpy(eth_frame + sizeof(eth_hdr), msg, msg_len);
}

static int avb_net_send(FAR struct avb_state_s *state, void *msg, uint16_t msg_len, struct timespec *ts)
{
  uint8_t eth_frame[msg_len + ETH_HEADER_LEN];
  avb_create_eth_frame(state, eth_frame, msg, msg_len);

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

  int ret = write(state->avb_l2if, &msg_ext_buff, 0);

  // check if write was successful, ts exists and ts_info is valid
  if (ret > 0 && ts && ts_info->type == L2TAP_IREC_TIME_STAMP)
    {
      *ts = *(struct timespec *)ts_info->data;
      ESP_LOGD("net_send", "ts is %lld.%09ld", (long long)ts->tv_sec, ts->tv_nsec);
    }

  return ret;
}

static int ptp_net_recv(FAR struct avb_state_s *state, void *msg, uint16_t msg_len, struct timespec *ts)
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

  int ret = read(state->avb_l2if, &msg_ext_buff, 0);

  // check if read was successful, ts exists and ts_info is valid
  if (ret > 0 && ts && ts_info->type == L2TAP_IREC_TIME_STAMP)
    {
      *ts = *(struct timespec *)ts_info->data;
      ESP_LOGD("net_recv", "ts is %lld.%09ld", (long long)ts->tv_sec, ts->tv_nsec);
    }

  memcpy(msg, &eth_frame[ETH_HEADER_LEN], ret);

  return ret;
}

/* Initialize AVB state and create L2TAP FD - FIXME 3 FDs needed */
static int avb_initialize_state(FAR struct avb_state_s *state,
                                FAR const char *interface)
{
  state->avb_l2if = open("/dev/net/tap", 0);
  if (state->avb_l2if < 0)
  {
      ptperr("Failed to create tx l2if: %d\n", errno);
      return ERROR;
  }

  // Set Ethernet interface on which to get raw frames
  if (ioctl(state->avb_l2if, L2TAP_S_INTF_DEVICE, interface) < 0)
  {
    ptperr("failed to set network interface at l2if: %d\n", errno);
    return ERROR;
  }

  // Set the Ethertype filter (frames with this type will be available through the state->tx_l2if)
  uint16_t ethertype_filter = ETHERTYPE_AVTP; // TODO: Change to MSRP or MVRP
  if (ioctl(state->avb_l2if, L2TAP_S_RCV_FILTER, &ethertype_filter) < 0)
  {
    ptperr("failed to set Ethertype filter: %d\n", errno);
    return ERROR;
  }
  // Enable time stamping in driver
  esp_eth_handle_t eth_handle;
  if (ioctl(state->avb_l2if, L2TAP_G_DEVICE_DRV_HNDL, &eth_handle) < 0)
  {
    ptperr("failed to get l2if eth_handle %d\n", errno);
    return ERROR;
  }
  // esp_eth_clock_cfg_t clk_cfg = {
  //   .eth_hndl = eth_handle,
  // };
  // esp_eth_clock_init(CLOCK_PTP_SYSTEM, &clk_cfg);

  // Enable time stamping in L2TAP
  if(ioctl(state->avb_l2if, L2TAP_S_TIMESTAMP_EN) < 0)
  {
    ptperr("failed to enable time stamping in L2TAP FD: %d\n", errno);
    return ERROR;
  }

  // get HW address
  esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, &state->internal_mac_addr);

  s_state = state;
  return OK;
}

/* Destroy l2ifs */

static int avb_destroy_state(struct avb_state_s *state)
{
  if (state->avb_l2if > 0)
  {
    close(state->avb_l2if);
    state->avb_l2if = -1;
  }
  return OK;
}

/* Send MSRP Domain message */

static int avb_send_msrp_domain(struct avb_state_s *state)
{
  struct avb_msrp_domain_s msg;
  struct timespec ts;
  int ret;

  memset(&msg, 0, sizeof(msg));
  msg.header.messagetype = AVB_MSGTYPE_MSRP_DOMAIN;
  
  ret = avb_net_send(state, &msg, sizeof(msg), &ts);
  if (ret < 0)
    {
      avberr("send failed: %d", errno);
    }
  else
    {
      avbinfo("Sent MSRP Domain message");
    }
  return ret;
}

/* Check if we need to send periodic messages */
static int avb_periodic_send(struct avb_state_s *state)
{
  struct timespec time_now;
  struct timespec delta;

  clock_gettime(CLOCK_MONOTONIC, &time_now);

  clock_timespec_subtract(&time_now,
    &state->last_transmitted_msrp_domain, &delta);
  if (timespec_to_ms(&delta) > CONFIG_ESP_AVB_MSRP_DOMAIN_INTERVAL_MSEC)
    {
      state->last_transmitted_msrp_domain = time_now;
      avb_send_msrp_domain(state);
    }

  clock_timespec_subtract(&time_now,
    &state->last_transmitted_mvrp_vlan_identifier, &delta);
  if (timespec_to_ms(&delta) > CONFIG_ESP_AVB_MVRP_VLAN_IDENTIFIER_INTERVAL_MSEC)
    {
      state->last_transmitted_mvrp_vlan_identifier = time_now;
      avb_send_mvrp_vlan_identifier(state);
    }
  return OK;
}

/* Process received PTP sync packet */
static int avb_process_msrp_domain(struct avb_state_s *state,
                            FAR struct ptp_sync_s *msg)
{
  // TODO: Implement sync packet processing
  return OK;
}

/* Determine received packet type and process it */

static int avb_process_rx_packet(struct avb_state_s *state,
                                 ssize_t length)
{
  if (length < sizeof(avtp_header_s)) // TODO: Change to MSRP or MVRP
    {
      avbwarn("Ignoring invalid message, length only %d bytes\n",
              (int)length);
      return OK;
    }

  clock_gettime(CLOCK_MONOTONIC, &state->last_received_multicast);

  /* Rout the packet to the appropriate handler */

  switch (state->rxbuf.header.messagetype & PTP_MSGTYPE_MASK)
  {
    case AVTP_MSGTYPE_ANNOUNCE:
      ptpinfo("Got MSRP Domain message, seq %ld\n",
              (long)ptp_get_sequence(&state->rxbuf.header));
      return avb_process_msrp_domain(state, &state->rxbuf.announce);

    case PTP_MSGTYPE_SYNC:
      ptpinfo("Got MVRP VLAN Identifier message, seq %ld\n",
              (long)ptp_get_sequence(&state->rxbuf.header));
      if (!state->selected_source_valid) { return OK; } // ignore if operating as a server
      return avb_process_mvrp_vlan_identifier(state, &state->rxbuf.sync);

    default:
      ptpinfo("Ignoring unknown message type: 0x%02x\n",
              state->rxbuf.header.messagetype);
      return OK;
  }
}

/* Process status information request */
static void avb_process_statusreq(struct avb_state_s *state)
{
  struct avb_status_s *status;

  if (!state->status_req.dest)
    {
      return; /* No active request */
    }

  status = state->status_req.dest;

  /* TODO: Copy required info to the state structure */

  /* Post semaphore to inform that we are done */
  if (state->status_req.done)
    {
      sem_post(state->status_req.done);
    }

  state->status_req.done = NULL;
  state->status_req.dest = NULL;
}

/* Main AVB task */
static void avb_task(void *task_param)
{
  const char *interface = "eth0";
  struct avb_state_s *state;
  struct pollfd pollfds[1]; // everything is received over one fd
  int ret;
  state = calloc(1, sizeof(struct avb_state_s));

  if (task_param != NULL)
    {
      interface = task_param;
    }
  if (avb_initialize_state(state, interface) != OK)
    {
      avberr("Failed to initialize AVB state, exiting\n");
      avb_destroy_state(state);
      free(state);
      goto err;
    }

  pollfds[0].events = POLLIN;
  pollfds[0].fd = state->avb_l2if;

  while (!state->stop)
    {
      // reset any values that need to be reset
      pollfds[0].revents = 0;

      /* Wait for a message */
	    ret = poll(pollfds, 1, AVB_POLL_INTERVAL_MS);
      if (pollfds[0].revents)
        {
          /* Receive a message */
          ret = avb_net_recv(state, &state->rxbuf, sizeof(state->rxbuf), &state->rxtime);
          if (ret > 0)
            {
              /* Process the received message */
              avb_process_rx_packet(state, ret);
            }
        }

      // Send periodic messages such as announcing entity available, etc
      avb_periodic_send(state);

      // do any validation checking for timed out connections, etc

      // Process status requests
      avb_process_statusreq(state);

    } // while (!state->stop)

  ptp_destroy_state(state);
  free(state);

err:
  s_state = NULL;
  vTaskDelete(NULL);
#else
  return 0;
#endif // ESP_PTP
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* @brief Start the PTP daemon and bind it to specified interface
 * 
 * @param interface Name of the network interface to bind to, e.g. "eth0"
 * 
 * @return Non-negative task ID on success, negative errno on failure
 * 
 * @note Only one instance of AVB task can run at a time. Attempting to 
 *       start multiple instances will fail with an error.
 */
int avb_start(FAR const char *interface)
{
  if (s_state == NULL) {
    xTaskCreate(avb_task, "avb_task", CONFIG_NETUTILS_PTPD_STACKSIZE,
              (void *)interface, tskIDLE_PRIORITY + 2, NULL);
    return 1;
  }
  ESP_LOGE(TAG, "Other instance of PTP is already running");
  return -1;

}

/* @brief Query status from a running AVB task
 * 
 * @param pid Task ID of the AVB task to query
 * @param status Pointer to status storage structure
 * 
 * @return OK on success, negative errno on failure
 * 
 * @note Multiple threads with priority less than CONFIG_ESP_AVB_TASKPRIO
 *       can request status simultaneously. If higher priority threads 
 *       request status simultaneously, some of the requests may timeout.
 */
int avb_status(int pid, struct avb_status_s *status)
{
#ifdef ESP_PTP
  int ret = 0;
  sem_t donesem;
  struct ptpd_statusreq_s req;
  struct timespec timeout;

  /* Fill in the status request */

  memset(status, 0, sizeof(struct ptpd_status_s));
  sem_init(&donesem, 0, 0);
  req.done = &donesem;
  req.dest = status;

  s_state->status_req = req;

  /* Wait for status request to be handled */
  clock_gettime(CLOCK_REALTIME, &timeout); // sem_timedwait uses CLOCK_REALTIME
  timeout.tv_sec += 1;

  if (sem_timedwait(&donesem, &timeout) != 0)
    {
      req.done = NULL;
      req.dest = NULL;
      s_state->status_req = req;
      ret = -errno;
    }
  sem_destroy(&donesem);

  return ret;
#endif

#ifndef CONFIG_BUILD_FLAT

  /* TODO: Use SHM memory to pass the status information if processes
   * do not share the same memory space.
   */

  return -ENOTSUP;

#endif /* CONFIG_BUILD_FLAT */
}

/* @brief Stop the AVB task
 * 
 * @return OK on success, negative errno on failure
 **/
int avb_stop()
{
  s_state->stop = true;
  return OK;
}
