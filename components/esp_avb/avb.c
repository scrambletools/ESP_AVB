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

#include "esp_avb.h"
#include "avb.h" // Internal API

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define clock_timespec_subtract(ts1, ts2, ts3) timespecsub(ts1, ts2, ts3)
#define clock_timespec_add(ts1, ts2, ts3) timespecadd(ts1, ts2, ts3)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* AVB debug messages are enabled by either CONFIG_DEBUG_NET_INFO
 * or separately by CONFIG_ESP_AVB_DEBUG. This simplifies
 * debugging without having excessive amount of logging from net.
 */

static const char *TAG = "AVB";
#define avbinfo(format, ...) ESP_LOGI(TAG, format, ##__VA_ARGS__)
#define avbwarn(format, ...) ESP_LOGW(TAG, format, ##__VA_ARGS__)
#define avberr(format, ...) ESP_LOGE(TAG, format, ##__VA_ARGS__)

struct avb_state_s *s_state;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Initialize AVB state and create L2TAP FD - FIXME 3 FDs needed */
static int avb_initialize_state(struct avb_state_s *state,
                                const char *interface) {
  /* Create a handle to the Ethernet driver */
  esp_eth_handle_t eth_handle;

  // Create 3 L2TAP interfaces (FDs) for AVTP, MSRP, and MVRP
  for (int i = 0; i < 3; i++) {

    int fd = open("/dev/net/tap", 0);
    avbinfo("fd: %d", fd);
    state->l2if[i] = fd;
    if (state->l2if[i] < 0) {
      avberr("Failed to create tx l2if: %d\n", errno);
      return ERROR;
    }
    // Set Ethernet interface on which to get raw frames
    if (ioctl(state->l2if[i], L2TAP_S_INTF_DEVICE, interface) < 0) {
      avberr("failed to set network interface at l2if: %d\n", errno);
      return ERROR;
    }

    // Set the ethertype based on the protocol index
    uint16_t ethertype;
    switch(i) {
      case AVTP:
        ethertype = ethertype_avtp;
        break;
      case MSRP:
        ethertype = ethertype_msrp;
        break;
      case MVRP:
        ethertype = ethertype_mvrp;
        break;
      default:
        avberr("Invalid protocol index\n");
        return ERROR;
    }

    // Set the Ethertype filter (frames with this type will be available through the state->tx_l2if)
    uint16_t ethertype_filter = ETHERTYPE_AVTP; // TODO: Change to MSRP or MVRP
    if (ioctl(state->l2if[i], L2TAP_S_RCV_FILTER, &ethertype) < 0) {
      avberr("failed to set Ethertype filter: %d\n", errno);
      return ERROR;
    }
    // Enable time stamping in driver
    if (ioctl(state->l2if[i], L2TAP_G_DEVICE_DRV_HNDL, &eth_handle) < 0) {
      avberr("failed to get l2if eth_handle %d\n", errno);
      return ERROR;
    }
    // esp_eth_clock_cfg_t clk_cfg = {
    //   .eth_hndl = eth_handle,
    // };
    // esp_eth_clock_init(CLOCK_AVB_SYSTEM, &clk_cfg);

    // Enable time stamping in L2TAP
    if(ioctl(state->l2if[i], L2TAP_S_TIMESTAMP_EN) < 0) {
      avberr("failed to enable time stamping in L2TAP FD: %d\n", errno);
      return ERROR;
    }
  }

  // get HW address
  esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, &state->internal_mac_addr);

  s_state = state;
  return OK;
}

/* Destroy l2ifs */
static int avb_destroy_state(struct avb_state_s *state) {

  for (int i = 0; i < AVB_NUM_PROTOCOLS; i++) {
    if (state->l2if[i] > 0) {
      close(state->l2if[i]);
      state->l2if[i] = -1;
    }
  }
  return OK;
}

/* Send MSRP Domain message */
static int avb_send_msrp_domain(struct avb_state_s *state) {
  msrp_domain_message_s msg;
  struct timespec ts;
  int ret;

  memset(&msg, 0, sizeof(msg));
  msg.attr1_type = msrp_attr_type_domain;
  
  ret = avb_net_send(state, state->l2if[MSRP], &msg, sizeof(msg), &ts);
  if (ret < 0) {
      avberr("send failed: %d", errno);
    }
  else {
      avbinfo("Sent MSRP Domain message");
    }
  return ret;
}

/* Check if we need to send periodic messages */
static int avb_periodic_send(struct avb_state_s *state) {
  struct timespec time_now;
  struct timespec delta;

  clock_gettime(CLOCK_MONOTONIC, &time_now);

  clock_timespec_subtract(&time_now,
    &state->last_transmitted_msrp_domain, &delta);
  if (timespec_to_ms(&delta) > MSRP_DOMAIN_INTERVAL_MSEC) {
      state->last_transmitted_msrp_domain = time_now;
      avb_send_msrp_domain(state);
    }

  // TODO: Add other periodic messages
  return OK;
}

/* Process received MSRP Domain message */
static int avb_process_msrp_domain(struct avb_state_s *state,
                            msrp_domain_message_s *msg) {
  // TODO: Implement sync packet processing
  return OK;
}

/* Determine received message type and process it */
static int avb_process_rx_message(struct avb_state_s *state,
                                 int protocol_idx,
                                 ssize_t length) {
  // Check if the message length is valid
  // if (length < sizeof(state->rxbuf[protocol_idx].header)) {
  //   avbwarn("Ignoring invalid message, length only %d bytes\n",
  //           (int)length);
  //       return OK;
  // }
  //clock_gettime(CLOCK_MONOTONIC, &state->last_received_multicast);

  switch (protocol_idx) {
    case AVTP:
      avtp_msgbuf_u *msg = (avtp_msgbuf_u *)&state->rxbuf[protocol_idx].avtp;

      /* Route the message to the appropriate handler */
      switch (msg->subtype) {
        case avtp_subtype_adp:
          avbinfo("Got ADP message\n");
          //return avb_process_msrp_domain(state, &state->rxbuf[protocol_idx].announce);
          break;
        default:
          avbinfo("Ignoring unknown AVTP message subtype: 0x%02x\n",
                  msg->subtype);
          return OK;
      }
    case MSRP:
      // TODO: Implement MSRP message processing
      return OK;
    case MVRP:
      // TODO: Implement MVRP message processing
      return OK;
    default:
      avbwarn("Unknown protocol index: %d", protocol_idx);
      return OK;
  }
}

/* Process status information request */
static void avb_process_statusreq(struct avb_state_s *state) {
  struct avb_status_s *status;

  if (!state->status_req.dest) {
      return; /* No active request */
    }

  status = state->status_req.dest;

  /* TODO: Copy required info to the state structure */

  /* Post semaphore to inform that we are done */
  if (state->status_req.done) {
      sem_post(state->status_req.done);
    }

  state->status_req.done = NULL;
  state->status_req.dest = NULL;
}

/* Main AVB task */
static void avb_task(void *task_param) {
  const char *interface = "eth0"; // default interface
  struct avb_state_s *state;
  struct pollfd pollfds[AVB_NUM_PROTOCOLS]; // one for AVTP, one for MSRP, one for MVRP
  int ret = 0;
  state = calloc(1, sizeof(struct avb_state_s));
  avbinfo("state size: %d", sizeof(struct avb_state_s));

  // Get interface from task_param if provided
  if (task_param != NULL) {
    interface = task_param;
  }

  // Initialize AVB state
  if (avb_initialize_state(state, interface) != OK) {
    avberr("Failed to initialize AVB state, exiting\n");
    avb_destroy_state(state);
    free(state);
    goto err;
  }

  // Set up pollfds for each L2TAP interface
  for (int i = 0; i < AVB_NUM_PROTOCOLS; i++) {
    pollfds[i].events = POLLIN;
    pollfds[i].fd = state->l2if[i];
  }

  // Main AVB loop
  while (!state->stop) {

    /* Wait for a message on any of the L2TAP interfaces */
    for (int i = 0; i < AVB_NUM_PROTOCOLS; i++) {

      pollfds[i].revents = 0; // reset revents
      ret = poll(pollfds, AVB_NUM_PROTOCOLS, AVB_POLL_INTERVAL_MS);
      if (pollfds[i].revents) {
        /* Receive a message from the L2TAP interface */
        ret = avb_net_recv(state, state->l2if[i], &state->rxbuf[i], AVB_MAX_MSG_LEN, &state->rxtime[i]);
        if (ret > 0) {
          /* Process the received message */
          avb_process_rx_message(state, i, ret);
        }
      }
    }

    // Send periodic messages such as announcing entity available, etc
    avb_periodic_send(state);

    // do any validation checking for timed out connections, etc
    // TBD

    // Process status requests
    avb_process_statusreq(state);
  } // while (!state->stop)

  avb_destroy_state(state);
  free(state);

err:
  s_state = NULL;
  vTaskDelete(NULL);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* @brief Start the AVB task and bind it to a specified interface
 * 
 * @param interface Name of the network interface to bind to, e.g. "eth0"
 * 
 * @return OK on success, ERROR on failure
 * 
 * @note Only one instance of AVB task can run at a time. Attempting to 
 *       start multiple instances will fail with an error. CONFIG_ESP_AVB_STACKSIZE
 */
int avb_start(const char *interface)
{
  if (s_state == NULL) {
    xTaskCreate(avb_task, "AVB", 6144,
              (void *)interface, 20, NULL);
    return OK;
  }
  ESP_LOGE(TAG, "Another instance of AVB is already running");
  return ERROR;
}

/* @brief Query status from a running AVB task
 * 
 * @param status Pointer to status storage structure
 * 
 * @return OK on success, negative errno on failure
 * 
 * @note Multiple threads with priority less than CONFIG_ESP_AVB_TASKPRIO
 *       can request status simultaneously. If higher priority threads 
 *       request status simultaneously, some of the requests may timeout.
 */
int avb_status(struct avb_status_s *status)
{
  int ret = 0;
  sem_t donesem;
  struct avb_statusreq_s req;
  struct timespec timeout;

  /* Fill in the status request */
  memset(status, 0, sizeof(struct avb_status_s));
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
