/*
 * Copyright 2024 Scramble Tools
 * License: MIT
 *
 * ESP_AVB Component
 *
 * This component provides an implementation of an AVB talker and listener.
 */

#include "esp_avb.h" // External API
#include "avb.h" // Internal API

/* Global state */
struct avb_state_s *s_state;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Initialize AVB state and create L2TAP FD - FIXME 3 FDs needed */
static int avb_initialize_state(struct avb_state_s *state,
                                const char *interface) {
  int ret = 0;

  // Initialize the network interface
  ret = avb_net_init(state, interface);
  if (ret < 0) {
    avberr("Failed to initialize network interface");
    return ERROR;
  }

  // Set any other state variables: TBD

  // Set stop to false
  state->stop = false;

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

/* Send periodic messages */
static int avb_periodic_send(struct avb_state_s *state) {
  struct timespec time_now;
  struct timespec delta;

  clock_gettime(CLOCK_MONOTONIC, &time_now);

  clock_timespec_subtract(&time_now, &state->last_transmitted_mvrp_vlan_id, &delta);
  if (timespec_to_ms(&delta) > MVRP_VLAN_ID_INTERVAL_MSEC) {
    state->last_transmitted_mvrp_vlan_id = time_now;
    avb_send_mvrp_vlan_id(state);
  }

  clock_timespec_subtract(&time_now, &state->last_transmitted_msrp_domain, &delta);
  if (timespec_to_ms(&delta) > MSRP_DOMAIN_INTERVAL_MSEC) {
    state->last_transmitted_msrp_domain = time_now;
    avb_send_msrp_domain(state);
  }

  clock_timespec_subtract(&time_now, &state->last_transmitted_msrp_talker_adv, &delta);
  if (timespec_to_ms(&delta) > MSRP_TALKER_ADV_INTERVAL_MSEC) {
    state->last_transmitted_msrp_talker_adv = time_now;
    avb_send_msrp_talker_adv(state, msrp_attr_event_new);
  }

  clock_timespec_subtract(&time_now, &state->last_transmitted_adp_entity_avail, &delta);
  if (timespec_to_ms(&delta) > ADP_ENTITY_AVAIL_INTERVAL_MSEC) {
    state->last_transmitted_adp_entity_avail = time_now;
    avb_send_adp_entity_available(state);
  }
  return OK;
}

/* Determine received message type and process it */
static int avb_process_rx_message(struct avb_state_s *state,
                                 int protocol_idx,
                                 ssize_t length) {
  // Check if the message length is valid
  if (length <= ETH_HEADER_LEN) {
    avbwarn("Ignoring invalid message, length only %d bytes\n",
            (int)length);
        return OK;
  }
  //clock_gettime(CLOCK_MONOTONIC, &state->last_received_multicast);

  /* Route the message to the appropriate handler */
  switch (protocol_idx) {
    case AVTP: {
      avtp_msgbuf_u *msg = (avtp_msgbuf_u *)&state->rxbuf[protocol_idx].avtp;
      switch (msg->subtype) {
        case avtp_subtype_aaf:
          avbinfo("Got an AAF message\n");
          return avb_process_aaf(state, &msg->aaf);
          break;
        case avtp_subtype_maap:
          avbinfo("Got a MAAP message\n");
          return avb_process_maap(state, &msg->maap);
          break;
        case avtp_subtype_adp:
          avbinfo("Got an ADP message\n");
          return avb_process_adp(state, &msg->adp);
          break;
        case avtp_subtype_aecp:
          avbinfo("Got an AECP message\n");
          return avb_process_aecp(state, &msg->aecp);
          break;
        case avtp_subtype_acmp:
          avbinfo("Got an ACMP message\n");
          return avb_process_acmp(state, &msg->acmp);
          break;
        default:
          avbinfo("Ignoring unknown AVTP message subtype: 0x%02x\n",
                  msg->subtype);
      }
      break;
    }
    case MSRP: {
      msrp_msgbuf_s *msg = (msrp_msgbuf_s *)&state->rxbuf[protocol_idx].msrp;
      
      // Get the first attribute type
      msrp_attr_header_s header;
      int offset = 0;
      int count = 0;
      memcpy(&header, &msg->messages_raw[offset], sizeof(msrp_attr_header_s));

      // Process all attributes in the message
      while (header.attr_type != msrp_attr_type_none && offset < sizeof(msrp_msgbuf_s)) {
        size_t attr_size = octets_to_uint(header.attr_list_len, 2) + 4; // 4 bytes for the header w/o vechead
        switch (header.attr_type) {
          case msrp_attr_type_domain:
            avbinfo("Got an MSRP domain message\n");
            avb_process_msrp_domain(state, msg, offset, attr_size);
            break;
          case msrp_attr_type_talker_advertise:
            avbinfo("Got an MSRP talker advertise message\n");
            avb_process_msrp_talker(state, msg, offset, attr_size, false);
            break;
          case msrp_attr_type_talker_failed:
            avbinfo("Got an MSRP talker failed message\n");
            avb_process_msrp_talker(state, msg, offset, attr_size, true);
            break;
          case msrp_attr_type_listener:
            avbinfo("Got an MSRP listener message\n");
            avb_process_msrp_listener(state, msg, offset, attr_size);
            break;
          default:
            avbinfo("Ignoring unknown MSRP message atribute type: 0x%02x\n",
                    header.attr_type);
        }
        // Get the next attribute offset
        offset += attr_size;
        // Get the next attribute header
        memcpy(&header, &msg->messages_raw[offset], sizeof(msrp_attr_header_s));
        count++;
      }
      //avbinfo("Processed %d attributes in MSRP message", count);
      break;
    }
    case MVRP: {
      /* only VLAN identifier is needed */
      mvrp_vlan_id_message_s *msg = &state->rxbuf[protocol_idx].mvrp;
      if (msg->header.attr_type == mvrp_attr_type_vlan_identifier) {
          avbinfo("Got an MVRP VLAN identifier message\n");
          return avb_process_mvrp_vlan_id(state, msg);
      }
      else {
          avbinfo("Ignoring unknown MVRP message attribute type: 0x%02x\n",
                  msg->header.attr_type);
      }
      break;
    default:
      avbwarn("Ignoring unknown protocol index: %d", protocol_idx);
    }
  }
  return OK;
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
    // Validate file descriptor
    if (pollfds[i].fd < 0) {
      avberr("Invalid file descriptor for interface %d: fd=%d", i, pollfds[i].fd);
      continue;
    }
  }

  // Main AVB loop
  while (!state->stop) {

    /* Wait for a message on any of the L2TAP interfaces */
    if (poll(pollfds, AVB_NUM_PROTOCOLS, AVB_POLL_INTERVAL_MS)) {
      /* Check for events on each L2TAP interface */
      for (int i = 0; i < AVB_NUM_PROTOCOLS; i++) {
        if (pollfds[i].revents) {
          /* Get a message from the L2TAP interface */
          ret = avb_net_recv(state, state->l2if[i], &state->rxbuf[i], AVB_MAX_MSG_LEN, &state->rxtime[i]);
          if (ret > 0) {
            /* Process the received message */
            avb_process_rx_message(state, i, ret);
          }
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
  avberr("Another instance of AVB is already running");
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
