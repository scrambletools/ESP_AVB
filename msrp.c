/*
 * Copyright 2024-2026 Scramble Tools
 * License: MIT
 *
 * SPDX-License-Identifier: MIT
 *
 * ESP_AVB SRP — Stream Reservation Protocol family
 *
 * MSRP (Multiple Stream Reservation Protocol)
 * MVRP (Multiple VLAN Registration Protocol)
 * MAAP (Multicast Address Acquisition Protocol)
 *
 * Per IEEE 802.1Q-2018 Clause 35 (SRP) and IEEE 1722-2016 Annex B (MAAP).
 * Together these run the bridge-local attribute registration that
 * decides which streams are reserved on a given port and at what
 * bandwidth, and they assign destination MAC ranges for talker
 * stream egress.
 *
 * Bridge-only admission control (the 75 % link-rate cap per SR class)
 * also lives in this file — it hooks into the MSRP TALKER_ADVERTISE
 * handler and is excluded from endpoint builds via
 * CONFIG_ESP_AVB_ROLE_BRIDGE.
 *
 * Previously these functions were spread between avtp.c (which is
 * properly the IEEE 1722 AVTPDU stream code) and atdecc.c (which is
 * IEEE 1722.1 control). Consolidating into msrp.c keeps SRP code
 * together and makes the role-conditional build cleaner: bridge
 * compiles msrp.c without avtp.c's stream sections; endpoint
 * compiles both.
 */

#include "avb.h"
#include "avbbridge.h"
#include "esp_log.h"
#include "esp_timer.h"

/* SRP class-mapping helpers. SR Class A = mapping index 0 (sr_class_id 6),
 * SR Class B = mapping index 1 (sr_class_id 5). The class_id values
 * match IEEE 802.1Q-2018 Table 35-7. */

static uint16_t avb_msrp_mapping_index_for_class(bool class_b) {
  return class_b ? 1 : 0;
}

static uint8_t avb_msrp_class_id_for_index(uint16_t mapping_index) {
  return mapping_index == 1 ? 5 : 6;
}

static uint16_t avb_msrp_mapping_index_for_class_id(uint8_t class_id) {
  return class_id == 5 ? 1 : 0;
}

/* CIP SFC → sample rate, used by both the AVTP talker (avtp.c) and the
 * MSRP TALKER advertise (this file) for bandwidth calculation.
 * Declared in avb.h so avtp.c can also call it. */
uint32_t cip_sfc_to_sample_rate(uint8_t sfc) {
  switch (sfc) {
  case cip_sfc_sample_rate_32k:
    return 32000;
  case cip_sfc_sample_rate_44_1k:
    return 44100;
  case cip_sfc_sample_rate_48k:
    return 48000;
  case cip_sfc_sample_rate_88_2k:
    return 88200;
  case cip_sfc_sample_rate_96k:
    return 96000;
  case cip_sfc_sample_rate_176_4k:
    return 176400;
  case cip_sfc_sample_rate_192k:
    return 192000;
  default:
    return 48000;
  }
}

int avb_send_mvrp_vlan_id(avb_state_s *state, mrp_attr_event_t attr_event,
                          bool leave_all) {
  mvrp_vlan_id_message_s msg;
  struct timespec ts;
  int ret;
  uint16_t mapping_index = avb_msrp_mapping_index_for_class(false);
  memset(&msg, 0, sizeof(msg));

  // Populate the message
  msg.protocol_ver = 0;
  msg.header.attr_type = mvrp_attr_type_vlan_identifier;
  msg.header.attr_len = 2;
  msg.header.vechead_leaveall = leave_all;
  msg.header.vechead_padding = 0;
  msg.header.vechead_num_vals = 1;
  memcpy(msg.vlan_id, state->msrp_mappings[mapping_index].vlan_id, 2);
  msg.attr_event[0] = int_to_3pe(attr_event, 0, 0);
  uint16_t msg_len =
      8 + 2 + 2; // all of the above + end mark list and end mark msg

  // send the message
  ret = avb_net_send(state, ethertype_mvrp, &msg, msg_len, &ts);
  if (ret < 0) {
    avberr("send MVRP VLAN ID failed: %d", errno);
    return ret;
  }
  return OK;
}

/* Send MSRP domain message */
// TODO: add support for multiple domains
int avb_send_msrp_domain(avb_state_s *state, mrp_attr_event_t attr_event,
                         bool leave_all) {
  msrp_domain_message_s msg;
  struct timespec ts;
  int ret;
  uint16_t mapping_index = avb_msrp_mapping_index_for_class(false);
  memset(&msg, 0, sizeof(msg));

  // Populate the message
  msg.header.attr_type = msrp_attr_type_domain;
  msg.header.attr_len = 4;
  int attr_list_len = 9; // includes vechead, attr_event and vec end mark
  int_to_octets(&attr_list_len, msg.header.attr_list_len, 2);
  msg.header.vechead_leaveall = leave_all;
  msg.header.vechead_padding = 0;
  msg.header.vechead_num_vals = 1;
  msg.sr_class_id = avb_msrp_class_id_for_index(mapping_index);
  msg.sr_class_priority = state->msrp_mappings[mapping_index].priority;
  memcpy(msg.sr_class_vid, state->msrp_mappings[mapping_index].vlan_id,
         sizeof(msg.sr_class_vid));
  msg.attr_event[0] = int_to_3pe(attr_event, 0, 0);

  // Create an MSRP message buffer
  msrp_msgbuf_s msrp_msg;
  memset(&msrp_msg, 0, sizeof(msrp_msg));
  msrp_msg.protocol_ver = 0;
  memcpy(msrp_msg.messages_raw, &msg, sizeof(msg));
  uint16_t msg_len = 5 + attr_list_len + 2; // header + attr_list_len + end mark

  // send the message
  ret = avb_net_send(state, ethertype_msrp, &msrp_msg, msg_len, &ts);
  if (ret < 0) {
    avberr("send MSRP Domain failed: %d", errno);
    perror("send MSRP Domain failed");
  }
  return ret;
}


/* Compute the MSRP TSpec MaxFrameSize for an output stream based on its
 * current stream_format. Per IEEE 802.1Qat, MaxFrameSize is the L2 frame
 * length excluding FCS: ETH header + VLAN tag + AVTP header + payload. */
uint16_t avb_compute_tspec_max_frame_size(avb_state_s *state, uint16_t index) {
  avtp_stream_format_s *fmt = &state->output_streams[index].stream_format;
  bool class_b = state->output_streams[index].stream_info_flags.class_b;
  int interval_us = class_b ? 250 : 125;
  int channels, bytes_per_sample, sample_rate, avtp_hdr;
  uint8_t subtype = fmt->aaf_pcm.subtype;
  if (subtype == avtp_subtype_crf) {
    /* CRF: ETH + VLAN + AVTP CRF header + one 64-bit timestamp. */
    return (uint16_t)(14 /*ETH*/ + 4 /*VLAN*/ + 28 /*CRF AVTPDU*/);
  }
  if (subtype == avtp_subtype_61883) {
    channels = fmt->am824.dbs;
    bytes_per_sample = 4; /* AM824 always carries 32-bit quadlets */
    sample_rate = cip_sfc_to_sample_rate(fmt->am824.fdf_sfc);
    avtp_hdr = 24 + 8; /* AVTP common + CIP header */
  } else {
    channels =
        (fmt->aaf_pcm.chan_per_frame_h << 2) | fmt->aaf_pcm.chan_per_frame;
    int bit_depth = fmt->aaf_pcm.bit_depth;
    bytes_per_sample = (bit_depth == 24) ? 4 : (bit_depth / 8);
    sample_rate = aaf_code_to_sample_rate(fmt->aaf_pcm.sample_rate);
    avtp_hdr = 24;
  }
  if (channels <= 0 || sample_rate <= 0)
    return 0;
  int samples_per_packet = sample_rate / (1000000 / interval_us);
  int payload = samples_per_packet * channels * bytes_per_sample;
  return (uint16_t)(14 /*ETH*/ + 4 /*VLAN*/ + avtp_hdr + payload);
}

static int avb_send_msrp_attr(avb_state_s *state, void *attr,
                              int attr_list_len, const char *label) {
  size_t attr_size = 4 + attr_list_len; /* attr hdr w/o vechead + attr list */
  struct timespec ts;
  int ret;

  if (state->avb_lite) {
    return avb_send_cvu_srp_attr(state, attr, attr_list_len, label);
  }

  msrp_msgbuf_s msrp_msg;
  memset(&msrp_msg, 0, sizeof(msrp_msg));
  msrp_msg.protocol_ver = 0;
  memcpy(msrp_msg.messages_raw, attr, attr_size);
  uint16_t msg_len = 5 + attr_list_len + 2; // header + attr_list_len + end mark

  ret = avb_net_send(state, ethertype_msrp, &msrp_msg, msg_len, &ts);
  if (ret < 0) {
    avberr("send MSRP %s failed: %d", label, errno);
  }
  return ret;
}

/* Send MSRP talker advertise message with appropriate event */
int avb_send_msrp_talker(avb_state_s *state, mrp_attr_event_t attr_event,
                         bool leave_all, bool is_failed, unique_id_t *stream_id,
                         eth_addr_t *stream_dest_addr, uint8_t *vlan_id,
                         uint16_t max_frame_size) {
  msrp_talker_message_u msg;
  int attr_list_len;
  memset(&msg, 0, sizeof(msg));

  // Populate the message
  if (!is_failed) {
    msg.header.attr_type = msrp_attr_type_talker_advertise;
    msg.header.attr_len = 25;
    attr_list_len = 29; // includes vechead, attr_event and vec end mark
  } else {
    msg.header.attr_type = msrp_attr_type_talker_failed;
    msg.header.attr_len = 34;
    attr_list_len = 38; // includes vechead, attr_event and vec end mark
  }
  int_to_octets(&attr_list_len, msg.header.attr_list_len, 2);
  msg.header.vechead_leaveall = leave_all;
  msg.header.vechead_num_vals = 1;
  memcpy(msg.talker.info.stream_id, stream_id, UNIQUE_ID_LEN);
  memcpy(msg.talker.info.stream_dest_addr, stream_dest_addr, ETH_ADDR_LEN);
  memcpy(msg.talker.info.vlan_id, vlan_id, 2);
  int tspec_max_frame_size = max_frame_size;
  int_to_octets(&tspec_max_frame_size, msg.talker.info.tspec_max_frame_size, 2);
  int tspec_max_frame_interval = 1;
  int_to_octets(&tspec_max_frame_interval,
                msg.talker.info.tspec_max_frame_interval, 2);
  uint16_t mapping_index = 0;
  for (uint16_t i = 0; i < state->msrp_mappings_count; i++) {
    if (memcmp(vlan_id, state->msrp_mappings[i].vlan_id, 2) == 0) {
      mapping_index = i;
      break;
    }
  }
  msg.talker.info.priority = state->msrp_mappings[mapping_index].priority;
  msg.talker.info.rank = 1;        // rank 1: emergency-capable stream
  int accumulated_latency = 15000; // ~15μs worst-case talker latency
  int_to_octets(&accumulated_latency, msg.talker.info.accumulated_latency, 4);
  if (is_failed) {
    memcpy(msg.talker_failed.failure_bridge_id, &EMPTY_ID, UNIQUE_ID_LEN);
    msg.talker_failed.failure_code = 0;
    msg.talker_failed.event_data[0] = int_to_3pe(attr_event, 0, 0);
  } else {
    msg.talker.event_data[0] = int_to_3pe(attr_event, 0, 0);
  }

  return avb_send_msrp_attr(state, &msg, attr_list_len,
                            is_failed ? "Talker Failed"
                                      : "Talker Advertise");
}

/* Send MSRP listener message with appropriate event */
int avb_send_msrp_listener(avb_state_s *state, mrp_attr_event_t attr_event,
                           msrp_listener_event_t listener_event, bool leave_all,
                           unique_id_t *stream_id) {
  msrp_listener_message_s msg;
  memset(&msg, 0, sizeof(msg));

  // Populate the message
  msg.header.attr_type = msrp_attr_type_listener;
  msg.header.attr_len = 8;
  int attr_list_len = 14; // includes vechead, attr_event and vec end mark
  int_to_octets(&attr_list_len, msg.header.attr_list_len, 2);
  msg.header.vechead_leaveall = leave_all;
  msg.header.vechead_padding = 0;
  msg.header.vechead_num_vals = 1; // only attribute event counts
  memcpy(msg.stream_id, stream_id, UNIQUE_ID_LEN);
  msg.event_decl_data[0].event = int_to_3pe(attr_event, 0, 0);
  msg.event_decl_data[1].declaration.event1 = listener_event;

  return avb_send_msrp_attr(state, &msg, attr_list_len, "Listener");
}

/* MAAP timing constants (IEEE 1722-2016 Annex B) */
#define MAAP_PROBE_RETRANSMITS 3
#define MAAP_PROBE_INTERVAL_BASE_MS 500
#define MAAP_PROBE_INTERVAL_VAR_MS 100
#define MAAP_ANNOUNCE_INTERVAL_BASE_MS 30000
#define MAAP_ANNOUNCE_INTERVAL_VAR_MS 2000
/* MAAP dynamic allocation pool: 91:E0:F0:00:00:00 to 91:E0:F0:00:FD:FF */
#define MAAP_POOL_SIZE 0xFE00

/* Generate a deterministic address within the MAAP pool from the device
 * MAC. Deterministic across boots is important: switches cache per-
 * stream_id MSRP state, and if our picked Stream DA changed each boot,
 * the switch's stale registration would block forwarding for the new
 * address. MAAP PROBE still resolves cross-device conflicts if two
 * devices happen to hash to the same slot. */
static void maap_generate_addr(avb_state_s *state, eth_addr_t *addr,
                               int stream_index) {
  uint32_t seed = state->port[0].internal_mac_addr[2] ^ state->port[0].internal_mac_addr[3];
  seed = (seed << 8) | state->port[0].internal_mac_addr[4];
  seed = (seed << 8) | state->port[0].internal_mac_addr[5];
  seed += stream_index;
  uint16_t offset = seed % MAAP_POOL_SIZE;
  uint8_t *a = (uint8_t *)addr;
  a[0] = 0x91;
  a[1] = 0xe0;
  a[2] = 0xf0;
  a[3] = 0x00;
  a[4] = (offset >> 8) & 0xFF;
  a[5] = offset & 0xFF;
}

/* Get a random timer jitter value */
static int64_t maap_jitter_us(int base_ms, int var_ms) {
  uint32_t r = (uint32_t)esp_timer_get_time();
  int jitter = (r % (var_ms + 1));
  return (int64_t)(base_ms + jitter) * 1000;
}

/* Check if two single-address ranges conflict */
static bool maap_addr_conflicts(eth_addr_t *a, eth_addr_t *b) {
  return memcmp(a, b, ETH_ADDR_LEN) == 0;
}

/* Send a MAAP message */
int avb_send_maap_msg(avb_state_s *state, maap_msg_type_t msg_type,
                      eth_addr_t *req_addr, uint16_t req_count,
                      eth_addr_t *confl_addr, uint16_t confl_count) {
  maap_message_s msg;
  struct timespec ts;
  memset(&msg, 0, sizeof(msg));

  msg.subtype = avtp_subtype_maap;
  msg.msg_type = msg_type;
  msg.sv = 0;
  msg.version = 0;
  msg.maap_version = 1;
  msg.control_data_len = 16;
  memcpy(msg.req_start_addr, req_addr, ETH_ADDR_LEN);
  int_to_octets(&req_count, msg.req_count, 2);
  if (confl_addr)
    memcpy(msg.confl_start_addr, confl_addr, ETH_ADDR_LEN);
  if (confl_count > 0)
    int_to_octets(&confl_count, msg.confl_count, 2);

  int ret = avb_net_send(state, ethertype_avtp, &msg, sizeof(msg), &ts);
  if (ret < 0) {
    avberr("send MAAP %s failed: %d",
           msg_type == maap_msg_type_probe     ? "probe"
           : msg_type == maap_msg_type_defend   ? "defend"
           : msg_type == maap_msg_type_announce ? "announce"
                                                : "unknown",
           errno);
  }
  return ret;
}

/* Initialize MAAP state for all output streams */
void avb_maap_init(avb_state_s *state) {
  for (int i = 0; i < state->num_output_streams; i++) {
    maap_stream_state_s *m = &state->maap[i];
    m->state = maap_state_initial;
    m->acquired = false;
    maap_generate_addr(state, &m->acquired_addr, i);
    m->probe_count = MAAP_PROBE_RETRANSMITS;
    m->timer_expiry_us =
        esp_timer_get_time() + maap_jitter_us(MAAP_PROBE_INTERVAL_BASE_MS,
                                              MAAP_PROBE_INTERVAL_VAR_MS);
    m->state = maap_state_probe;
    avb_send_maap_msg(state, maap_msg_type_probe, &m->acquired_addr, 1, NULL,
                      0);
    avbinfo("MAAP: probing %02x:%02x:%02x:%02x:%02x:%02x for stream %d",
            m->acquired_addr[0], m->acquired_addr[1], m->acquired_addr[2],
            m->acquired_addr[3], m->acquired_addr[4], m->acquired_addr[5], i);
  }
}

/* MAAP periodic tick — called from AVB main loop.
 * Handles probe retransmits and announce refresh. */
void avb_maap_tick(avb_state_s *state) {
  int64_t now = esp_timer_get_time();

  for (int i = 0; i < state->num_output_streams; i++) {
    maap_stream_state_s *m = &state->maap[i];
    if (now < m->timer_expiry_us)
      continue;

    if (m->state == maap_state_probe) {
      m->probe_count--;
      if (m->probe_count == 0) {
        /* Probing complete — address acquired */
        m->state = maap_state_defend;
        m->acquired = true;
        memcpy(state->output_streams[i].stream_dest_addr, m->acquired_addr,
               ETH_ADDR_LEN);
        avb_send_maap_msg(state, maap_msg_type_announce, &m->acquired_addr, 1,
                          NULL, 0);
        m->timer_expiry_us =
            now + maap_jitter_us(MAAP_ANNOUNCE_INTERVAL_BASE_MS,
                                 MAAP_ANNOUNCE_INTERVAL_VAR_MS);
        avbinfo("MAAP: acquired %02x:%02x:%02x:%02x:%02x:%02x for stream %d",
                m->acquired_addr[0], m->acquired_addr[1], m->acquired_addr[2],
                m->acquired_addr[3], m->acquired_addr[4], m->acquired_addr[5],
                i);
      } else {
        /* Send another probe */
        avb_send_maap_msg(state, maap_msg_type_probe, &m->acquired_addr, 1,
                          NULL, 0);
        m->timer_expiry_us =
            now + maap_jitter_us(MAAP_PROBE_INTERVAL_BASE_MS,
                                 MAAP_PROBE_INTERVAL_VAR_MS);
      }
    } else if (m->state == maap_state_defend) {
      /* Periodic announce refresh */
      avb_send_maap_msg(state, maap_msg_type_announce, &m->acquired_addr, 1,
                        NULL, 0);
      m->timer_expiry_us =
          now + maap_jitter_us(MAAP_ANNOUNCE_INTERVAL_BASE_MS,
                               MAAP_ANNOUNCE_INTERVAL_VAR_MS);
    }
  }
}

/* Process received MSRP domain message */
int avb_process_msrp_domain(avb_state_s *state, msrp_msgbuf_s *msg, int offset,
                            size_t length) {
  msrp_domain_message_s domain;
  memset(&domain, 0, sizeof(domain));
  memcpy(&domain, &msg->messages_raw[offset], sizeof(msrp_domain_message_s));

  uint16_t vid = octets_to_uint(domain.sr_class_vid, 2);
  uint16_t mapping_index =
      avb_msrp_mapping_index_for_class_id(domain.sr_class_id);
  uint16_t our_vid = octets_to_uint(state->msrp_mappings[mapping_index].vlan_id, 2);

  /* Log and validate — the network's SR class must match ours */
  if ((domain.sr_class_id == 6 || domain.sr_class_id == 5) && vid != our_vid) {
    avberr("MSRP domain: class %d network VLAN %d != our VLAN %d",
           domain.sr_class_id, vid, our_vid);
  }
  return OK;
}

/* Process received MSRP talker advertise message */
int avb_process_msrp_talker(avb_state_s *state, msrp_msgbuf_s *msg_data,
                            int offset, size_t length, bool is_failed,
                            eth_addr_t *src_addr) {
  msrp_talker_message_u msg;
  memset(&msg, 0, sizeof(msrp_talker_message_u));
  memcpy(&msg, &msg_data->messages_raw[offset], sizeof(msrp_talker_message_u));

  // get the talker addr from the stream id
  eth_addr_t talker_addr;
  memcpy(&talker_addr, msg.talker.info.stream_id, ETH_ADDR_LEN);

#ifdef CONFIG_ESP_AVB_ROLE_BRIDGE
  /* Bridge admission control. A TALKER_ADVERTISE arriving on one port
   * would be re-declared by the bridge MAP on the other port; before
   * propagating, check that the egress port has the budget for it. If
   * not, the bridge re-declares as TALKER_FAILED with failure_code =
   * insufficient_bandwidth_for_traffic_class (IEEE 802.1Q-2018
   * §35.2.2.8.4 and Table 35-7).
   *
   * is_failed=true means the upstream side already gave up; nothing
   * to admit.
   *
   * Phase 5d: we don't yet know the ingress port (the L2 forwarder
   * wires that in Phase 5b). Until then assume worst-case egress —
   * the Wi-Fi port (port 1 when NUM_PORTS=2). */
  if (!is_failed && CONFIG_ESP_AVB_NUM_PORTS > 1) {
    avb_sr_class_e cls =
        (msg.talker.info.priority == 3) ? AVB_SR_CLASS_A : AVB_SR_CLASS_B;
    uint16_t mfs =
        (uint16_t)octets_to_uint(msg.talker.info.tspec_max_frame_size, 2);
    uint16_t mfi = (uint16_t)octets_to_uint(
        msg.talker.info.tspec_max_frame_interval, 2);
    /* Observation interval per SR class:
     *   Class A = 125 µs → 8000 intervals/s
     *   Class B = 250 µs → 4000 intervals/s
     * Bandwidth = max_frame_size × frames_per_interval × intervals/s × 8. */
    uint32_t intervals_per_sec = (cls == AVB_SR_CLASS_A) ? 8000u : 4000u;
    uint32_t request_bps =
        (uint32_t)mfs * (uint32_t)mfi * intervals_per_sec * 8u;

    int egress_port = 1; /* Phase 5b will replace with !ingress_port */
    int adm = avb_srp_admission_try_admit(egress_port, cls, request_bps);
    if (adm < 0) {
      avbwarn("MSRP TALKER over budget on port %d class %s: %u bps; "
              "re-declaring as TALKER_FAILED",
              egress_port, (cls == AVB_SR_CLASS_A) ? "A" : "B",
              (unsigned)request_bps);

      /* Re-declare as TALKER_FAILED on the egress side. Build the
       * full failed message directly so we can set failure_code =
       * insufficient_bandwidth_for_traffic_class (the existing
       * avb_send_msrp_talker hard-codes failure_code = 0). */
      msrp_talker_message_u failed;
      memset(&failed, 0, sizeof(failed));
      failed.header.attr_type = msrp_attr_type_talker_failed;
      failed.header.attr_len = 34;
      int attr_list_len = 38;
      int_to_octets(&attr_list_len, failed.header.attr_list_len, 2);
      failed.header.vechead_num_vals = 1;
      memcpy(&failed.talker_failed.info, &msg.talker.info,
             sizeof(talker_adv_info_s));
      memcpy(failed.talker_failed.failure_bridge_id,
             state->port[0].internal_mac_addr, ETH_ADDR_LEN);
      failed.talker_failed.failure_code =
          insufficient_bandwidth_for_traffic_class;
      failed.talker_failed.event_data[0] = int_to_3pe(mrp_attr_event_new, 0, 0);
      avb_send_msrp_attr(state, &failed, attr_list_len, "Talker Failed");
      return OK;
    }
  }
#endif /* CONFIG_ESP_AVB_ROLE_BRIDGE */

  // If the talker is known then update talker info
  int index =
      avb_find_entity_by_addr(state, &talker_addr, avb_entity_type_talker);
  if (index >= 0) {
    memcpy(&state->talkers[index].info, &msg.talker.info,
           sizeof(talker_adv_info_s));
  }

  /* Propagate the SRP-accumulated latency onto any listener input_stream
   * with a matching stream_id. This is the authoritative end-to-end
   * max_transit_time (talker origination + bridge hops), per
   * IEEE 802.1Qat / Milan §5.4. avb_start_stream_in reads it at connect
   * time to size the presentation-time startup fill.
   * Guard against a zero-stream-id malformed MSRP packet matching an
   * unconnected slot's zeroed stream_id. */
  static const uint8_t zero_stream_id[UNIQUE_ID_LEN] = {0};
  if (memcmp(msg.talker.info.stream_id, zero_stream_id, UNIQUE_ID_LEN) != 0) {
    for (int i = 0; i < AVB_MAX_NUM_INPUT_STREAMS; i++) {
      if (memcmp(state->input_streams[i].stream_id, msg.talker.info.stream_id,
                 UNIQUE_ID_LEN) == 0) {
        memcpy(state->input_streams[i].msrp_accumulated_latency,
               msg.talker.info.accumulated_latency, 4);
        break;
      }
    }
  }
  return OK;
}

/* Find a connected listener by MAC address in a stream's listener list.
 * Returns index or -1 if not found. */
static int find_connected_listener(avb_talker_stream_s *stream,
                                   eth_addr_t *mac_addr) {
  uint16_t count = octets_to_uint(stream->connection_count, 2);
  for (int i = 0; i < count && i < AVB_MAX_NUM_CONNECTED_LISTENERS; i++) {
    if (memcmp(stream->connected_listeners[i].mac_addr, mac_addr,
               ETH_ADDR_LEN) == 0) {
      return i;
    }
  }
  return -1;
}

static int find_connected_listener_by_entity(avb_talker_stream_s *stream,
                                             const unique_id_t entity_id) {
  static const unique_id_t zero_id = {0};
  if (memcmp(entity_id, zero_id, UNIQUE_ID_LEN) == 0)
    return -1;
  uint16_t count = octets_to_uint(stream->connection_count, 2);
  for (int i = 0; i < count && i < AVB_MAX_NUM_CONNECTED_LISTENERS; i++) {
    if (memcmp(stream->connected_listeners[i].identity.id, entity_id,
               UNIQUE_ID_LEN) == 0) {
      return i;
    }
  }
  return -1;
}

/* Add a listener to a stream's connected_listeners list.
 * Returns index of added entry, or -1 if full. */
static int add_connected_listener(avb_talker_stream_s *stream,
                                  eth_addr_t *mac_addr) {
  uint16_t count = octets_to_uint(stream->connection_count, 2);
  if (count >= AVB_MAX_NUM_CONNECTED_LISTENERS)
    return -1;
  memcpy(stream->connected_listeners[count].mac_addr, mac_addr, ETH_ADDR_LEN);
  count++;
  int_to_octets(&count, stream->connection_count, 2);
  return count - 1;
}

/* Resolve/merge an MSRP listener declaration (known by source MAC) with any
 * existing ACMP CONNECT_TX state (known by listener entity_id/uid).  Streaming
 * is allowed only when both halves are present: MSRP Ready + ACMP connected. */
static int find_or_add_msrp_listener(avb_state_s *state,
                                     avb_talker_stream_s *stream,
                                     eth_addr_t *mac_addr) {
  int idx = find_connected_listener(stream, mac_addr);
  if (idx >= 0)
    return idx;

  int listener_idx = avb_find_entity_by_addr(state, mac_addr,
                                             avb_entity_type_listener);
  if (listener_idx != NOT_FOUND) {
    idx = find_connected_listener_by_entity(
        stream, state->listeners[listener_idx].entity_id);
    if (idx >= 0) {
      memcpy(stream->connected_listeners[idx].mac_addr, mac_addr,
             ETH_ADDR_LEN);
      return idx;
    }
  }

  idx = add_connected_listener(stream, mac_addr);
  if (idx >= 0 && listener_idx != NOT_FOUND) {
    memcpy(stream->connected_listeners[idx].identity.id,
           state->listeners[listener_idx].entity_id, UNIQUE_ID_LEN);
    memset(stream->connected_listeners[idx].identity.uid, 0, 2);
  }
  return idx;
}

static bool any_listener_ready(avb_talker_stream_s *stream) {
  uint16_t count = octets_to_uint(stream->connection_count, 2);
  for (int i = 0; i < count && i < AVB_MAX_NUM_CONNECTED_LISTENERS; i++) {
    if (stream->connected_listeners[i].msrp_ready) {
      return true;
    }
  }
  return false;
}

static bool any_listener_acmp_connected(avb_talker_stream_s *stream) {
  uint16_t count = octets_to_uint(stream->connection_count, 2);
  for (int i = 0; i < count && i < AVB_MAX_NUM_CONNECTED_LISTENERS; i++) {
    if (stream->connected_listeners[i].acmp_connected) {
      return true;
    }
  }
  return false;
}


/* Process received MSRP listener message.
 * As a talker, this tells us a listener has declared ready/asking_failed
 * for one of our streams. Drives streaming start/stop decisions. */
int avb_process_msrp_listener(avb_state_s *state, msrp_msgbuf_s *msg,
                              int offset, size_t length,
                              eth_addr_t *src_addr) {
  msrp_listener_message_s listener_msg;
  memset(&listener_msg, 0, sizeof(listener_msg));
  memcpy(&listener_msg, &msg->messages_raw[offset],
         sizeof(msrp_listener_message_s));

  /* Decode the MRP attribute event and listener declaration */
  int attr_event = 0, unused1 = 0, unused2 = 0;
  three_pe_to_int(listener_msg.event_decl_data[0].event, &attr_event, &unused1,
                  &unused2);
  uint8_t listener_decl = listener_msg.event_decl_data[1].declaration.event1;

  /* Check if this listener declaration is for one of our output streams */
  for (int i = 0; i < state->num_output_streams; i++) {
    avb_talker_stream_s *stream = &state->output_streams[i];
    if (memcmp(listener_msg.stream_id, stream->stream_id, UNIQUE_ID_LEN) != 0)
      continue;

    /* Listener leaving — clear MSRP readiness, but keep ACMP identity/state.
     * ACMP DISCONNECT_TX is authoritative for removing the listener entry. */
    if (attr_event == mrp_attr_event_lv) {
      int idx = find_connected_listener(stream, src_addr);
      if (idx >= 0) {
        avbinfo("MSRP: listener leaving stream %d", i);
        stream->connected_listeners[idx].msrp_ready = false;
        stream->connected_listeners[idx].asking_failed = false;
        bool should_stop = !any_listener_ready(stream) ||
                           (!state->config.milan_compliant &&
                            !any_listener_acmp_connected(stream));
        if (should_stop && stream->streaming) {
          avb_stop_stream_out(state, i);
        }
      }
      return OK;
    }

    /* Listener ready — add/merge MSRP state. Only start once ACMP has also
     * connected this listener. This prevents stale periodic MSRP Ready
     * re-declarations from restarting a stream after ACMP disconnect/stop. */
    if (listener_decl == msrp_listener_event_ready) {
      int idx = find_or_add_msrp_listener(state, stream, src_addr);
      if (idx < 0) {
        avberr("MSRP: connected_listeners full for stream %d", i);
        return OK;
      }
      bool was_ready = stream->connected_listeners[idx].msrp_ready;
      stream->connected_listeners[idx].msrp_ready = true;
      if (!was_ready) {
        avbinfo("MSRP: listener ready for stream %d (count=%d, acmp=%d)", i,
                octets_to_uint(stream->connection_count, 2),
                stream->connected_listeners[idx].acmp_connected);
      }
      /* Clear any stale asking_failed state — the listener moved out of
       * that declaration and into Ready. */
      stream->connected_listeners[idx].asking_failed = false;
      /* Plain AVB starts only when both ACMP and MSRP agree that the listener is
       * connected/ready. Milan Talkers do not maintain ACMP listener state;
       * MSRP Listener Ready alone drives streaming. */
      bool should_start = state->config.milan_compliant
                              ? stream->connected_listeners[idx].msrp_ready
                              : stream->connected_listeners[idx].acmp_connected;
      if (should_start && !stream->streaming) {
        avb_start_stream_out(state, i);
      }
      return OK;
    }

    /* Listener asking_failed — track per-listener so ACMP
     * GET_TX_STATE_RESPONSE can set REGISTERING_FAILED per Milan
     * v1.3 Table 5.23. ready_failed is a transient MRP event with
     * the same meaning at the talker side; treat identically. */
    if (listener_decl == msrp_listener_event_asking_failed ||
        listener_decl == msrp_listener_event_ready_failed) {
      int idx = find_or_add_msrp_listener(state, stream, src_addr);
      if (idx < 0) {
        avberr("MSRP: connected_listeners full for stream %d", i);
        return OK;
      }
      if (!stream->connected_listeners[idx].msrp_ready) {
        /* New listener arriving directly in a failed state. Track it, but
         * don't start streaming. */
        stream->connected_listeners[idx].msrp_ready = false;
      }
      stream->connected_listeners[idx].asking_failed = true;
      avbinfo("MSRP: listener %s for stream %d",
              listener_decl == msrp_listener_event_asking_failed
                  ? "asking_failed"
                  : "ready_failed",
              i);
      return OK;
    }
    return OK;
  }

  return OK;
}

/* Process received AVTP IEC 61883 message */
int avb_process_iec_61883(avb_state_s *state, iec_61883_6_message_s *msg) {
  return OK;
}

/* Process received AVTP AAF PCM message */
int avb_process_aaf(avb_state_s *state, aaf_pcm_message_s *msg) {
  return OK;
}

/* Process received AVTP MAAP message */
int avb_process_maap(avb_state_s *state, maap_message_s *msg) {
  for (int i = 0; i < state->num_output_streams; i++) {
    maap_stream_state_s *m = &state->maap[i];

    /* Only handle messages that conflict with our address */
    if (!maap_addr_conflicts(&m->acquired_addr, (eth_addr_t *)msg->req_start_addr))
      continue;

    switch (msg->msg_type) {
    case maap_msg_type_probe:
      /* Someone is probing our address — defend it if we own it */
      if (m->state == maap_state_defend) {
        avbinfo("MAAP: defending stream %d address against probe", i);
        avb_send_maap_msg(state, maap_msg_type_defend,
                          (eth_addr_t *)msg->req_start_addr, 1,
                          &m->acquired_addr, 1);
      } else if (m->state == maap_state_probe) {
        /* We're also probing — lower MAC yields. Compare our MAC vs sender.
         * For simplicity, restart with a new address. */
        avbinfo("MAAP: probe conflict on stream %d, selecting new address", i);
        maap_generate_addr(state, &m->acquired_addr, i);
        m->probe_count = MAAP_PROBE_RETRANSMITS;
        m->timer_expiry_us =
            esp_timer_get_time() +
            maap_jitter_us(MAAP_PROBE_INTERVAL_BASE_MS,
                           MAAP_PROBE_INTERVAL_VAR_MS);
        avb_send_maap_msg(state, maap_msg_type_probe, &m->acquired_addr, 1,
                          NULL, 0);
      }
      break;

    case maap_msg_type_defend:
      /* Our probe was defended — pick a new address */
      if (m->state == maap_state_probe) {
        avbinfo("MAAP: probe defended on stream %d, selecting new address", i);
        maap_generate_addr(state, &m->acquired_addr, i);
        m->probe_count = MAAP_PROBE_RETRANSMITS;
        m->timer_expiry_us =
            esp_timer_get_time() +
            maap_jitter_us(MAAP_PROBE_INTERVAL_BASE_MS,
                           MAAP_PROBE_INTERVAL_VAR_MS);
        avb_send_maap_msg(state, maap_msg_type_probe, &m->acquired_addr, 1,
                          NULL, 0);
      }
      break;

    case maap_msg_type_announce:
      /* Conflicting announce — yield and restart with new address */
      if (m->state == maap_state_defend) {
        avbinfo("MAAP: conflicting announce on stream %d, restarting", i);
        m->acquired = false;
        maap_generate_addr(state, &m->acquired_addr, i);
        m->state = maap_state_probe;
        m->probe_count = MAAP_PROBE_RETRANSMITS;
        m->timer_expiry_us =
            esp_timer_get_time() +
            maap_jitter_us(MAAP_PROBE_INTERVAL_BASE_MS,
                           MAAP_PROBE_INTERVAL_VAR_MS);
        avb_send_maap_msg(state, maap_msg_type_probe, &m->acquired_addr, 1,
                          NULL, 0);
      }
      break;

    default:
      break;
    }
  }
  return OK;
}

/* ======================================================================
 * Bridge-only: MSRP admission control with 75 % link-rate cap per SR class
 *
 * Per IEEE 802.1Q-2018 §35.2.2.8.4, SR Class A is allotted at most
 * 75 % of the port bandwidth, and Class B shares that 75 % budget.
 * The bridge tracks per-port-per-class running bandwidth and rejects
 * a TALKER_ADVERTISE propagation when admitted + new would exceed the
 * cap. Hooked into avb_process_msrp_talker (Phase 5d) so an
 * over-budget request propagates as TALKER_FAILED with failure_code =
 * insufficient_bandwidth_for_traffic_class (avb.h:407).
 *
 * On Wi-Fi the bridge admits Class B only; Class A reservations
 * propagating from the wired side onto the wireless egress are
 * rejected at admission rather than dropped at forwarding time.
 *
 * Excluded from endpoint builds via CONFIG_ESP_AVB_ROLE_BRIDGE.
 * ====================================================================== */

#ifdef CONFIG_ESP_AVB_ROLE_BRIDGE

static avb_admission_class_s
    s_admission[CONFIG_ESP_AVB_NUM_PORTS][AVB_SR_CLASS_COUNT];

int avb_srp_admission_init(avb_state_s *state) {
  for (int p = 0; p < CONFIG_ESP_AVB_NUM_PORTS; p++) {
    for (int c = 0; c < AVB_SR_CLASS_COUNT; c++) {
      uint32_t link_rate = (state->port[p].medium == avb_port_medium_ethernet)
                               ? 1000000000u
                               : 0u;
      s_admission[p][c].link_rate_bps = link_rate;
      s_admission[p][c].admitted_bps = 0;
      s_admission[p][c].cap_bps = link_rate * 3u / 4u;
    }
  }
  ESP_LOGI("avb_srp", "MSRP admission init; 75%% cap per SR class per port");
  return 0;
}

void avb_srp_admission_stop(avb_state_s *state) { (void)state; }

int avb_srp_admission_try_admit(int port_index, avb_sr_class_e cls,
                                uint32_t request_bps) {
  if (port_index < 0 || port_index >= CONFIG_ESP_AVB_NUM_PORTS) return -1;
  if (cls < 0 || cls >= AVB_SR_CLASS_COUNT) return -1;
  avb_admission_class_s *e = &s_admission[port_index][cls];
  if (e->cap_bps == 0) {
    return -2; /* -ENOSPC: link rate not yet known (Wi-Fi port not up) */
  }
  if (e->admitted_bps + request_bps > e->cap_bps) {
    return -2; /* -ENOSPC: would exceed 75 % cap */
  }
  e->admitted_bps += request_bps;
  /* Push the new aggregate idle_slope into the FQTSS shaper so the
   * credit refill rate matches the admitted bandwidth. */
  avb_fqtss_set_idle_slope(port_index, cls, (int64_t)e->admitted_bps);
  return 0;
}

void avb_srp_admission_release(int port_index, avb_sr_class_e cls,
                               uint32_t request_bps) {
  if (port_index < 0 || port_index >= CONFIG_ESP_AVB_NUM_PORTS) return;
  if (cls < 0 || cls >= AVB_SR_CLASS_COUNT) return;
  avb_admission_class_s *e = &s_admission[port_index][cls];
  if (e->admitted_bps > request_bps) {
    e->admitted_bps -= request_bps;
  } else {
    e->admitted_bps = 0;
  }
  avb_fqtss_set_idle_slope(port_index, cls, (int64_t)e->admitted_bps);
}

#endif /* CONFIG_ESP_AVB_ROLE_BRIDGE */
