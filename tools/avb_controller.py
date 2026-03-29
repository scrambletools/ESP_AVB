#!/usr/bin/env python3
"""
AVB Controller - ATDECC controller for managing AVB stream connections.

Communicates using raw AF_PACKET sockets with ethertype 0x22F0 (AVTP/ATDECC).
Requires root privileges or CAP_NET_RAW capability.

Protocol references:
  - IEEE 1722-2016 (AVTP)
  - IEEE 1722.1-2021 (ATDECC: ADP, ACMP, AECP)

Usage:
  sudo python3 avb_controller.py discover
  sudo python3 avb_controller.py connect <talker_entity_id> <listener_entity_id>
  sudo python3 avb_controller.py disconnect <talker_entity_id> <listener_entity_id>
  sudo python3 avb_controller.py --interface eno1 discover
"""

import argparse
import fcntl
import os
import select
import socket
import struct
import sys
import time
import uuid

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

ETH_P_AVTP = 0x22F0
ETH_ADDR_LEN = 6
UNIQUE_ID_LEN = 8
ETH_HLEN = 14

# Multicast destination for ADP / ACMP
ACMP_MULTICAST = b"\x91\xe0\xf0\x01\x00\x00"

# AVTP subtypes
AVTP_SUBTYPE_ADP = 0xFA
AVTP_SUBTYPE_AECP = 0xFB
AVTP_SUBTYPE_ACMP = 0xFC

# ADP message types
ADP_MSG_ENTITY_AVAILABLE = 0
ADP_MSG_ENTITY_DEPARTING = 1
ADP_MSG_ENTITY_DISCOVER = 2

# ACMP message types
ACMP_MSG_CONNECT_TX_COMMAND = 0
ACMP_MSG_CONNECT_TX_RESPONSE = 1
ACMP_MSG_DISCONNECT_TX_COMMAND = 2
ACMP_MSG_DISCONNECT_TX_RESPONSE = 3
ACMP_MSG_GET_TX_STATE_COMMAND = 4
ACMP_MSG_GET_TX_STATE_RESPONSE = 5
ACMP_MSG_CONNECT_RX_COMMAND = 6
ACMP_MSG_CONNECT_RX_RESPONSE = 7
ACMP_MSG_DISCONNECT_RX_COMMAND = 8
ACMP_MSG_DISCONNECT_RX_RESPONSE = 9
ACMP_MSG_GET_RX_STATE_COMMAND = 10
ACMP_MSG_GET_RX_STATE_RESPONSE = 11

# AECP message types
AECP_MSG_AEM_COMMAND = 0
AECP_MSG_AEM_RESPONSE = 1

# AECP command codes
AECP_CMD_READ_DESCRIPTOR = 0x0004
AECP_CMD_SET_STREAM_FORMAT = 0x0008
AECP_CMD_GET_STREAM_INFO = 0x000F

# AEM descriptor types
AEM_DESC_TYPE_ENTITY = 0x0000
AEM_DESC_TYPE_STREAM_INPUT = 0x0005
AEM_DESC_TYPE_STREAM_OUTPUT = 0x0006

# Well-known stream format presets (8 bytes each)
# IEC 61883-6 AM824: subtype=0, vendor=0, format=0x10, sf=1, fdf_sfc, fdf_evt=0, dbs=8, ...
# AAF PCM: subtype=2, vendor=0, sample_rate, format=0x02, bit_depth=24, channels=8
# Well-known stream format presets (8 bytes, matching C bitfield layout on LE)
STREAM_FORMATS = {
    "am824-44.1k": bytes([0x00, 0xa0, 0x01, 0x08, 0x40, 0x00, 0x08, 0x00]),
    "am824-48k":   bytes([0x00, 0xa0, 0x02, 0x08, 0x40, 0x00, 0x08, 0x00]),
    "am824-96k":   bytes([0x00, 0xa0, 0x04, 0x08, 0x40, 0x00, 0x08, 0x00]),
    "aaf-44.1k":   bytes([0x02, 0x04, 0x02, 0x18, 0x02, 0x00, 0x60, 0x00]),
    "aaf-48k":     bytes([0x02, 0x05, 0x02, 0x18, 0x02, 0x00, 0x60, 0x00]),
    "aaf-96k":     bytes([0x02, 0x07, 0x02, 0x18, 0x02, 0x00, 0x60, 0x00]),
}
FORMAT_ALIASES = {"am824": "am824-48k", "aaf": "aaf-48k", "61883": "am824-48k"}

# ACMP control_data_len per IEEE 1722.1-2021
ACMP_CONTROL_DATA_LEN = 84

# SIOCGIFHWADDR ioctl number
SIOCGIFHWADDR = 0x8927

# ACMP status names
ACMP_STATUS_NAMES = {
    0: "SUCCESS",
    1: "LISTENER_UNKNOWN_ID",
    2: "TALKER_UNKNOWN_ID",
    3: "TALKER_DEST_MAC_FAIL",
    4: "TALKER_NO_STREAM_INDEX",
    5: "TALKER_NO_BANDWIDTH",
    6: "TALKER_EXCLUSIVE",
    7: "LISTENER_TALKER_TIMEOUT",
    8: "LISTENER_EXCLUSIVE",
    9: "STATE_UNAVAILABLE",
    10: "NOT_CONNECTED",
    11: "NO_SUCH_CONNECTION",
    12: "COULD_NOT_SEND_MESSAGE",
    13: "TALKER_MISBEHAVING",
    14: "LISTENER_MISBEHAVING",
    15: "RESERVED",
    16: "CONTROLLER_NOT_AUTHORIZED",
    17: "INCOMPATIBLE_REQUEST",
    31: "NOT_SUPPORTED",
}


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def format_entity_id(eid: bytes) -> str:
    """Format an 8-byte entity ID as colon-separated hex."""
    return ":".join(f"{b:02x}" for b in eid)


def parse_entity_id(s: str) -> bytes:
    """Parse a colon-separated hex entity ID string to 8 bytes."""
    parts = s.split(":")
    if len(parts) != 8:
        raise ValueError(f"Entity ID must be 8 colon-separated hex bytes, got: {s}")
    return bytes(int(p, 16) for p in parts)


def get_mac_address(interface: str) -> bytes:
    """Get the MAC address of a network interface."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        info = fcntl.ioctl(
            s.fileno(),
            SIOCGIFHWADDR,
            struct.pack("256s", interface.encode("utf-8")[:15]),
        )
        return info[18:24]
    finally:
        s.close()


def mac_to_entity_id(mac: bytes) -> bytes:
    """Derive an 8-byte entity ID from a 6-byte MAC address.

    Inserts 0xFF 0xFE in the middle (EUI-48 to EUI-64 conversion).
    """
    return mac[:3] + b"\xff\xfe" + mac[3:6]


def format_mac(mac: bytes) -> str:
    """Format a MAC address as colon-separated hex."""
    return ":".join(f"{b:02x}" for b in mac)


# ---------------------------------------------------------------------------
# ATDECC header encoding / decoding
# ---------------------------------------------------------------------------
# The ATDECC common header is 4 bytes:
#   byte 0: subtype (8 bits)
#   byte 1: sv(1) | version(3) | msg_type(4)    -- note bit ordering in C struct
#   byte 2: status_valtime(5) | control_data_len_h(3)
#   byte 3: control_data_len (low 8 bits)
#
# C struct uses bitfields with LSB-first ordering within each byte on
# little-endian (ESP32 / x86), so the wire layout from the code is:
#   byte 1 bits [7..4] = msg_type, [3..1] = version, [0] = sv
#     but on the wire (big-endian network order) it is:
#       bit7 = sv, bit6..4 = version, bit3..0 = msg_type
#   byte 2 bits [7..3] = status_valtime, [2..0] = control_data_len_h
#
# Looking at the C bitfield layout and how ESP-IDF (little-endian target)
# lays this out on the wire (the struct is memcpy'd directly):
#   byte 1: msg_type(4 high bits) | version(3 bits) | sv(1 low bit)
#           i.e. (msg_type << 4) | (version << 1) | sv
#   byte 2: control_data_len_h(3 high bits) | status_valtime(5 low bits)
#           i.e. (cdl_h << 5) | status_valtime
#
# Actually, C bitfield ordering on little-endian:
#   struct { uint8_t msg_type:4; uint8_t version:3; uint8_t sv:1; }
#   means msg_type is in bits 0-3, version in bits 3-5, sv in bit 7
#   So byte = (sv << 7) | (version << 4) | msg_type
#
#   struct { uint8_t control_data_len_h:3; uint8_t status_valtime:5; }
#   means cdl_h is in bits 0-2, status_valtime in bits 3-7
#   So byte = (status_valtime << 3) | cdl_h


def encode_atdecc_header(subtype: int, msg_type: int, sv: int, version: int,
                         status_valtime: int, control_data_len: int) -> bytes:
    """Encode a 4-byte ATDECC header."""
    cdl_h = (control_data_len >> 8) & 0x07
    cdl_l = control_data_len & 0xFF
    byte1 = (sv << 7) | (version << 4) | (msg_type & 0x0F)
    byte2 = (status_valtime << 3) | cdl_h
    return struct.pack("BBBB", subtype, byte1, byte2, cdl_l)


def decode_atdecc_header(data: bytes):
    """Decode a 4-byte ATDECC header.

    Returns (subtype, msg_type, sv, version, status_valtime, control_data_len).
    """
    subtype = data[0]
    byte1 = data[1]
    byte2 = data[2]
    cdl_l = data[3]

    sv = (byte1 >> 7) & 0x01
    version = (byte1 >> 4) & 0x07
    msg_type = byte1 & 0x0F

    status_valtime = (byte2 >> 3) & 0x1F
    cdl_h = byte2 & 0x07
    control_data_len = (cdl_h << 8) | cdl_l

    return subtype, msg_type, sv, version, status_valtime, control_data_len


# ---------------------------------------------------------------------------
# AECP AEM header encoding
# ---------------------------------------------------------------------------
# aecp_common_aem_s is 2 bytes:
#   byte 0: command_type_h(6 high bits) | unsolicited(1) | cr(1 low bit)
#   byte 1: command_type (low 8 bits)
#
# C bitfield: { uint8_t cr:1; uint8_t unsolicited:1; uint8_t command_type_h:6; }
# On little-endian: cr in bit 0, unsolicited in bit 1, command_type_h in bits 2-7
# So byte0 = (command_type_h << 2) | (unsolicited << 1) | cr

def encode_aecp_aem_header(command_type: int, cr: int = 0, unsolicited: int = 0) -> bytes:
    """Encode a 2-byte AECP AEM command header."""
    command_type_h = (command_type >> 8) & 0x3F
    command_type_l = command_type & 0xFF
    byte0 = (command_type_h << 2) | (unsolicited << 1) | cr
    return struct.pack("BB", byte0, command_type_l)


# ---------------------------------------------------------------------------
# Socket helpers
# ---------------------------------------------------------------------------

def open_raw_socket(interface: str) -> socket.socket:
    """Open a raw AF_PACKET socket bound to the given interface for AVTP."""
    try:
        sock = socket.socket(
            socket.AF_PACKET, socket.SOCK_RAW, socket.htons(ETH_P_AVTP)
        )
    except PermissionError:
        print("Error: raw sockets require root privileges (sudo) or CAP_NET_RAW.",
              file=sys.stderr)
        sys.exit(1)
    sock.bind((interface, ETH_P_AVTP))
    # Join the ATDECC multicast group
    # Use PACKET_ADD_MEMBERSHIP with struct packet_mreq
    ifindex = socket.if_nametoindex(interface)
    # struct packet_mreq: int ifindex, unsigned short type, unsigned short alen, unsigned char address[8]
    PACKET_MR_MULTICAST = 0
    SOL_PACKET = 263  # Linux constant, not always in Python's socket module
    PACKET_ADD_MEMBERSHIP = 1
    mreq = struct.pack("IHH8s", ifindex, PACKET_MR_MULTICAST, 6,
                        ACMP_MULTICAST + b"\x00\x00")
    sock.setsockopt(SOL_PACKET, PACKET_ADD_MEMBERSHIP, mreq)
    sock.setblocking(False)
    return sock


def send_frame(sock: socket.socket, interface: str, dest_mac: bytes,
               src_mac: bytes, payload: bytes) -> None:
    """Send an Ethernet frame with AVTP ethertype."""
    eth_header = struct.pack("!6s6sH", dest_mac, src_mac, ETH_P_AVTP)
    frame = eth_header + payload
    sock.sendto(frame, (interface, ETH_P_AVTP))


def recv_frame(sock: socket.socket, timeout: float = 0.1):
    """Receive an Ethernet frame. Returns (src_mac, payload) or None."""
    ready, _, _ = select.select([sock], [], [], timeout)
    if not ready:
        return None
    try:
        data = sock.recv(2048)
    except BlockingIOError:
        return None
    if len(data) < ETH_HLEN + 4:
        return None
    dst_mac = data[0:6]
    src_mac = data[6:12]
    ethertype = struct.unpack("!H", data[12:14])[0]
    if ethertype != ETH_P_AVTP:
        return None
    payload = data[ETH_HLEN:]
    return src_mac, payload


# ---------------------------------------------------------------------------
# ADP parsing
# ---------------------------------------------------------------------------

def parse_adp_entity(payload: bytes) -> dict:
    """Parse an ADP ENTITY_AVAILABLE message payload.

    ADP message layout (after ATDECC 4-byte header):
      bytes 0-7:   entity_id (8)
      bytes 8-15:  entity_model_id (8)
      bytes 16-19: entity_capabilities (4)
      bytes 20-21: talker_stream_sources (2)
      bytes 22-23: talker_capabilities (2)
      bytes 24-25: listener_stream_sinks (2)
      bytes 26-27: listener_capabilities (2)
      bytes 28-31: controller_capabilities (4)
      bytes 32-35: available_index (4)
      bytes 36-43: gptp_gm_id (8)
      byte  44:    gptp_domain_num
      byte  45:    reserved
      bytes 46-47: current_config_index (2)
      bytes 48-49: identify_control_index (2)
      bytes 50-51: interface_index (2)
      bytes 52-59: association_id (8)
      bytes 60-63: reserved (4)
    Total body: 64 bytes (header.control_data_len = 56 for entity summary part)
    """
    if len(payload) < 4:
        return None

    subtype, msg_type, sv, version, status_valtime, cdl = decode_atdecc_header(payload[:4])

    if subtype != AVTP_SUBTYPE_ADP:
        return None
    if msg_type != ADP_MSG_ENTITY_AVAILABLE:
        return None

    body = payload[4:]
    if len(body) < 56:
        return None

    entity_id = body[0:8]
    model_id = body[8:16]
    entity_caps = body[16:20]
    talker_stream_sources = struct.unpack("!H", body[20:22])[0]
    talker_caps = body[22:24]
    listener_stream_sinks = struct.unpack("!H", body[24:26])[0]
    listener_caps = body[26:28]
    controller_caps = body[28:32]
    available_index = struct.unpack("!I", body[32:36])[0]

    # Parse capability flags
    # talker_capabilities byte 0 (C bitfield on LE):
    #   bit 0: reserved, bit 1: other_source, bit 2: control_source,
    #   bit 3: media_clock_source, bit 4: smpte_source, bit 5: midi_source,
    #   bit 6: audio_source, bit 7: video_source
    # talker_capabilities byte 1:
    #   bit 0: implemented, bits 1-7: reserved
    talker_implemented = bool(talker_caps[1] & 0x01)
    talker_audio = bool(talker_caps[0] & 0x40)

    listener_implemented = bool(listener_caps[1] & 0x01)
    listener_audio = bool(listener_caps[0] & 0x40)

    # Controller implemented is in byte 3 bit 0 of controller_caps
    controller_implemented = bool(controller_caps[3] & 0x01)

    valid_time = status_valtime * 2  # seconds

    roles = []
    if talker_implemented:
        roles.append("Talker")
    if listener_implemented:
        roles.append("Listener")
    if controller_implemented:
        roles.append("Controller")

    return {
        "entity_id": entity_id,
        "model_id": model_id,
        "entity_id_str": format_entity_id(entity_id),
        "model_id_str": format_entity_id(model_id),
        "talker_implemented": talker_implemented,
        "talker_audio": talker_audio,
        "talker_stream_sources": talker_stream_sources,
        "listener_implemented": listener_implemented,
        "listener_audio": listener_audio,
        "listener_stream_sinks": listener_stream_sinks,
        "controller_implemented": controller_implemented,
        "roles": ", ".join(roles) if roles else "None",
        "available_index": available_index,
        "valid_time": valid_time,
    }


# ---------------------------------------------------------------------------
# AECP: READ_DESCRIPTOR for entity name
# ---------------------------------------------------------------------------

def build_aecp_read_descriptor(controller_id: bytes, target_id: bytes,
                               seq_id: int, descriptor_type: int,
                               descriptor_index: int = 0) -> bytes:
    """Build an AECP READ_DESCRIPTOR command message."""
    # AECP common: header(4) + target_entity_id(8) + controller_entity_id(8) + seq_id(2) = 22
    # AEM header: 2 bytes
    # configuration_index(2) + reserved(2) + descriptor_type(2) + descriptor_index(2) = 8
    # Total body (control_data_len) = 22 - 4 + 2 + 8 = 28
    # control_data_len = total after header = 18 + 2 + 8 = 28
    control_data_len = 28  # bytes after the 4-byte header

    header = encode_atdecc_header(AVTP_SUBTYPE_AECP, AECP_MSG_AEM_COMMAND,
                                  0, 0, 0, control_data_len)
    aem = encode_aecp_aem_header(AECP_CMD_READ_DESCRIPTOR)
    msg = (header
           + target_id
           + controller_id
           + struct.pack("!H", seq_id)
           + aem
           + struct.pack("!HH", 0, 0)  # configuration_index, reserved
           + struct.pack("!HH", descriptor_type, descriptor_index))
    return msg


def parse_aecp_read_descriptor_response(payload: bytes) -> dict:
    """Parse an AECP READ_DESCRIPTOR response to extract entity name."""
    if len(payload) < 4:
        return None

    subtype, msg_type, sv, version, status_valtime, cdl = decode_atdecc_header(payload[:4])
    if subtype != AVTP_SUBTYPE_AECP:
        return None
    if msg_type != AECP_MSG_AEM_RESPONSE:
        return None

    # AECP common after header: target(8) + controller(8) + seq(2) = 18
    # AEM header: 2
    # config_index(2) + reserved(2) + desc_type(2) + desc_index(2) = 8
    # Total to descriptor_data start: 4 + 18 + 2 + 8 = 32
    if len(payload) < 32:
        return None

    target_id = payload[4:12]
    seq_id = struct.unpack("!H", payload[20:22])[0]
    desc_type = struct.unpack("!H", payload[28:30])[0]
    desc_index = struct.unpack("!H", payload[30:32])[0]

    result = {
        "target_id": target_id,
        "seq_id": seq_id,
        "descriptor_type": desc_type,
        "descriptor_index": desc_index,
    }

    # For entity descriptor, the descriptor_data starts at offset 32.
    # Entity descriptor layout: summary(40) + detail(272)
    # detail starts at offset 32+40 = 72
    # detail: association_id(8) + entity_name(64) ...
    # entity_name starts at offset 72 + 8 = 80
    if desc_type == AEM_DESC_TYPE_ENTITY and len(payload) >= 80 + 64:
        name_bytes = payload[80:144]
        entity_name = name_bytes.split(b"\x00", 1)[0].decode("utf-8", errors="replace")
        result["entity_name"] = entity_name

    return result


# ---------------------------------------------------------------------------
# AECP: SET_STREAM_FORMAT
# ---------------------------------------------------------------------------

def build_aecp_set_stream_format(controller_id: bytes, target_id: bytes,
                                  seq_id: int, descriptor_type: int,
                                  descriptor_index: int,
                                  stream_format: bytes) -> bytes:
    """Build an AECP SET_STREAM_FORMAT command.

    Wire layout (aecp_stream_format_s = 36 bytes):
      header(4) + target_entity_id(8) + controller_entity_id(8) + seq_id(2)
      + aem_header(2) + descriptor_type(2) + descriptor_index(2) + stream_format(8)
    control_data_len = 36 - 4 = 32
    """
    control_data_len = 32

    header = encode_atdecc_header(AVTP_SUBTYPE_AECP, AECP_MSG_AEM_COMMAND,
                                  0, 0, 0, control_data_len)
    aem = encode_aecp_aem_header(AECP_CMD_SET_STREAM_FORMAT)
    msg = (header
           + target_id
           + controller_id
           + struct.pack("!H", seq_id)
           + aem
           + struct.pack("!HH", descriptor_type, descriptor_index)
           + stream_format[:8])
    return msg


def _resolve_mac(sock, interface: str, mac: bytes, entity_id: bytes,
                 duration: float = 2.0) -> bytes:
    """Resolve entity ID to MAC address via ADP, fall back to EUI-64 derivation."""
    start = time.monotonic()
    while time.monotonic() - start < duration:
        result = recv_frame(sock, timeout=0.2)
        if result is None:
            continue
        src_mac, payload = result
        if len(payload) >= 12 and payload[0] == AVTP_SUBTYPE_ADP:
            adp_eid = payload[4:12]
            if adp_eid == entity_id:
                return src_mac
    # Fallback: standard EUI-64 to EUI-48 (remove FF:FE at bytes 3-4)
    if entity_id[3:5] == b'\xff\xfe':
        return entity_id[:3] + entity_id[5:8]
    # Non-standard entity ID — use first 6 bytes as best guess
    return entity_id[:6]


def send_set_stream_format(sock, interface: str, mac: bytes, controller_id: bytes,
                           target_id: bytes, target_mac: bytes,
                           descriptor_type: int, descriptor_index: int,
                           stream_format: bytes, seq_id: int) -> bool:
    """Send SET_STREAM_FORMAT and wait for response. Returns True on success."""
    msg = build_aecp_set_stream_format(
        controller_id, target_id, seq_id,
        descriptor_type, descriptor_index, stream_format)
    send_frame(sock, interface, target_mac, mac, msg)

    # Wait for response
    start = time.monotonic()
    while time.monotonic() - start < 3.0:
        result = recv_frame(sock, timeout=0.2)
        if result is None:
            continue
        src_mac_rx, payload = result
        if len(payload) < 4:
            continue
        subtype, msg_type, sv, version, status_valtime, cdl = decode_atdecc_header(payload[:4])
        if subtype == AVTP_SUBTYPE_AECP and msg_type == AECP_MSG_AEM_RESPONSE:
            # Check it's a SET_STREAM_FORMAT response for our target
            if len(payload) >= 22 and payload[4:12] == target_id:
                status = status_valtime
                status_name = {0: "SUCCESS", 1: "NOT_IMPLEMENTED", 2: "NO_SUCH_DESCRIPTOR",
                               9: "NOT_SUPPORTED", 12: "ENTITY_MISBEHAVING"}.get(status, f"ERROR({status})")
                print(f"  SET_STREAM_FORMAT response from {format_entity_id(target_id)}: {status_name}")
                return status == 0

    print(f"  SET_STREAM_FORMAT timeout for {format_entity_id(target_id)}")
    return False


# ---------------------------------------------------------------------------
# ACMP message building
# ---------------------------------------------------------------------------

def build_acmp_message(msg_type: int, controller_id: bytes,
                       talker_id: bytes, listener_id: bytes,
                       talker_uid: int = 0, listener_uid: int = 0,
                       connection_count: int = 0, seq_id: int = 0,
                       stream_id: bytes = None) -> bytes:
    """Build an ACMP message.

    ACMP layout:
      header(4) + stream_id(8) + controller_entity_id(8) +
      talker_entity_id(8) + listener_entity_id(8) +
      talker_uid(2) + listener_uid(2) + stream_dest_addr(6) +
      connection_count(2) + seq_id(2) + flags(2) +
      stream_vlan_id(2) + conn_listeners_entries(2)
    Total body: 52 bytes, but IEEE 1722.1-2021 specifies control_data_len = 84
    """
    if stream_id is None:
        stream_id = b"\x00" * 8

    header = encode_atdecc_header(AVTP_SUBTYPE_ACMP, msg_type, 0, 0, 0,
                                  ACMP_CONTROL_DATA_LEN)
    msg = (header
           + stream_id                          # stream_id (8)
           + controller_id                      # controller_entity_id (8)
           + talker_id                          # talker_entity_id (8)
           + listener_id                        # listener_entity_id (8)
           + struct.pack("!H", talker_uid)      # talker_uid (2)
           + struct.pack("!H", listener_uid)    # listener_uid (2)
           + b"\x00" * 6                        # stream_dest_addr (6)
           + struct.pack("!H", connection_count) # connection_count (2)
           + struct.pack("!H", seq_id)          # seq_id (2)
           + struct.pack("!H", 0)               # flags (2)
           + struct.pack("!H", 0)               # stream_vlan_id (2)
           + struct.pack("!H", 0))              # conn_listeners_entries (2)

    # Pad to full ACMP size: header(4) + control_data_len(84) = 88 bytes total
    # But our ACMP wire size should be 4 (header) + 84 = 88
    # What we built: 4 + 8+8+8+8+2+2+6+2+2+2+2+2 = 4 + 52 = 56
    # Pad remaining 32 bytes (extended ACMP fields or padding)
    if len(msg) < 4 + ACMP_CONTROL_DATA_LEN:
        msg += b"\x00" * (4 + ACMP_CONTROL_DATA_LEN - len(msg))

    return msg


def parse_acmp_response(payload: bytes) -> dict:
    """Parse an ACMP response message."""
    if len(payload) < 56:
        return None

    subtype, msg_type, sv, version, status_valtime, cdl = decode_atdecc_header(payload[:4])
    if subtype != AVTP_SUBTYPE_ACMP:
        return None

    body = payload[4:]
    stream_id = body[0:8]
    controller_id = body[8:16]
    talker_id = body[16:24]
    listener_id = body[24:32]
    talker_uid = struct.unpack("!H", body[32:34])[0]
    listener_uid = struct.unpack("!H", body[34:36])[0]
    stream_dest_addr = body[36:42]
    connection_count = struct.unpack("!H", body[42:44])[0]
    seq_id = struct.unpack("!H", body[44:46])[0]
    flags = struct.unpack("!H", body[46:48])[0]
    stream_vlan_id = struct.unpack("!H", body[48:50])[0]

    status_name = ACMP_STATUS_NAMES.get(status_valtime, f"UNKNOWN({status_valtime})")

    return {
        "msg_type": msg_type,
        "status": status_valtime,
        "status_name": status_name,
        "stream_id": format_entity_id(stream_id),
        "controller_id": format_entity_id(controller_id),
        "talker_id": format_entity_id(talker_id),
        "listener_id": format_entity_id(listener_id),
        "talker_uid": talker_uid,
        "listener_uid": listener_uid,
        "stream_dest_addr": format_mac(stream_dest_addr),
        "connection_count": connection_count,
        "seq_id": seq_id,
        "flags": flags,
        "stream_vlan_id": stream_vlan_id,
    }


# ---------------------------------------------------------------------------
# AECP: GET_STREAM_INFO
# ---------------------------------------------------------------------------

def build_aecp_get_stream_info(controller_id: bytes, target_id: bytes,
                               seq_id: int, descriptor_type: int,
                               descriptor_index: int = 0) -> bytes:
    """Build an AECP GET_STREAM_INFO command."""
    # Uses aecp_aem_short_s format: common(22) + aem(2) + desc_type(2) + desc_index(2) = 28
    # control_data_len = 28 - 4 = 24
    control_data_len = 24

    header = encode_atdecc_header(AVTP_SUBTYPE_AECP, AECP_MSG_AEM_COMMAND,
                                  0, 0, 0, control_data_len)
    aem = encode_aecp_aem_header(AECP_CMD_GET_STREAM_INFO)
    msg = (header
           + target_id
           + controller_id
           + struct.pack("!H", seq_id)
           + aem
           + struct.pack("!HH", descriptor_type, descriptor_index))
    return msg


# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------

def cmd_discover(interface: str, duration: float = 5.0) -> None:
    """Discover AVB entities on the network via ADP."""
    mac = get_mac_address(interface)
    controller_id = mac_to_entity_id(mac)
    print(f"Interface:     {interface}")
    print(f"MAC address:   {format_mac(mac)}")
    print(f"Controller ID: {format_entity_id(controller_id)}")
    print(f"Discovering entities for {duration:.0f} seconds...\n")

    sock = open_raw_socket(interface)
    entities = {}  # entity_id_str -> entity info dict
    entity_names = {}  # entity_id_str -> name
    aecp_seq_id = 1
    name_requests_sent = set()  # entity_id_str

    start = time.monotonic()
    try:
        while time.monotonic() - start < duration:
            result = recv_frame(sock, timeout=0.1)
            if result is None:
                continue

            src_mac, payload = result
            if len(payload) < 4:
                continue

            subtype = payload[0]

            # Handle ADP
            if subtype == AVTP_SUBTYPE_ADP:
                entity = parse_adp_entity(payload)
                if entity is not None:
                    eid_str = entity["entity_id_str"]
                    entity["src_mac"] = format_mac(src_mac)
                    entities[eid_str] = entity

                    # Request entity name if not already done
                    if eid_str not in name_requests_sent:
                        name_requests_sent.add(eid_str)
                        msg = build_aecp_read_descriptor(
                            controller_id, entity["entity_id"],
                            aecp_seq_id, AEM_DESC_TYPE_ENTITY, 0
                        )
                        send_frame(sock, interface, src_mac, mac, msg)
                        aecp_seq_id += 1

            # Handle AECP responses (for entity names)
            elif subtype == AVTP_SUBTYPE_AECP:
                resp = parse_aecp_read_descriptor_response(payload)
                if resp and "entity_name" in resp:
                    tid_str = format_entity_id(resp["target_id"])
                    entity_names[tid_str] = resp["entity_name"]

    except KeyboardInterrupt:
        pass
    finally:
        sock.close()

    if not entities:
        print("No entities discovered.")
        return

    # Merge names
    for eid_str, info in entities.items():
        info["entity_name"] = entity_names.get(eid_str, "")

    # Print table
    print(f"{'Entity ID':<26} {'Name':<24} {'Model ID':<26} {'Roles':<22} {'MAC':<18} {'Streams'}")
    print("-" * 140)
    for eid_str, info in sorted(entities.items()):
        streams = []
        if info["talker_stream_sources"] > 0:
            streams.append(f"TX:{info['talker_stream_sources']}")
        if info["listener_stream_sinks"] > 0:
            streams.append(f"RX:{info['listener_stream_sinks']}")
        streams_str = ", ".join(streams) if streams else "-"

        print(f"{eid_str:<26} {info['entity_name']:<24} {info['model_id_str']:<26} "
              f"{info['roles']:<22} {info['src_mac']:<18} {streams_str}")

    print(f"\nTotal: {len(entities)} entity(ies) discovered.")


def cmd_stream_info(interface: str, entity_id_str: str) -> None:
    """Query stream info for all input and output streams of an entity."""
    mac = get_mac_address(interface)
    controller_id = mac_to_entity_id(mac)
    entity_id = parse_entity_id(entity_id_str)

    print(f"Interface:     {interface}")
    print(f"Entity ID:     {entity_id_str}")

    sock = open_raw_socket(interface)

    # Resolve MAC
    entity_mac = _resolve_mac(sock, interface, mac, entity_id, duration=3.0)
    print(f"Entity MAC:    {format_mac(entity_mac)}")

    seq_id = int(time.monotonic() * 1000) & 0xFFFF

    # Query output streams 0-3 and input streams 0-3
    for desc_type, desc_name in [(AEM_DESC_TYPE_STREAM_OUTPUT, "OUTPUT"),
                                  (AEM_DESC_TYPE_STREAM_INPUT, "INPUT")]:
        for idx in range(4):
            msg = build_aecp_get_stream_info(
                controller_id, entity_id, seq_id, desc_type, idx)
            send_frame(sock, interface, entity_mac, mac, msg)
            seq_id += 1

            # Wait for response
            start = time.monotonic()
            got_response = False
            while time.monotonic() - start < 2.0:
                result = recv_frame(sock, timeout=0.2)
                if result is None:
                    continue
                src_mac_rx, payload = result
                if len(payload) < 4:
                    continue
                subtype, msg_type, sv, version, status, cdl = decode_atdecc_header(payload[:4])
                if subtype != AVTP_SUBTYPE_AECP or msg_type != AECP_MSG_AEM_RESPONSE:
                    continue
                # Check target matches
                if len(payload) < 12 or payload[4:12] != entity_id:
                    continue

                got_response = True
                if status != 0:
                    # Non-zero status means no such descriptor — stop querying this type
                    break

                # Parse GET_STREAM_INFO response
                # After header(4) + target(8) + controller(8) + seq(2) + aem(2) = 24
                # descriptor_type(2) + descriptor_index(2) = 4
                # stream_info_flags(4) + stream_format(8) + stream_id(8) +
                # msrp_acc_lat(4) + dest_addr(6) + msrp_failure_code(1) + reserved(1) +
                # msrp_failure_bridge_id(8) + vlan_id(2)
                body = payload[28:]  # after header+aecp_common+aem+desc_type+desc_index
                if len(body) < 42:
                    print(f"\n  STREAM {desc_name}[{idx}]: response too short ({len(body)} bytes)")
                    break

                # Parse flags (4 bytes, little-endian bitfield layout)
                flags_raw = body[0:4]
                # Byte 0 (LE bitfield): connected(1), msrp_failure_valid(1),
                #   stream_dest_mac_valid(1), msrp_acc_lat_valid(1),
                #   stream_id_valid(1), stream_format_valid(1), reserved(2)
                connected = bool(flags_raw[0] & 0x01)
                msrp_fail_valid = bool(flags_raw[0] & 0x02)
                dest_mac_valid = bool(flags_raw[0] & 0x04)
                acc_lat_valid = bool(flags_raw[0] & 0x08)
                stream_id_valid = bool(flags_raw[0] & 0x10)
                fmt_valid = bool(flags_raw[0] & 0x20)
                # Byte 2: class_b(1), fast_connect(1), saved_state(1),
                #   streaming_wait(1), supports_encrypted(1), talker_failed(1), reserved(2)
                class_b = bool(flags_raw[2] & 0x01)
                streaming_wait = bool(flags_raw[2] & 0x08)
                talker_failed = bool(flags_raw[2] & 0x20)
                # Byte 3: no_srp(1)
                no_srp = bool(flags_raw[3] & 0x01)

                stream_format = body[4:12]
                stream_id = body[12:20]
                acc_latency = struct.unpack("!I", body[20:24])[0]
                dest_addr = body[24:30]
                msrp_fail_code = body[30]
                vlan_id = struct.unpack("!H", body[39:41])[0] if len(body) > 40 else 0

                # Determine format type from first byte
                fmt_subtype = stream_format[0] & 0x7F
                if fmt_subtype == 0x00:
                    # AM824
                    sfc = stream_format[2] & 0x07
                    dbs = stream_format[3]
                    sfc_names = {0:"32k", 1:"44.1k", 2:"48k", 3:"88.2k", 4:"96k"}
                    fmt_str = f"AM824 {sfc_names.get(sfc, '?')} DBS={dbs}"
                elif fmt_subtype == 0x02:
                    # AAF
                    sr_code = stream_format[1] & 0x0F
                    sr_names = {1:"8k",2:"16k",3:"32k",4:"44.1k",5:"48k",6:"88.2k",7:"96k"}
                    bit_depth = stream_format[3]
                    ch_h = stream_format[4]
                    ch_l = (stream_format[5] >> 6) & 0x03
                    channels = (ch_h << 2) | ch_l
                    fmt_str = f"AAF {sr_names.get(sr_code,'?')} {bit_depth}bit {channels}ch"
                else:
                    fmt_str = f"unknown (subtype=0x{fmt_subtype:02x})"

                print(f"\n  STREAM {desc_name}[{idx}]:")
                print(f"    Format:        {fmt_str} {'(valid)' if fmt_valid else '(invalid)'}")
                print(f"    Stream ID:     {format_entity_id(stream_id)} {'(valid)' if stream_id_valid else ''}")
                print(f"    Connected:     {connected}")
                print(f"    Dest MAC:      {format_mac(dest_addr)} {'(valid)' if dest_mac_valid else ''}")
                print(f"    VLAN ID:       {vlan_id}")
                print(f"    Class:         {'B' if class_b else 'A'}")
                print(f"    Talker Failed: {talker_failed}")
                print(f"    Streaming Wait:{streaming_wait}")
                print(f"    MSRP Fail:     code={msrp_fail_code} {'(valid)' if msrp_fail_valid else ''}")
                print(f"    Acc Latency:   {acc_latency}ns {'(valid)' if acc_lat_valid else ''}")
                print(f"    Raw format:    {stream_format.hex()}")
                break

            if not got_response:
                break  # no more streams of this type
            if status != 0:
                break

    sock.close()


def cmd_connect(interface: str, talker_id_str: str, listener_id_str: str,
                format_name: str = None) -> None:
    """Connect a talker to a listener via ACMP CONNECT_RX_COMMAND.

    If format_name is given, sets stream format on both talker (output) and
    listener (input) before connecting.
    """
    if format_name:
        # Resolve aliases
        format_name = FORMAT_ALIASES.get(format_name, format_name)
        if format_name not in STREAM_FORMATS:
            avail = ", ".join(sorted(list(STREAM_FORMATS.keys()) + list(FORMAT_ALIASES.keys())))
            print(f"Unknown format '{format_name}'. Available: {avail}")
            sys.exit(1)

        stream_format = STREAM_FORMATS[format_name]
        mac = get_mac_address(interface)
        controller_id = mac_to_entity_id(mac)
        talker_id = parse_entity_id(talker_id_str)
        listener_id = parse_entity_id(listener_id_str)

        print(f"Setting stream format to '{format_name}' on both endpoints...")

        sock = open_raw_socket(interface)
        seq_id = int(time.monotonic() * 1000) & 0xFFFF

        # We need the MAC addresses of the target devices to send AECP unicast.
        # First try to find them from a quick ADP discovery, fall back to
        # EUI-64 → EUI-48 derivation (remove FF:FE bytes 3-4).
        talker_mac = _resolve_mac(sock, interface, mac, talker_id, duration=2.0)
        listener_mac = _resolve_mac(sock, interface, mac, listener_id, duration=2.0)

        # Set format on talker output stream (descriptor_type=STREAM_OUTPUT, index=0)
        ok1 = send_set_stream_format(sock, interface, mac, controller_id,
                                      talker_id, talker_mac,
                                      AEM_DESC_TYPE_STREAM_OUTPUT, 0,
                                      stream_format, seq_id)
        seq_id += 1

        # Set format on listener input stream (descriptor_type=STREAM_INPUT, index=0)
        ok2 = send_set_stream_format(sock, interface, mac, controller_id,
                                      listener_id, listener_mac,
                                      AEM_DESC_TYPE_STREAM_INPUT, 0,
                                      stream_format, seq_id + 1)
        sock.close()

        if not ok1:
            print("Warning: failed to set format on talker")
        if not ok2:
            print("Warning: failed to set format on listener")
        print()

    _acmp_command(interface, talker_id_str, listener_id_str,
                  ACMP_MSG_CONNECT_RX_COMMAND, "CONNECT_RX_COMMAND",
                  ACMP_MSG_CONNECT_RX_RESPONSE, "CONNECT_RX_RESPONSE")


def cmd_disconnect(interface: str, talker_id_str: str, listener_id_str: str) -> None:
    """Disconnect a talker from a listener via ACMP DISCONNECT_RX_COMMAND."""
    _acmp_command(interface, talker_id_str, listener_id_str,
                  ACMP_MSG_DISCONNECT_RX_COMMAND, "DISCONNECT_RX_COMMAND",
                  ACMP_MSG_DISCONNECT_RX_RESPONSE, "DISCONNECT_RX_RESPONSE")


def _acmp_command(interface: str, talker_id_str: str, listener_id_str: str,
                  cmd_type: int, cmd_name: str,
                  rsp_type: int, rsp_name: str) -> None:
    """Send an ACMP command and wait for a response."""
    mac = get_mac_address(interface)
    controller_id = mac_to_entity_id(mac)
    talker_id = parse_entity_id(talker_id_str)
    listener_id = parse_entity_id(listener_id_str)

    print(f"Interface:     {interface}")
    print(f"Controller ID: {format_entity_id(controller_id)}")
    print(f"Talker ID:     {talker_id_str}")
    print(f"Listener ID:   {listener_id_str}")
    print(f"Sending {cmd_name}...")

    sock = open_raw_socket(interface)
    seq_id = int(time.monotonic() * 1000) & 0xFFFF

    msg = build_acmp_message(
        msg_type=cmd_type,
        controller_id=controller_id,
        talker_id=talker_id,
        listener_id=listener_id,
        talker_uid=0,
        listener_uid=0,
        connection_count=1,
        seq_id=seq_id,
    )

    send_frame(sock, interface, ACMP_MULTICAST, mac, msg)

    # Wait for response (timeout 5 seconds)
    timeout = 5.0
    start = time.monotonic()
    try:
        while time.monotonic() - start < timeout:
            result = recv_frame(sock, timeout=0.2)
            if result is None:
                continue

            src_mac, payload = result
            if len(payload) < 4:
                continue

            subtype = payload[0]
            if subtype != AVTP_SUBTYPE_ACMP:
                continue

            resp = parse_acmp_response(payload)
            if resp is None:
                continue

            # Accept any ACMP response related to our command
            if resp["msg_type"] in (rsp_type,
                                     ACMP_MSG_CONNECT_TX_RESPONSE,
                                     ACMP_MSG_DISCONNECT_TX_RESPONSE):
                rtype_name = {
                    ACMP_MSG_CONNECT_RX_RESPONSE: "CONNECT_RX_RESPONSE",
                    ACMP_MSG_DISCONNECT_RX_RESPONSE: "DISCONNECT_RX_RESPONSE",
                    ACMP_MSG_CONNECT_TX_RESPONSE: "CONNECT_TX_RESPONSE",
                    ACMP_MSG_DISCONNECT_TX_RESPONSE: "DISCONNECT_TX_RESPONSE",
                }.get(resp["msg_type"], f"type={resp['msg_type']}")

                print(f"\nReceived {rtype_name}:")
                print(f"  Status:           {resp['status_name']}")
                print(f"  Stream ID:        {resp['stream_id']}")
                print(f"  Talker:           {resp['talker_id']}")
                print(f"  Listener:         {resp['listener_id']}")
                print(f"  Stream Dest MAC:  {resp['stream_dest_addr']}")
                print(f"  Connection Count: {resp['connection_count']}")
                print(f"  VLAN ID:          {resp['stream_vlan_id']}")

                if resp["msg_type"] == rsp_type:
                    sock.close()
                    if resp["status"] == 0:
                        print("\nCommand completed successfully.")
                    else:
                        print(f"\nCommand failed: {resp['status_name']}")
                        sys.exit(1)
                    return

        print(f"\nTimeout: no {rsp_name} received within {timeout:.0f} seconds.")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nAborted.")
    finally:
        sock.close()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="AVB Controller - manage AVB stream connections via ATDECC",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
examples:
  sudo python3 avb_controller.py discover
  sudo python3 avb_controller.py connect 00:1b:21:ff:fe:01:02:03 00:1b:21:ff:fe:04:05:06
  sudo python3 avb_controller.py disconnect 00:1b:21:ff:fe:01:02:03 00:1b:21:ff:fe:04:05:06
  sudo python3 avb_controller.py --interface eno1 discover
""",
    )
    parser.add_argument(
        "--interface", "-i", default="enp5s0f3u4",
        help="Network interface to use (default: enp5s0f3u4)",
    )

    subparsers = parser.add_subparsers(dest="command", required=True)

    # discover
    sp_discover = subparsers.add_parser("discover", help="Discover AVB entities on the network")
    sp_discover.add_argument(
        "--duration", "-d", type=float, default=5.0,
        help="Discovery duration in seconds (default: 5)",
    )

    # connect
    sp_connect = subparsers.add_parser("connect", help="Connect a talker to a listener")
    sp_connect.add_argument("talker_entity_id", help="Talker entity ID (colon-separated hex)")
    sp_connect.add_argument("listener_entity_id", help="Listener entity ID (colon-separated hex)")
    sp_connect.add_argument("--format", "-f", default=None,
                           help="Stream format to set on both endpoints before connecting. "
                                "Options: am824-48k, am824-44.1k, am824-96k, aaf-48k, aaf-44.1k, aaf-96k, "
                                "am824 (alias for am824-48k), aaf (alias for aaf-48k), 61883 (alias for am824-48k)")

    # disconnect
    sp_disconnect = subparsers.add_parser("disconnect", help="Disconnect a talker from a listener")
    sp_disconnect.add_argument("talker_entity_id", help="Talker entity ID (colon-separated hex)")
    sp_disconnect.add_argument("listener_entity_id", help="Listener entity ID (colon-separated hex)")

    # stream-info
    sp_info = subparsers.add_parser("stream-info", help="Query stream info for an entity")
    sp_info.add_argument("entity_id", help="Entity ID to query (colon-separated hex)")

    args = parser.parse_args()

    # Check we are running as root
    if os.geteuid() != 0:
        print("Warning: this tool requires root privileges for raw socket access.",
              file=sys.stderr)
        print("Run with: sudo python3 avb_controller.py ...", file=sys.stderr)

    if args.command == "discover":
        cmd_discover(args.interface, args.duration)
    elif args.command == "connect":
        cmd_connect(args.interface, args.talker_entity_id, args.listener_entity_id,
                    format_name=args.format)
    elif args.command == "disconnect":
        cmd_disconnect(args.interface, args.talker_entity_id, args.listener_entity_id)
    elif args.command == "stream-info":
        cmd_stream_info(args.interface, args.entity_id)


if __name__ == "__main__":
    main()
