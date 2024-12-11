#include "avtp.h"

// Define logging tag
static const char *TAG = "AVTP";

////////////
// MSRP messages
////////////

// Test msrp domain
static uint8_t frame_template_msrp_domain[] = {
    0x00, // protocol version
    0x04, // attribute type: domain (4)
    0x04, // attribute length: 4
    0x00,0x09, // attribute list length: 9
    0x20,0x01, // vector header: 0x2001 (leave all, number of values: 1)
    0x06, // sr class ID: sr class A (6)
    0x03, // sr class priority: 3
    0x00,0x02, // sr class VID: 2
    0x24, // attribute event: JoinIn (1)
    0x00,0x00, // end mark for list
    0x04, // attribute type: domain (4)
    0x04, // attribute length: 4
    0x00,0x09, // attribute list length: 9
    0x00,0x01, // vector header: 0x0001 (leave all: null, number of values: 1)
    0x05, // sr class ID: sr class B (5)
    0x02, // sr class priority: 2
    0x00,0x02, // sr class VID: 2
    0x24, // attribute event: JoinIn (1)
    0x00,0x00, // end mark for list
    0x00,0x00, // end mark for pdu
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // padding
    0x00,0x00,0x00,0x00,0x00 // padding
}; // 50 bytes

// Test msrp talker advertise
static uint8_t frame_template_msrp_talker_advertise[] = {
    0x00, // protocol version
    0x01, // attribute type: talker advertise (1)
    0x19, // attribute length: 25
    0x00,0x1d, // attribute list length: 29
    0x20,0x00, // vector header: 0x2000
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // stream id: 0
    0x00,0x00,0x00,0x00,0x00,0x00, // stream da: 00:00:00:00:00:00
    0x00,0x00, // vlan id: 0
    0x00,0x00, // tspec max frame size: 0
    0x00,0x00, // tspec max frame interval: 0
    0x00, // priority: unknown (0), rank: emergency (0), reserved: 0
    0x00,0x00,0x00,0x00, // accumulated latency:0
    0x00,0x00, // end mark for list
    0x00,0x00, // end mark for pdu
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 // padding
}; // 50 bytes

// Test msrp listener
static uint8_t frame_template_msrp_listener[] = {
    0x00, // protocol version
    0x03, // attribute type: listener (3)
    0x08, // attribute length: 8
    0x00,0x0c, // attribute list length: 12
    0x20,0x00, // vector header: 0x2000
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // stream id: 0
    0x00,0x00, // end mark for list
    0x00,0x00, // end mark for pdu
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // padding
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 // padding
}; // 50 bytes

////////////
// MVRP messages
////////////

// Template mvrp vlan identifier
static uint8_t frame_template_mvrp_vlan_identifier[] = {
    0x00, // protocol version
    0x01, // attribute type: vlan identifier (1)
    0x02, // attribute length: 2
    0x00,0x01, // vector header: 0x0001 (leave all event: null, number of values: 1)
    0x00,0x02, // vlan id: 2
    0x24, // attribute event: JoinIn (1)
    0x00,0x00, // end mark of list
    0x00,0x00, // end mark of pdu
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // padding
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // padding
    0x00,0x00,0x00,0x00,0x00,0x00 // padding
}; // 50 bytes

////////////
// AVTP streams
////////////

// Template AAF stream
static uint8_t frame_template_aaf_stream[] = {};

////////////
// MAAP messages
////////////

// Template maap probe
static uint8_t frame_template_maap_probe[] = {};

// Template maap defend
static uint8_t frame_template_maap_defend[] = {};

// Template maap announce
static uint8_t frame_template_maap_announce[] = {
    0xfe, // avtp subtype: MAAP (0xfe)
    0x03, // stream id value: false, avtp version: 0x0, msg type: maap_announce (0x3)
    0x08,0x10, // maap version: 0x01, data length: 0x010
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // stream id: 0
    0x91,0xe0,0xf0,0x00,0x3b,0xc0, // requested start addr: 1722-bcast
    0x00,0x40, // request count: 0x0040
    0x00,0x00,0x00,0x00,0x00,0x00, // conflict start addr: 00:00:00:00:00:00
    0x00,0x00, // conflict count: 0
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // padding
    0x00,0x00 // padding
}; // 46 bytes

// Append avtpdu to a frame based on message type and set the payload_size
void append_avtpdu(avtp_subtype_t type, eth_frame_t *frame) {
    switch (type) {
        case avtp_subtype_aaf:
            memcpy(frame->payload, frame_template_aaf_stream, sizeof(frame_template_aaf_stream));
            frame->payload_size = sizeof(frame_template_aaf_stream);
            break;
        case avtp_subtype_maap:
            uint8_t message_type;
            memcpy(&message_type, frame->payload + 1, (1)); // 4 bits
            message_type = message_type & 0x0f; // mask out the left 4 bits
            switch(message_type) {
                case maap_message_type_probe:
                    memcpy(frame->payload, frame_template_maap_probe, sizeof(frame_template_maap_probe));
                    frame->payload_size = sizeof(frame_template_maap_probe);
                    break;
                case maap_message_type_defend:
                    memcpy(frame->payload, frame_template_maap_defend, sizeof(frame_template_maap_defend));
                    frame->payload_size = sizeof(frame_template_maap_defend);
                    break;
                case maap_message_type_announce:
                    memcpy(frame->payload, frame_template_maap_announce, sizeof(frame_template_maap_announce));
                    frame->payload_size = sizeof(frame_template_maap_announce);
                    break;
                default:
                    ESP_LOGE(TAG, "Can't create MAAP frame with unkown message type: 0x%x", message_type);
            }
            break;
        default:
            ESP_LOGE(TAG, "Can't create AVTPDU with unkown subtype: 0x%x", type);
    }
}

// Append msrpdu to a frame based on attribute type and set the payload_size
void append_msrpdu(msrp_attribute_type_t type, eth_frame_t *frame) {
    switch (type) {
        case msrp_attribute_type_talker_advertise:
            memcpy(frame->payload, frame_template_msrp_talker_advertise, sizeof(frame_template_msrp_talker_advertise));
            frame->payload_size = sizeof(frame_template_msrp_talker_advertise);
            break;
        case msrp_attribute_type_listener:
            memcpy(frame->payload, frame_template_msrp_listener, sizeof(frame_template_msrp_listener));
            frame->payload_size = sizeof(frame_template_msrp_listener);
            break;
        case msrp_attribute_type_domain:
            memcpy(frame->payload, frame_template_msrp_domain, sizeof(frame_template_msrp_domain));
            frame->payload_size = sizeof(frame_template_msrp_domain);
            break;
        default:
            ESP_LOGE(TAG, "Can't create MSRPDU with unkown attribute type: 0x%x", type);
    }
}

// Append mvrpdu to a frame based on attribute type and set the payload_size
void append_mvrpdu(msrp_attribute_type_t type, eth_frame_t *frame) {
    switch (type) {
        case mvrp_attribute_type_vlan_identifier:
            memcpy(frame->payload, frame_template_mvrp_vlan_identifier, sizeof(frame_template_mvrp_vlan_identifier));
            frame->payload_size = sizeof(frame_template_mvrp_vlan_identifier);
            break;
        default:
            ESP_LOGE(TAG, "Can't create MVRPDU with unkown attribute type: 0x%x", type);
    }
}

// Detect which kind of frame it is; AVTP, MSRP or MVRP
avb_frame_type_t detect_avtp_frame_type(ethertype_t *type, eth_frame_t *frame, ssize_t size) {
    ethertype_t ethertype = *type;
    avb_frame_type_t frame_type = avb_frame_unknown;
    if (size <= ETH_HEADER_LEN) {
        ESP_LOGI(TAG, "Can't detect frame, too small: %d bytes", size);
    }
    else {
        uint8_t attribute_type; // Used for MSRP and MVRP
        memcpy(&attribute_type, frame->payload + 1, (1));
        switch(ethertype) {
            case ethertype_avtp:
                uint8_t subtype;
                memcpy(&subtype, frame->payload, (1));
                switch(subtype) {
                    case avtp_subtype_aaf: // only supporting AAF at this point
                        frame_type = avb_frame_avtp_stream;
                        break;
                    case avtp_subtype_maap:
                        uint8_t message_type;
                        memcpy(&message_type, frame->payload + 1, (1)); // 4 bits
                        message_type = message_type & 0x0f; // mask out the left 4 bits
                        switch (message_type) {
                            case 0x03: // maap announce
                                frame_type = avb_frame_maap_announce;
                                break;
                            default:
                                ESP_LOGI(TAG, "Can't detect maap frame with unknown message type: %d", message_type);
                        }
                        break;
                    default:
                        ESP_LOGI(TAG, "Can't detect avtp frame with unknown subtype: %d", subtype);
                }
                break;
            case ethertype_msrp:
                switch (attribute_type) {
                    case msrp_attribute_type_talker_advertise:
                        frame_type = avb_frame_msrp_talker_advertise;
                        break;
                    case msrp_attribute_type_talker_failed:
                        frame_type = avb_frame_msrp_talker_failed;
                        break;
                    case msrp_attribute_type_listener:
                        frame_type = avb_frame_msrp_listener;
                        break;
                    case msrp_attribute_type_domain:
                        frame_type = avb_frame_msrp_domain;
                        break;
                    default:
                        ESP_LOGI(TAG, "Can't detect msrp frame with unknown attribute type: %d", attribute_type);
                }
                break;
            case ethertype_mvrp:
                if (attribute_type == mvrp_attribute_type_vlan_identifier) {
                    frame_type = avb_frame_mvrp_vlan_identifier;
                }
                else {
                    ESP_LOGI(TAG, "Can't detect mvrp frame with unknown attribute type: %d", attribute_type);
                }
                break;
            default:
                ESP_LOGI(TAG, "Can't detect frame with unknown ethertype: %d", ethertype);
        }
    }
    return frame_type;
}

// Print out the frame details for AVTP streams, MSRP and MVRP frames
void print_avtp_frame(avb_frame_type_t type, eth_frame_t *frame, int format) {

    ssize_t size = frame->payload_size + ETH_HEADER_LEN;
    if (frame->payload_size < 2) {
        ESP_LOGE(TAG, "Can't print frame, payload is too small: %d", frame->payload_size);
    }
    else {
        if (format == 0) { // short
            ESP_LOGI(TAG, "%s from %s", get_frame_type_name(type), mac_address_to_string(frame->header.src.addr));
        }
        else {
            ESP_LOGI(TAG, "*** Print Frame - %s (%d) ***", get_frame_type_name(type), size);
                    ESP_LOG_BUFFER_HEX("           destination", &frame->header.dest, (6));
                    ESP_LOG_BUFFER_HEX("                source", &frame->header.src, (6));
                    ESP_LOG_BUFFER_HEX("             etherType", &frame->header.type, (2));
            switch (type) {
                case avb_frame_avtp_stream:
                    ESP_LOG_BUFFER_HEX("               subType", frame->payload, (1));
                    ESP_LOG_BUFFER_HEX("             remainder", frame->payload + 1, (size - ETH_HEADER_LEN - 1));
                    // Need to break out all fields
                    break;
                case avb_frame_maap_announce:
                    ESP_LOG_BUFFER_HEX("               subType", frame->payload, (1));
                    ESP_LOG_BUFFER_HEX("             remainder", frame->payload + 1, (size - ETH_HEADER_LEN - 1));
                    // Need to break out all fields
                    break;
                case avb_frame_msrp_domain:
                    ESP_LOG_BUFFER_HEX("       protocolVersion", frame->payload, (1));
                    ESP_LOG_BUFFER_HEX("         attributeType", frame->payload + 1, (1));
                    ESP_LOG_BUFFER_HEX("             remainder", frame->payload + 2, (size - ETH_HEADER_LEN - 2));
                    // Need to break out all fields
                    break;
                case avb_frame_msrp_talker_advertise:
                    ESP_LOG_BUFFER_HEX("       protocolVersion", frame->payload, (1));
                    ESP_LOG_BUFFER_HEX("         attributeType", frame->payload + 1, (1));
                    ESP_LOG_BUFFER_HEX("             remainder", frame->payload + 2, (size - ETH_HEADER_LEN - 2));
                    // Need to break out all fields
                    break;
                case avb_frame_msrp_listener:
                    ESP_LOG_BUFFER_HEX("       protocolVersion", frame->payload, (1));
                    ESP_LOG_BUFFER_HEX("         attributeType", frame->payload + 1, (1));
                    ESP_LOG_BUFFER_HEX("             remainder", frame->payload + 2, (size - ETH_HEADER_LEN - 2));
                    // Need to break out all fields
                    break;
                case avb_frame_mvrp_vlan_identifier:
                    ESP_LOG_BUFFER_HEX("       protocolVersion", frame->payload, (1));
                    ESP_LOG_BUFFER_HEX("         attributeType", frame->payload + 1, (1));
                    ESP_LOG_BUFFER_HEX("             remainder", frame->payload + 2, (size - ETH_HEADER_LEN - 2));
                    // Need to break out all fields
                    break;
                default:
                    ESP_LOGI(TAG, "Can't print frame with unknown Ethertype.");
            }
            ESP_LOGI(TAG, "*** End of Frame ***");
        }
    }
}