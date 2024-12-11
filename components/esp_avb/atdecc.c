#include "atdecc.h"

// Define logging tag
static const char *TAG = "DECC";

////////////
// ATDECC discovery messages
////////////

// Template entity available
static uint8_t frame_template_adp_entity_available[] = {
    0xfa, // avtp subtype: atdecc discovery (0xfa)
    0x00, // atvp stream id valid: false (0... ....), avtp version: 0x0 (.000 ....), msg type: entity available (.... 0000)
    0x40,0x38, // valid time: 8 (0100 0... .... ....),control data length: 56 (.... .000 0011 1000)
    0x15,0x98,0x77,0x40,0xc7,0x88,0x80,0x00, // entity id
    0x00,0x0d,0x93,0x00,0x00,0x00,0x00,0x08, // entity model id
    0x00,0x00,0x85,0x8a, // entity capabilities: ADRESS_ACCESS=true,AEM=true,VENDOR_UNIQUE=true,CLASS_A=true,gPTP=true
    0x00,0x01, // talker stream sources: 1
    0x48,0x01, // talker capabilities: IMPLEMENTED=true,MEDIA_CLOCK_SOURCE=true,AUDIO_SOURCE=true
    0x00,0x00, // listener stream sinks: 0
    0x48,0x01, // listener capabilities: IMPLEMENTED=true,MEDIA_CLOCK_SINK=true,AUDIO_SINK=true
    0x00,0x00,0x00,0x00, // controller capabilities: not implemented
    0x00,0x00,0x15,0x80, // available index
    0x14,0x98,0x77,0x40,0xc7,0x88,0x00,0x02, // gPTP grandmaster id
    0x00, // gptp_domain_number: 0
    0x00, // reserved
    0x00,0x00, // current_configuration_index: 0
    0x00,0x00, // identify_control_index: 0
    0x00,0x00, // interface_index: 0
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff, // association id
    0x00,0x00,0x00,0x00 // reserved
}; // 68 bytes

// Template entity departing
static uint8_t frame_template_adp_entity_departing[] = {};

// Template entity discover
static uint8_t frame_template_adp_entity_discover[] = {
    0xfa, // avtp subtype: atdecc discovery (0xfa)
    0x02, // atvp stream id valid: false (0... ....), avtp version: 0x0 (.000 ....), msg type: entity discover (.... 0010)
    0x40,0x38, // valid time: 8 (0100 0... .... ....),control data length: 56 (.... .000 0011 1000)
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // entity id
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // entity model id
    0x00,0x00,0x00,0x00, // entity capabilities: all false
    0x00,0x00, // talker stream sources: 0
    0x00,0x00, // talker capabilities: not implemented
    0x00,0x00, // listener stream sinks: 0
    0x00,0x00, // listener capabilities: not implemented
    0x00,0x00,0x00,0x00, // controller capabilities: not implemented
    0x00,0x00,0x00,0x00, // available index
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // gPTP grandmaster id
    0x00, // gptp_domain_number: 0
    0x00, // reserved
    0x00,0x00, // current_configuration_index: 0
    0x00,0x00, // identify_control_index: 0
    0x00,0x00, // interface_index: 0
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // association id
    0x00,0x00,0x00,0x00 // reserved
}; // 68 bytes

////////////
// ATDECC enumeration & control messages
////////////

// Note for atdecc: The control_data_length field for AECP is the number of octets following 
// the target_entity_id, but is limited to a maximum of 524.

// Template acquire entity command
static uint8_t frame_template_aecp_command_acquire_entity[] = {};

// Template acquire entity response
static uint8_t frame_template_aecp_response_acquire_entity[] = {};

// Template lock entity command
static uint8_t frame_template_aecp_command_lock_entity[] = {};

// Template lock entity response
static uint8_t frame_template_aecp_response_lock_entity[] = {};

// Template entity available command
static uint8_t frame_template_aecp_command_entity_available[] = {};

// Template entity available response
static uint8_t frame_template_aecp_response_entity_available[] = {};

// Template controller available command
static uint8_t frame_template_aecp_command_controller_available[] = {};

// Template controller available response
static uint8_t frame_template_aecp_response_controller_available[] = {};

// Template get configuration command
static uint8_t frame_template_aecp_command_get_configuration[] = {
    0xfb, // avtp subtype: atdecc enum and control (0xfb)
    0x00, // stream id valid: false, avtp version: 0x0, message type: aem_command (0)
    0x00,0x0c, // status: success (0x00), control data length: 12
    0x15,0x98,0x77,0x40,0xc7,0x88,0x00,0x00, // target guid
    0x1d,0x57,0xdc,0x6f,0xb4,0x19,0x80,0x00, // controller guid
    0x0a,0x74, // sequence id: 2676
    0x00,0x07, // u flag: false, command type: get_configuration (0x0007)
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // padding (not counted in control data length)
    0x00,0x00,0x00,0x00,0x00,0x00 // padding (not counted in control data length)
}; // 46 bytes

// Template get configuration response
static uint8_t frame_template_aecp_response_get_configuration[] = {
    0xfb, // avtp subtype: atdecc enum and control (0xfb)
    0x01, // stream id valid: false, avtp version: 0x0, message type: aem_response (1)
    0x00,0x22, // status: success (0x00), control data length: 16
    0x1d,0x57,0xdc,0x6f,0xb4,0x19,0x80,0x00, // target guid
    0x15,0x98,0x77,0x40,0xc7,0x88,0x00,0x00, // controller guid
    0x00,0x47, // sequence id: 71
    0x00,0x07, // u flag: false, command type: get_configuration (0x0007)
    0x00,0x00, // reserved
    0x00,0x00, // configuration_index: 0
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // padding (not counted in control data length)
    0x00,0x00 // padding (not counted in control data length)
}; // 46 bytes

// Template read entity descriptor command
static uint8_t frame_template_aecp_command_read_descriptor_entity[] = {
    0xfb, // avtp subtype: atdecc enum and control (0xfb)
    0x00, // stream id valid: false, avtp version: 0x0, message type: aem_command (0)
    0x00,0x14, // status: success (0x00), control data length: 20
    0x15,0x98,0x77,0x40,0xc7,0x88,0x80,0x00, // target guid
    0x1c,0x57,0xdc,0x6f,0xb4,0x19,0x00,0x00, // controller guid
    0x01,0xc3, // sequence ID: 451
    0x00,0x04, // u flag: false, command type: read_descriptor (0x0004)
    0x00,0x00, // configuration: 0
    0x00,0x00, // reserved
    0x00,0x00, // descriptor type: entity (0x0000)
    0x00,0x00, // descriptor index: 0x0000
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 // padding (not counted in control data length)
}; // 46 bytes

// Template read entity descriptor response
static uint8_t frame_template_aecp_response_read_descriptor_entity[] = {
    0xfb, // avtp subtype: atdecc enum and control (0xfb)
    0x01, // stream id valid: false, avtp version: 0x0, message type: aem_response (1)
    0x01,0x48, // status: success (0x00), control data length: 328
    0x15,0x98,0x77,0x40,0xc7,0x88,0x80,0x00, // target guid
    0x1c,0x57,0xdc,0x6f,0xb4,0x19,0x00,0x00, // controller guid
    0x01,0xc3, // sequence ID: 451
    0x00,0x04, // u flag: flase, command type: read_descriptor (0x0004)
    0x00,0x00, // configuration: 0
    0x00,0x00, // reserved
    0x00,0x00, // descriptor type: entity (0x0000)
    0x00,0x00, // descriptor index: 0x0000
    0x15,0x98,0x77,0x40,0xc7,0x88,0x80,0x00, // entity ID
    0x00,0x0d,0x93,0x00,0x00,0x00,0x00,0x08, // entity model ID
    0x00,0x00,0x85,0x8a, // entity capabilities: address_access: true, aem: true, vendor_unique: true, class_a: true, gptp_support: true
    0x00,0x02, // talker stream sources: 2
    0x48,0x01, // talker capabilities: implemented: true, media_clock_source: true, audio_source: true
    0x00,0x02, // listener stream sinks: 2
    0x48,0x01, // listener capabilities: implemented: true, media_clock_sink: true, audio_sink: true
    0x00,0x00,0x00,0x00, // controller capabilities: implemented: false, layer3_proxy: false
    0x00,0x00,0x13,0xc9, // available index: 0x000013c9
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff, // association ID
    0x53,0x63,0x72,0x61,0x6d,0x62,0x6c,0x65,0x20,0x54,0x68,0x69,0x6e,0x67,0x00,0x00, // entity name: Scramble Thing
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // entity name: Scramble Thing
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // entity name: Scramble Thing
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // entity name: Scramble Thing
    0x00,0x00, // vendor name string (ptr): 0
    0x00,0x01, // model name string (ptr): 1
    0x56,0x65,0x72,0x73,0x69,0x6f,0x6e,0x20,0x31,0x34,0x2e,0x34,0x2e,0x31,0x20,0x28, // firmware version: Version 14.4.1 (Build 23E224)
    0x42,0x75,0x69,0x6c,0x64,0x20,0x32,0x33,0x45,0x32,0x32,0x34,0x29,0x00,0x00,0x00, // firmware version: Version 14.4.1 (Build 23E224)
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // firmware version: Version 14.4.1 (Build 23E224)
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // firmware version: Version 14.4.1 (Build 23E224)
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // group name: 
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // group name:
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // group name:
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // group name:
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // serial number: 
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // serial number: 
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // serial number: 
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // serial number: 
    0x00,0x01, // configurations count: 1
    0x00,0x00 // current configuration: 0
}; // 340 bytes

// Template read configuration descriptor command
static uint8_t frame_template_aecp_command_read_descriptor_configuration[] = {
    0xfb, // avtp subtype: atdecc enum and control (0xfb)
    0x00, // stream id valid: false, avtp version: 0x0, message type: aem_command (0)
    0x00,0x14, // status: success (0x00), control data length: 20
    0x15,0x98,0x77,0x40,0xc7,0x88,0x80,0x00, // target guid
    0x1c,0x57,0xdc,0x6f,0xb4,0x19,0x00,0x00, // controller guid
    0x01,0xc4, // sequence ID: 452
    0x00,0x04, // u flag: false, command type: read_descriptor (0x0004)
    0x00,0x00, // configuration: 0
    0x00,0x00, // ?
    0x00,0x01, // descriptor type: configuration (0x0001)
    0x00,0x07, // descriptor index: 0x0007
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 // padding
}; // 46 bytes

// Template read configuration descriptor response
static uint8_t frame_template_aecp_response_read_descriptor_configuration[] = {
    0xfb, // avtp subtype: atdecc enum and control (0xfb)
    0x01, // stream id valid: false, avtp version: 0x0, message type: aem_response (1)
    0x00,0x86, // status: success (0x00), control data length: 134 ?
    0x15,0x98,0x77,0x40,0xc7,0x88,0x80,0x00, // target guid
    0x1c,0x57,0xdc,0x6f,0xb4,0x19,0x00,0x00, // controller guid
    0x01,0xc4, // sequence ID: 452
    0x00,0x04, // u flag: false, command type: read_descriptor (0x0004)
    0x00,0x00, // configuration: 0
    0x00,0x00, // ?
    0x00,0x01, // descriptor type: configuration (0x0001)
    0x00,0x00, // descriptor index: 0x0000
    0x31,0x20,0x69,0x6e,0x2c,0x20,0x31,0x20,0x6f,0x75,0x74,0x2c,0x20,0x32,0x20,0x63, // object name: 1 in, 1 out, 2 ch streams
    0x68,0x20,0x73,0x74,0x72,0x65,0x61,0x6d,0x73,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // object name: 1 in, 1 out, 2 ch streams
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // object name: 1 in, 1 out, 2 ch streams
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // object name: 1 in, 1 out, 2 ch streams
    0x00,0x02, // localized description: 2 ?
    0x00,0x0b, // descriptor counts count: 8
    0x00,0x4a, // descriptor counts offset: 74 ?
    0x00,0x02, // descriptor type: audio_unit (0x0002)
    0x00,0x01, // count: 1
    0x00,0x05, // descriptor type: stream_input (0x0005)
    0x00,0x02, // count: 2
    0x00,0x06, // descriptor type: stream_output (0x0006)
    0x00,0x02, // count: 2
    0x00,0x09, // descriptor type: avb_interface (0x0009)
    0x00,0x01, // count: 1
    0x00,0x0a, // descriptor type: clock_source (0x000a)
    0x00,0x03, // count: 3
    0x00,0x24, // descriptor type: clock_domain (0x0024)
    0x00,0x01, // count: 1
    0x00,0x26, // descriptor type: timing (0x0026)
    0x00,0x01, // count: 1
    0x00,0x27, // descriptor type: ptp_instance (0x0027)
    0x00,0x01 // count: 1
}; // 146 bytes ?

// Template read audio_unit descriptor response
static uint8_t frame_template_aecp_response_read_descriptor_audio_unit[] = {
    0xfb, // avtp subtype: atdecc enum and control (0xfb)
    0x01, // stream id valid: false, avtp version: 0x0, message type: aem_response (1)
    0x00,0xb8, // status: success (0x00), control data length: 184 ?
    0x15,0x98,0x77,0x40,0xc7,0x88,0x80,0x00, // target guid
    0x1c,0x57,0xdc,0x6f,0xb4,0x19,0x00,0x00, // controller guid
    0x01,0xce, // sequence ID: 462
    0x00,0x04, // u flag: false, command type: read_descriptor (0x0004)
    0x00,0x00, // configuration: 0
    0x00,0x00, // ?
    0x00,0x02, // descriptor type: audio_unit (0x0002)
    0x00,0x00, // descriptor index: 0x0000
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // object name
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // object name
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // object name
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // object name
    0x00,0x03, // localized description: 3 ?
    0x00,0x00, // clock domain ID: 0
    0x00,0x01, // number of stream input ports: 1
    0x00,0x00, // base stream input port: 0
    0x00,0x01, // number of stream output ports: 1
    0x00,0x00, // base stream output port: 0
    0x00,0x00, // number of external input ports: 0
    0x00,0x00, // base external input port: 0
    0x00,0x00, // number of external output ports: 0
    0x00,0x00, // base external output port: 0
    0x00,0x00, // number of internal input ports: 0
    0x00,0x00, // base internal input port: 0
    0x00,0x00, // number of internal output ports: 0
    0x00,0x00, // base internal output port: 0
    0x00,0x00, // number of controls: 0
    0x00,0x00, // base control: 0
    0x00,0x00, // number of signal selectors: 0
    0x00,0x00, // base signal selector: 0
    0x00,0x00, // number of mixers: 0
    0x00,0x00, // base mixer: 0
    0x00,0x00, // number of matrices: 0
    0x00,0x00, // base matrix: 0
    0x00,0x00, // number of splitters: 0
    0x00,0x00, // base splitter: 0
    0x00,0x00, // number of combiners: 0
    0x00,0x00, // base combiner: 0
    0x00,0x00, // number of demultiplexers: 0
    0x00,0x00, // base demultiplexer: 0
    0x00,0x00, // number of multiplexers: 0
    0x00,0x00, // base multiplexer: 0
    0x00,0x00, // number of transcoders: 0
    0x00,0x00, // base transcoder: 0
    0x00,0x00, // number of control blocks: 0
    0x00,0x00, // base control block: 0
    0x00,0x00,0xbb,0x80, // current sample rate: 48000
    0x00,0x90, // sample rates offset: 144
    0x00,0x06, // sample rates count: 3
    0x00,0x00,0xac,0x44, // pull field: 1.0 (0x0), base freqency: 44100
    0x00,0x00,0xbb,0x80, // pull field: 1.0 (0x0), base freqency: 48000
    0x00,0x01,0x77,0x00, // pull field: 1.0 (0x0), base freqency: 96000
}; // 196 bytes ?

// Note for stream formats below: the ut field indicates that less channels can be used
// in a stream always set to 0 if its the current format

// nsr field values for sample rate:
// 0x0 = User specified
// 0x1 = 8 kHz
// 0x2 = 16 kHz
// 0x3 = 32 kHz
// 0x4 = 44.1 kHz
// 0x5 = 48 kHz
// 0x6 = 88.2 kHz
// 0x7 = 96 kHz
// 0x8 = 176.4 kHz
// 0x9 = 192 kHz
// 0xA = 24 kHz

// format field values for aaf stream:
// 0x0 = USER User specified PCM
// 0x1 = FLOAT_32BIT 32-bit floating point. See 7.2.3.2. PCM
// 0x2 = INT_32BIT 32-bit integer PCM
// 0x3 = INT_24BIT 24-bit integer PCM
// 0x4 = INT_16BIT 16-bit integer PCM
// 0x5 = AES3_32BIT 32-bit AES3 format. See 7.4 and Annex K. AES3

// Template read stream_input descriptor response (reduced number of formats)
static uint8_t frame_template_aecp_response_read_descriptor_stream_input[] = {
    0xfb, // avtp subtype: atdecc enum and control (0xfb)
    0x01, // stream id valid: false, avtp version: 0x0, message type: aem_response (1)
    0x00,0xb2, // status: success (0x00), control_data_length: 178
    0x15,0x98,0x77,0x40,0xc7,0x88,0x80,0x00, // target guid
    0x1c,0x57,0xdc,0x6f,0xb4,0x19,0x00,0x00, // controller guid
    0x01,0xcf, // sequence ID: 463
    0x00,0x04, // u flag: false, command type: read_descriptor (0x0004)
    0x00,0x00, // configuration: 0
    0x00,0x00, // reserved
    0x00,0x05, // descriptor type: stream_input (0x0005)
    0x00,0x00, // descriptor index: 0x0000
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // object name
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // object name
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // object name
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // object name
    0x00,0x0a, // localized description: 10 (see locale descriptor)
    0x00,0x00, // clock domain ID: 0
    0x20,0x02, // stream flags: class A: true
    0x02,0x05,0x02,0x18,0x00,0x40,0x60,0x00, // current stream format: AAF, 24/48, 32bit
    0x00,0x8a, // formats offset: 138
    0x00,0x10, // number of formats: 3
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // primary backup talker guid
    0x00,0x00, // primary backup talker unique ID
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // secondary backup talker guid
    0x00,0x00, // secondary backup talker unique ID
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // tertiary backup talker guid
    0x00,0x00, // tertiary backup talker unique ID
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // backedup talker guid
    0x00,0x00, // backedup talker unique ID
    0x00,0x00, // avb interface ID: 0
    0x0b,0x9f,0x76,0xc0, // mac ingress buffer length: 195000000 nanoseconds
    0x01,0x1a, // redundant_offset: 162 (end of descriptor)
    0x00,0x00, // number_of_redundant_streams: 0
    0x00,0x00, // timing: 0
    0x02, // 1st stream format: version: 0x0, subtype: 0x02 (AAF)
    0x04, // 1st stream format: ut: 0, nsr: 0x4 (44.1 kHz)
    0x02, // 1st stream format: format: 0x02 (int32, pcm)
    0x18, // 1st stream format: bit_depth: 24 bit (0x18)
    0x00,0x40,0x60,0x00, // 1st stream format: channels_per_frame: 1, samples_per_frame: 6
    0x02, // 2nd stream format: version: 0x0, subtype: 0x02 (AAF)
    0x05, // 2nd stream format: ut: 0, nsr: 0x5 (48 kHz)
    0x02, // 2nd stream format: format: 0x02 (int32, pcm)
    0x18, // 2nd stream format: bit_depth: 24 bit (0x18)
    0x00,0x40,0x60,0x00, // 2nd stream format: channels_per_frame: 1, samples_per_frame: 6
    0x02, // 3rd stream format: version: 0x0, subtype: 0x02 (AAF)
    0x07, // 3rd stream format: ut: 0, nsr: 0x7 (96 kHz)
    0x02, // 3rd stream format: format: 0x02 (int32, pcm)
    0x18, // 3rd stream format: bit_depth: 24 bit (0x18)
    0x00,0x40,0xc0,0x00 // 3rd stream format: channels_per_frame: 1, samples_per_frame: 12
}; // 190 bytes

// Template read stream_output descriptor response (reduced number of formats)
static uint8_t frame_template_aecp_response_read_descriptor_stream_output[] = {
    0xfb, // avtp subtype: atdecc enum and control (0xfb)
    0x01, // stream id valid: false, avtp version: 0x0, message type: aem_response (1)
    0x00,0xb2, // status: success (0x00), control_data_length: 178
    0x15,0x98,0x77,0x40,0xc7,0x88,0x80,0x00, // target guid
    0x1c,0x57,0xdc,0x6f,0xb4,0x19,0x00,0x00, // controller guid
    0x01,0xcf, // sequence ID: 463
    0x00,0x04, // u flag: false, command type: read_descriptor (0x0004)
    0x00,0x00, // configuration: 0
    0x00,0x00, // reserved
    0x00,0x06, // descriptor type: stream_output (0x0006)
    0x00,0x00, // descriptor index: 0x0000
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // object name
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // object name
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // object name
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // object name
    0x00,0x0c, // localized description: 12 (see locale descriptor)
    0x00,0x00, // clock domain ID: 0
    0x20,0x03, // stream flags: clock sync source: true, class A: true
    0x02,0x05,0x02,0x18,0x00,0x40,0x60,0x00, // current stream format: AAF, 24/48, 32bit
    0x00,0x8a, // formats offset: 138
    0x00,0x10, // number of formats: 3
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // primary backup talker guid
    0x00,0x00, // primary backup talker unique ID
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // secondary backup talker guid
    0x00,0x00, // secondary backup talker unique ID
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // tertiary backup talker guid
    0x00,0x00, // tertiary backup talker unique ID
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // backedup talker guid
    0x00,0x00, // backedup talker unique ID
    0x00,0x00, // avb interface ID: 0
    0x00,0x00,0x00,0x00, // mac egress buffer length: 0 nanoseconds
    0x01,0x1a, // redundant_offset: 162 (end of descriptor)
    0x00,0x00, // number_of_redundant_streams: 0
    0x00,0x00, // timing: 0
    0x02, // 1st stream format: version: 0x0, subtype: 0x02 (AAF)
    0x04, // 1st stream format: ut: 0, nsr: 0x4 (44.1 kHz)
    0x02, // 1st stream format: format: 0x02 (int32, pcm)
    0x18, // 1st stream format: bit_depth: 24 bit (0x18)
    0x00,0x40,0x60,0x00, // 1st stream format: channels_per_frame: 1, samples_per_frame: 6
    0x02, // 2nd stream format: version: 0x0, subtype: 0x02 (AAF)
    0x05, // 2nd stream format: ut: 0, nsr: 0x5 (48 kHz)
    0x02, // 2nd stream format: format: 0x02 (int32, pcm)
    0x18, // 2nd stream format: bit_depth: 24 bit (0x18)
    0x00,0x40,0x60,0x00, // 2nd stream format: channels_per_frame: 1, samples_per_frame: 6
    0x02, // 3rd stream format: version: 0x0, subtype: 0x02 (AAF)
    0x07, // 3rd stream format: ut: 0, nsr: 0x7 (96 kHz)
    0x02, // 3rd stream format: format: 0x02 (int32, pcm)
    0x18, // 3rd stream format: bit_depth: 24 bit (0x18)
    0x00,0x40,0xc0,0x00 // 3rd stream format: channels_per_frame: 1, samples_per_frame: 12
}; // 190 bytes

// Template read avb_interface descriptor response
static uint8_t frame_template_aecp_response_read_descriptor_avb_interface[] = {
    0xfb, // avtp subtype: atdecc enum and control (0xfb)
    0x01, // stream id valid: false, avtp version: 0x0, message type: aem_response (1)
    0x00,0x76, // status: success (0x00), control data length: 118
    0x15,0x98,0x77,0x40,0xc7,0x88,0x80,0x00, // target guid
    0x1c,0x57,0xdc,0x6f,0xb4,0x19,0x00,0x00, // controller guid
    0x01,0xce, // sequence ID: 462
    0x00,0x04, // u flag: false, command type: read_descriptor (0x0004)
    0x00,0x00, // configuration: 0
    0x00,0x00, // reserved
    0x00,0x09, // descriptor type: avb_interface (0x0009)
    0x00,0x00, // descriptor index: 0x0000
    0x45,0x74,0x68,0x65,0x72,0x6e,0x65,0x74,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // interface name: Ethernet
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // interface name: Ethernet
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // interface name: Ethernet
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // interface name: Ethernet
    0x00,0x06, // interface name string: 6
    0x14,0x98,0x77,0x40,0xc7,0x88, // mac address
    0x00,0x07, // interface flags: 7
    0x14,0x98,0x77,0x40,0xc7,0x88,0x00,0x02, // clock identity
    0x00, // priority1: 0
    0x00, // clock class: 0
    0x43,0x6a, // scaled log variance: 17258
    0x00, // clock accuracy: 0
    0x00, // priority2: 0
    0x00, // domain number: 0
    0xfd, // log sync interval: -3
    0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00 // padding
}; // 130 bytes

// Template read clock_source descriptor response (3 are sent out by Apple)
static uint8_t frame_template_aecp_response_read_descriptor_clock_source[] = {
    0xfb, // avtp subtype: atdecc enum and control (0xfb)
    0x01, // stream id valid: false, avtp version: 0x0, message type: aem_response (1)
    0x00,0x66, // status: success (0x00), control data length: 102
    0x15,0x98,0x77,0x40,0xc7,0x88,0x80,0x00, // target guid
    0x1c,0x57,0xdc,0x6f,0xb4,0x19,0x00,0x00, // controller guid
    0x01,0xce, // sequence ID: 462
    0x00,0x04, // u flag: false, command type: read_descriptor (0x0004)
    0x00,0x00, // configuration: 0
    0x00,0x00, // reserved
    0x00,0x0a, // descriptor type: clock_source (0x000a)
    0x00,0x00, // descriptor index: 0x0000
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // object name
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // object name
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // object name
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // object name
    0x00,0x0a, // localized description: 10
    0x00,0x02, // clock source flags: 0x0002
    0x00,0x02, // clock source type: input stream (0x0002)
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff, // clock source ID
    0x00,0x05, // clock source location type: stream_input (0x0005)
    0x00,0x00 // clock source location ID: 0
}; // 114 bytes

// Template read clock_domain descriptor response
static uint8_t frame_template_aecp_response_read_descriptor_clock_domain[] = {
    0xfb, // avtp subtype: atdecc enum and control (0xfb)
    0x01, // stream id valid: false, avtp version: 0x0, message type: aem_response (1)
    0x00,0x62, // status: success (0x00), control data length: 98
    0x15,0x98,0x77,0x40,0xc7,0x88,0x80,0x00, // target guid
    0x1c,0x57,0xdc,0x6f,0xb4,0x19,0x00,0x00, // controller guid
    0x01,0xce, // sequence ID: 462
    0x00,0x04, // u flag: false, command type: read_descriptor (0x0004)
    0x00,0x00, // configuration: 0
    0x00,0x00, // reserved
    0x00,0x24, // descriptor type: clock_domain (0x0024)
    0x00,0x00, // descriptor index: 0x0000
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // object name
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // object name
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // object name
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // object name
    0x00,0x11, // localized description: 17
    0x00,0x02, // clock source index: 2
    0x00,0x4c, // clock sources offset: 76
    0x00,0x03, // clock sources count: 3
    0x00,0x00, // clock sources array item: 0
    0x00,0x01, // clock sources array item: 1
    0x00,0x02 // clock sources array item: 2
}; // 110 bytes

// Template read timing descriptor response
static uint8_t frame_template_aecp_response_read_descriptor_timing[] = {
    0xfb, // avtp subtype: atdecc enum and control (0xfb)
    0x01, // stream id valid: false, avtp version: 0x0, message type: aem_response (1)
    0x00,0x5e, // status: success (0x00), control data length: 94
    0x15,0x98,0x77,0x40,0xc7,0x88,0x80,0x00, // target guid
    0x1c,0x57,0xdc,0x6f,0xb4,0x19,0x00,0x00, // controller guid
    0x01,0xce, // sequence ID: 462
    0x00,0x04, // u flag: false, command type: read_descriptor (0x0004)
    0x00,0x00, // configuration: 0
    0x00,0x00, // reserved
    0x00,0x26, // descriptor type: timing (0x0026)
    0x00,0x00, // descriptor index: 0x0000
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // object name
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // object name
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // object name
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // object name
    0x00,0x12, // localized description: 18
    0x00,0x00, // algorithm: 0 (single, only 1 instance is used for timing)
    0x00,0x4c, // ptp_instances_offset: 76
    0x00,0x01, // number of ptp instances: 1
    0x00,0x00 // 1st ptp_instance index: 0
}; // 106 bytes

// Template read ptp_instance descriptor response
static uint8_t frame_template_aecp_response_read_descriptor_ptp_instance[] = {
    0xfb, // avtp subtype: atdecc enum and control (0xfb)
    0x01, // stream id valid: false, avtp version: 0x0, message type: aem_response (1)
    0x00,0x6a, // status: success (0x00), control data length: 106
    0x15,0x98,0x77,0x40,0xc7,0x88,0x80,0x00, // target guid
    0x1c,0x57,0xdc,0x6f,0xb4,0x19,0x00,0x00, // controller guid
    0x01,0xce, // sequence ID: 462
    0x00,0x04, // u flag: false, command type: read_descriptor (0x0004)
    0x00,0x00, // configuration: 0
    0x00,0x00, // reserved
    0x00,0x27, // descriptor type: timing (0x0027)
    0x00,0x00, // descriptor index: 0x0000
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // object name
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // object name
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // object name
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // object name
    0x00,0x13, // localized description: 19
    0x14,0x98,0x77,0x40,0xc7,0x88,0x00,0x02, // clock_identity of this ptp instance
    0x80,0x00,0x00,0x00, // flags for this ptp instance: grandmaster capable (0x80000000)
    0x00,0x00, // number of controls within this ptp instance
    0x00,0x00, // base control (index of the first control descriptor)
    0x00,0x01, // number of ptp ports within this ptp instance
    0x00,0x00 // base ptp port (index of first ptp_port descriptor)
}; // 118 bytes

// Template set clock source unsolicited controller request
static uint8_t frame_template_aecp_response_set_clock_source[] = {
    0xfb, // avtp subtype: atdecc enum and control (0xfb)
    0x01, // stream id valid: false, avtp version: 0x0, message type: aem_response (1)
    0x00,0x14, // status: success (0x00), control data length: 20
    0x15,0x98,0x77,0x40,0xc7,0x88,0x80,0x00, // target guid
    0x1c,0x57,0xdc,0x6f,0xb4,0x19,0x00,0x00, // controller guid
    0x00,0x01, // sequence ID: 1
    0xc0,0x16, // u+cr flag: true (11.. ....), command type: set_clock_source (0x0016) (..00 0000 0001 0110)
    0x00,0x24, // descriptor_type: clock_domain (0x0024)
    0x00,0x00, // descriptor_index: 0
    0x00,0x02, // clock_source_index: 2
    0x00,0x00, // reserved
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // padding
    0x00,0x00,0x00,0x00,0x00,0x00 // padding
}; // 46 bytes

// Template register unsolicited notification command
static uint8_t frame_template_aecp_command_register_unsol_notification[] = {
    0xfb, // avtp subtype: atdecc enum and control (0xfb)
    0x00, // stream id valid: false, avtp version: 0x0, message type: aem_command (0)
    0x00,0x10, // status: success (0x00), control data length: 16
    0x1d,0x57,0xdc,0x6f,0xb4,0x19,0x80,0x00, // target guid
    0x15,0x98,0x77,0x40,0xc7,0x88,0x00,0x01, // controller guid
    0x03,0xcc, // sequence ID: 972
    0x00,0x24, // u+cr flag: false (00.. ....), command type: register_unsol_notification (0x0024) (..00 0000 0010 0100)
    0x00,0x00,0x00,0x01, // flags: time_limited: true
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // padding
    0x00,0x00,0x00,0x00,0x00,0x00 // padding
}; // 46 bytes

// Template register unsolicited notification response
static uint8_t frame_template_aecp_response_register_unsol_notification[] = {
    0xfb, // avtp subtype: atdecc enum and control (0xfb)
    0x01, // stream id valid: false, avtp version: 0x0, message type: aem_response (1)
    0x00,0x22, // status: success (0x00), control data length: 34
    0x1d,0x57,0xdc,0x6f,0xb4,0x19,0x80,0x00, // target guid
    0x15,0x98,0x77,0x40,0xc7,0x88,0x00,0x01, // controller guid
    0x03,0xcc, // sequence ID: 972
    0x00,0x24, // u+cr flag: false (00.. ....), command type: register_unsol_notification (0x0024) (..00 0000 0010 0100)
    0x00,0x00,0x00,0x01, // flags: time_limited: true
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // padding
    0x00,0x00,0x00,0x00,0x00,0x00 // padding
}; // 46 bytes

// Template deregister unsolicited notification command
static uint8_t frame_template_aecp_command_deregister_unsol_notification[] = {
    0xfb, // avtp subtype: atdecc enum and control (0xfb)
    0x00, // stream id valid: false, avtp version: 0x0, message type: aem_command (0)
    // TBD
}; // 46 bytes

// Template deregister unsolicited notification response
static uint8_t frame_template_aecp_response_deregister_unsol_notification[] = {
    0xfb, // avtp subtype: atdecc enum and control (0xfb)
    0x01, // stream id valid: false, avtp version: 0x0, message type: aem_response (1)
    // TBD
}; // 46 bytes

////////////
// ATDECC connection management messages
////////////

// Template connect tx command
static uint8_t frame_template_acmp_connect_tx_command[] = {
    0xfc, // avtp subtype: atdecc connection management (0xfc)
    0x00, // stream id valid: false, avtp version: 0x0, message type: connect_tx_command (0)
    0x00,0x2c, // status: success (0x00), control data length: 44
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // stream ID
    0x1d,0x57,0xdc,0x6f,0xb4,0x19,0x00,0x00, // controller ID
    0x1d,0x57,0xdc,0x6f,0xb4,0x19,0x00,0x00, // talker guid
    0x15,0x98,0x77,0x40,0xc7,0x88,0x80,0x00, // listener guid
    0x00,0x01, // talker unique ID: 1
    0x00,0x01, // listener unique ID: 1
    0x00,0x00,0x00,0x00,0x00,0x00, // destination mac addr: 00:00:00:00:00:00
    0x00,0x00, // connection count: 0
    0x02,0xa2, // sequence ID: 0x02a2
    0x00,0x00, // flags: all false
    0x00,0x00, // stream vlan ID: 0
    0x00,0x00 // connected_listeners_entries: 0
}; // 56 bytes

// Template connect tx response
static uint8_t frame_template_acmp_connect_tx_response[] = {
    0xfc, // avtp subtype: atdecc connection management (0xfc)
    0x01, // stream id valid: false, avtp version: 0x0, message type: connect_tx_response (1)
    0x00,0x2c, // status: success (0x00), control data length: 44
    0x14,0x98,0x77,0x40,0xc7,0x88,0x00,0x03, // stream ID
    0x1d,0x57,0xdc,0x6f,0xb4,0x19,0x00,0x00, // controller ID
    0x15,0x98,0x77,0x40,0xc7,0x88,0x80,0x00, // talker guid
    0x1d,0x57,0xdc,0x6f,0xb4,0x19,0x80,0x00, // listener guid
    0x00,0x01, // talker unique ID: 1
    0x00,0x01, // listener unique ID: 1
    0x91,0xe0,0xf0,0x00,0x5d,0x4c, // destination mac addr: IEEE1722 bcast
    0x00,0x01, // connection count: 1
    0x00,0x28, // sequence ID: 0x0028
    0x00,0x00, // flags: all false
    0x00,0x00, // stream vlan ID: 0
    0x00,0x00 // connected_listeners_entries: 0
}; // 56 bytes

// Template disconnect tx command
static uint8_t frame_template_acmp_disconnect_tx_command[] = {};

// Template disconnect tx response
static uint8_t frame_template_acmp_disconnect_tx_response[] = {};

// Template get tx state command
static uint8_t frame_template_acmp_get_tx_state_command[] = {};

// Template get tx state response
static uint8_t frame_template_acmp_get_tx_state_response[] = {};

// Template connect rx command
static uint8_t frame_template_acmp_connect_rx_command[] = {
    0xfc, // avtp subtype: atdecc connection management (0xfc)
    0x06, // stream id valid: false, avtp version: 0x0, message type: connect_rx_command (6)
    0x00,0x2c, // status: success (0x00), control data length: 44
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // stream ID
    0x15,0x98,0x77,0x40,0xc7,0x88,0x80,0x00, // controller ID
    0x15,0x98,0x77,0x40,0xc7,0x88,0x80,0x00, // talker guid
    0x1d,0x57,0xdc,0x6f,0xb4,0x19,0x80,0x00, // listener guid
    0x00,0x01, // talker unique ID: 1
    0x00,0x01, // listener unique ID: 1
    0x00,0x00,0x00,0x00,0x00,0x00, // destination mac addr: 00:00:00:00:00:00
    0x00,0x00, // connection count: 0
    0x02,0xc4, // sequence ID: 0x02c4
    0x00,0x00, // flags: all false
    0x00,0x00, // stream vlan ID: 0
    0x00,0x00 // connected_listeners_entries: 0
}; // 56 bytes

// Template connect rx response
static uint8_t frame_template_acmp_connect_rx_response[] = {
    0xfc, // avtp subtype: atdecc connection management (0xfc)
    0x07, // stream id valid: false, avtp version: 0x0, message type: connect_rx_response (7)
    00,0x2c, // status: success (0x00), control data length: 44
    0x14,0x98,0x77,0x40,0xc7,0x88,0x00,0x01, // stream ID
    0x1d,0x57,0xdc,0x6f,0xb4,0x19,0x00,0x00, // controller ID
    0x15,0x98,0x77,0x40,0xc7,0x88,0x80,0x00, // talker guid
    0x1d,0x57,0xdc,0x6f,0xb4,0x19,0x80,0x00, // listener guid
    0x00,0x00, // talker unique ID: 0
    0x00,0x00, // listener unique ID: 0
    0x91,0xe0,0xf0,0x00,0x5d,0x4b, // destination mac addr: IEEE1722 bcast
    0x00,0x01, // connection count: 1
    0x00,0x2d, // sequence ID: 0x002d
    0x00,0x00, // flags: all false
    0x00,0x00, // stream vlan ID: 0
    0x00,0x00 // connected_listeners_entries: 0
}; // 56 bytes

// Template disconnect rx command
static uint8_t frame_template_acmp_disconnect_rx_command[] = {};

// Template disconnect rx response
static uint8_t frame_template_acmp_disconnect_rx_response[] = {};

// Template get rx state command
static uint8_t frame_template_acmp_get_rx_state_command[] = {};

// Template get rx state response
static uint8_t frame_template_acmp_get_rx_state_response[] = {};

// Append adpdu to a frame based on message type and set the payload_size
void append_adpdu(adp_message_type_t type, eth_frame_t *frame) {
    switch (type) {
        case adp_message_type_entity_available:
            memcpy(frame->payload, frame_template_adp_entity_available, sizeof(frame_template_adp_entity_available));
            frame->payload_size = sizeof(frame_template_adp_entity_available);
            break;
        case adp_message_type_entity_departing:
            memcpy(frame->payload, frame_template_adp_entity_departing, sizeof(frame_template_adp_entity_departing));
            frame->payload_size = sizeof(frame_template_adp_entity_departing);
            break;
        case adp_message_type_entity_discover:
            memcpy(frame->payload, frame_template_adp_entity_discover, sizeof(frame_template_adp_entity_discover));
            frame->payload_size = sizeof(frame_template_adp_entity_discover);
            break;
        default:
            ESP_LOGE(TAG, "Can't create ADPDU, with uknown message type: 0x%x", type);
    }
}

// Append aecpdu to a frame based on message type and set the payload_size
void append_aecpdu(aecp_message_type_t type, eth_frame_t *frame) {
    static uint8_t *frame_template;
    static ssize_t frame_size = 0;
    static uint16_t command_code;
    memcpy(&command_code, frame->payload + 21, (2)); // 14 bits but left 2 bits are likely 0
    command_code = ntohs(command_code); // flip endianness
    switch (type) {
        case aecp_message_type_aem_command:
            switch (command_code) {
                case aecp_command_code_acquire_entity:
                    frame_template = frame_template_aecp_command_acquire_entity;
                    frame_size = sizeof(frame_template_aecp_command_acquire_entity);
                    break;
                case aecp_command_code_lock_entity:
                    frame_template = frame_template_aecp_command_lock_entity;
                    frame_size = sizeof(frame_template_aecp_command_lock_entity);
                    break;
                case aecp_command_code_entity_available:
                    frame_template = frame_template_aecp_command_entity_available;
                    frame_size = sizeof(frame_template_aecp_command_entity_available);
                    break;
                case aecp_command_code_controller_available:
                    frame_template = frame_template_aecp_command_controller_available;
                    frame_size = sizeof(frame_template_aecp_command_controller_available);
                    break;
                case aecp_command_code_read_descriptor:
                    frame_template = frame_template_aecp_command_read_descriptor_entity;
                    frame_size = sizeof(frame_template_aecp_command_read_descriptor_entity);
                    break;
                default:
                    ESP_LOGE(TAG, "Can't create AECP command, with unknown command code: 0x%x", command_code);
            }
            break;
        case aecp_message_type_aem_response:
            switch(command_code) {
                case aecp_command_code_acquire_entity:
                    frame_template = frame_template_aecp_response_acquire_entity;
                    frame_size = sizeof(frame_template_aecp_response_acquire_entity);
                    break;
                case aecp_command_code_lock_entity:
                    frame_template = frame_template_aecp_response_lock_entity;
                    frame_size = sizeof(frame_template_aecp_response_lock_entity);
                    break;
                case aecp_command_code_entity_available:
                    frame_template = frame_template_aecp_response_entity_available;
                    frame_size = sizeof(frame_template_aecp_response_entity_available);
                    break;
                case aecp_command_code_controller_available:
                    frame_template = frame_template_aecp_response_controller_available;
                    frame_size = sizeof(frame_template_aecp_response_controller_available);
                    break;
                case aecp_command_code_read_descriptor:
                    frame_template = frame_template_aecp_response_read_descriptor_entity;
                    frame_size = sizeof(frame_template_aecp_response_read_descriptor_entity);
                    break;
                default:
                    ESP_LOGE(TAG, "Can't create AECP response, with unknown command code: 0x%x", command_code);
            }
            break;
        default:
            ESP_LOGE(TAG, "Can't create AECPDU, with unknown message type: 0x%x", type);
    }
    if (frame_size > 0) {
        memcpy(frame->payload, frame_template, frame_size);
        frame->payload_size = frame_size;
    }
}

// Append acmpdu to a frame based on message type and set the payload_size
void append_acmpdu(acmp_message_type_t type, eth_frame_t *frame) {
    static uint8_t *frame_template = {};
    static ssize_t frame_size = 0;
    switch (type) {
        case acmp_message_type_connect_tx_command:
            frame_template = frame_template_acmp_connect_tx_command;
            frame_size = sizeof(frame_template_acmp_connect_tx_command);
            break;
        case acmp_message_type_connect_tx_response:
            frame_template = frame_template_acmp_connect_tx_response;
            frame_size = sizeof(frame_template_acmp_connect_tx_response);
            break;
        case acmp_message_type_disconnect_tx_command:
            frame_template = frame_template_acmp_disconnect_tx_command;
            frame_size = sizeof(frame_template_acmp_disconnect_tx_command);
            break;
        case acmp_message_type_disconnect_tx_response:
            frame_template = frame_template_acmp_disconnect_tx_response;
            frame_size = sizeof(frame_template_acmp_disconnect_tx_response);
            break;
	    case acmp_message_type_get_tx_state_command:
            frame_template = frame_template_acmp_get_tx_state_command;
            frame_size = sizeof(frame_template_acmp_get_tx_state_command);
            break;
	    case acmp_message_type_get_tx_state_response:
            frame_template = frame_template_acmp_get_tx_state_response;
            frame_size = sizeof(frame_template_acmp_get_tx_state_response);
            break;
	    case acmp_message_type_connect_rx_command:
            frame_template = frame_template_acmp_connect_rx_command;
            frame_size = sizeof(frame_template_acmp_connect_rx_command);
            break;
        case acmp_message_type_connect_rx_response:
            frame_template = frame_template_acmp_connect_rx_response;
            frame_size = sizeof(frame_template_acmp_connect_rx_response);
            break;
        case acmp_message_type_disconnect_rx_command:
            frame_template = frame_template_acmp_disconnect_rx_command;
            frame_size = sizeof(frame_template_acmp_disconnect_rx_command);
            break;
	    case acmp_message_type_disconnect_rx_response:
            frame_template = frame_template_acmp_disconnect_rx_response;
            frame_size = sizeof(frame_template_acmp_disconnect_rx_response);
            break;
	    case acmp_message_type_get_rx_state_command:
            frame_template = frame_template_acmp_get_rx_state_command;
            frame_size = sizeof(frame_template_acmp_get_rx_state_command);
            break;
	    case acmp_message_type_get_rx_state_response:
            frame_template = frame_template_acmp_get_rx_state_response;
            frame_size = sizeof(frame_template_acmp_get_rx_state_response);
            break;
        default:
            ESP_LOGE(TAG, "Can't create ACMPDU, with uknown message type: 0x%x", type);
    }
    if (frame_size > 0) {
        memcpy(frame->payload, frame_template, frame_size);
        frame->payload_size = frame_size;
    }
}

// Detect which kind of ATDECC frame it is; assumes Ethertype is AVTP
avb_frame_type_t detect_atdecc_frame_type(eth_frame_t *frame, ssize_t size) {
    avb_frame_type_t frame_type = avb_frame_unknown;
    if (size <= ETH_HEADER_LEN) {
        ESP_LOGI(TAG, "Can't detect frame, too small: %d bytes", size);
    }
    else {
        uint8_t subtype;
        uint8_t message_type;
        memcpy(&subtype, frame->payload, (1));
        memcpy(&message_type, frame->payload + 1, (1)); // 4 bits
        message_type = message_type & 0x0f; // mask out the left 4 bits
        switch(subtype) {
            case avtp_subtype_adp:
                switch (message_type) {
                    case adp_message_type_entity_available:
                        frame_type = avb_frame_adp_entity_available;
                        break;
                    case adp_message_type_entity_departing:
                        frame_type = avb_frame_adp_entity_departing;
                        break;
                    case adp_message_type_entity_discover:
                        frame_type = avb_frame_adp_entity_discover;
                        break;
                    default:
                        ESP_LOGI(TAG, "Can't detect ADP frame with unknown message type: 0x%x", message_type);
                }
                break;
            case avtp_subtype_aecp:
                uint16_t command_code;
                memcpy(&command_code, frame->payload + 21, (2)); // 14 bits but left 2 bits are likely 0
                command_code = ntohs(command_code); // flip endianness
                switch (message_type) {
                    case aecp_message_type_aem_command:
                        switch (command_code) {
                            case aecp_command_code_acquire_entity:
                                frame_type = avb_frame_aecp_command_acquire_entity;
                                break;
                            case aecp_command_code_lock_entity:
                                frame_type = avb_frame_aecp_command_lock_entity;
                                break;
                            case aecp_command_code_entity_available:
                                frame_type = avb_frame_aecp_command_entity_available;
                                break;
                            case aecp_command_code_controller_available:
                                frame_type = avb_frame_aecp_command_controller_available;
                                break;
                            case aecp_command_code_read_descriptor:
                                frame_type = avb_frame_aecp_command_read_descriptor;
                                break;
                            default:
                                ESP_LOGI(TAG, "Can't detect aecp command frame with unknown command type: 0x%x", command_code);
                        }
                        break;
                    case aecp_message_type_aem_response:
                        switch (command_code) {
                            case aecp_command_code_acquire_entity:
                                frame_type = avb_frame_aecp_response_acquire_entity;
                                break;
                            case aecp_command_code_lock_entity:
                                frame_type = avb_frame_aecp_response_lock_entity;
                                break;
                            case aecp_command_code_entity_available:
                                frame_type = avb_frame_aecp_response_entity_available;
                                break;
                            case aecp_command_code_controller_available:
                                frame_type = avb_frame_aecp_response_controller_available;
                                break;
                            case aecp_command_code_read_descriptor:
                                frame_type = avb_frame_aecp_response_read_entity;
                                break;
                            default:
                                ESP_LOGI(TAG, "Can't detect aecp response frame with unknown command type: 0x%x", command_code);
                        }
                        break;
                    default:
                        ESP_LOGI(TAG, "Can't detect aecp frame with unknown message type: 0x%x", message_type);
                }
                break;
            case avtp_subtype_acmp:
                switch (message_type) {
                    case acmp_message_type_connect_tx_command:
                        frame_type = avb_frame_acmp_connect_tx_command;
                        break;
                    case acmp_message_type_connect_tx_response:
                        frame_type = avb_frame_acmp_connect_tx_response;
                        break;
                    case acmp_message_type_disconnect_tx_command:
                        frame_type = avb_frame_acmp_disconnect_tx_command;
                        break;
                    case acmp_message_type_disconnect_tx_response:
                        frame_type = avb_frame_acmp_disconnect_tx_response;
                        break;
                    case acmp_message_type_get_tx_state_command:
                        frame_type = avb_frame_acmp_get_tx_state_command;
                        break;
                    case acmp_message_type_get_tx_state_response:
                        frame_type = avb_frame_acmp_get_tx_state_response;
                        break;
                    case acmp_message_type_connect_rx_command:
                        frame_type = avb_frame_acmp_connect_rx_command;
                        break;
                    case acmp_message_type_connect_rx_response:
                        frame_type = avb_frame_acmp_connect_rx_response;
                        break;
                    case acmp_message_type_disconnect_rx_command:
                        frame_type = avb_frame_acmp_disconnect_rx_command;
                        break;
                    case acmp_message_type_disconnect_rx_response:
                        frame_type = avb_frame_acmp_disconnect_rx_response;
                        break;
                    case acmp_message_type_get_rx_state_command:
                        frame_type = avb_frame_acmp_get_rx_state_command;
                        break;
                    case acmp_message_type_get_rx_state_response:
                        frame_type = avb_frame_acmp_get_rx_state_response;
                        break;
                    default:
                        ESP_LOGI(TAG, "Can't detect ACMP frame with unknown message type: 0x%x", message_type);
                }
                break;
            default:
                ESP_LOGI(TAG, "Can't detect AVTP frame with unknown subtype: 0x%x", subtype);
        }
    }
    return frame_type;
}

// Print out the frame details
void print_atdecc_frame(avb_frame_type_t type, eth_frame_t *frame, int format) {
    
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
                // all adp message types use the same format
                case avb_frame_adp_entity_available ... avb_frame_adp_entity_discover:
                    ESP_LOG_BUFFER_HEX("               subType", frame->payload, (1));
                    ESP_LOG_BUFFER_HEX(" streamValidVerMsgType", frame->payload + 1, (1));
                    ESP_LOG_BUFFER_HEX("  validTimeCtrlDataLen", frame->payload + 2, (2));
                    ESP_LOG_BUFFER_HEX("              entityID", frame->payload + 4, (8));
                    ESP_LOG_BUFFER_HEX("         entityModelID", frame->payload + 12, (8));
                    ESP_LOG_BUFFER_HEX("    entityCapabilities", frame->payload + 20, (4));
                    ESP_LOG_BUFFER_HEX("   talkerStreamSources", frame->payload + 24, (2));
                    ESP_LOG_BUFFER_HEX("    talkerCapabilities", frame->payload + 26, (2));
                    ESP_LOG_BUFFER_HEX("   listenerStreamSinks", frame->payload + 28, (2));
                    ESP_LOG_BUFFER_HEX("  listenerCapabilities", frame->payload + 30, (2));
                    ESP_LOG_BUFFER_HEX("controllerCapabilities", frame->payload + 32, (4));
                    ESP_LOG_BUFFER_HEX("        availableIndex", frame->payload + 36, (4));
                    ESP_LOG_BUFFER_HEX("     gptpGrandmasterID", frame->payload + 40, (8));
                    ESP_LOG_BUFFER_HEX("      gptpDomainNumber", frame->payload + 48, (1));
                    ESP_LOG_BUFFER_HEX("         reserved8bits", frame->payload + 49, (1));
                    ESP_LOG_BUFFER_HEX("    currentConfigIndex", frame->payload + 50, (2));
                    ESP_LOG_BUFFER_HEX("  identifyControlIndex", frame->payload + 52, (2));
                    ESP_LOG_BUFFER_HEX("        interfaceIndex", frame->payload + 54, (2));
                    ESP_LOG_BUFFER_HEX("         associationID", frame->payload + 56, (8));
                    ESP_LOG_BUFFER_HEX("        reserved32bits", frame->payload + 64, (4));
                    break;
                case avb_frame_aecp_command_acquire_entity ... avb_frame_aecp_response_acquire_entity:
                    ESP_LOG_BUFFER_HEX("               subType", frame->payload, (1));
                    ESP_LOG_BUFFER_HEX("             remainder", frame->payload + 1, (size - ETH_HEADER_LEN - 1));
                    // Need to break out all fields
                    break;
                case avb_frame_aecp_command_lock_entity ... avb_frame_aecp_response_lock_entity:
                    ESP_LOG_BUFFER_HEX("               subType", frame->payload, (1));
                    ESP_LOG_BUFFER_HEX("             remainder", frame->payload + 1, (size - ETH_HEADER_LEN - 1));
                    // Need to break out all fields
                    break;
                case avb_frame_aecp_command_entity_available:
                    ESP_LOG_BUFFER_HEX("               subType", frame->payload, (1));
                    ESP_LOG_BUFFER_HEX("             remainder", frame->payload + 1, (size - ETH_HEADER_LEN - 1));
                    // Need to break out all fields
                    break;
                case avb_frame_aecp_response_entity_available:
                    ESP_LOG_BUFFER_HEX("               subType", frame->payload, (1));
                    ESP_LOG_BUFFER_HEX("             remainder", frame->payload + 1, (size - ETH_HEADER_LEN - 1));
                    // Need to break out all fields
                    break;
                case avb_frame_aecp_command_controller_available ... avb_frame_aecp_response_controller_available:
                    ESP_LOG_BUFFER_HEX("               subType", frame->payload, (1));
                    ESP_LOG_BUFFER_HEX("             remainder", frame->payload + 1, (size - ETH_HEADER_LEN - 1));
                    // Need to break out all fields
                    break;
                case avb_frame_aecp_command_read_descriptor:
                    ESP_LOG_BUFFER_HEX("               subType", frame->payload, (1));
                    ESP_LOG_BUFFER_HEX("             remainder", frame->payload + 1, (size - ETH_HEADER_LEN - 1));
                    // Need to break out all fields
                    break;
                case avb_frame_aecp_response_read_entity:
                    ESP_LOG_BUFFER_HEX("               subType", frame->payload, (1));
                    ESP_LOG_BUFFER_HEX("             remainder", frame->payload + 1, (size - ETH_HEADER_LEN - 1));
                    // Need to break out all fields
                    break;
                case avb_frame_acmp_connect_tx_command ... avb_frame_acmp_get_rx_state_response:
                    ESP_LOG_BUFFER_HEX("               subType", frame->payload, (1));
                    ESP_LOG_BUFFER_HEX("             remainder", frame->payload + 1, (size - ETH_HEADER_LEN - 1));
                    // Need to break out all fields
                    break;
                default:
                    ESP_LOGI(TAG, "Can't print frame with unknown Ethertype.");
            }
            ESP_LOGI(TAG, "*** End of Frame ***");  
        }
    }
}

// Get the name of the ADP message type
const char* get_adp_message_type_name(adp_message_type_t message_type) 
{
    switch (message_type) {
        case adp_message_type_entity_available: return "ADP Entity Available";
        case adp_message_type_entity_departing: return "ADP Entity Departing";
        case adp_message_type_entity_discover: return "ADP Entity Discover";
        default: return "Unknown";
    }
}

// Get the name of the AECP command code
const char* get_aecp_command_code_name(aecp_command_code_t command_code) 
{
    switch (command_code) {
        case aecp_command_code_acquire_entity: return "AECP Acquire Entity";
        case aecp_command_code_lock_entity: return "AECP Lock Entity";
        case aecp_command_code_entity_available: return "AECP Entity Available";
        case aecp_command_code_controller_available: return "AECP Controller Available";
        case aecp_command_code_read_descriptor: return "AECP Read Descriptor";
        default: return "Unknown";
    }
}

// Get the name of the ACMP message type
const char* get_acmp_message_type_name(acmp_message_type_t message_type) 
{
    switch (message_type) {
        case acmp_message_type_connect_tx_command: return "ACMP Connect TX Command";
        case acmp_message_type_connect_tx_response: return "ACMP Connect TX Response";
        case acmp_message_type_disconnect_tx_command: return "ACMP Disconnect TX Command";
        case acmp_message_type_disconnect_tx_response: return "ACMP Disconnect TX Response";
        case acmp_message_type_get_tx_state_command: return "ACMP Get TX State Command";
        case acmp_message_type_get_tx_state_response: return "ACMP Get TX State Response";
        case acmp_message_type_connect_rx_command: return "ACMP Connect RX Command";
        case acmp_message_type_connect_rx_response: return "ACMP Connect RX Response";
        case acmp_message_type_disconnect_rx_command: return "ACMP Disconnect RX Command";
        case acmp_message_type_disconnect_rx_response: return "ACMP Disconnect RX Response";
        case acmp_message_type_get_rx_state_command: return "ACMP Get RX State Command";
        case acmp_message_type_get_rx_state_response: return "ACMP Get RX State Response";
        default: return "Unknown";
    }
}