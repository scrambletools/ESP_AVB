#ifndef ESP_AVB_SIMPLE_TALKER_INCLUDE_CONFIG_H_
#define ESP_AVB_SIMPLE_TALKER_INCLUDE_CONFIG_H_

// Configuration options for an AVB audio talker (single stream)
#define CONFIG_ENTITY_ID 0xf3c0401c5c114d2a // uint64_t
#define CONFIG_ENTITY_MODEL_ID 0x0000000000000000 // uint64_t
#define CONFIG_ASSOCIATION_ID 0x0000000000000000 // uint64_t
#define CONFIG_CLOCK_ID 0x0001a2b3c4d5e6f7 // uint64_t
#define CONFIG_PORT_ID 0x0001 // uint16_t
#define CONFIG_STREAM_ID 0x0000000000000000 // uint64_t
#define CONFIG_ENTITY_NAME "Simple Talker"
#define CONFIG_VENDOR_NAME "ACME"
#define CONFIG_MODEL_NAME "ESP_AVB Simple Talker"
#define CONFIG_GROUP_NAME ""
#define CONFIG_FIRMWARE_VERSION ""
#define CONFIG_SERIAL_NUMBER ""
#define CONFIG_AVB_INTERFACE_NAME "AVB Interface"
#define CONFIG_STREAM_FORMAT 0x0205021800406000 // uint64_t (AAF, 24/48, 32bit)
#define CONFIG_USE_NTP false // Use NTP to set system time
#define CONFIG_SYSTEM_LATENCY_COMP 250 // Timestamp compensation (for testing on sw timestamp systems)
#define CONFIG_GM_CAPABLE false // Will announce GM capability
#define CONFIG_PDELAY_REQUEST_PERIOD 1e3 // Pdelay request period in msec  
#define CONFIG_ANNOUNCE_PERIOD 3e3 // Announce period in msec
#define CONFIG_SYNC_PERIOD 125 // Sync period in msec
#define CONFIG_GM_TIMEOUT 3 // Timeout for GM (missing announcements)
#define CONFIG_PDELAY_LIST_SIZE 3 // Number of recent pDelay interactions to keep in list
#define CONFIG_SYNC_LIST_SIZE 3 // Number of recent sync actions to keep in list
#define CONFIG_GM_LIST_SIZE 3 // Number of valid GMs to keep in list
#define CONFIG_ANNOUNCE_LIST_SIZE 3 // Number of recent announce timestamps to keep per GM
#define CONFIG_PATH_TRACE_LIMIT 5 // Number of path trace IDs to keep in list

#endif /* ESP_AVB_SIMPLE_TALKER_INCLUDE_CONFIG_H_ */