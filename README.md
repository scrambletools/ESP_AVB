# AVB Talker, Listener, and Bridge Component

This component provides an AVB (Audio Video Bridging) stack with talker,
listener, and L2 bridge support. It runs as a wired endpoint, a Wi-Fi
endpoint, or as an Ethernet ↔ Wi-Fi bridge depending on per-port
configuration.

For a demo on how to use it, see the endpoint app on Github
(scrambletools/ESP-AVB-Endpoint). For the bridge counterpart, see
scrambletools/ESP-AVB-Bridge.

This component is available via the ESP Component Registry
(<https://components.espressif.com>).

## Roles

The role is selected automatically from the per-port configuration owned
by `esp_ptp` (see *Configuration* below). No separate AVB role switch is
needed.

- **endpoint_wired** — talker/listener on a wired EMAC port. Default.
  Full ATDECC entity, codec/I2S enabled.
- **endpoint_wireless** — talker/listener on a Wi-Fi STA port. Same
  AVB stack on the wireless data plane via `esp_wifi_internal_tx` /
  `esp_wifi_internal_reg_rxcb`. Codec is optional; disable it via
  `avb_config_s::codec_disabled = true` for boards without audio
  hardware.
- **bridge** — transparent L2 AVB bridge between an Ethernet port and a
  Wi-Fi SoftAP. Forwards AVTP control (ADP/AECP/ACMP/MAAP), MSRP, MVRP,
  and VLAN-tagged stream frames in both directions. No codec, no own
  ATDECC entity. Includes FQTSS Credit-Based Shaper and MSRP admission
  control on the egress port (Class B-only on Wi-Fi per the v1 plan;
  Class A reservations propagating to Wi-Fi are rejected with
  `insufficient_bandwidth_for_traffic_class`).

## Current features

Streaming:

- 1 Talker, 1 Listener (run simultaneously on endpoints)
- Up to 2 channels per stream
- AAF PCM audio, 24 bit, 48 kHz

Bridging:

- Bidirectional L2 forwarder between Ethernet and Wi-Fi (SoftAP) ports
- Bridge classifier per IEEE 802.1Q (PTP/MSRP/MVRP terminate locally;
  AVTP control + VLAN-tagged streams forward)
- FQTSS shaping decisions and MSRP admission control on Wi-Fi egress
- Beacon Vendor IE publish for gPTP `FollowUpInformation` transport
  (provided by `esp_ptp`)

Mediums:

- Ethernet (EMAC) — endpoint and bridge
- Wi-Fi (native, e.g. ESP32-C6) — endpoint
- Wi-Fi (`esp_wifi_remote` over ESP-Hosted SDIO/SPI) — bridge

## Configuration

Per-port topology (number of ports, medium, type, Wi-Fi role) is
configured in `esp_ptp`'s Kconfig under *PTP Daemon Configuration*. The
matching `ESP_AVB_*` symbols (`NUM_PORTS`, `PORT{0,1}_MEDIUM_*`,
`TIME_SOURCE_*`, `ROLE_*`) are derived automatically — they have no
prompt and cannot drift from the `esp_ptp` selection.

Topology examples:

| Build | esp_ptp settings | Resulting AVB role |
| --- | --- | --- |
| Wired endpoint | `NUM_PORTS=1`, `PORT0_MEDIUM=ethernet`, `PORT0_TYPE=primary` | `endpoint_wired` |
| Wireless endpoint | `NUM_PORTS=1`, `PORT0_MEDIUM=wifi`, `PORT0_TYPE=primary`, `PORT0_WIFI_ROLE=sta` | `endpoint_wireless` |
| Bridge | `NUM_PORTS=2`, `PORT0_MEDIUM=ethernet` + `TYPE=bridged`, `PORT1_MEDIUM=wifi_cp` + `TYPE=bridged` + `WIFI_ROLE=ap` | `bridge` |

AVB-specific knobs (stream VLAN, Class A/B PCP, MILAN compliance, codec
selection) remain in this component's own Kconfig.

## Tested with

Targets:

- ESP32-P4 — wired endpoint, bridge host
- ESP32-C6 — wireless endpoint (native Wi-Fi), bridge Wi-Fi coprocessor
  (over ESP-Hosted SDIO)

Boards:

- Waveshare ESP32-P4-ETH (wired endpoint with onboard ES8311)
- Waveshare ESP32-P4-WiFi6-PoE-ETH (bridge: P4 host + onboard C6 Wi-Fi)
- ESP32-C6 dev board (wireless endpoint)

Network peers:

- MOTU AVB switch
- MOTU 8D AVB endpoint
- Apple Mac Mini M1 and M4
- Sonnet Thunderbolt AVB Adapter
- Hive AVB Controller

## ATDECC support

Note that ATDECC support is limited to functionality that is required
for an AVB talker and listener. No effort has been made to implement
functionality that is needed to operate as a controller. The bridge role
is transparent at L2 and exposes no ATDECC entity of its own.

- Discovery
    Entity Available
- Enumeration and Control
  - Descriptors
    - Entity
    - Configuration
    - Audio Unit
    - Stream Input
    - Stream Output
    - AVB Interface
    - Clock Source
    - Memory Object
    - Locale
    - Strings
    - Stream Port Input
    - Stream Port Output
    - Audio Cluster
    - Audio Map
    - Control
    - Clock Domain
  - Commands
    - Acquire Entity
    - Lock Entity
    - Entity Available
    - Controller Available
    - Read Descriptor
    - Get Configuration
    - Set Stream Format
    - Get Stream Info
    - Register Unsolicited Notification
    - Deregister Unsolicited Notification
    - Get Counters
- Connection Management
  - Connect TX
  - Disconnect TX
  - Get TX State
  - Connect RX
  - Disconnect RX
  - Get RX State
  - Get TX Connection
- Milan Vendor Unique (MVU, Milan v1.3 §5)
  - Get Milan Info
  - Get System Unique ID
  - Get Media Clock Reference Info
  - Set Media Clock Reference Info
  - Bind Stream
  - Unbind Stream
  - Get Stream Input Info Ex
- AVB Community Vendor Unique (CVU)
  - Tunnels SRP attributes over AECP Vendor Unique on networks without
    native MSRP. Used when the PTP daemon falls back from gPTP to
    standard PTP (`ESP_AVB_AVB_LITE_COMPLIANT`).
  - Talker Advertise
  - Talker Failed
  - Listener (Asking Failed / Ready / Ready Failed / Ignore)
