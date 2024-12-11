#include "network.h"

// Ethernet driver handle
static esp_eth_handle_t eth_handle = NULL;

// Define logging tag
static const char *TAG = "NETWORK";

// Managed mac addresses
static uint8_t local_eth_mac_addr[ETH_ADDR_LEN] = { 0 };
static uint8_t local_wifi_mac_addr[ETH_ADDR_LEN] = { 0 };
static uint8_t controller_mac_addr[ETH_ADDR_LEN] = { 0 };
static uint8_t listener_mac_addr[ETH_ADDR_LEN] = { 0 }; // remote device
static uint8_t talker_mac_addr[ETH_ADDR_LEN] = { 0 }; // remote device

// L2tap file descriptors for each Ethertype; start out as invalid
static int l2tap_gptp_fd = INVALID_FD;
static int l2tap_avtp_fd = INVALID_FD;
static int l2tap_msrp_fd = INVALID_FD;
static int l2tap_mvrp_fd = INVALID_FD;

// Set the Ethernet driver handle
void set_eth_handle(esp_netif_iodriver_handle handle) {
    eth_handle = handle;
    // Set the local Ethernet mac address
    esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, local_eth_mac_addr);
}

// Get the frame from the fd
esp_err_t get_frame(ethertype_t ethertype, eth_frame_t *frame) {

    // Select the fd based on the given Ethertype
    int fd;
    switch (ethertype) {
        case ethertype_gptp:
            fd = l2tap_gptp_fd;
            break;
        case ethertype_avtp:
            fd = l2tap_avtp_fd;
            break;
        case ethertype_msrp:
            fd = l2tap_msrp_fd;
            break;
        case ethertype_mvrp:
            fd = l2tap_mvrp_fd;
            break;
        default:
            ESP_LOGE(TAG, "Cannot get frame with an unkown Ethertype, 0x%x.", ethertype);
            return ESP_FAIL;
    }
    
    // Create timeval
    struct timeval tv;
    tv.tv_sec = 5;
    tv.tv_usec = 0;

    // Create an fd set
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);

    // Create a buffer to read the frame from the fd
    uint8_t rx_buffer[1500]; //128

    // Use select to setup a timeout for the read
    int result = select(fd + 1, &rfds, NULL, NULL, &tv);
    if (result > 0) {

        // Read a frame from the fd, if any
        ssize_t len = read(fd, rx_buffer, sizeof(rx_buffer));
        if (len > 0) {
            // Save the timestamp of receipt (needs to be done immediately after read)
            gettimeofday(&frame->time_received, NULL);

            // Copy frame data to frame struct
            memcpy(frame, (eth_frame_t *)rx_buffer, len);

            // Detect the frame type for proper handling
            avb_frame_type_t frame_type = avb_frame_unknown;
            switch (ethertype) {
                case ethertype_gptp:
                    frame_type = detect_gptp_frame_type(frame, len);
                    break;
                case ethertype_avtp:
                    uint8_t subtype;
                    memcpy(&subtype, &frame->payload, (1));
                    if (subtype == avtp_subtype_adp || subtype == avtp_subtype_aecp || subtype == avtp_subtype_acmp) {
                        frame_type = detect_atdecc_frame_type(frame, len);
                    }
                    else {
                        frame_type = detect_avtp_frame_type(&ethertype, frame, len);
                    }
                    break;
                case ethertype_msrp:
                    frame_type = detect_avtp_frame_type(&ethertype, frame, len);
                    break;
                case ethertype_mvrp:
                    frame_type = detect_avtp_frame_type(&ethertype, frame, len);
                    break;
                default:
                    ESP_LOGE(TAG, "Cannot detect frame with an unkown Ethertype, 0x%x.", ethertype);
                    return ESP_FAIL;
            }
            
            // Fill in the frame type and payload size
            frame->frame_type = frame_type;
            frame->payload_size = len - ETH_HEADER_LEN;
        } else {
            ESP_LOGE(TAG, "L2 TAP fd %d read error: errno %d", fd, errno);
            return ESP_FAIL;
        }
    } else if (result == 0) {
        ESP_LOGD(TAG, "L2 TAP select timeout");
    } else {
        ESP_LOGE(TAG, "L2 TAP select error: errno %d", errno);
        return ESP_FAIL;
    }
    return ESP_OK;
}

// Set the destination, source (local mac) and Ethertype in a frame, based on params
void set_frame_header(uint8_t *dest_addr, ethertype_t ethertype, eth_frame_t *frame) {
    // Add the destination
    memcpy(frame->header.dest.addr, dest_addr, ETH_ADDR_LEN);
    // Add local mac address as the source
    memcpy(frame->header.src.addr, local_eth_mac_addr, ETH_ADDR_LEN);
    // Add the Ethertype
    ethertype_t type = ntohs(ethertype);
    memcpy(&frame->header.type, &type, sizeof(ethertype));
}

// Send an Ethernet frame
esp_err_t send_frame(eth_frame_t *frame) {
    
    // Select the fd based on the frame's Ethertype
    ethertype_t ethertype = ntohs(frame->header.type);

    // Select the fd based on the given Ethertype
    int fd;
    switch (ethertype) {
        case ethertype_gptp:
            fd = l2tap_gptp_fd;
            break;
        case ethertype_msrp:
            fd = l2tap_msrp_fd;
            break;
        case ethertype_mvrp:
            fd = l2tap_mvrp_fd;
            break;
        case ethertype_avtp:
            fd = l2tap_avtp_fd;
            break;
        default:
            ESP_LOGE(TAG, "Cannot send frame with an unkown Ethertype, 0x%x.", ethertype);
            return ESP_FAIL;
    }

    // Save timestamp for follow up (needs to be done immediately before write)
    gettimeofday(&frame->time_sent, NULL);

    // Send away!
    ssize_t len = write(fd, frame, frame->payload_size + ETH_HEADER_LEN);
    if (len < 0) {
        ESP_LOGE(TAG, "L2 TAP fd %d write error, errno, %d", fd, errno);
        return ESP_FAIL;
    }
    return ESP_OK;
}

// Opens and configures L2 TAP file descriptor; flags=0 is blocking, 1 is non-blocking
int init_l2tap_fd(int flags, ethertype_t ethertype)
{ 
    // Open a file descriptor using the flags
    int fd = open("/dev/net/tap", flags);
    if (fd < 0) {
        ESP_LOGE(TAG, "Unable to open L2 TAP interface, errno %d", errno);
        goto error;
    }
    ESP_LOGI(TAG, "/dev/net/tap fd %d successfully opened", fd);

    // Configure the fd to use the default Ethernet interface
    int result;
    if ((result = ioctl(fd, L2TAP_S_INTF_DEVICE, ETH_INTERFACE)) == INVALID_FD) {
        ESP_LOGE(TAG, "Unable to bound L2 TAP fd %d with Ethernet device: errno %d", fd, errno);
        goto error;
    }
    ESP_LOGI(TAG, "L2 TAP fd %d successfully bound to the default Ethernet interface", fd);

    // Configure Ethernet frames we want to filter out
    if ((result = ioctl(fd, L2TAP_S_RCV_FILTER, &ethertype)) == INVALID_FD) {
        ESP_LOGE(TAG, "Unable to configure fd %d Ethernet type receive filter: errno %d", fd, errno);
        goto error;
    }
    ESP_LOGI(TAG, "L2 TAP fd %d Ethernet type filter configured to 0x%x", fd, ethertype);

    return fd;
error:
    if (fd != INVALID_FD) {
        close(fd);
    }
    return INVALID_FD;
}

// Print out the frame (format: 0 for short (default), 1 for long format)
void print_frame_of_type(avb_frame_type_t type, eth_frame_t *frame, int format) {
    if (frame->payload_size < 2) {
        ESP_LOGI(TAG, "Can't print frame, payload is too small: %d", frame->payload_size);
    }
    else {
        switch (type) {
            case avb_frame_gptp_announce ... avb_frame_gptp_pdelay_response_follow_up:
                print_gptp_frame(type, frame, format);
                break;
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

// Setup l2tap file descripters (fd) for all necessary Ethertypes
esp_err_t init_all_l2tap_fds() {
    //Setup gPTP fd
    if (l2tap_gptp_fd == INVALID_FD) {
        if ((l2tap_gptp_fd = init_l2tap_fd(O_NONBLOCK, ethertype_gptp)) == INVALID_FD) {
            return ESP_FAIL;
        }
    }
    // Setup AVTP fd
    if (l2tap_avtp_fd == INVALID_FD) {
        if ((l2tap_avtp_fd = init_l2tap_fd(O_NONBLOCK, ethertype_avtp)) == INVALID_FD) {
            return ESP_FAIL;
        }
    }
    // Setup MSRP fd
    if (l2tap_msrp_fd == INVALID_FD) {
        if ((l2tap_msrp_fd = init_l2tap_fd(O_NONBLOCK, ethertype_msrp)) == INVALID_FD) {
            return ESP_FAIL;
        }
    }
    // Setup MVRP fd
    if (l2tap_mvrp_fd == INVALID_FD) {
        if ((l2tap_mvrp_fd = init_l2tap_fd(O_NONBLOCK, ethertype_mvrp)) == INVALID_FD) {
            return ESP_FAIL;
        }
    }
    return ESP_OK;
}

// Close l2tap file descripters (fd) for all necessary Ethertypes
void close_all_l2tap_fds() {
    // Close gPTP fd
    if (l2tap_gptp_fd != INVALID_FD) {
        close(l2tap_gptp_fd);
    }
    // Close AVTP fd
    if (l2tap_avtp_fd != INVALID_FD) {
        close(l2tap_avtp_fd);
    }
    // Close MSRP fd
    if (l2tap_msrp_fd != INVALID_FD) {
        close(l2tap_msrp_fd);
    }
    // Close MVRP fd
    if (l2tap_mvrp_fd != INVALID_FD) {
        close(l2tap_mvrp_fd);
    }
}