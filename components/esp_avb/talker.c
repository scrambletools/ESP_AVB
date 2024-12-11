#include "talker.h"

// Queue message structure
struct queue_message_t {
 uint8_t message_id;
 uint8_t message_type;
 uint8_t message_data[ 20 ];
};

// Queue for inter-task messaging
QueueHandle_t queue_for_main_task;
QueueHandle_t queue_for_gptp_task;
QueueHandle_t queue_for_avtp_task;
QueueHandle_t queue_for_msrp_task;
QueueHandle_t queue_for_mvrp_task;

// Ethernet driver handle
static esp_eth_handle_t eth_handle = NULL;

// Define logging tag
static const char *TAG = "TALKER";

// gPTP communication management task
static void gptp_task(void *pvParameters) {

    // Define task specific logging tag
    static const char *TAG = "TASK-GPTP";
    ESP_LOGI(TAG, "Started GPTP Management Task.");

    // Get gPTP state and current GM info
    gptp_state_info_t *gptp_state = get_gptp_state();
    gptp_gm_info_t *current_gm = get_current_gm();
    gptp_pdelay_info_t *current_pdelay = get_current_pdelay();
    gptp_sync_info_t *current_sync = get_current_sync();
    gptp_gm_info_t *gm_list = get_gm_list();
    gptp_pdelay_info_t *pdelay_list = get_pdelay_list();
    gptp_sync_info_t *sync_list = get_sync_list();

    // Message for rx queue
    struct queue_message_t *p_message_for_gptp_task;

    // Create a message to send to tx queue
    struct queue_message_t message_for_main_task = {0,0,{0}};
    struct queue_message_t *p_message_for_main_task = &message_for_main_task;

    // Create a queue capable of containing 10 pointers to queue_message_t structures.
    // These should be passed by pointer as they contain a lot of data.
    queue_for_main_task = xQueueCreate(10, sizeof(struct queue_message_t *));
    if (queue_for_main_task == 0) {
        ESP_LOGE(TAG, "Failed to create the queue: gptp->main.");
        goto error;
    }

    // Send a pointer to a struct queue_message_t object.  Don't block if the queue is full.
    xQueueSend(queue_for_main_task, (void *) &p_message_for_main_task, (TickType_t) 0);

    // Create a timer for pdelay requests
    const esp_timer_create_args_t pdelay_request_timer_args = {
            .callback = &send_gptp_pdelay_request,
            .name = "pdelay-request-timer" // for debugging
    };
    esp_timer_handle_t pdelay_request_timer;
    ESP_ERROR_CHECK(esp_timer_create(&pdelay_request_timer_args, &pdelay_request_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(pdelay_request_timer, CONFIG_PDELAY_REQUEST_PERIOD * 1e3));

    // If GM capable then setup timers for local gm checks, 
    // as well as sending announce, sync and follow up messages
    if (CONFIG_GM_CAPABLE) {

        // Create a timer to check if local gm is needed
        const esp_timer_create_args_t local_gm_timer_args = {
                .callback = &check_local_gm,
                .name = "local-gm-timer" // for debugging
        };
        esp_timer_handle_t local_gm_timer;
        int local_gm_check_period = CONFIG_GM_TIMEOUT * current_gm->announce_period;
        ESP_ERROR_CHECK(esp_timer_create(&local_gm_timer_args, &local_gm_timer));
        ESP_ERROR_CHECK(esp_timer_start_periodic(local_gm_timer, local_gm_check_period * 1e3));

        // Create a timer for announce messages
        const esp_timer_create_args_t announce_timer_args = {
                .callback = &send_gptp_announce,
                .name = "announce-timer" // for debugging
        };
        esp_timer_handle_t announce_timer;
        ESP_ERROR_CHECK(esp_timer_create(&announce_timer_args, &announce_timer));
        ESP_ERROR_CHECK(esp_timer_start_periodic(announce_timer, CONFIG_ANNOUNCE_PERIOD * 1e3));

        // Create a timer for sync messages
        const esp_timer_create_args_t sync_timer_args = {
                .callback = &send_gptp_sync,
                .name = "sync-timer" // for debugging
        };
        esp_timer_handle_t sync_timer;
        ESP_ERROR_CHECK(esp_timer_create(&sync_timer_args, &sync_timer));
        ESP_ERROR_CHECK(esp_timer_start_periodic(sync_timer, CONFIG_SYNC_PERIOD * 1e3));
    }

    // Listen loop
    while (1) {

        // Check if queue_for_gptp_task has an incoming message
        if (queue_for_gptp_task != 0) {
            // Receive a message on the queue. Don't block if a message is not available.
            if (xQueueReceive(queue_for_gptp_task, &(p_message_for_gptp_task), (TickType_t) 0)) {
                // p_message_for_gptp_task now points to the struct queue_message_t variable posted
                // by another task, and the item is removed from the queue.
                // you can do something with the message here.
                ESP_LOGI(TAG, "Received queue message: ID=%d (Type=%d)", p_message_for_gptp_task->message_id, p_message_for_gptp_task->message_type);
            }
        }

        // Get next frame from the fd, detect the frame type and set the timestamp of receipt
        eth_frame_t frame;
        if (get_frame(ethertype_gptp, &frame) == ESP_FAIL) {
            break;
        }

        // Log all received frames
        //print_frame(&frame, PRINT_SUMMARY);

        // Handle each type of frame that needs action
        switch(frame.frame_type) {
            case avb_frame_gptp_pdelay_request:
                //print_frame(&frame, PRINT_SUMMARY);
                // Create a task to send the response at high priority
                BaseType_t result = xTaskCreatePinnedToCore(
                    pdelay_response_task,   // Task function
                    "gptp-pdelay-response", // Task name
                    6144,                   // Stack size (bytes)
                    &frame,                 // Parameters
                    21,                     // High priority (20)
                    NULL,                   // Task handle
                    PRO_CPU_NUM             // Pin to protocol CPU core
                );
                if (result != pdPASS) {
                    ESP_LOGE(TAG, "Failed to create high priority task");
                }
                break;
            case avb_frame_gptp_pdelay_response:
                //print_frame(&frame, PRINT_SUMMARY);
                // Check if the sequence_id = last request sent from local
                if (octets_to_uint(frame.payload + 30, 2) == current_pdelay->sequence_id) {

                    // Save the requeset receipt timestamp from the response frame
                    current_pdelay->timestamp_request_receipt.tv_sec = octets_to_uint(frame.payload + 34, 6);
                    current_pdelay->timestamp_request_receipt.tv_usec = (int)(octets_to_uint(frame.payload + 40, 4) / 1000);
                    
                    // Save the pdelay response receipt timestamp to current_pdelay
                    current_pdelay->timestamp_response_receipt.tv_sec = frame.time_received.tv_sec;
                    current_pdelay->timestamp_response_receipt.tv_usec = frame.time_received.tv_usec;
                }
                // If not, then discard the response
                break;
            case avb_frame_gptp_pdelay_response_follow_up:
                //print_frame(&frame, PRINT_SUMMARY);
                // Check if the sequence_id = last request sent from local
                if (octets_to_uint(frame.payload + 30, 2) == current_pdelay->sequence_id) {

                    // Save the response origin timestamp to current_pdelay
                    octets_to_timeval(frame.payload + 34, &current_pdelay->timestamp_response_transmission);
                    
                    // Calculate the peer delay and save it to current_pdelay
                    current_pdelay->calculated_pdelay = calculate_pdelay(current_pdelay);
                }
                // If not, then discard the response follow up
                break;
            case avb_frame_gptp_announce:
                print_frame(&frame, PRINT_SUMMARY);
                // Check if the announce message is for same GM as current GM, if so do nothing
                if (octets_to_uint(frame.payload + 53, 8) == current_gm->clock_id) {
                    //ESP_LOGI(TAG, "Announce message is for same GM as current GM.");
                }
                else {
                    // Extract the GM info from the announce message into a new GM struct
                    gptp_gm_info_t new_gm;
                    new_gm.clock_id = octets_to_uint(frame.payload + 53, 8);
                    new_gm.priority_1 = octets_to_uint(frame.payload + 47, 1);
                    new_gm.clock_class = octets_to_uint(frame.payload + 48, 1);
                    new_gm.clock_accuracy = octets_to_uint(frame.payload + 49, 1);
                    new_gm.clock_variance = octets_to_uint(frame.payload + 50, 2);
                    new_gm.priority_2 = octets_to_uint(frame.payload + 52, 1);
                    new_gm.steps_removed = octets_to_uint(frame.payload + 61, 2);
                    new_gm.time_source = octets_to_uint(frame.payload + 63, 1);
                    new_gm.port_id = octets_to_uint(frame.payload + 56, 2);

                    // Insert the sequence id
                    new_gm.announce_sequence_id = octets_to_uint(frame.payload + 30, 2);
                    
                    // Insert the announce period
                    new_gm.announce_period = log_period_to_msec(octets_to_uint(frame.payload + 33, 1));

                    // Insert the announce timestamp into the first slot of the timestamp_announce array
                    add_to_list_front(&frame.time_received, 
                                        &new_gm.timestamp_announce, 
                                        sizeof(struct timeval), 
                                        CONFIG_ANNOUNCE_LIST_SIZE
                    );

                    // Insert the path trace IDs
                    for (int i = 0; i <= new_gm.steps_removed; i++) {

                        // Dont go over the limit of the path trace IDs array
                        if (i >= CONFIG_PATH_TRACE_LIMIT) { break; }
                        new_gm.path_trace_ids[i] = octets_to_uint(frame.payload + 68 + (i * 8), 8);
                    }
                    // Add the new GM to the GM list
                    int index = add_to_gm_list(&new_gm);
                    if (index != -1) {
                        ESP_LOGI(TAG, "Announce message is from a better GM than current GM!");
                        if (index == 0) {

                            // New best GM is now current GM
                            ESP_LOGI(TAG, "Newly announced GM is now current GM!");

                            // Reset sync list
                            *sync_list = (gptp_sync_info_t){0};
                        }
                    }
                }
                break;
            case avb_frame_gptp_sync:
                print_frame(&frame, PRINT_SUMMARY);
                // Check if the sync message is from current GM
                if (octets_to_uint(frame.payload + 20, 8) == current_gm->clock_id) {

                    //ESP_LOGI(TAG, "Sync message is from current GM.");
                    // If the sequence id is greater than the current sync sequence id
                    if (octets_to_uint(frame.payload + 30, 2) > current_sync->sequence_id) {

                        // Create a new sync info struct
                        gptp_sync_info_t new_sync;
                        // Insert the sync sequence id
                        new_sync.sequence_id = octets_to_uint(frame.payload + 30, 2);
                        // Insert the sync period
                        new_sync.period = log_period_to_msec(octets_to_uint(frame.payload + 33, 1));
                        // Insert the sync receipt timestamp
                        new_sync.timestamp_sync_receipt.tv_sec = frame.time_received.tv_sec;
                        new_sync.timestamp_sync_receipt.tv_usec = frame.time_received.tv_usec;
                        // Add the new sync info to the sync list
                        add_to_sync_list(&new_sync);
                    }
                }
                else {
                    ESP_LOGI(TAG, "Sync message is not from current GM, will ignore.");
                }
                break;
            case avb_frame_gptp_follow_up:
                print_frame(&frame, PRINT_SUMMARY);
                // If the sync message is from current GM
                //if (false) {
                    if (octets_to_uint(frame.payload + 20, 8) == current_gm->clock_id) {

                    //ESP_LOGI(TAG, "Follow up message is from current GM.");
                    // If the sequence id = the current sync sequence id
                    if (octets_to_uint(frame.payload + 30, 2) == current_sync->sequence_id) {

                        // Insert the sync transmission timestamp into current_sync
                        octets_to_timeval(frame.payload + 34, &current_sync->timestamp_sync_transmission);

                        // Insert the scaled rate offset and scaled freq change
                        current_sync->scaled_rate_offset = scaled_to_ppm(octets_to_uint(frame.payload + 54, 4));
                        current_sync->scaled_freq_change = scaled_to_ppm(octets_to_uint(frame.payload + 72, 4));

                        // Calculate the offset and insert it into current_sync
                        current_sync->calculated_offset = calculate_offset(current_sync, current_pdelay);
                        //ESP_LOGI(TAG, "Calculated offset: %lld ns", current_sync->calculated_offset);

                        // Calculate new mean offset from sync list and save it to gptp state
                        uint64_t new_mean_offset = 0;
                        size_t count = 0;
                        for (int i = 0; i < CONFIG_SYNC_LIST_SIZE; i++) {
                            if (sync_list[i].sequence_id == 0) { break; } // ignore empty slots
                            new_mean_offset += sync_list[i].calculated_offset;
                            count++;
                        }
                        gptp_state->mean_offset = new_mean_offset / count;
                        //ESP_LOGI(TAG, "New mean offset: %lld ns", gptp_state->mean_offset);
                    }
                }
                else {
                    ESP_LOGI(TAG, "Follow up message is not from current GM, will ignore.");
                }
                break;
            default:
        }
    }
error:
    vTaskDelete(NULL);
}

// Check if local GM needed
static void check_local_gm(void* arg) {

    // Get needed gPTP variables
    gptp_state_info_t *gptp_state = get_gptp_state();
    gptp_gm_info_t *gm_list = get_gm_list();
    gptp_gm_info_t *current_gm = get_current_gm();

    // If all good do nothing  
    if (gptp_state->gm_is_present && gptp_state->gm_is_local) {
        return;
    }
    // If local GM but not present (unlikely), then set to present
    if (gptp_state->gm_is_local) {      
        gptp_state->gm_is_present = true;
        return;
    }
    // If GM is present but not local (most common case), then check if GM timed out 
    if (gptp_state->gm_is_present) {
        struct timeval timeout_time;
        gettimeofday(&timeout_time, NULL);
        gptp_gm_info_t *current_gm = get_current_gm();
        // Set timeout time to GM_TIMEOUT * GM's announce period (rounded down to whole seconds)
        timeout_time.tv_sec -= CONFIG_GM_TIMEOUT * (int)(current_gm->announce_period / 1e3);
        // If current GM timed out
        if (compare_timeval(current_gm->timestamp_announce[0], timeout_time) < 0) {
            // Remove it from the list of GMs; and next best GM becomes current GM at gm_list[0]
            esp_err_t err = remove_from_gm_list(current_gm);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "GM timeout, but failed to remove GM from list.");
            }
            else {
                ESP_LOGI(TAG, "Dropped current GM due to timeout.");
            }
            // Get new current GM, reset sequence id, timestamp_announce list
            current_gm = get_current_gm();
            current_gm->announce_sequence_id = 0;
            memset(&current_gm->timestamp_announce, 0, sizeof(current_gm->timestamp_announce));
            gettimeofday(&current_gm->timestamp_announce[0], NULL);
            // Reset sync list
            gptp_sync_info_t *sync_list = get_sync_list();
            *sync_list = (gptp_sync_info_t){0};
            ESP_LOGI(TAG, "GM is now %lld", current_gm->clock_id);

            // If new current GM is the local clock, then set GM is local
            if (current_gm->clock_id == CONFIG_CLOCK_ID) {
                gptp_state->gm_is_local = true;
                ESP_LOGI(TAG, "GM fell back to local clock.");
            }
        }
    }
    // If GM is not present and not local (startup case)
    else {
        if (CONFIG_GM_CAPABLE) {
            // Local clock is already set to current_gm (gm_list[0]) by default
            // So just add an announce timestamp to gm_list[0] and set the gPTP state variables
            gettimeofday(&current_gm->timestamp_announce[0], NULL);
            gptp_state->gm_is_present = true;
            gptp_state->gm_is_local = true;
            ESP_LOGI(TAG, "Local clock is now current GM.");
        }
    }
}

// Send gPTP peer delay request message
static void send_gptp_pdelay_request(void* arg) {

    // Get the needed variables
    gptp_pdelay_info_t *pdelay_list = get_pdelay_list();
    gptp_pdelay_info_t *current_pdelay = get_current_pdelay();

    // Create a new frame, set the header and append the request message
    eth_frame_t frame;
    set_frame_header(lldp_mcast_mac_addr, ethertype_gptp, &frame);
    frame.frame_type = avb_frame_gptp_pdelay_request;
    append_gptpdu(frame.frame_type, &frame);

    // Increment the sequence_id and insert it into the frame
    uint16_t request_sequence_id = current_pdelay->sequence_id;
    if (request_sequence_id == 0xFFFF) { request_sequence_id = 1; }
    else { request_sequence_id += 1; }
    int_to_octets(&request_sequence_id, frame.payload + 30, 2);

    // Send response
    send_frame(&frame);
    //print_frame(&frame, PRINT_SUMMARY);

    // Create a new pdelay info struct
    gptp_pdelay_info_t new_pdelay;
    new_pdelay.sequence_id = request_sequence_id;
    new_pdelay.period = CONFIG_PDELAY_REQUEST_PERIOD; // Set to default period

    // Save the request sent timestamp from the frame that was just sent
    new_pdelay.timestamp_request_transmission.tv_sec = frame.time_sent.tv_sec;
    new_pdelay.timestamp_request_transmission.tv_usec = frame.time_sent.tv_usec;

    // Add the new pdelay info to the pdelay list
    add_to_pdelay_list(&new_pdelay);
}

// Send gPTP peer delay response and follow up messages
static void pdelay_response_task(void *pvParameters) {

    // Get the needed variables
    eth_frame_t *req_frame = (eth_frame_t *)pvParameters;

    // Create a new frame, set the header and append the response message
    eth_frame_t frame;
    set_frame_header(lldp_mcast_mac_addr, ethertype_gptp, &frame);
    frame.frame_type = avb_frame_gptp_pdelay_response;
    append_gptpdu(frame.frame_type, &frame);

    // -- Process response based on request frame data --

    // Insert the sequence_id from the request frame    
    memcpy(frame.payload + 30, req_frame->payload + 30, (2));

    // Compensate for system latency (for testing on sw timestamp systems)
    // Time received is adjusted to be earlier to account for estimated system latency
    struct timeval adjusted_time_received = req_frame->time_received;
    adjusted_time_received.tv_usec = (int)(adjusted_time_received.tv_usec - CONFIG_SYSTEM_LATENCY_COMP);

    // Insert the requeset_receipt_timestamp from the request time of receipt
    timeval_to_octets(&adjusted_time_received, frame.payload + 34, frame.payload + 40);

    // Insert the requesting_source_port_identity from the request frame clock_identity
    memcpy(frame.payload + 44, req_frame->payload + 20, (8));

    // Insert the requesting_source_port_id from the request frame source_port_id
    memcpy(frame.payload + 52, req_frame->payload + 28, (2));

    // Send response
    send_frame(&frame);
    //print_frame(&frame, PRINT_SUMMARY);

    // --- Process response follow up based on request and response frame data ---
    
    // Reuse the frame; change payload to response follow up
    // Re-insert the sequence_id, requesting_source_port_identity, and requesting_source_port_id
    frame.frame_type = avb_frame_gptp_pdelay_response_follow_up;
    append_gptpdu(frame.frame_type, &frame);
    memcpy(frame.payload + 30, req_frame->payload + 30, (2));
    memcpy(frame.payload + 44, req_frame->payload + 20, (8));
    memcpy(frame.payload + 52, req_frame->payload + 28, (2));

    // Compensate for system latency (for testing on sw timestamp systems)
    // Time sent is adjusted to be later to account for estimated system latency
    struct timeval adjusted_time_sent = frame.time_sent;
    adjusted_time_sent.tv_usec = (int)(adjusted_time_sent.tv_usec + CONFIG_SYSTEM_LATENCY_COMP);

    // Insert the response_origin_timestamp from the response time of transmission
    // using the time_sent from the response that was just sent
    timeval_to_octets(&frame.time_sent, frame.payload + 34, frame.payload + 40);
    
    // Send response follow up
    send_frame(&frame);
    //print_frame(&frame, PRINT_SUMMARY);

    // Delete the task when done
    vTaskDelete(NULL);
}

// Send gPTP announce message (if GM is local)
static void send_gptp_announce(void* arg) {

    // If GM is present and local
    gptp_state_info_t *gptp_state = get_gptp_state();
    if (gptp_state->gm_is_present && gptp_state->gm_is_local) {

        // Create a new frame, set the header and append the announce message
        eth_frame_t frame;
        set_frame_header(lldp_mcast_mac_addr, ethertype_gptp, &frame);
        frame.frame_type = avb_frame_gptp_announce;
        append_gptpdu(frame.frame_type, &frame);

        // Increment the sequence_id and insert it into the frame
        gptp_gm_info_t *current_gm = get_current_gm();
        if (current_gm->announce_sequence_id == 0xFFFF) { current_gm->announce_sequence_id = 1; }
        else { current_gm->announce_sequence_id += 1; }
        int_to_octets(&current_gm->announce_sequence_id, frame.payload + 30, 2);

        // Insert the log_period
        int8_t log_period = msec_to_log_period(current_gm->announce_period);
        int_to_octets(&log_period, frame.payload + 33, 1);

        // Insert the local gm info
        int_to_octets(&current_gm->priority_1, frame.payload + 47, 1);
        int_to_octets(&current_gm->clock_class, frame.payload + 48, 1);
        int_to_octets(&current_gm->clock_accuracy, frame.payload + 49, 1);
        int_to_octets(&current_gm->clock_variance, frame.payload + 50, 2);
        int_to_octets(&current_gm->priority_2, frame.payload + 52, 1);
        int_to_octets(&current_gm->clock_id, frame.payload + 53, 8);
        int_to_octets(&current_gm->steps_removed, frame.payload + 61, 2);
        int_to_octets(&current_gm->time_source, frame.payload + 63, 1);
        int_to_octets(&current_gm->clock_id, frame.payload + 68, 8); // pathTraceTlv(pathSequence)

        // Send the announce message
        send_frame(&frame);
        print_frame(&frame, PRINT_SUMMARY);
    }
}

// Send gPTP sync message
static void send_gptp_sync(void* arg) {

    // If GM is present and local
    gptp_state_info_t *gptp_state = get_gptp_state();
    if (gptp_state->gm_is_present && gptp_state->gm_is_local) {

        // Get the sync list
        gptp_sync_info_t *sync_list = get_sync_list();
        gptp_sync_info_t *current_sync = get_current_sync();

        // --- Process sync message ---

        // Create a new frame, set the header and append the sync template
        eth_frame_t frame;
        set_frame_header(lldp_mcast_mac_addr, ethertype_gptp, &frame);
        frame.frame_type = avb_frame_gptp_sync;
        append_gptpdu(frame.frame_type, &frame);

        // Increment the sequence_id and insert into the frame
        // Don't change current_sync->sequence_id as that is now old
        uint16_t sync_sequence_id = current_sync->sequence_id;
        if (sync_sequence_id == 0xFFFF) { sync_sequence_id = 1; }
        else { sync_sequence_id += 1; }
        int_to_octets(&sync_sequence_id, frame.payload + 30, 2);

        // Insert the sync period
        int8_t log_period = msec_to_log_period(CONFIG_SYNC_PERIOD);
        int_to_octets(&log_period, frame.payload + 33, 1);

        // Send the sync message
        send_frame(&frame);
        print_frame(&frame, PRINT_SUMMARY);

        // Create a new sync and add it to the sync list
        gptp_sync_info_t new_sync;
        new_sync.sequence_id = sync_sequence_id;

        new_sync.timestamp_sync_transmission.tv_sec = frame.time_sent.tv_sec;
        new_sync.timestamp_sync_transmission.tv_usec = frame.time_sent.tv_usec;
        add_to_sync_list(&new_sync);

        // --- Process follow up using current sync data ---

        // Reuse the frame; change payload to follow up message
        frame.frame_type = avb_frame_gptp_follow_up;
        append_gptpdu(frame.frame_type, &frame);

        // Re-insert the sequence_id and log_period
        int_to_octets(&sync_sequence_id, frame.payload + 30, 2);
        uint8_t no_period = 127; // 127 = no period
        int_to_octets(&no_period, frame.payload + 33, 1); 

        // Insert the transmission timestamp from the sync message that was just sent
        timeval_to_octets(&frame.time_sent, frame.payload + 34, frame.payload + 40);

        // Insert the organization extension tlv data
        int32_t scaled_rate_offset = ppm_to_scaled(gptp_state->mean_rate_ratio);
        int_to_octets(&scaled_rate_offset, frame.payload + 54, 4);
        int32_t scaled_freq_change = ppm_to_scaled(gptp_state->mean_freq_change);
        int_to_octets(&scaled_freq_change, frame.payload + 74, 4);

        // Send the follow up message
        send_frame(&frame);
        print_frame(&frame, PRINT_SUMMARY);
    }
}

// AVTP communication management task
static void avtp_task(void *pvParameters) {

    // Define task specific logging tag
    static const char *TAG = "TASK-AVTP";
    ESP_LOGI(TAG, "Started AVTP Management Task.");

    // Message for rx queue
    struct queue_message_t *p_message_for_avtp_task;

    // Create a message to send to tx queue
    struct queue_message_t message_for_main_task = {0, 0, { 0 }};
    struct queue_message_t *p_message_for_main_task = &message_for_main_task;

    // Create a queue capable of containing 10 pointers to queue_message_t structures.
    // These should be passed by pointer as they contain a lot of data.
    queue_for_main_task = xQueueCreate(10, sizeof(struct queue_message_t *));
    if (queue_for_main_task == 0) {
        ESP_LOGE(TAG, "Failed to create the queue: avtp->main.");
        goto error;
    }

    // Send a pointer to a struct queue_message_t object.  Don't block if the queue is full.
    xQueueSend(queue_for_main_task, (void *) &p_message_for_main_task, (TickType_t) 0);

    // Listen loop
    while (1) {

        // Check if queue_for_avtp_task has an incoming message
        if (queue_for_avtp_task != 0) {
            // Receive a message on the queue. Don't block if a message is not available.
            if (xQueueReceive(queue_for_avtp_task, &(p_message_for_avtp_task), (TickType_t) 0)) {
                // message_for_avtp_task now points to the struct queue_message_t variable posted
                // by another task, and the item is removed from the queue.
                // you can do something with the message here.
                ESP_LOGI(TAG, "I got a text!");
            }
        }
        // Get the frame from the fd, detect the frame type and set the timestamp of receipt
        eth_frame_t frame;
        if (get_frame(ethertype_avtp, &frame) == ESP_FAIL) {
            break;
        }
        // Log the received frame
        print_frame(&frame, PRINT_SUMMARY);

        // Do something with the frame
    }
error:
    vTaskDelete(NULL);
}

// MSRP communication management task
static void msrp_task(void *pvParameters) {

    // Define task specific logging tag
    static const char *TAG = "TASK-MSRP";
    ESP_LOGI(TAG, "Started MSRP Management Task.");

    // Message for rx queue
    struct queue_message_t *p_message_for_msrp_task;

    // Create a message to send to tx queue
    struct queue_message_t message_for_main_task = { 0, 0, { 0 }};
    struct queue_message_t *p_message_for_main_task = &message_for_main_task;

    // Create a queue capable of containing 10 pointers to queue_message_t structures.
    // These should be passed by pointer as they contain a lot of data.
    queue_for_main_task = xQueueCreate(10, sizeof(struct queue_message_t *));
    if (queue_for_main_task == 0) {
        ESP_LOGE(TAG, "Failed to create the queue: msrp->main.");
        goto error;
    }

    // Send a pointer to a struct queue_message_t object.  Don't block if the queue is full.
    xQueueSend(queue_for_main_task, (void *) &p_message_for_main_task, (TickType_t) 0);

    // Listen loop
    while (1) {

        // Check if queue_for_msrp_task has an incoming message
        if (queue_for_msrp_task != 0) {
            // Receive a message on the queue. Don't block if a message is not available.
            if (xQueueReceive(queue_for_msrp_task, &(p_message_for_msrp_task), (TickType_t) 0)) {
                // message_for_msrp_task now points to the struct queue_message_t variable posted
                // by another task, and the item is removed from the queue.
                // you can do something with the message here.
                ESP_LOGI(TAG, "I got a text!");
            }
        }
        // Get the frame from the fd, detect the frame type and set the timestamp of receipt
        eth_frame_t frame;
        if (get_frame(ethertype_msrp, &frame) == ESP_FAIL) {
            break;
        }
        // Log the received frame
        print_frame(&frame, PRINT_SUMMARY);

        // Do something with the frame
    }
error:
    vTaskDelete(NULL);
}

// MVRP communication management task
static void mvrp_task(void *pvParameters) {

    // Define task specific logging tag
    static const char *TAG = "TASK-MVRP";
    ESP_LOGI(TAG, "Started MVRP Management Task.");

    // Message for rx queue
    struct queue_message_t *p_message_for_mvrp_task;

    // Create a message to send to tx queue
    struct queue_message_t message_for_main_task = {0,0,{0}};
    struct queue_message_t *p_message_for_main_task = &message_for_main_task;

    // Create a queue capable of containing 10 pointers to queue_message_t structures.
    // These should be passed by pointer as they contain a lot of data.
    queue_for_main_task = xQueueCreate(10, sizeof(struct queue_message_t *));
    if (queue_for_main_task == 0) {
        ESP_LOGE(TAG, "Failed to create the queue: mvrp->main.");
        goto error;
    }

    // Send a pointer to a struct queue_message_t object.  Don't block if the queue is full.
    xQueueSend(queue_for_main_task, (void *) &p_message_for_main_task, (TickType_t) 0);

    // Listen loop
    while (1) {

        // Check if queue_for_mvrp_task has an incoming message
        if (queue_for_mvrp_task != 0) {
            // Receive a message on the queue. Don't block if a message is not available.
            if (xQueueReceive(queue_for_mvrp_task, &(p_message_for_mvrp_task), (TickType_t) 0)) {
                // message_for_mvrp_task now points to the struct queue_message_t variable posted
                // by another task, and the item is removed from the queue.
                // you can do something with the message here.
                ESP_LOGI(TAG, "I got a text!");
            }
        }
        // Get the frame from the fd, detect the frame type and set the timestamp of receipt
        eth_frame_t frame;
        if (get_frame(ethertype_mvrp, &frame) == ESP_FAIL) {
            break;
        }
        // Log the received frame
        print_frame(&frame, PRINT_SUMMARY);

        // Do something with the frame
    }
error:
    vTaskDelete(NULL);
}

// Starup the talker
void start_talker(esp_netif_iodriver_handle handle) {
    
    ESP_LOGI(TAG, "Starting Talker...");
    
    // Set the Ethernet driver handle (passed from main app)
    set_eth_handle(handle);

    // Open and configure L2 TAP file descriptors
    esp_err_t err = init_all_l2tap_fds();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize FDs, %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "All FDs are open.");
        
        // Start gPTP manager task
        xTaskCreate(gptp_task, "gptp_task", 6144, NULL, 21, NULL);

        // Start AVTP manager task
        //xTaskCreate(avtp_task, "avtp_task", 6144, NULL, 5, NULL);

        // Start MSRP manager task
        //xTaskCreate(msrp_task, "msrp_task", 6144, NULL, 5, NULL);

        // Start MVRP manager task
        //xTaskCreate(mvrp_task, "mvrp_task", 6144, NULL, 5, NULL);
    }
}

// Stop the talker
void stop_talker() {
    ESP_LOGI(TAG, "Stopping Talker...");
    //close_all_l2tap_fds();
    ESP_LOGI(TAG, "All FDs are closed.");
}