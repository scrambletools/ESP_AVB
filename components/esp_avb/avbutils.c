/*
 * Copyright 2024 Scramble Tools
 * License: MIT
 *
 * ESP_AVB Component
 *
 * This component provides an implementation of an AVB talker and listener.
 * 
 * This file provides utility functions for the AVB component.
 */

#include "avbutils.h"

/* Convert an octet buffer to a hex string 
 * 
 * buffer: the octet buffer to convert
 * size: the size of the buffer
 * hex_string: the resulting hex string (must be at least size * 3 + 1 bytes)
 * delimiter: the delimiter to use between octets (default is space)
 */
void octets_to_hex_string(const uint8_t* buffer, size_t size, char* hex_string, unsigned char delimiter) {
    if (!buffer || size == 0) {
        return;
    }
    if (!delimiter || delimiter == '\0') {
        delimiter = ' ';
    }
    hex_string[0] = '\0'; // Initialize as an empty string
    for (size_t i = 0; i < size; i++) {
       char hexBuffer[4]; // 2 for hex, 1 for delimiter, 1 for null terminator
       char delimStr[2] = {delimiter, '\0'}; // Create proper string for delimiter
       snprintf(hexBuffer, sizeof(hexBuffer), "%02X%s", (unsigned char)buffer[i], 
               (i < size - 1) ? delimStr : "");
       strcat(hex_string, hexBuffer);
   }
}

// Convert octet buffer to uint64_t, uint32_t, uint16_t or uint8_t; assumes big-endian buffer
// size is buffer size in bytes: 8 (default) >= size >= 0
uint64_t octets_to_uint(const uint8_t *buffer, size_t size) {
    uint64_t result = 0;
    if (buffer == NULL || size == 0 || size > 8) {
        return result; // Return 0 for 0 bytes or gracfully handle invalid input
    }
    // If 1 byte, truncate the result to 8 bits
    if (size == 1) {
        return (uint8_t)result;
    }
    // Combine bytes into a uint64_t
    for (size_t i = 0; i < size; i++) {
        result |= ((uint64_t)buffer[i] << (8 * (size - 1 - i))); // Big-endian order
    }
    // If 2 bytes, truncate the result to 16 bits
    if (size == 2) {
        return (uint16_t)result;
    }
    // If 4 bytes or less, truncate the result to 32 bits
    if (size <= 4) {
        return (uint32_t)result;
    }
    // Otherwise, return as a 64-bit integer
    return result;
}

// Convert an octet buffer to a timeval
void octets_to_timeval(const uint8_t *buffer, struct timeval *tv) {
    tv->tv_sec = octets_to_uint(buffer, 6);
    tv->tv_usec = (int)(octets_to_uint(buffer + 6, 4) / 1e3);
}

// Reverses the order of octets in a buffer
void reverse_octets(uint8_t *buffer, size_t size) {
    if (buffer == NULL || size <= 1) {
        return; // Nothing to do for NULL or single-item buffers
    }
    for (size_t i = 0; i < size / 2; i++) {
        // Swap items at index i and (size - 1 - i)
        uint8_t temp = buffer[i];
        buffer[i] = buffer[size - 1 - i];
        buffer[size - 1 - i] = temp;
    }
}

// Generates a string of bits from a uint8_t buffer
void octets_to_binary_string(const uint8_t *buffer, size_t size, char *bit_string) {
    if (buffer == NULL || bit_string == NULL || size == 0) {
        return; // Handle invalid input gracefully
    }
    // Convert each byte to its binary representation
    char *ptr = bit_string;
    for (size_t i = 0; i < size; i++) {
        for (int bit = 7; bit >= 0; bit--) { // MSB to LSB
            *ptr++ = (buffer[i] & (1 << bit)) ? '1' : '0';
        }
    }
    // Null-terminate the string
    *ptr = '\0';
}

// Converts an integer to a buffer of octets; reverses the order of the octets
void int_to_octets(void *value, void *buffer, size_t size) {
    //ESP_LOGI(TAG, "int_to_octets: %lld", (long long)*value);
    memcpy(buffer, value, size);
    reverse_octets((uint8_t *)buffer, size);
}

// Generates a string of bits fron a uint64_t; num_bits can be 64,32,16 or 8
void int_to_binary_string(uint64_t value, int num_bits, char *bit_string, bool reverse_order) {
    if (bit_string == NULL || num_bits <= 0 || num_bits > 64) {
        return; // Handle invalid input gracefully
    }

    // Generate the binary string
    for (int i = 0; i < num_bits; i++) {
        if (!reverse_order) {
            // Big-endian: MSB first
            bit_string[i] = (value & (1ULL << (num_bits - 1 - i))) ? '1' : '0';
        } else {
            // Little-endian: LSB first
            bit_string[i] = (value & (1ULL << i)) ? '1' : '0';
        }
    }
    // Null-terminate the string
    bit_string[num_bits] = '\0';
}

// Helper function for uint64_t
void int_to_binary_string_64(uint64_t value, char *bit_string) {
    int_to_binary_string(value, 64, bit_string, false);
}

// Helper function for uint32_t
void int_to_binary_string_32(uint32_t value, char *bit_string) {
    int_to_binary_string(value, 32, bit_string, false);
}

// Helper function for uint16_t
void int_to_binary_string_16(uint16_t value, char *bit_string) {
    int_to_binary_string(value, 16, bit_string, false);
}

// Reverse endianness of a uint64_t; num_bits can be 64,32,16 or 8
uint64_t reverse_endianness(uint64_t value, int num_bits) {
    switch (num_bits) {
        case 16:
            return ((value & 0x00FF) << 8) |  // Move byte 0 to byte 1
                   ((value & 0xFF00) >> 8);  // Move byte 1 to byte 0
        case 32:
            return ((value & 0x000000FF) << 24) |  // Move byte 0 to byte 3
                   ((value & 0x0000FF00) << 8)  |  // Move byte 1 to byte 2
                   ((value & 0x00FF0000) >> 8)  |  // Move byte 2 to byte 1
                   ((value & 0xFF000000) >> 24);   // Move byte 3 to byte 0
        case 64:
            return ((value & 0x00000000000000FFULL) << 56) |  // Move byte 0 to byte 7
                   ((value & 0x000000000000FF00ULL) << 40) |  // Move byte 1 to byte 6
                   ((value & 0x0000000000FF0000ULL) << 24) |  // Move byte 2 to byte 5
                   ((value & 0x00000000FF000000ULL) << 8)  |  // Move byte 3 to byte 4
                   ((value & 0x000000FF00000000ULL) >> 8)  |  // Move byte 4 to byte 3
                   ((value & 0x0000FF0000000000ULL) >> 24) |  // Move byte 5 to byte 2
                   ((value & 0x00FF000000000000ULL) >> 40) |  // Move byte 6 to byte 1
                   ((value & 0xFF00000000000000ULL) >> 56);   // Move byte 7 to byte 0
        default:
            // Unsupported size
            return value;
    }
}

// Helper function for 64-bit integers using the general function
uint64_t reverse_endianness_64(uint64_t value) {
    return reverse_endianness(value, 64);
}

// Helper function for 32-bit integers using the general function
uint32_t reverse_endianness_32(uint32_t value) {
    return (uint32_t)reverse_endianness((uint64_t)value, 32);
}

// Helper function for 16-bit integers using the general function
uint16_t reverse_endianness_16(uint16_t value) {
    return (uint16_t)reverse_endianness((uint64_t)value, 16);
}

// Convert timeval to octets; assumes big-endian buffers of size 6 and 4
void timeval_to_octets(struct timeval *tv, uint8_t *buffer_sec, uint8_t *buffer_nsec) {
    //ESP_LOGI(TAG, "timeval_to_octets: %lld.%ld", tv->tv_sec, tv->tv_usec);
    int64_t tv_sec = (int64_t)tv->tv_sec;
    int64_t tv_nsec = (int64_t)tv->tv_usec * 1000L;
    memcpy(buffer_sec, &tv_sec, 6);
    memcpy(buffer_nsec, &tv_nsec, 4);
    //ESP_LOG_BUFFER_HEX("sec:", buffer_sec, (6));
    //ESP_LOG_BUFFER_HEX("nsec:", buffer_nsec, (4));
    reverse_octets(buffer_sec, (6));
    reverse_octets(buffer_nsec, (4));
    //ESP_LOG_BUFFER_HEX("rev_sec:", buffer_sec, (6));
    //ESP_LOG_BUFFER_HEX("rev_nsec:", buffer_nsec, (4));
}

// Add two timevals, handling overflow and normalization
void timeval_add(struct timeval *result, 
                             struct timeval *a, 
                             struct timeval *b) {
    result->tv_sec = a->tv_sec + b->tv_sec;
    result->tv_usec = a->tv_usec + b->tv_usec;
    
    // Normalize if microseconds overflow
    if (result->tv_usec >= 1000000) {
        result->tv_sec += result->tv_usec / 1000000;
        result->tv_usec %= 1000000;
    }
}

// Subtract timevals (result = a - b), handling underflow and normalization
void timeval_subtract(struct timeval *result,
                                  struct timeval *a,
                                  struct timeval *b) {
    result->tv_sec = a->tv_sec - b->tv_sec;
    result->tv_usec = a->tv_usec - b->tv_usec;
    
    // Normalize if microseconds underflow
    if (result->tv_usec < 0) {
        result->tv_sec--;
        result->tv_usec += 1000000;
    }
}

// Divide a timeval by an integer, maintaining precision
struct timeval timeval_divide_by_int(struct timeval tv, int divisor) {
    struct timeval result;
    
    // Convert all to microseconds to maintain precision
    long long total_usec = (tv.tv_sec * 1000000LL) + tv.tv_usec;
    
    // Perform the division
    total_usec /= divisor;
    
    // Convert back to seconds and microseconds
    result.tv_sec = total_usec / 1000000;
    result.tv_usec = total_usec % 1000000;
    
    return result;
}

// Compare two timevals
// Returns: -1 if t1 < t2, 0 if equal, 1 if t1 > t2
int compare_timeval(struct timeval t1, struct timeval t2) {
    if (t1.tv_sec < t2.tv_sec) return -1;
    if (t1.tv_sec > t2.tv_sec) return 1;
    if (t1.tv_usec < t2.tv_usec) return -1;
    if (t1.tv_usec > t2.tv_usec) return 1;
    return 0;
}

// Add an item to the front of a list
esp_err_t add_to_list_front(void *item_to_add, void *list, size_t item_size, size_t list_size) {
    if (item_to_add == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    // Move existing items back by one position
    memmove(list + item_size, list, item_size * (list_size - 1));
    // Add new item to front
    memcpy(list, item_to_add, item_size);
    return ESP_OK;
}

// Convert period in msec to log period
int8_t msec_to_log_period(uint16_t msec_period) {
    // logMessagePeriod = log2(interval_seconds)
    // Clamp between -128 and 127 as per IEEE 1588
    double log2_value = log2(msec_period / 1e3);
    // Round to nearest integer
    int8_t log_period = (int8_t)round(log2_value);
    // Clamp to valid range
    if (log_period < -128) return -128;
    if (log_period > 127) return 127;
    
    return log_period;
}

// Convert log period to period in msec
uint16_t log_period_to_msec(int8_t log_period) {
    // interval = 2^logMessagePeriod
    return (uint16_t)(pow(2.0, log_period) * 1e3);
}

// Convert scaled to PPM
double scaled_to_ppm(int32_t scaled_value) {
    // Convert from 2^41 scaled value to PPM
    return ((double)scaled_value / pow(2, 41)) * 1e6;  // multiply by 1e6 for PPM
}

// Convert PPM to scaled
int32_t ppm_to_scaled(double ppm_value) {
    // Convert from PPM to 2^41 scaled value
    return (int32_t)((ppm_value / 1e6) * pow(2, 41));
}

// Convert a timespec to milliseconds
int64_t timespec_to_ms(const struct timespec *ts)
{
  return ts->tv_sec * MSEC_PER_SEC + (ts->tv_nsec / NSEC_PER_MSEC);
}

// Function to add milliseconds to a timeval
void timeval_add_ms(struct timeval *tv, int ms) {
    tv->tv_sec += ms / 1000;
    tv->tv_usec += (ms % 1000) * 1000;
    if (tv->tv_usec >= 1000000) {
        tv->tv_sec++;
        tv->tv_usec -= 1000000;
    }
}

/* Convert three integers to a 3PE value */
uint8_t int_to_3pe(int value1, int value2, int value3) {
    return (value1 * 36) + (value2 * 6) + value3;
}

/* Convert a 3PE value to three integers */
void three_pe_to_int(uint8_t value, int *value1, int *value2, int *value3) {
    *value1 = value / 36;
    *value2 = (value % 36) / 6;
    *value3 = value % 6;
}

/* Check if a value is in an array of integers */
bool in_array_of_int(int value, int *array, size_t size) {
    bool result = false;
    for (size_t i = 0; i < size; i++) { 
        if (array[i] == value) { 
            result = true; 
            break; 
        } 
    }
    return result;
}
