/*
 * Copyright 2024 Scramble Tools
 * License: MIT
 *
 * ESP_AVB Component
 *
 * This component provides an implementation of an AVB talker and listener.
 */

#ifndef ESP_AVB_AVBUTILS_H_
#define ESP_AVB_AVBUTILS_H_

#include <esp_log.h>
#include <esp_eth.h>
#include <lwip/prot/ethernet.h> // Ethernet header
#include <lwip/prot/ieee.h>
#include <arpa/inet.h> // ntohs
#include <string.h>
#include <math.h>

#define avbinfo(format, ...) ESP_LOGI("AVB", format, ##__VA_ARGS__)
#define avbwarn(format, ...) ESP_LOGW("AVB", format, ##__VA_ARGS__)
#define avberr(format, ...) ESP_LOGE("AVB", format, ##__VA_ARGS__)

#define ETH_MAX_PAYLOAD_LENGTH 1486

/* Error and OK definitions */
#define ERROR ESP_FAIL
#define NOT_FOUND -1
#define OK ESP_OK

#define UNUSED (void)

/* Time constants */
#define MSEC_PER_SEC 1000
#define NSEC_PER_USEC 1000
#define NSEC_PER_MSEC 1000000ll
#define NSEC_PER_SEC 1000000000ll

// bitswap helpers
#define bitswap_64(x)   ((uint64_t)                 \
     ((((x) & 0xff00000000000000ull) >> 56) |       \
      (((x) & 0x00ff000000000000ull) >> 40) |       \
      (((x) & 0x0000ff0000000000ull) >> 24) |       \
      (((x) & 0x000000ff00000000ull) >>  8) |       \
      (((x) & 0x00000000ff000000ull) <<  8) |       \
      (((x) & 0x0000000000ff0000ull) << 24) |       \
      (((x) & 0x000000000000ff00ull) << 40) |       \
      (((x) & 0x00000000000000ffull) << 56)))

#define bitswap_32(x)    ((uint32_t)                \
    ((((x) & 0xff000000) >> 24) |                   \
     (((x) & 0x00ff0000) >>  8) |                   \
     (((x) & 0x0000ff00) <<  8) |                   \
     (((x) & 0x000000ff) << 24)))

#define bitswap_16(x)   ((uint16_t)                 \
    ((((x) & 0xff00) >> 8) |                        \
     (((x) & 0x00ff) << 8)))

// Functions
uint64_t octets_to_uint(const uint8_t *buffer, size_t size);
void octets_to_timeval(const uint8_t *buffer, struct timeval *tv);
void reverse_octets(uint8_t *buffer, size_t size);
void octets_to_hex_string(const uint8_t* buffer, size_t size, char* hex_string, unsigned char delimiter);
void octets_to_binary_string(const uint8_t *buffer, size_t size, char *bit_string);
void int_to_octets(void *value, uint8_t *buffer, size_t size);
void int_to_binary_string(uint64_t value, int num_bits, char *bit_string, bool reverse_order);
void int_to_binary_string_64(uint64_t value, char *bit_string);
void int_to_binary_string_32(uint32_t value, char *bit_string);
void int_to_binary_string_16(uint16_t value, char *bit_string);
uint64_t reverse_endianness(uint64_t value, int num_bits);
uint64_t reverse_endianness_64(uint64_t value);
uint32_t reverse_endianness_32(uint32_t value);
uint16_t reverse_endianness_16(uint16_t value);
void timeval_to_octets(struct timeval *tv, uint8_t *buffer_sec, uint8_t *buffer_nsec);
void timeval_add(struct timeval *result, struct timeval *a, struct timeval *b);
void timeval_subtract(struct timeval *result, struct timeval *a, struct timeval *b);
struct timeval timeval_divide_by_int(struct timeval tv, int divisor);
int compare_timeval(struct timeval t1, struct timeval t2);
esp_err_t add_to_list_front(void *item_to_add, void *list, size_t item_size, size_t list_size);
int8_t msec_to_log_period(uint16_t msec_period);
uint16_t log_period_to_msec(int8_t log_period);
double scaled_to_ppm(int32_t scaled_value);
int32_t ppm_to_scaled(double ppm_value);
int64_t timespec_to_ms(const struct timespec *ts);
uint8_t int_to_3pe(int value1, int value2, int value3);
void three_pe_to_int(uint8_t value, int *value1, int *value2, int *value3);

#endif /* ESP_AVB_AVBUTILS_H_ */