// Elmo Trolla, 2019
// License: pick one - public domain / UNLICENSE (https://www.unlicense.org) / MIT (https://opensource.org/licenses/MIT).

// # metavar: sync

#pragma once

#include <stdint.h>
#include <stddef.h> // size_t


uint32_t cobs_encode_crc16(const uint8_t* __restrict src, uint32_t src_len,
                          uint8_t* __restrict dst,
                          uint32_t start_write_pos, uint32_t write_mask);
int32_t  cobs_decode_crc16(const uint8_t* __restrict src, size_t len, uint8_t* __restrict dst);
uint32_t cobs_encode_crc8(const uint8_t* __restrict src, uint32_t src_len,
                          uint8_t* __restrict dst,
                          uint32_t start_write_pos, uint32_t write_mask);
int32_t  cobs_decode_crc8(const uint8_t* __restrict src, size_t len, uint8_t* __restrict dst);


// Calculate num of additional bytes per packet if you want to send src_len bytes in a packet.
// Overhead is minimum 4 or 5 bytes (one byte for COBS, one or two for crc, two for begin/end framing zeroes), plus one
// byte for every 254 src + crc bytes. Quite a complicated value to calculate, so we just use this dirty method that
// gives nearly 2 bytes of overhead per 254 bytes, instead of just 1.
//inline uint32_t cobs_encode_crc16_frame_overhead(uint32_t src_len) { return 3 + 2 + (src_len >> 8); }
//inline uint32_t cobs_encode_crc8_frame_overhead(uint32_t src_len)  { return 3 + 1 + (src_len >> 8); }
// if using the inline versions in "char bla[11 + cobs_encode_crc8_frame_overhead(11)]" then compiler gave error
//     undefined reference to `__cxa_end_cleanup'
// so, use the defines, and this way there's no dependency on -lstdc++
#define cobs_encode_crc16_frame_overhead(src_len) (3 + 2 + ((src_len) >> 8))
#define cobs_encode_crc8_frame_overhead(src_len)  (3 + 1 + ((src_len) >> 8))

#define cobs_encode_crc16_with_frame_overhead(src_len) (cobs_encode_crc16_frame_overhead(src_len) + src_len)
#define cobs_encode_crc8_with_frame_overhead(src_len)  (cobs_encode_crc8_frame_overhead(src_len) + src_len)
