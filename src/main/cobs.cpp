// Elmo Trolla, 2019
// License: pick one - public domain / UNLICENSE (https://www.unlicense.org) / MIT (https://opensource.org/licenses/MIT).

// # metavar: sync

#include "cobs.h"

#include "crccalc.h"
#include "sys_globals.h" // ASSERT, IS_POWER_OF_TWO


// Packetizes given data while writing the result to the given ringbuf. Suports arbitrary packet lengths.
// COBS-encodes the data.
//
// NB! make sure that the dst has enough room for the packet with all its overhead/framing/crc. Use the function
//     cobs_encode_crc16_frame_overhead to get a bit exaggerated, but a fast and usable value.
//
// https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing
// This function is originally from http://www.jacquesf.com/2011/03/consistent-overhead-byte-stuffing/
// (the original code had a problem with src_len == 0)
//
// params:
//     dst             : ringbuf start address
//     start_write_pos : (start_write_pos & write_mask) is the index of the first free byte into dst
//     write_mask      : dst ringbuf size-1, and assumes size is a power of two,
//                       or we'd have to use modulo instead of simple masking.
// return:
//     0 if src_len is 0.
//     num bytes written to dst otherwise.
uint32_t cobs_encode_crc16(const uint8_t* __restrict src, uint32_t src_len,
                           uint8_t* __restrict dst,
                           uint32_t start_write_pos, uint32_t write_mask)
{
	ASSERT(IS_POWER_OF_TWO(write_mask+1));

	uint32_t read_index = 0;
	uint32_t write_pos  = start_write_pos + 1; // to get the index, always mask it with write_mask
	uint32_t code_pos   = start_write_pos;
	uint8_t  code       = 1;
	uint16_t crc        = 0xffff;

	if (src_len == 0) return 0;

	while (read_index < src_len) {
		uint8_t b = src[read_index++];
		if (b == 0) {
			// our current chunk just finished. set value of the first byte of the output chunk to chunk len.
			dst[code_pos & write_mask] = code;
			code = 1;
			code_pos = write_pos++; // remember the start index of the new chunk in the write buffer and move write cursors.
		} else {
			dst[write_pos++ & write_mask] = b;
			if (++code == 0xff) {
				dst[code_pos & write_mask] = code;
				code = 1;
				code_pos = write_pos++;
			}
		}
		crc = crccalc_crc16_step(crc, b);
	}

	// now append two bytes of crc

	read_index = 0;
	uint8_t crcbuf[2] = {(uint8_t)(crc >> 8), (uint8_t)(crc & 0xff)};

	while (read_index < 2) {
		uint8_t b = crcbuf[read_index++];
		if (b == 0) {
			dst[code_pos & write_mask] = code;
			code = 1;
			code_pos = write_pos++;
		} else {
			dst[write_pos++ & write_mask] = b;
			if (++code == 0xff) {
				dst[code_pos & write_mask] = code;
				code = 1;
				code_pos = write_pos++;
			}
		}
	}

	dst[code_pos & write_mask] = code;

	return write_pos - start_write_pos;
}

// Decode COBS-encoded data. Supports arbitrary packet length.
// https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing
// This function is originally from http://www.jacquesf.com/2011/03/consistent-overhead-byte-stuffing/
// but with a little fix. This function is also ~20% faster than the pointer-based wikipedia
// version for data that contains relatively few zeroes.
//
// NB! assumes input data does not contain any zeroes.
// Remove the "restrict" qualifiers if compiling with a pre-C99 C dialect.
//
// For documentation, look into l_cobs_decode(..). This function is a simple mod to add crc calculation.
// Zeroes in input data is an error, but sometimes detected, sometimes not.
// return: -1 on decode error, -2 on crc error, num bytes written to dst otherwise (2 bytes of crc are included).
//         on success, output length starts from 3 and is always at least one byte smaller than input length.
int32_t cobs_decode_crc16(const uint8_t* __restrict src, size_t len, uint8_t* __restrict dst)
{
	size_t read_index = 0;
	size_t write_index = 0;
	uint16_t crc = 0xffff; // initial crc value

	if (len < 4) // cobs start overhead, two crc bytes, at least one byte of payload
		return -1;

	while (read_index < len) {
		uint8_t code = src[read_index];

		if (read_index + code > len || code == 0)
			return -1;

		read_index++;

		for (int i = 1; i < code; i++) {
			uint8_t b = src[read_index++];
			dst[write_index++] = b;
			crc = crccalc_crc16_step(crc, b);
		}

		if (code != 0xff && read_index != len) {
			dst[write_index++] = 0;
			crc = crccalc_crc16_step(crc, 0);
		}
	}

	if (crc != 0)
		return -2;

	return write_index;
}

// Same as the 16-bit crc version of l_cobs_encode_crc, but for 8-bit crc.
uint32_t cobs_encode_crc8(const uint8_t* __restrict src, uint32_t src_len,
                          uint8_t* __restrict dst,
                          uint32_t start_write_pos, uint32_t write_mask)
{
	ASSERT(IS_POWER_OF_TWO(write_mask+1));
	uint32_t read_index = 0;
	uint32_t write_pos  = start_write_pos + 1; // to get the index, always mask it with write_mask
	uint32_t code_pos   = start_write_pos;
	uint8_t  code       = 1;
	uint8_t  crc        = 0x01; // initial crc value

	if (src_len == 0) return 0;

	while (read_index < src_len) {
		uint8_t b = src[read_index++];
		if (b == 0) {
			// our current chunk just finished. set value of the first byte of the output chunk to chunk len.
			dst[code_pos & write_mask] = code;
			code = 1;
			code_pos = write_pos++; // remember the start index of the new chunk in the write buffer and move write cursors.
		} else {
			dst[write_pos++ & write_mask] = b;
			if (++code == 0xff) {
				dst[code_pos & write_mask] = code;
				code = 1;
				code_pos = write_pos++;
			}
		}
		crc = crccalc_crc8_step(crc, b);
	}

	// now append the crc byte

	if (crc == 0) {
		dst[code_pos & write_mask] = code;
		dst[write_pos++ & write_mask] = 1;
	} else {
		dst[write_pos++ & write_mask] = crc;
		if (++code == 0xff) {
			dst[code_pos & write_mask] = code;
			code = 1;
			code_pos = write_pos++;
		}
		dst[code_pos & write_mask] = code;
	}

	return write_pos - start_write_pos;
}

// Same as the 16-bit crc version of l_cobs_decode_crc, but for 8-bit crc.
int32_t cobs_decode_crc8(const uint8_t* __restrict src, size_t len, uint8_t* __restrict dst)
{
	size_t read_index = 0;
	size_t write_index = 0;
	uint8_t crc = 0x01; // initial crc value

	if (len < 3) // cobs start overhead, one crc bytes, at least one byte of payload
		return -1;

	while (read_index < len) {
		uint8_t code = src[read_index];

		if (read_index + code > len || code == 0)
			return -1;

		read_index++;

		for (int i = 1; i < code; i++) {
			uint8_t b = src[read_index++];
			dst[write_index++] = b;
			crc = crccalc_crc8_step(crc, b);
		}

		if (code != 0xff && read_index != len) {
			dst[write_index++] = 0;
			crc = crccalc_crc8_step(crc, 0);
		}
	}

	if (crc != 0)
		return -2;

	return write_index;
}
