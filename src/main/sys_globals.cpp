// Elmo Trolla, 2019, 2020
// License: pick one - public domain / UNLICENSE (https://www.unlicense.org) / MIT (https://opensource.org/licenses/MIT).
//          except for g_timestamp_us_to_time(). that one is heavily modified and from https://sourceware.org/newlib/
//              newlib\libc\time\gmtime_r.c and should have a BSD license. check that repo for the awesome authors.

#include "sys_globals.h"

#include <stdio.h>
#include <string.h> // memset, strlen
#include <sys/time.h> // gettimeofday

//#include "esp_eth.h"
#include "esp_system.h" // esp_read_mac, ESP_MAC_WIFI_STA
#include "esp_log.h"


#ifdef USE_FREERTOS
	#include "freertos/FreeRTOS.h"
	#include "freertos/task.h"

	portMUX_TYPE g_mutex = portMUX_INITIALIZER_UNLOCKED;
#endif


static const char* TAG = "sys_globals";


static u8   l_mac_addr[6] = {0};
static char l_mac_addr_str[] = {'0','0',':','0','0',':','0','0',':','0','0',':','0','0',':','0','0','\0'}; // 17 chars + 1 zeroterm

// Return wifi mac.
const char* g_mac_addr_get_str() {
	static bool mac_read;
	if (!mac_read) {
		if (esp_read_mac(l_mac_addr, ESP_MAC_WIFI_STA) != ESP_OK) {
			ESP_LOGE(TAG, "failed to get mac addr");
		} else {
			u8* m = l_mac_addr;
			sprintf(l_mac_addr_str, "%02x:%02x:%02x:%02x:%02x:%02x", m[0], m[1], m[2], m[3], m[4], m[5]);
			mac_read = true;
		}
	}
	return l_mac_addr_str;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// clamp/normalize
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// g_normalize_angle_0_360f: this may be a faster version, depending on how large the angle is:
// while (a < 0) a += 360.f; while (a >= 360) a -= 360.f; return a;
// g_normalize_angle_180f: this may be a faster version, depending on how large the angle is:
// while (a < -180.f) a += 360.f; while (a >= 180.f) a -= 360.f; return a;

float  g_normalize_angle_0_360f(float  a)   { a = fmodf(a, 360.f); return a < 0.f ? a + 360.f : a; }
double g_normalize_angle_0_360d(double a)   { a = fmod (a, 360); return a < 0 ? a + 360 : a; }
float  g_normalize_angle_360_360f(float  a) { return fmodf(a, 360); }
double g_normalize_angle_360_360d(double a) { return fmod(a, 360); }
float  g_normalize_angle_180f(float  a)     { a = fmodf(a + 180.f, 360.f); return a < 0.f ? a + 180.f : a - 180.f; }
double g_normalize_angle_180d(double a)     { a = fmod (a + 180, 360); return a < 0 ? a + 180 : a - 180; }
float  g_angledifff(float  a1, float  a2)   { return g_normalize_angle_180f(a2 - a1); }
double g_anglediffd(double a1, double a2)   { return g_normalize_angle_180d(a2 - a1); }

float g_clampf(float v, float lo, float hi)
{
	if (lo > hi) { float t = lo; lo = hi; hi = t; }
	if (v > hi) return hi;
	if (v < lo) return lo;
	return v;
}

double g_clampd(double v, double lo, double hi)
{
	if (lo > hi) { double t = lo; lo = hi; hi = t; }
	if (v > hi) return hi;
	if (v < lo) return lo;
	return v;
	// could use this, but for our case, hi can be lower than lo.
	//v = v > lo ? v : lo;
	//return v < hi ? v : hi;
}

// g_clampf( 90, 50) ->  50
// g_clampf(-90, 50) -> -50
// g_clampf( 10, 50) ->  10
// g_clampf(-10, 50) -> -10
float g_clampf(float v, float limit) // clamp between -limit..+limit
{
	limit = limit < 0 ? -limit: limit;
	if (v > limit) return limit;
	if (v < -limit) return -limit;
	return v;
}

double g_clampd(double v, double limit) // clamp between -limit..+limit
{
	limit = limit < 0 ? -limit: limit;
	if (v > limit) return limit;
	if (v < -limit) return -limit;
	return v;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// time
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// return current time. unix time in microseconds.
i64 g_timestamp_us() {
	struct timeval tv_now;
	gettimeofday(&tv_now, NULL);
	return (i64)tv_now.tv_sec * 1000000L + (i64)tv_now.tv_usec;
}

//i64 esp_timer_get_time() // microseconds since boot


// Wikipedia:
//   When a leap second occurs, so that the UTC day is not exactly 86,400 seconds long, a discontinuity occurs in the Unix time number.
//   Observe that when a positive leap second occurs (i.e., when a leap second is inserted) the Unix time numbers repeat themselves.
//
// Original code from: https://gmbabar.wordpress.com/2010/12/01/mktime-slow-use-custom-function/
//
// month and day start from 1.
// usec can be negative. But make sure the wanted time is after the 1970 epoch. And i32 can represent
//     only 0xffffffff / 1000000 / 2 = +/-2147 seconds.
//
// return microseconds
// return 0 if year is < 1970 or month is > 12.
i64 g_time_to_timestamp_us(u16 year, u8 month, u8 day, u8 hour, u8 minute, i32 usec)
{
	// cumulative days per month, using values {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}
	const int cumulative_month_days[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365};

	if (year < 1970 || month > 12) return 0;

	uint32_t tyears = year - 1970;
	uint32_t leaps = (tyears + 2) / 4;
	if ((tyears+2) % 4 == 0 && month <= 2) leaps--;
	// uncomment these next two lines after year 2100?
	//uint32_t i = (year - 100) / 100;
	//leaps -= ( (i/4)*3 + i%4 );
	uint32_t tdays = cumulative_month_days[month-1] + (day - 1) + (tyears * 365) + leaps;

	return ((int64_t)tdays * 86400 + (int64_t)hour * 3600 + (int64_t)minute * 60) * 1000000 + usec;

	// some more random info:
	//http://pubs.opengroup.org/onlinepubs/9699919799/basedefs/V1_chap04.html#tag_04_15
	//tm_sec + tm_min*60 + tm_hour*3600 + tm_yday*86400 +
	//    (tm_year-70)*31536000 + ((tm_year-69)/4)*86400 -
	//    ((tm_year-1)/100)*86400 + ((tm_year+299)/400)*86400
}

// Move epoch from 01.01.1970 to 01.03.0000 (yes, Year 0) - this is the first
// day of a 400-year long "era", right after additional day of leap year.
// This adjustment is required only for date calculation, so instead of
// modifying time_t value (which would require 64-bit operations to work
// correctly) it's enough to adjust the calculated number of days since epoch.
#define EPOCH_ADJUSTMENT_DAYS 719468L

#define ADJUSTED_EPOCH_YEAR   0               // year to which the adjustment was made
#define ADJUSTED_EPOCH_WDAY   3               // 1st March of year 0 is Wednesday
#define DAYS_PER_ERA          146097L         // there are 97 leap years in 400-year periods. ((400 - 97) * 365 + 97 * 366)
#define DAYS_PER_CENTURY      36524L          // there are 24 leap years in 100-year periods. ((100 - 24) * 365 + 24 * 366)
#define DAYS_PER_4_YEARS      (3 * 365 + 366) // there is one leap year every 4 years
#define DAYS_PER_YEAR         365             // number of days in a non-leap year
#define DAYS_IN_JANUARY       31              // number of days in January
#define DAYS_IN_FEBRUARY      28              // number of days in non-leap February
#define YEARS_PER_ERA         400             // number of years per era

#define USECSPERMIN           (60L * 1000000L)
#define MINSPERHOUR           60L
#define HOURSPERDAY           24L
#define USECSPERHOUR          ((int64_t)USECSPERMIN * MINSPERHOUR)
#define USECSPERDAY           ((int64_t)USECSPERHOUR * HOURSPERDAY)
#define DAYSPERWEEK           7
#define MONSPERYEAR           12

#define YEAR_BASE                      0
#define EPOCH_YEAR                     1970
#define EPOCH_WDAY                     4
#define EPOCH_YEARS_SINCE_LEAP         2
#define EPOCH_YEARS_SINCE_CENTURY      70
#define EPOCH_YEARS_SINCE_LEAP_CENTURY 370

//#define isleap(y) ((((y) % 4) == 0 && ((y) % 100) != 0) || ((y) % 400) == 0)

/*
 gmtime_r.c
 Original Author: Adapted from tzcode maintained by Arthur David Olson.
 Modifications:
 - Changed to mktm_r and added __tzcalc_limits - 04/10/02, Jeff Johnston
 - Fixed bug in mday computations - 08/12/04, Alex Mogilnikov <alx@intellectronika.ru>
 - Fixed bug in __tzcalc_limits - 08/12/04, Alex Mogilnikov <alx@intellectronika.ru>
 - Move code from _mktm_r() to gmtime_r() - 05/09/14, Freddie Chopin <freddie_chopin@op.pl>
 - Fixed bug in calculations for dates after year 2069 or before year 1901. Ideas for
   solution taken from musl's __secs_to_tm() - 07/12/2014, Freddie Chopin <freddie_chopin@op.pl>
 - Use faster algorithm from civil_from_days() by Howard Hinnant - 12/06/2014, Freddie Chopin <freddie_chopin@op.pl>
..
 - Add microsecond precision, un-standardize, and break some important things for sure.. 2019-06-12 Elmo Trolla.
*/

// should work after year 2038
void g_timestamp_us_to_time(int64_t timestamp_us, g_time_struct_t* res)
{
	int32_t days;
	int64_t rem;
	int32_t era, year;
	uint32_t erayear, yearday, month, day;
	uint32_t eraday;

	//uint64_t timestamp_s = timestamp_us / 1000000; // can't use uint32_t here because of we need it to work after year 2038

	days = timestamp_us / USECSPERDAY + EPOCH_ADJUSTMENT_DAYS;
	rem  = timestamp_us % USECSPERDAY;
	if (rem < 0) {
		rem += USECSPERDAY;
		--days;
	}

	// Compute hour, min, and sec
	res->hour   = rem / USECSPERHOUR;
	rem        %= USECSPERHOUR;
	res->minute = rem / USECSPERMIN;
	res->usec   = rem % USECSPERMIN;

	// Compute year, month, day & day of year. For description of this algorithm see
	// http://howardhinnant.github.io/date_algorithms.html#civil_from_days
	era     = (days >= 0 ? days : days - (DAYS_PER_ERA - 1)) / DAYS_PER_ERA;
	eraday  = days - era * DAYS_PER_ERA; // [0, 146096]
	erayear = (eraday - eraday / (DAYS_PER_4_YEARS - 1) + eraday / DAYS_PER_CENTURY - eraday / (DAYS_PER_ERA - 1)) / 365; // [0, 399]
	yearday = eraday - (DAYS_PER_YEAR * erayear + erayear / 4 - erayear / 100); // [0, 365]
	month   = (5 * yearday + 2) / 153; // [0, 11]
	day     = yearday - (153 * month + 2) / 5 + 1; // [1, 31]
	month  += month < 10 ? 2 : -10;
	year    = ADJUSTED_EPOCH_YEAR + erayear + era * YEARS_PER_ERA + (month <= 1);

	res->year  = year - YEAR_BASE;
	res->month = month + 1;
	res->mday  = day;
}

// "2019-03-08T22:23:15.123Z". 25 characters with zero-termination.
void g_timestamp_us_to_iso8601(i64 timestamp_us, char* out_str, i32 str_capacity) {
	if (str_capacity < 25) { out_str[0] = 0; return; }
	g_time_struct_t t;
	g_timestamp_us_to_time(timestamp_us, &t);
	i32 sec = t.usec / 1000000; // 0..59
	i32 msec = t.usec % 1000000 / 1000;
	snprintf(out_str, str_capacity, "%04i-%02i-%02iT%02i:%02i:%02i.%03iZ", t.year, t.month, t.mday, t.hour, t.minute, sec, msec);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// uuid
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// return 0..15 for valid hex charanters. larger values for non-hex characters.
static u8 l_hexchar2int(char c) {
	// '0' = 48, '9' = 57, 'A' = 65, 'F' = 70, 'a' = 97, 'f' = 102
	u8 h = (u8)c;
	if (h  < '0') return 255;
	if (h <= '9') return c - '0';
	if (h  < 'A') return 255;
	if (h <= 'F') return c - 'A' + 10;
	if (h  < 'a') return 255;
	if (h <= 'f') return c - 'a' + 10;
	return 255;
}

bool g_uuid_t::is_equal(g_uuid_t* uuid) { return memcmp(uuid->buf, buf, sizeof(buf)) == 0; }
void g_uuid_t::from_buf(const u8* src_16bytes) {
	if (src_16bytes) memcpy(buf, src_16bytes, sizeof(buf));
	else             memset(buf, 0, sizeof(buf));
}
// Input : 16 bytes (buf)
// Output: 36 bytes ascii (out_str), "123e4567-e89b-12d3-a456-426655440000", with following caveats:
//
// Return:  0 on success (str_capacity >= 36)
//            First 36 bytes are filled with the uuid, 37-th byte (if available) with the zero-termination.
//         -1 if str_capacity < 36. Fills out_str with minus signs up to str_capacity.
//         -2 if no previous error but uuid is nullptr. Fills out_str with '0' characters. Append zero-term if room.
//            So this is a shortcut for "no input means uuid filled with zeros".
//
// out_str shall not be nullptr.
int g_uuid_bin_to_str_canonical(u8* uuid, char* out_str, u8 str_capacity) {
	assert(out_str);
	if (str_capacity < 36) {
		memset(out_str, '-', str_capacity);
		return -1;
	}
	if (!uuid) {
		memset(out_str, '0', 36);
		if (str_capacity > 36)
			out_str[36] = 0;
		return -2;
	}
	// "123e4567-e89b-12d3-a456-426655440000"
	u8* buf = uuid;
	u8 chunks[5] = { 4, 2, 2, 2, 6 };
	u8 i = 0;
	u8 k = 0;
	while (k < sizeof(chunks)) {
		if (k > 0)
			*out_str++ = '-';
		u8 j = 0;
		while (j++ < chunks[k]) {
			const char* hexes = "0123456789abcdef";
			u8 b = *buf++;
			*out_str++ = hexes[b >> 4];
			*out_str++ = hexes[b & 0xf];
		}
		k++;
	}
	if (str_capacity > 36)
		*out_str = 0;
	return 0;
}

// doc: g_uuid_bin_to_str_canonical(u8* uuid, char* out_str, u8 str_capacity)
int g_uuid_bin_to_str_canonical(g_uuid_t* uuid, char* out_str, u8 str_capacity) {
	return g_uuid_bin_to_str_canonical(uuid ? uuid->buf : nullptr, out_str, str_capacity);
}

// Input : uuid strings (uuid_str, zero-terminated) in the following formats
//         "123e4567-e89b-12d3-a456-426655440000" (36 bytes without zero-termination)
//         "123e4567e89b12d3a456426655440000" (32 bytes without zero-termination)
// Output: 16 bytes (out_buf)
//
// Return:  0 on success
//         -1 if uuid_str == nullptr or out_buf == nullptr.
//         -2 if input format is wrong or has the wrong length. out_buf is filled with 16 zeros.
int g_uuid_str_to_bin(const char* uuid_str, g_uuid_t* out_buf) {
	if (!uuid_str || !out_buf)
		return -1;
	return g_uuid_strn_to_bin(uuid_str, strlen(uuid_str), out_buf->buf);
}

int g_uuid_strn_to_bin(const char* uuid_str, u8 len, g_uuid_t* out_buf) {
	if (!uuid_str || !out_buf)
		return -1;
	return g_uuid_strn_to_bin(uuid_str, len, out_buf->buf);
}

int g_uuid_strn_to_bin(const char* uuid_str, u8 len, u8* out_buf) {
	if (!uuid_str || !out_buf)
		return -1;

	u8* dst = out_buf;

	if (len == 32) {
		// "123e4567e89b12d3a456426655440000"
		u8 i = 0;
		while (i < len) {
			u8 v1 = l_hexchar2int(uuid_str[i++]);
			u8 v2 = l_hexchar2int(uuid_str[i++]);
			if (v1 >= 16 || v2 >= 16)
				goto error_label;
			*dst++ = (v1 << 4) | v2;
		}
	} else if (len == 36) {
		// "123e4567-e89b-12d3-a456-426655440000"
		u8 chunks[5] = { 4, 2, 2, 2, 6 };
		u8 i = 0, k = 0;
		while (k < sizeof(chunks)) {
			if (i > 0 && uuid_str[i++] != '-')
				goto error_label;
			u8 j = 0;
			while (j++ < chunks[k]) {
				u8 v1 = l_hexchar2int(uuid_str[i++]);
				u8 v2 = l_hexchar2int(uuid_str[i++]);
				if (v1 >= 16 || v2 >= 16)
					goto error_label;
				*dst++ = (v1 << 4) | v2;
			}
			k++;
		}
	} else {
		goto error_label;
	}
	return 0;
error_label:
	memset(out_buf, 0, 16);
	return -2;
}

// esp-idf:
//     If the RF subsystem is not used by the program, the function bootloader_random_enable() can be called to enable
//     an entropy source. bootloader_random_disable() must be called before RF subsystem or I2S peripheral are used.
void g_uuid_generate(g_uuid_t* out_uuid) {
	esp_fill_random(out_uuid->buf, sizeof(out_uuid->buf));
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// text
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// Output in format "00011100_01010101". Always appends zero-termination (except when dst is nullptr or dst size is 0).
void g_generate_binary_string(char* dst, u8 dst_max_len, u64 val, u8 num_bits, u8 underscore_step) {
	if (!dst || dst_max_len == 0) return;

	if (num_bits == 0) {
		*dst = 0;
		return;
	}

	u32 num_underscores = underscore_step > 0 ? (num_bits - 1) / underscore_step : 0;
	u32 total_dst_chars = num_bits + num_underscores + 1; // with terminating zero

	if (total_dst_chars > dst_max_len)
		total_dst_chars = dst_max_len;

	char* cur = dst + total_dst_chars - 1; // move cursor to the end of the bitstring
	*cur-- = 0;
	u32 i = 0;

	while (cur >= dst) {
		*cur-- = val & 1 ? '1' : '0';
		val >>= 1;
		if (++i == underscore_step && cur >= dst) {
			*cur-- = '_';
			i = 0;
		}
	}
}

// Convert hex string to bytes. Ignores spaces. Hex characters have to come in doubles.
// valid input  : " 12d4   3f    "
// invalid input: " 12d4   3 f   "
// Return : num of bytes written
//          -1 on error.
int g_hex2buf(const char* src, u32 src_len, u8* dst, u32 dst_max_len) {
	if (dst_max_len < 2) return 0;

	u32 isrc = 0;
	u32 idst = 0;

	while (isrc < src_len) {
		if (src[isrc] == ' ') {
			isrc++;
			// if no characters left, end parsing. if only one character left, error.
			continue;
		}

		// makes sure we still have at least 2 characters left.
		if (isrc + 1 >= src_len)
			goto error_label;

		// next byte doesn't fit to dst?
		if (idst >= dst_max_len)
			goto error_label;

		u8 v1 = l_hexchar2int(src[isrc++]);
		u8 v2 = l_hexchar2int(src[isrc++]);
		if (v1 >= 16 || v2 >= 16)
			goto error_label;
		dst[idst++] = (v1 << 4) | v2;
	}

	return idst;

error_label:
	return -1;
}

// Convert binary buf to hex string with a space between every chunk_len input bytes. No spaces are inserted if
// chunk_len is 0. NB! no terminating zero written!
//
// Return : num of characters written
// Example:
//   input: "yamamotoche", 11 bytes, no terminating zero.
//   output: in case chunk_len is 4
//       if dst_max_len == 0,1 : ''
//       if dst_max_len ==  2  : '..'
//       if dst_max_len ==  3  : '7..'
//       if dst_max_len == 23  : '79616d61 6d6f746f 636..'
//       if dst_max_len >= 24  : '79616d61 6d6f746f 636865'
int g_buf2hex(const u8* src, u32 src_len, char* dst, u32 dst_max_len, u32 chunk_len) {
	if (dst_max_len < 2) return 0;

	u32 isrc = 0;
	u32 idst = 0;

	while (isrc < src_len) {
		const char* hexes = "0123456789abcdef";
		u8 b = src[isrc++];
		dst[idst++] = hexes[b >> 4];
		dst[idst++] = hexes[b & 0xf];

		if (isrc != src_len) {
			// insert a space after every fourth number
			if (chunk_len && isrc % chunk_len == 0 && idst < dst_max_len)
				dst[idst++] = ' ';

			// there are more bytes in input buf, but no more room in output buf. change last two chars to ellipsis.
			if (idst > dst_max_len - 2) {
				dst[dst_max_len - 1] = '.';
				dst[dst_max_len - 2] = '.';
				idst = dst_max_len;
				break;
			}
		}
	}
	return idst;
}

// Like g_buf2hex, but always zero-terminates the output and returns the dst pointer.
// Undefined behaviour if dst_max_len == 0 or out is NULL
char* g_buf2hexstr(const u8* src, u32 src_len, char* dst, u32 dst_max_len, u32 chunk_len) {
	if (dst && dst_max_len) {
		int l = g_buf2hex(src, src_len, dst, dst_max_len - 1, chunk_len);
		dst[l] = 0;
	}
	return dst;
}

// example:
//
//    u8 buf[100]
//
//    void fun() {
//        buf_wrap_t topic{buf, sizeof(buf)};
//        i32 i = topic.len;
//        i += topic.append("mugabe/");
//        i += topic.append("i_command_you");
//        if (i > topic.max_len)
//            bailout("overflow!");
//    }
i32 buf_wrap_t::append(u8* src, u32 len) {
	if (!src) return this->len;
	i32 copylen = len;
	if (copylen > this->max_len - this->len)
		copylen = this->max_len - this->len;
	memcpy(this->buf + this->len, src, copylen);
	this->len += copylen;
	return len;
};

i32 buf_wrap_t::append(buf_wrap_t* src)     { return this->append(src->buf, src->len); }
i32 buf_wrap_t::append(const char* src_str) { return this->append((u8*)src_str, strlen(src_str)); }

// Copy src to somewhere in dst. src and dst can clip.
// Overflow occurred if (return value > dst->max_len).
// Return:
//    * Index in dst of one byte past the last byte "copied" as if dst was infinite in both directions.
//      This allows chaining insert calls and checking for overflow only for the last call.
//    * dst_pos, if dst or src are NULL.
i32 g_buf_wrap_insert(buf_wrap_t* dst, u8* src, u32 len, i32 dst_pos) {
	if (!dst || !src) return dst_pos;
	i32 i1 = dst_pos;
	i32 i2 = dst_pos + len;
	if (i1 < 0) i1 = 0;
	if (i2 > dst->max_len) i2 = dst->max_len;
	if (i1 < i2) memcpy(dst->buf + i1, src, i2 - i1);
	return dst_pos + len;
};

i32 g_buf_wrap_insert(buf_wrap_t* dst, buf_wrap_t* src, i32 dst_pos=0) {
	return g_buf_wrap_insert(dst, src->buf, src->len, dst_pos);
}

i32 g_buf_wrap_insert(buf_wrap_t* dst, const char* src_str, i32 dst_pos=0) {
	return g_buf_wrap_insert(dst, (u8*)src_str, strlen(src_str), dst_pos);
}
