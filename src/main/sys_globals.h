// Elmo Trolla, 2019, 2020
// License: pick one - public domain / UNLICENSE (https://www.unlicense.org) / MIT (https://opensource.org/licenses/MIT).

#pragma once

#include "version.h"

#include <stdlib.h> // exit
#include <stdint.h>
#include <math.h> // isnormal
#include <limits.h> // CHAR_MIN
#include <assert.h>
#include <esp_log.h>

#include "stdints.h"

#ifndef __cplusplus
	#include <stdbool.h>
#endif

#define USE_FREERTOS

// ------------------------------------------------------------------------------------------------------------------

#if CHAR_MIN != 0
	#error this program was developed with a compiler that had the char type as unsigned by default, but right not the char type is signed. you can probably just disable this check and see if compiler gives any sign warnings anywhere and then fix those.
#endif

#if defined(PART_TM4C123AH6PM) || defined(PART_TM4C123GH6PM) || defined(PART_TM4C129ENCPDT)

	#include "driverlib/interrupt.h"
	#include "driverlib/sysctl.h"
	#define INTERRUPTS_SAVEDISABLE() volatile bool _intrstate = IntMasterDisable()
	#define INTERRUPTS_RESTORE()     if (!_intrstate) IntMasterEnable()

#elif defined(USE_FREERTOS)

	#include "freertos/FreeRTOS.h"
	#include "freertos/task.h"
	// TODO: multicore support in that the mux should be separate for every ActiveObject.
	extern portMUX_TYPE g_mutex;
	// seems that portENTER_CRITICAL can be nested in esp-idf freertos version
	// also https://www.esp32.com/viewtopic.php?t=12621
	// freertos/portmacro.h
	#define INTERRUPTS_SAVEDISABLE() portENTER_CRITICAL(&g_mutex);
	#define INTERRUPTS_RESTORE()     portEXIT_CRITICAL(&g_mutex);

#else

	#error unsupported platform
	//#define INTERRUPTS_SAVEDISABLE() bool _intrstate = false;
	//#define INTERRUPTS_RESTORE()     if (_intrstate) {}
#endif

#define ELEMENTS_IN_ARRAY(array) (sizeof(array) / sizeof(array[0]))
#define MEMBER_SIZE(type, member) sizeof(((type *)0)->member)
// Convert integers to string literals.
// #define VERSION "appname-" STR(5) // results in "appname-5"
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
// example:
//     #define GENLINE(txt) CONCAT(txt_, __LINE__)
//     line 13: GENLINE(int varname_); // results in line 13 being: int varname_13;
#define CONCAT_HELPER(x, y) x##y
#define CONCAT(x, y) CONCAT_HELPER(x, y)
// http://www.pixelbeat.org/programming/gcc/static_assert.html
// http://stackoverflow.com/questions/19401887/how-to-check-the-size-of-a-structure-at-compile-time
// usage:
//     DUMB_STATIC_ASSERT(sizeof(can_aerospace_packet) == 12);
//     // if you get here an error, like "the size of an array must be greater than zero",
//     // then it means the size of the structure is not what we wanted.
#define DUMB_STATIC_ASSERT(test) typedef char CONCAT(asserter_, __LINE__)[( !!(test) )*2-1 ]
#define IS_POWER_OF_TWO(v) ((v) && !((v) & ((v) - 1)))
// to get rid of these warnings if for example not using logging but don't want to remove the params one-by-one:
//     warning #179-D: variable "txt" was declared but never referenced
//     warning #552-D: variable "txt" was set but never used
// usage:
//     int UNUSED(a) = calculate_val_for_logger();
#define UNUSED(x) x __attribute__((unused))
// Find next power of 2. 0->0, 1->2, 2->2, 3->4.. https://lists.freebsd.org/pipermail/freebsd-current/2007-February/069093.html
#define _helper_b2(x)  (           (x) | (           (x) >> 1) )
#define _helper_b4(x)  ( _helper_b2(x) | ( _helper_b2(x) >> 2) )
#define _helper_b8(x)  ( _helper_b4(x) | ( _helper_b4(x) >> 4) )
#define _helper_b16(x) ( _helper_b8(x) | ( _helper_b8(x) >> 8) )
#define _helper_b32(x) (_helper_b16(x) | (_helper_b16(x) >>16) )
#define NEXT_POWER_OF_2(x) (_helper_b32(x-1) + 1)

//#define ASSERT assert

#undef  ASSERT
#define ASSERT(val)    do { if (!(val)) { ESP_LOGE("ass", "%s:%i %s", __FILE__, __LINE__, "ASSERT\n"); } } while(0)

// little-endian note: least-significant bytes are on lower memory addresses.
union typeconv_t {
	uint8_t  u8;
	int8_t   i8;
	uint16_t u16;
	int16_t  i16;
	uint32_t u32;
	int32_t  i32;
	uint64_t u64;
	int64_t  i64;
	float    f32;
	float    f64;
	uint8_t  u8v[8];
	int8_t   i8v[8];
	uint16_t u16v[4];
	int16_t  i16v[4];
	uint32_t u32v[2];
	int32_t  i32v[2];
	uint64_t u64v[1];
	int64_t  i64v[1];
	float    f32v[2];
	float    f64v[1];
};

// Error codes from tinyos. In c++ programs we usually have to use the namespace because some values are alreay Defined somewhere.
namespace E {
	enum {
		SUCCESS =  0,
		FAIL    =  1, // Generic condition: backwards compatible
		SIZE    =  2, // Parameter passed in was too big.
		CANCEL  =  3, // Operation cancelled by a call.
		OFF     =  4, // Subsystem is not active
		BUSY    =  5, // The underlying system is busy; retry later
		INVAL   =  6, // An invalid parameter was passed
		RETRY   =  7, // A rare and transient failure: can retry
		RESERVE =  8, // Reservation required before usage
		ALREADY =  9, // The device state you are requesting is already set
		NOMEM   = 10, // Memory required not available
		NOACK   = 11, // A packet was not acknowledged
		VERSION = 12, // Wrong version
		LAST    = 12  // Last enum value
	};
};

typedef int error_t;

// ------------------------------------------------------------------------------------------------------------------

const char* g_mac_addr_get_str();

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// float/clamp/normalize
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// floating point:
//
//     https://stackoverflow.com/questions/2083550/flush-to-zero-behavior-in-floating-point-arithmetic
//     question: can I type a denormal floating point in source code? ie 0.000111f?
//     question: should i accept denormal numbers from outside and just let the cpu decide if it should be clipped to zero or not?
//
//     At startup, Floating-Point Status Control (FPSC) register value is zero (read with CCS debugger when the CPU is paused).
//     That means that the CPU is configured to not ignore/flush-to-zero denormal floats.

// Return true if input is not infinite, and not NaN.
inline bool  g_isvalidf(float v) { return !(isnan(v) || isinf(v)); }
inline float g_validatef(float v, float def_val) { if (isnan(v) || isinf(v)) return def_val; else return v; }
//inline bool g_isvalidf(float v) { return isnormal(v) || v == 0.f; }
//inline bool g_isvalidf(float v) { int c = fpclassify(v); return c == FP_NORMAL || c == FP_ZERO;  } // FP_SUBNORMAL is rejected. right or not? should convert to 0?

// Convenience functions. Feel free to copy and inline implementations to where necessary.
float  g_normalize_angle_0_360f(float a);
double g_normalize_angle_0_360d(double a);
float  g_normalize_angle_360_360f(float a);
double g_normalize_angle_360_360d(double a);
float  g_normalize_angle_180f(float a);       // -180..+180
double g_normalize_angle_180d(double a);      // -180..+180
float  g_angledifff(float a1, float a2);
double g_anglediffd(double a1, double a2);
float  g_clampf(float v, float lo, float hi); // if hi < lo, they're automatically swapped.
double g_clampd(double v, double lo, double hi);
float  g_clampf(float v, float limit);        // clamp between -limit..+limit
double g_clampd(double v, double limit);      // clamp between -limit..+limit

//char*  g_itoa(int val, int base, int max_txt_width=0);
//void   g_itoa2(int val, char* buf, int txt_width, char fillchar=' ');
//int    g_ftoa(float f, char* buf, int dplaces, int pad);


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// time conversion
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


struct g_time_struct_t {
	u16 year;
	u8  month; // [1, 12]
	u8  mday;  // [1, 31]
	u8  hour;
	u8  minute;
	i32 usec;  // 0..1e6-1
};

i64  g_timestamp_us(); // return current time. unix time in microseconds.
i64  g_time_to_timestamp_us(u16 year, u8 month, u8 day, u8 hour, u8 minute, i32 usec);
void g_timestamp_us_to_time(i64 timestamp_us, g_time_struct_t* res);
// "2019-03-08T22:23:15.123Z". 25 characters with zero-termination.
void g_timestamp_us_to_iso8601(i64 timestamp_us, char* out_str, i32 str_capacity);

// esp-idf: ets_delay_us


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// uuid
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//typedef struct g_uuid_t { u8 buf[16]; } g_uuid_t;
struct g_uuid_t {
	u8 buf[16];
	//g_uuid_t() {}
	//g_uuid_t(const u8* src_16bytes) { from_buf(src_16bytes); }
	void from_buf(const u8* src_16bytes);
	bool is_equal(g_uuid_t* uuid);
};

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
int g_uuid_bin_to_str_canonical(u8* uuid, char* out_str, u8 str_capacity);
// doc: g_uuid_bin_to_str_canonical(u8* uuid, char* out_str, u8 str_capacity)
int g_uuid_bin_to_str_canonical(g_uuid_t* buf, char* out_str, u8 str_capacity);
// Input : uuid strings (uuid_str, zero-terminated) in the following formats
//         "123e4567-e89b-12d3-a456-426655440000" (36 bytes without zero-termination)
//         "123e4567e89b12d3a456426655440000" (32 bytes without zero-termination)
// Output: 16 bytes (out_buf)
//
// Return:  0 on success
//         -1 if input format is wrong or has the wrong length. out_buf is filled with 16 zeros.
int g_uuid_str_to_bin(const char* uuid_str, g_uuid_t* out_buf);
int g_uuid_strn_to_bin(const char* uuid_str, u8 len, u8* out_buf);
int g_uuid_strn_to_bin(const char* uuid_str, u8 len, g_uuid_t* out_buf);
// esp-idf:
//     If the RF subsystem is not used by the program, the function bootloader_random_enable() can be called to enable
//     an entropy source. bootloader_random_disable() must be called before RF subsystem or I2S peripheral are used.
void g_uuid_generate(g_uuid_t* out_uuid);


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// text
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// Output in format "00011100_01010101". Always appends zero-termination (except when dst is nullptr or dst size is 0).
void g_generate_binary_string(char* dst, u8 dst_max_len, u64 val, u8 num_bits, u8 underscore_step=0);

// Convert hex string to bytes. Ignores spaces. Hex characters have to come in doubles.
// valid input  : " 12d4   3f    "
// invalid input: " 12d4   3 f   "
// Return : num of bytes written
//          -1 on error.
int g_hex2buf(const char* src, u32 src_len, u8* dst, u32 dst_max_len);

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
int g_buf2hex(const u8* src, u32 src_len, char* dst, u32 dst_max_len, u32 chunk_len=4);
// Like g_buf2hex, but always zero-terminates the output and returns the dst pointer.
// Undefined behaviour if dst_max_len == 0 or out is NULL
char* g_buf2hexstr(const u8* src_buf, u32 src_buf_len, char* dst_str, u32 dst_str_max_len, u32 chunk_len=4);

struct buf_wrap_t {
	buf_wrap_t() = default; // this won't initialize members
	buf_wrap_t(u8* buf, u32 max_len) : buf{ buf }, max_len{ max_len }, len{ 0 } {}
	u8* buf;
	u32 max_len; // capacity
	u32 len;
	//i32 insert(u8* src, u32 len, i32 dst_pos);
	//i32 insert(buf_wrap_t* src, i32 dst_pos);
	//i32 insert(const char* src_str, i32 dst_pos);
	i32 append(u8* src, u32 len);
	i32 append(buf_wrap_t* src);
	i32 append(const char* src_str);
};

// TODO: better doc (these won't change buf_wrap_t::len member for example), and intengrate with the struct.
//
// Copy src to somewhere in dst. src and dst can clip.
// Overflow occurred if (return value > dst->max_len).
// Return:
//    * Index in dst of one byte past the last byte "copied" as if dst was infinite in both directions.
//      This allows chaining insert calls and checking for overflow only for the last call.
//    * dst_pos, if dst or src are NULL.
i32 g_buf_wrap_insert(buf_wrap_t* dst, u8* src, u32 len, i32 dst_pos);
i32 g_buf_wrap_insert(buf_wrap_t* dst, buf_wrap_t* src, i32 dst_pos);
i32 g_buf_wrap_insert(buf_wrap_t* dst, const char* src_str, i32 dst_pos);

#define g_min(a,b) \
   ({ __typeof__ (a) _a = (a); \
      __typeof__ (b) _b = (b); \
      _a < _b ? _a : _b; })

#define g_max(a,b) \
   ({ __typeof__ (a) _a = (a); \
      __typeof__ (b) _b = (b); \
      _a > _b ? _a : _b; })
