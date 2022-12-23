// Elmo Trolla, 2019
// License: pick one - public domain / UNLICENSE (https://www.unlicense.org) / MIT (https://opensource.org/licenses/MIT).

// 2019-01-27

#include "leds.h"

#include <stdint.h>
#include <string.h> // memset
#include <inttypes.h> // PRIu64, ..

#include "freertos/FreeRTOS.h"
#include "freertos/task.h" // vTaskDelay

#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_pm.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "hardware.h"

// bikeep board, pins connected directly to cpu.
#define PIN_NUM_MISO -1
#define PIN_NUM_MOSI PINS_CPU.spi_mosi
#define PIN_NUM_CLK  PINS_CPU.spi_clk
#define PIN_NUM_CS   PINS_CPU.spi_cs1_led_module

// gpio_num_t spi_clk                     = GPIO_NUM_5;
// gpio_num_t spi_mosi                    = GPIO_NUM_14;
// gpio_num_t spi_cs0_e_ink               = GPIO_NUM_12;

// GpioExpander::Gpio eink_interrupt_input     = GpioExpander::P1_6;
// GpioExpander::Gpio eink_data_command_output = GpioExpander::P1_7;

#define LEDS_TIMER_HZ 50
#define LEDS_TIMER_PERIOD_US (u64)(1000 * 1000 / LEDS_TIMER_HZ)

#define l_timer_get_time(us) (esp_timer_get_time(us) / 1)

static spi_device_handle_t l_spi;

static const char* TAG = "leds";

#include "sys_globals.h"



/*

24 bitti

WS2813_V1.4_EN_18112018265398.pdf

hmmm mis spi sageduse ma peaks v6tma et k6ik variandid oleks detektitavad? 800 kHz?

0.2 microseconds tundub hea periood.

T0H 0.3
T1H 0.9 1.2 (1.2 is better. can use one byte for two bits, not one byte for 2.33 bits.
T0L 0.9 1.2
T1L 0.3

3.333 MHz?

need 2.631579 .. 4.545455 MHz

>>> 1./(1./1000000*0.22)
4545454.545454546
>>> 1./(1./1000000*0.38)
2631578.947368421
>>>
-

----

locked - red
free - green
unlocking - green or red? pulsating green.
locking - green or red? pulsating green.


  P_LED_SET_MAX_BRIGHTNESS u8 brightness # can lower for night-time.

  // state:
  // 0 - all off
  // 1 - free, green
  // 2 - locked - red
  // 3 - unlocking - pulsating green
  // 4 - locking - pulsating green
  P_LED_SET_STATE u8 state

*/

// TODO: update this doc
// layer 0 - hardware
// layer 1 - smoothen changes between wanted and previous colors.
// layer 2 - select wanted colors.
// layer 3 - handle overall state changes, info for layer 2.

#define	M_PIF   3.14159265358979323846f // pi
#define	M_PI_2F 1.57079632679489661923f // pi/2

#ifndef max
#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))
#endif

#define NUM_LEDS 8

#define LEDS_STATE_OFF             0 // off
#define LEDS_STATE_DOCKING_STARTED 1 // contracting green. unlocked. place the bike and lock the lever.
#define LEDS_STATE_DOCKING_ENDED   2 // expanding green. unlocked, safe to remove the bike
#define LEDS_STATE_FREE            3 // slow green?
#define LEDS_STATE_PARKING         4 // slow red?
#define LEDS_STATE_BOOKED          5 // slow yellow?
#define LEDS_STATE_DISABLED        6 // not implemented. blinking red dot?
#define LEDS_STATE_ERROR           7 // not implemented
#define LEDS_STATE_MAINTENANCE     8 // not implemented
#define LEDS_STATE_CHARGING        9 // not implemented

#define LEDS_STATE_NOT_AUTHORIZED 11 // not implemented. TODO: still need the next_state logic

static void l_leds_tick();
static bool l_calc_current_color(LedStateSetup* setup, int64_t cur_time_ms, u8 led_index, Color* out_color);
static inline void l_color_to_bitstream(u8 r, u8 g, u8 b, uint8_t* dstbuf);
static void l_color_to_bitstream(uint32_t color, uint8_t* dstbuf);
static void l_transfer_buf_to_spi(uint8_t* txdata, uint8_t len);
static void l_esp_timer_callback(void* arg);

esp_timer_handle_t   l_esp_timer_handle;
esp_pm_lock_handle_t l_esp_pm_lock_handle;

/*
 * The following 1024-element table was generated with this pyton script:

# Lookup table generator.
# Correcting for the eye's luminance response.
# Uses the CIE 1931 formula.
#
#   brightness - linear for the eye.
#   pwm (luminance) - linear for number of photons.

# Make a larger table than 256 because smaller table loses information if we use high-precision
# calculations up until to the table lookup in our c program.
# For example what if table[250]=237 and table[251]=240, and we need a value in table[250.5]?
NUM_ELEMENTS = 1024

table = []

for n in range(NUM_ELEMENTS):
    brightness = n / (NUM_ELEMENTS - 1.) * 100.  # convert brightness to 0..100
    pwm = pow( ((brightness + 16.) / 116.), 3 ) if brightness > 7.9996 else brightness / 903.3
    pwm = int(pwm * 255. + 0.5)  # convert to 0..255 with rounding
    table.append(pwm)

# write the table to file

with open("brightness_to_pwm.h", "wb") as f:
    txt = "const unsigned char brightness2pwm_table[] = {\n"
    ENTRIES_PER_LINE = 16
    for i in range(NUM_ELEMENTS / ENTRIES_PER_LINE):
        line = table[:ENTRIES_PER_LINE]
        table = table[ENTRIES_PER_LINE:]
        txt += "\t" + (", ".join(["0x%02x" % n for n in line])) + ",\n"

    txt = txt[:-2] + "\n\t};\n"  # cut off the last comma and add table ending
    f.write(txt)
*/
const unsigned char brightness2pwm_table[] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02,
	0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02,
	0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x03, 0x03, 0x03, 0x03, 0x03,
	0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
	0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
	0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
	0x04, 0x04, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05,
	0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06,
	0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07,
	0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x08, 0x08, 0x08, 0x08, 0x08,
	0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x09, 0x09, 0x09, 0x09, 0x09,
	0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x0a, 0x0a, 0x0a, 0x0a, 0x0a, 0x0a, 0x0a,
	0x0a, 0x0a, 0x0a, 0x0a, 0x0a, 0x0a, 0x0a, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b,
	0x0b, 0x0b, 0x0b, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0d,
	0x0d, 0x0d, 0x0d, 0x0d, 0x0d, 0x0d, 0x0d, 0x0d, 0x0d, 0x0d, 0x0e, 0x0e, 0x0e, 0x0e, 0x0e, 0x0e,
	0x0e, 0x0e, 0x0e, 0x0e, 0x0e, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x10,
	0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11,
	0x11, 0x11, 0x11, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x13, 0x13, 0x13, 0x13,
	0x13, 0x13, 0x13, 0x13, 0x13, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x15, 0x15, 0x15,
	0x15, 0x15, 0x15, 0x15, 0x15, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x17, 0x17, 0x17,
	0x17, 0x17, 0x17, 0x17, 0x17, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x19, 0x19, 0x19, 0x19,
	0x19, 0x19, 0x19, 0x19, 0x1a, 0x1a, 0x1a, 0x1a, 0x1a, 0x1a, 0x1a, 0x1b, 0x1b, 0x1b, 0x1b, 0x1b,
	0x1b, 0x1b, 0x1c, 0x1c, 0x1c, 0x1c, 0x1c, 0x1c, 0x1d, 0x1d, 0x1d, 0x1d, 0x1d, 0x1d, 0x1d, 0x1e,
	0x1e, 0x1e, 0x1e, 0x1e, 0x1e, 0x1e, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x20, 0x20, 0x20, 0x20,
	0x20, 0x20, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x23, 0x23,
	0x23, 0x23, 0x23, 0x23, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x25, 0x25, 0x25, 0x25, 0x25, 0x26,
	0x26, 0x26, 0x26, 0x26, 0x26, 0x27, 0x27, 0x27, 0x27, 0x27, 0x28, 0x28, 0x28, 0x28, 0x28, 0x29,
	0x29, 0x29, 0x29, 0x29, 0x29, 0x2a, 0x2a, 0x2a, 0x2a, 0x2a, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2c,
	0x2c, 0x2c, 0x2c, 0x2c, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x2e, 0x2e, 0x2e, 0x2e, 0x2e, 0x2f, 0x2f,
	0x2f, 0x2f, 0x2f, 0x30, 0x30, 0x30, 0x30, 0x31, 0x31, 0x31, 0x31, 0x31, 0x32, 0x32, 0x32, 0x32,
	0x32, 0x33, 0x33, 0x33, 0x33, 0x34, 0x34, 0x34, 0x34, 0x34, 0x35, 0x35, 0x35, 0x35, 0x36, 0x36,
	0x36, 0x36, 0x37, 0x37, 0x37, 0x37, 0x37, 0x38, 0x38, 0x38, 0x38, 0x39, 0x39, 0x39, 0x39, 0x3a,
	0x3a, 0x3a, 0x3a, 0x3b, 0x3b, 0x3b, 0x3b, 0x3c, 0x3c, 0x3c, 0x3c, 0x3d, 0x3d, 0x3d, 0x3d, 0x3e,
	0x3e, 0x3e, 0x3e, 0x3f, 0x3f, 0x3f, 0x3f, 0x40, 0x40, 0x40, 0x40, 0x41, 0x41, 0x41, 0x41, 0x42,
	0x42, 0x42, 0x42, 0x43, 0x43, 0x43, 0x43, 0x44, 0x44, 0x44, 0x44, 0x45, 0x45, 0x45, 0x46, 0x46,
	0x46, 0x46, 0x47, 0x47, 0x47, 0x47, 0x48, 0x48, 0x48, 0x49, 0x49, 0x49, 0x49, 0x4a, 0x4a, 0x4a,
	0x4b, 0x4b, 0x4b, 0x4b, 0x4c, 0x4c, 0x4c, 0x4d, 0x4d, 0x4d, 0x4d, 0x4e, 0x4e, 0x4e, 0x4f, 0x4f,
	0x4f, 0x50, 0x50, 0x50, 0x50, 0x51, 0x51, 0x51, 0x52, 0x52, 0x52, 0x53, 0x53, 0x53, 0x53, 0x54,
	0x54, 0x54, 0x55, 0x55, 0x55, 0x56, 0x56, 0x56, 0x57, 0x57, 0x57, 0x57, 0x58, 0x58, 0x58, 0x59,
	0x59, 0x59, 0x5a, 0x5a, 0x5a, 0x5b, 0x5b, 0x5b, 0x5c, 0x5c, 0x5c, 0x5d, 0x5d, 0x5d, 0x5e, 0x5e,
	0x5e, 0x5f, 0x5f, 0x5f, 0x60, 0x60, 0x60, 0x61, 0x61, 0x61, 0x62, 0x62, 0x62, 0x63, 0x63, 0x63,
	0x64, 0x64, 0x64, 0x65, 0x65, 0x65, 0x66, 0x66, 0x66, 0x67, 0x67, 0x68, 0x68, 0x68, 0x69, 0x69,
	0x69, 0x6a, 0x6a, 0x6a, 0x6b, 0x6b, 0x6b, 0x6c, 0x6c, 0x6d, 0x6d, 0x6d, 0x6e, 0x6e, 0x6e, 0x6f,
	0x6f, 0x6f, 0x70, 0x70, 0x71, 0x71, 0x71, 0x72, 0x72, 0x72, 0x73, 0x73, 0x74, 0x74, 0x74, 0x75,
	0x75, 0x76, 0x76, 0x76, 0x77, 0x77, 0x77, 0x78, 0x78, 0x79, 0x79, 0x79, 0x7a, 0x7a, 0x7b, 0x7b,
	0x7b, 0x7c, 0x7c, 0x7d, 0x7d, 0x7d, 0x7e, 0x7e, 0x7f, 0x7f, 0x7f, 0x80, 0x80, 0x81, 0x81, 0x81,
	0x82, 0x82, 0x83, 0x83, 0x84, 0x84, 0x84, 0x85, 0x85, 0x86, 0x86, 0x86, 0x87, 0x87, 0x88, 0x88,
	0x89, 0x89, 0x89, 0x8a, 0x8a, 0x8b, 0x8b, 0x8c, 0x8c, 0x8c, 0x8d, 0x8d, 0x8e, 0x8e, 0x8f, 0x8f,
	0x8f, 0x90, 0x90, 0x91, 0x91, 0x92, 0x92, 0x93, 0x93, 0x93, 0x94, 0x94, 0x95, 0x95, 0x96, 0x96,
	0x97, 0x97, 0x98, 0x98, 0x98, 0x99, 0x99, 0x9a, 0x9a, 0x9b, 0x9b, 0x9c, 0x9c, 0x9d, 0x9d, 0x9e,
	0x9e, 0x9e, 0x9f, 0x9f, 0xa0, 0xa0, 0xa1, 0xa1, 0xa2, 0xa2, 0xa3, 0xa3, 0xa4, 0xa4, 0xa5, 0xa5,
	0xa6, 0xa6, 0xa7, 0xa7, 0xa8, 0xa8, 0xa9, 0xa9, 0xaa, 0xaa, 0xab, 0xab, 0xab, 0xac, 0xac, 0xad,
	0xad, 0xae, 0xae, 0xaf, 0xaf, 0xb0, 0xb0, 0xb1, 0xb1, 0xb2, 0xb3, 0xb3, 0xb4, 0xb4, 0xb5, 0xb5,
	0xb6, 0xb6, 0xb7, 0xb7, 0xb8, 0xb8, 0xb9, 0xb9, 0xba, 0xba, 0xbb, 0xbb, 0xbc, 0xbc, 0xbd, 0xbd,
	0xbe, 0xbe, 0xbf, 0xc0, 0xc0, 0xc1, 0xc1, 0xc2, 0xc2, 0xc3, 0xc3, 0xc4, 0xc4, 0xc5, 0xc5, 0xc6,
	0xc7, 0xc7, 0xc8, 0xc8, 0xc9, 0xc9, 0xca, 0xca, 0xcb, 0xcb, 0xcc, 0xcd, 0xcd, 0xce, 0xce, 0xcf,
	0xcf, 0xd0, 0xd1, 0xd1, 0xd2, 0xd2, 0xd3, 0xd3, 0xd4, 0xd4, 0xd5, 0xd6, 0xd6, 0xd7, 0xd7, 0xd8,
	0xd9, 0xd9, 0xda, 0xda, 0xdb, 0xdb, 0xdc, 0xdd, 0xdd, 0xde, 0xde, 0xdf, 0xe0, 0xe0, 0xe1, 0xe1,
	0xe2, 0xe2, 0xe3, 0xe4, 0xe4, 0xe5, 0xe5, 0xe6, 0xe7, 0xe7, 0xe8, 0xe8, 0xe9, 0xea, 0xea, 0xeb,
	0xec, 0xec, 0xed, 0xed, 0xee, 0xef, 0xef, 0xf0, 0xf0, 0xf1, 0xf2, 0xf2, 0xf3, 0xf4, 0xf4, 0xf5,
	0xf5, 0xf6, 0xf7, 0xf7, 0xf8, 0xf9, 0xf9, 0xfa, 0xfb, 0xfb, 0xfc, 0xfc, 0xfd, 0xfe, 0xfe, 0xff
};

// current state info
//static u32   l_state;
static i64   l_state_start_t;     // time during initialization of state change
static Color l_state_start_color; // color during initialization of state change
static Color l_wanted_color;
static Color l_sent_color;        // real color sent to led
static const i64 l_statechange_smoothing_duration_us = 1 * 100000; // 0.1 second
static u32   l_next_state;
static u32   l_state_duration_ms;

static bool l_initialized;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



LedStateSetup LEDS_SETUP[] = {
	{
		// 0 LEDS_STATE_OFF

		NUM_LEDS,         // u8    total_leds;
		NUM_LEDS,         // u8    used_leds; // all unused leds are set to off
		0,                // u8    first_led; // starts from 0
		Color(0.f, 0.f, 0.f),    // Color color0; // start color
		Color(0.f, 0.f, 0.f),    // Color color1;
		0,                // u16   color0_duration_ms;
		0,                // u16   color1_duration_ms;

		0,                // u16   color10_change_duration_ms; // does not add to the period time. bites into the start of color0_duration_ms
		0,                // u16   color01_change_duration_ms; // does not add to the period time. bites into the start of color1_duration_ms

		0,                // u16   num_periods; // if 0xffff, then infinite. always finishes with leds off.
		0,                // u32   pattern_time_offset_ms; // in case you want to start from mid-pattern.

		0,                // i16   incremental_led_delay_ms; // in a multiled pattern, each led is delayed led_index * incremental_led_delay_ms milliseconds, where led_index 0 is at first_led.

		LEDS_PATTERN_TOCENTER,
	}, {
		// 1 LEDS_STATE_DOCKING_STARTED : green, moving from center to edges, 1 minute

		NUM_LEDS,         // total_leds
		NUM_LEDS,         // used_leds
		0,                // first_led
		Color(0.0f, 1.0f, 0.f),  // color0
		Color(0.f, 0.f, 0.f),    // color1
		300,              // color0_duration_ms
		1300,             // color1_duration_ms

		250,              // color10_change_duration_ms // fast start, slow fade
		500,              // color01_change_duration_ms // fast start, slow fade

		60000/(300+1300), // num_periods                // 1 minute by default
		0,                // pattern_time_offset_ms

		150,              // incremental_led_delay_ms

		LEDS_PATTERN_TOCENTER,
	}, {
		// 2 LEDS_STATE_DOCKING_ENDED : green, moving from edges to center, 1 minute

		NUM_LEDS,         // total_leds
		NUM_LEDS,         // used_leds
		0,                // first_led
		Color(0.0f, 1.0f, 0.f),  // color0
		Color(0.f, 0.f, 0.f),    // color1
		300,              // color0_duration_ms
		1300,             // color1_duration_ms

		250,              // color10_change_duration_ms // fast start, slow fade
		500,              // color01_change_duration_ms // fast start, slow fade

		60000/(300+1300), // num_periods                // 1 minute by default
		0,                // pattern_time_offset_ms

		150,              // incremental_led_delay_ms

		LEDS_PATTERN_FROMCENTER,
	}, {
		// 3 LEDS_STATE_FREE : all leds off.. don't know what to do here.
		//                     or.. infinite slow green pulsing, with a bit fainter green?

		NUM_LEDS,       // total_leds
		NUM_LEDS,       // used_leds
		0,              // first_led
		Color(0.0f, 1.0f, 0.f),// color0
		Color(0.f, 0.f, 0.f),  // color1
		2000,           // color0_duration_ms
		4000,           // color1_duration_ms

		2000,           // color10_change_duration_ms
		2000,           // color01_change_duration_ms

		0xffff,         // num_periods                // infinite
		0,              // pattern_time_offset_ms

		0,              // incremental_led_delay_ms

		LEDS_PATTERN_FROMCENTER,
	}, {
		// 4 LEDS_STATE_PARKING : all leds off.. don't know what to do here.
		//                        or.. infinite slow red pulsing, with a bit fainter red?

		NUM_LEDS,       // total_leds
		NUM_LEDS,       // used_leds
		0,              // first_led
		Color(1.0f, 0.f, 0.f), // color0
		Color(0.f, 0.f, 0.f),  // color1
		2000,           // color0_duration_ms
		4000,           // color1_duration_ms

		1000,           // color10_change_duration_ms
		1000,           // color01_change_duration_ms

		0xffff,         // num_periods                // infinite
		0,              // pattern_time_offset_ms

		0,              // incremental_led_delay_ms

		LEDS_PATTERN_FROMCENTER,
	}, {
		// 5 LEDS_STATE_BOOKED : all leds off.. don't know what to do here.
		//                       or.. infinite slow red pulsing, with a yellowish color?

		NUM_LEDS,       // total_leds
		NUM_LEDS,       // used_leds
		0,              // first_led
		Color(1.0f, 0.7f, 0.f),// color0
		Color(0.f, 0.f, 0.f),  // color1
		2000,           // color0_duration_ms
		4000,           // color1_duration_ms

		2000,           // color10_change_duration_ms
		2000,           // color01_change_duration_ms

		0xffff,         // num_periods                // infinite
		0,              // pattern_time_offset_ms

		0,              // incremental_led_delay_ms

		LEDS_PATTERN_FROMCENTER,
	}
};

static float clamp(float v, float lo, float hi)
{
	if (lo > hi) { float t = lo; lo = hi; hi = t; }
	if (v > hi) return hi;
	if (v < lo) return lo;
	return v;
}

// return 0..1. 0 if x <= edge0, 1 if x >= edge1.
static float smoothstep(float edge0, float edge1, float x) {
	// Scale, bias and saturate x to 0..1 range
	x = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
	// Evaluate polynomial
	return x * x * (3 - 2 * x);
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// public interface
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


static LedStateSetup l_active_ledsrule;
static bool          l_in_progress;


void leds_init() {
	if (l_initialized) return;

	esp_err_t           ret;

	/*
	spi_bus_config_t    buscfg = {
		.miso_io_num     = PIN_NUM_MISO,
		.mosi_io_num     = PIN_NUM_MOSI,
		.sclk_io_num     = PIN_NUM_CLK,
		.quadwp_io_num   = -1,
		.quadhd_io_num   = -1,
		.max_transfer_sz = 4
	};

	spi_device_interface_config_t devcfg = {
		.clock_speed_hz  = 2*1000*1000,                   // Clock out at 2 MHz
		.mode            = 0,                             // SPI mode 0
		.spics_io_num    = PIN_NUM_CS,                    // CS pin
		.queue_size      = 2,                             // We want to be able to queue 2? transactions at a time
	};
	*/

//	ret = gpio_set_direction(PIN_NUM_CS, GPIO_MODE_OUTPUT);
//	ESP_ERROR_CHECK(ret);

	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = 1ULL << PIN_NUM_CS;
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
	ret = gpio_config(&io_conf);
	ESP_ERROR_CHECK(ret);

	ret = gpio_set_level(PIN_NUM_CS, 0);
	ESP_ERROR_CHECK(ret);


	spi_bus_config_t buscfg = {};
	buscfg.miso_io_num     = PIN_NUM_MISO;
	buscfg.mosi_io_num     = PIN_NUM_MOSI;
	buscfg.sclk_io_num     = PIN_NUM_CLK;
	buscfg.quadwp_io_num   = -1;
	buscfg.quadhd_io_num   = -1;
	//buscfg.max_transfer_sz = 4;

	spi_device_interface_config_t devcfg = {};
	devcfg.clock_speed_hz  = 3300000;                   // 3.3 MHz is necessary for RGB leds. and also works for epaper.
	// mode: https://en.m.wikipedia.org/wiki/Serial_Peripheral_Interface#Clock_polarity_and_phase
	// https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_master.html
	devcfg.mode            = 0;                             // SPI mode 0
	devcfg.spics_io_num    = PIN_NUM_CS;                    // CS pin
	devcfg.queue_size      = 2;
	//// We want to be able to queue 2? transactions at a time

	// Initialize the SPI bus
	ret = spi_bus_initialize(VSPI_HOST, &buscfg, 1);
	ESP_ERROR_CHECK(ret);

	ret = spi_bus_add_device(VSPI_HOST, &devcfg, &l_spi);
	ESP_ERROR_CHECK(ret);

	const esp_timer_create_args_t timer_args = {
		.callback = &l_esp_timer_callback,
		.arg = nullptr,
		.dispatch_method = ESP_TIMER_TASK, // TODO: in the distant future, maybe esp-idf will implement ESP_TIMER_ISR
		.name = "leds_timer"
	};
	ESP_ERROR_CHECK(esp_timer_create(&timer_args, &l_esp_timer_handle));

	ret = esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 0, "leds", &l_esp_pm_lock_handle);

/*
	return 0;


	// random notes: data frame size is programmable from 4 to 16 bits. we use 8. should test with 16?
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_2);
	GPIOPinConfigure(GPIO_PF1_SSI1TX);
	GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_1);
	SSIConfigSetExpClk(SSI1_BASE, g_sys_clock_freq, SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 3300000, 8);
	SSIEnable(SSI1_BASE);
	leds_set_state(LEDS_STATE_OFF, 0);
	*/
	l_initialized = true;
}

// state_duration_ms 0 to use default duration.
void leds_set_state(u8 state, u32 state_duration_ms)
{
	if (!l_initialized) return;
	INTERRUPTS_SAVEDISABLE();

	if (state > ELEMENTS_IN_ARRAY(LEDS_SETUP)) state = ELEMENTS_IN_ARRAY(LEDS_SETUP);

	l_state_start_t = l_timer_get_time();

	if (!l_in_progress) {
		esp_timer_start_periodic(l_esp_timer_handle, LEDS_TIMER_PERIOD_US);
	}

	l_in_progress = true;

	l_active_ledsrule = LEDS_SETUP[state];
	// TODO: handle period special cases
	if (state_duration_ms != 0) {
		if (state_duration_ms == 0xffffffff) {
			l_active_ledsrule.num_periods = 0xffff; // infinite
		} else {
			u32 num_periods = state_duration_ms / (l_active_ledsrule.color0_duration_ms + l_active_ledsrule.color1_duration_ms);
			if (num_periods >= 0xffff) num_periods = 0xffff - 1;
			l_active_ledsrule.num_periods = num_periods;
		}
	}

	INTERRUPTS_RESTORE();
}

void leds_set_state_ex(LedStateSetup state) {
	if (!l_initialized) return;
	INTERRUPTS_SAVEDISABLE();

	l_state_start_t = l_timer_get_time();

	if (!l_in_progress) {
		esp_timer_start_periodic(l_esp_timer_handle, LEDS_TIMER_PERIOD_US);
	}

	l_in_progress = true;
	l_active_ledsrule = state;

	INTERRUPTS_RESTORE();
}

#define LEDS_MAX_NUM_LEDS 64
static u8 l_tx_buf[LEDS_MAX_NUM_LEDS * 12]; // raw buf sent to spi


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// private functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// call @ 50..100 Hz. 20 Hz is very flickery.
static void l_leds_tick()
{
	if (!l_initialized)
		return;
	if (!l_in_progress)
		return;

	i64 t = l_timer_get_time();
	i64 time_in_state_ms = (t - l_state_start_t) / 1000;

	// sanitize leds state

	LedStateSetup* ledsrule = &l_active_ledsrule;

	if (ledsrule->first_led >= LEDS_MAX_NUM_LEDS) ledsrule->first_led = ledsrule->used_leds = 0;
	ledsrule->total_leds = min(ledsrule->total_leds, LEDS_MAX_NUM_LEDS);
	ledsrule->used_leds  = min(ledsrule->used_leds, LEDS_MAX_NUM_LEDS - ledsrule->first_led);
	ledsrule->color10_change_duration_ms = min(ledsrule->color10_change_duration_ms, ledsrule->color0_duration_ms);
	ledsrule->color01_change_duration_ms = min(ledsrule->color01_change_duration_ms, ledsrule->color1_duration_ms);

	if (1) {
		l_in_progress = false;
		for (int i = 0; i < ledsrule->total_leds; i++) {
			l_in_progress |= l_calc_current_color(ledsrule, time_in_state_ms, i, &l_wanted_color);
			Color& col = l_wanted_color;

			// make brightness human-linear and convert from float to u8
			u8 r = brightness2pwm_table[(u32)(col.r * 1023.f + 0.5f) & 1023];
			u8 g = brightness2pwm_table[(u32)(col.g * 1023.f + 0.5f) & 1023];
			u8 b = brightness2pwm_table[(u32)(col.b * 1023.f + 0.5f) & 1023];

			l_color_to_bitstream(r, g, b, l_tx_buf + i*12);
		}

		l_transfer_buf_to_spi(l_tx_buf, ledsrule->total_leds * 12);
	}

	if (!l_in_progress) {
		esp_timer_stop(l_esp_timer_handle);
	}
}


// return blinking color state at timepos cur_time_us (microseconds).
// return false if pattern is finished.
// cur_time should start from 0
static bool l_calc_current_color(LedStateSetup* setup, int64_t cur_time_ms, u8 led_index, Color* out_color)
{
	// led is in the unused range. set color to 0 and indicate that the pattern has finished for this led.
	if (led_index < setup->first_led || led_index >= setup->first_led + setup->used_leds) {
		out_color->r = out_color->g = out_color->b = 0;
		return false;
	}

	led_index -= setup->first_led;

	// remap led indices. we can change led order, duplicate leds, generate different patterns with this.
	{
		u8 mid_led = setup->used_leds / 2;

		switch (setup->flags & 0x0f) {
		case LEDS_PATTERN_LEFTRIGHT:
			break;
		case LEDS_PATTERN_RIGHTLEFT:
			led_index = setup->used_leds - led_index - 1;
			break;
		case LEDS_PATTERN_FROMCENTER:
			if (led_index >= mid_led) led_index -= mid_led;
			else if (led_index < mid_led) led_index = mid_led - led_index - !(setup->used_leds & 1);
			break;
		case LEDS_PATTERN_TOCENTER:
			if (led_index >= mid_led) led_index -= mid_led;
			else if (led_index < mid_led) led_index = mid_led - led_index - !(setup->used_leds & 1);
			led_index = mid_led - led_index - !(setup->used_leds & 1);
			break;
		}
	}

	i32 led_time_offset_ms = -led_index * setup->incremental_led_delay_ms; // each next led is delayed x ms.
	cur_time_ms += led_time_offset_ms;

	i32 period_ms   = setup->color0_duration_ms + setup->color1_duration_ms;
	i64 end_time_ms = (i64)period_ms * setup->num_periods + setup->pattern_time_offset_ms;

	// time's up! all periods finished, or there were no periods!
	if ((cur_time_ms > end_time_ms && setup->num_periods != 0xffff) || setup->num_periods == 0) {
		out_color->r = out_color->g = out_color->b = 0;
		return false;
	}

	// pattern has not started for this led. keep it off and indicat that the pattern has not finished yet.
	if (cur_time_ms < 0) {
		out_color->r = out_color->g = out_color->b = 0;
		return true;
	}

	i32 period_timepos_ms  = (cur_time_ms + setup->pattern_time_offset_ms) % period_ms;
	//i32 current_period_num = period_timepos_ms / period_ms; // starts from 0

	f32 t = period_timepos_ms / 1000.f; // time in current period

	float color0_duration = setup->color0_duration_ms / 1000.f;
	//float color1_duration = setup->color1_duration_ms / 1000.f;
	float color10_change_duration = setup->color10_change_duration_ms / 1000.f;
	float color01_change_duration = setup->color01_change_duration_ms / 1000.f;
	//float pattern_time_offset   = setup->pattern_time_offset_ms / 1000.f;

	float d = smoothstep(0, color10_change_duration, t) *
	                     (1.f - smoothstep(color0_duration, color0_duration + color01_change_duration, t));

	//d = smoothstep(tp / color_change_duration) * (1.f - smoothstep((tp - color1_duration) / color_change_duration));
//	out_color = d * color1 + (1.f - d) * color2;

	// select d ratio of color0. if 1, select color0. if 0, select color 1.
	out_color->r = setup->color1.r + (setup->color0.r - setup->color1.r) * d;
	out_color->g = setup->color1.g + (setup->color0.g - setup->color1.g) * d;
	out_color->b = setup->color1.b + (setup->color0.b - setup->color1.b) * d;

	return true;
}

//// http://stackoverflow.com/questions/746171/best-algorithm-for-bit-reversal-from-msb-lsb-to-lsb-msb-in-c
//uint32_t l_flip_lowestbytebits(uint32_t b)
//{
//	return ((b * 0x0802LU & 0x22110LU) | (b * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16;
//}

//#define ENCODE_COLOR(r, g, b) ((g << 24) | (r << 16) | (b << 8))

// output: 12 bytes in dstbuf - grb encoded to bitstream to be sent out to SPI, high bits first.
static inline void l_color_to_bitstream(u8 r, u8 g, u8 b, uint8_t* dstbuf)
{
	// dstbuf is filled with data according to this logic:
	// every color bit takes 4 bits in the dstbuf.
	// 24 bits of color takes 4 * 24 = 96 bits = 12 bytes.
	// 1 byte input takes 4 bytes on SPI bus.

	// input : two color bits. output : one full byte for dstbuf.
	// this table should work for WS2813
	uint8_t table[] = {
		0b10001000, // color bit 00
		0b10001110, // color bit 01
		0b11101000, // color bit 10
		0b11101110, // color bit 11
	};

	*dstbuf++ = table[(g >> 6) & 0b11];
	*dstbuf++ = table[(g >> 4) & 0b11];
	*dstbuf++ = table[(g >> 2) & 0b11];
	*dstbuf++ = table[(g) & 0b11];
	*dstbuf++ = table[(r >> 6) & 0b11];
	*dstbuf++ = table[(r >> 4) & 0b11];
	*dstbuf++ = table[(r >> 2) & 0b11];
	*dstbuf++ = table[(r) & 0b11];
	*dstbuf++ = table[(b >> 6) & 0b11];
	*dstbuf++ = table[(b >> 4) & 0b11];
	*dstbuf++ = table[(b >> 2) & 0b11];
	*dstbuf++ = table[(b) & 0b11];
}

// color: rgbx
// output: 12 bytes in dstbuf - grb encoded to bitstream to be sent out to SPI, high bits first.
static void l_color_to_bitstream(uint32_t color, uint8_t* dstbuf)
{
	u8 r = (color & 0xff000000) >> 24;
	u8 g = (color & 0x00ff0000) >> 16;
	u8 b = (color & 0x0000ff00) >> 8;
	l_color_to_bitstream(r, g, b, dstbuf);

	// method 2
	//
	//// swap rgbx to grbx.
	//color = ((color & 0x00ff0000) << 8) || ((color & 0xff000000) >> 8) || (color & 0x0000ffff);
	//for (int i = 0; i < 12; i++) {
	//	*dstbuf++ = table[((color & 0b11000000000000000000000000000000ULL) >> 30 ) & 0b11];
	//	color <<= 2;
	//}
}

// http://stackoverflow.com/questions/746171/best-algorithm-for-bit-reversal-from-msb-lsb-to-lsb-msb-in-c
//static uint32_t l_flip_lowestbytebits(uint32_t b)
//{
//	return ((b * 0x0802LU & 0x22110LU) | (b * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16;
//}

//static void l_transfer_buf_to_spi(uint8_t* txdata, uint8_t len)
//{
//	uint32_t tx[256];
//
//	// convert tx data to u32
//	for (int i = 0; i < len; i++)
//		tx[i] = txdata[i];
//
//	for (int i = 0; i < len; i++)
//		//SSIDataPut(SSI1_BASE, l_flip_lowestbytebits(tx[i]));
//		SSIDataPut(SSI1_BASE, tx[i]);
//
//	// Wait until SSI0 is done transferring all the data in the transmit FIFO.
//	while (SSIBusy(SSI1_BASE));
//}

//void EpdIf::SpiTransfer(uint8_t* data, int32_t len) {
static void l_transfer_buf_to_spi(uint8_t* txdata, uint8_t len) {

	spi_transaction_t t;
	memset(&t, 0, sizeof(t));

	t.length    = 8 * len; // len in bits
	t.tx_buffer = txdata;
	t.user      = (void*)1; // data given to callbacks to identify the transaction.

	// Reference says that SPI master uses APB lock for transactions, but without the manual locking here,
	// dynamic cpu clocking and without ethernet initialized, leds glitched white quite often.
	// https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/power_management.html
	esp_pm_lock_acquire(l_esp_pm_lock_handle);

	esp_err_t ret = spi_device_polling_transmit(l_spi, &t);

	esp_pm_lock_release(l_esp_pm_lock_handle);
	assert( ret == ESP_OK );
	ESP_ERROR_CHECK(ret);
	//return *(uint32_t*)t.rx_data;
}

static void l_esp_timer_callback(void* arg) {
	l_leds_tick();
}
