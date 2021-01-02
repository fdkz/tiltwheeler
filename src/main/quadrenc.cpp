// Elmo Trolla, 2021-01-02
// License: pick one - public domain / UNLICENSE (https://www.unlicense.org) / MIT (https://opensource.org/licenses/MIT).

#include "quadrenc.h"

#include <limits>
#include <cstring>
//#include <cstdio>
//#include <sys/socket.h> // getsockopt

using namespace std;

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "esp_attr.h"
#include "esp_log.h"

#include "stdints.h"


static const char* TAG = "quadrenc";

#define PCNT_H_LIM_VAL      800
#define PCNT_L_LIM_VAL     -500

static void IRAM_ATTR    l_pcnt_int(void *arg);
static pcnt_isr_handle_t l_pcnt_isr_handle;
static i32               l_count_base[QUADRENC_NUM_ENCODERS];


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// public interface
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// gpio-s can be thought of like this:
//     gpio1 - Pulse Input GPIO
//     gpio2 - Control GPIO HIGH=count up, LOW=count down
//
void quadrenc_init_encoder(u32 pcnt_unit_num, u8 gpio1, u8 gpio2) {
	pcnt_config_t pcnt_config = {
		.pulse_gpio_num = gpio1,
		.ctrl_gpio_num = gpio2,
		// What to do when control input is low or high?
		.lctrl_mode = PCNT_MODE_REVERSE, // Reverse counting direction if ctrl_gpio is low
		.hctrl_mode = PCNT_MODE_KEEP,    // Keep the primary counter mode if ctrl_gpio is high
		// What to do on the positive / negative edge of pulse_gpio?
		.pos_mode = PCNT_COUNT_INC,      // Count up on the positive edge of pulse_gpio
		.neg_mode = PCNT_COUNT_DIS,      // Do not change counter value on the negative edge of pulse_gpio
		// Set the maximum and minimum limit values to watch
		.counter_h_lim = PCNT_H_LIM_VAL, // TODO: std::numeric_limits<int16_t>max(): ?
		.counter_l_lim = PCNT_L_LIM_VAL,
		.unit = (pcnt_unit_t)pcnt_unit_num,
		.channel = PCNT_CHANNEL_0,
	};

	pcnt_unit_config(&pcnt_config);

	pcnt_config = {
		.pulse_gpio_num = gpio2,
		.ctrl_gpio_num = gpio1,
		.lctrl_mode = PCNT_MODE_KEEP,
		.hctrl_mode = PCNT_MODE_REVERSE,
		.pos_mode = PCNT_COUNT_INC,
		.neg_mode = PCNT_COUNT_DIS,
		.counter_h_lim = PCNT_H_LIM_VAL,
		.counter_l_lim = PCNT_L_LIM_VAL,
		.unit = (pcnt_unit_t)pcnt_unit_num,
		.channel = PCNT_CHANNEL_1,
	};

	pcnt_unit_config(&pcnt_config);

	pcnt_event_enable(pcnt_unit_num, PCNT_EVT_H_LIM);
	pcnt_event_enable(pcnt_unit_num, PCNT_EVT_L_LIM);

	pcnt_counter_pause(pcnt_unit_num);
	pcnt_counter_clear(pcnt_unit_num);

	if (!l_pcnt_isr_handle)
		pcnt_isr_register(l_pcnt_int, NULL, 0, &l_pcnt_isr_handle);

	pcnt_intr_enable(pcnt_unit_num);

	pcnt_counter_resume(pcnt_unit_num);
}

i32 quadrenc_get_count(u32 encoder_num) {
	assert(encoder_num < QUADRENC_NUM_ENCODERS);
	i16 count = 0;
	pcnt_get_counter_value(encoder_num, &count);
	return count + l_count_base[encoder_num];
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// private functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


static void IRAM_ATTR l_pcnt_int(void *arg) {
	uint32_t intr_status = PCNT.int_st.val;

	for (int i = 0; i < QUADRENC_NUM_ENCODERS; i++) {
		if (intr_status & (BIT(i))) {
			u32 status = PCNT.status_unit[i].val;
			PCNT.int_clr.val = BIT(i);

			if (status & PCNT_EVT_L_LIM)
				l_count_base[i] += PCNT_L_LIM_VAL;
			if (status & PCNT_EVT_H_LIM)
				l_count_base[i] += PCNT_H_LIM_VAL;
		}
	}
}
