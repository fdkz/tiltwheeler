// Elmo Trolla, 2021-01-03
// License: pick one - public domain / UNLICENSE (https://www.unlicense.org) / MIT (https://opensource.org/licenses/MIT).

// H-bridge is DRV8871 and input configuration is: both high - brake, both low - freewheeling.
// This means we can't use one pin for PWM and the other for direction, because then in one direction the motor
// would brake, and in the other direction freewheel during pwm off-time. We have to switch PWM pins on dir change.
//
// DRV8871 automatically inserts dead time, so we don't need to use the deadband feature of the PWM generator.
// 229 ns. ~4 MHz.

/*

change of motor direction:

|
|    ___________    ____________   _____________  ______________ ___________________________________________________________________________
| __|           |__|            |_|             ||              |               .
|
|   __________________________________________________________________________________________ _____________  ____________   ___________
| _|                                                                                          |             ||            |_|           |__|
|

*/

#include "motor.h"

#include "driver/mcpwm.h"


#define PWM_FREQ 20000

struct motor_t {
	mcpwm_unit_t mcpwm_num;
	mcpwm_timer_t timer_num;
	mcpwm_duty_type_t duty_type;
	u8 gpio_a;
	u8 gpio_b;
};

motor_t l_motors[2] = {
	{
		.mcpwm_num = MCPWM_UNIT_0,
		.timer_num = MCPWM_TIMER_0,
		.duty_type = MCPWM_DUTY_MODE_1, // mode 1: duty cycle proportional to pin low time
		.gpio_a = GPIO_NUM_0,
		.gpio_b = GPIO_NUM_4,
	},
	{
		.mcpwm_num = MCPWM_UNIT_0,
		.timer_num = MCPWM_TIMER_1,
		.duty_type = MCPWM_DUTY_MODE_1,
		.gpio_a = GPIO_NUM_16,
		.gpio_b = GPIO_NUM_17,
	}
};

bool l_motor_num_invalid(u32 motor) { return motor >= 2; }


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// public interface
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void motor_init() {

	// NB! this is the only function here that doesn't automatically use the data in l_motors struct.
	//     So, when adding motors or changing conf, make sure to keep everything in this motor_init in sync
	//     with the l_motors struct.

	mcpwm_gpio_init(l_motors[0].mcpwm_num, MCPWM0A, l_motors[0].gpio_a);
	mcpwm_gpio_init(l_motors[0].mcpwm_num, MCPWM0B, l_motors[0].gpio_b);

	mcpwm_gpio_init(l_motors[1].mcpwm_num, MCPWM1A, l_motors[1].gpio_a);
	mcpwm_gpio_init(l_motors[1].mcpwm_num, MCPWM1B, l_motors[1].gpio_b);

	mcpwm_config_t pwm_config;
	pwm_config.frequency = PWM_FREQ * 2; // For symmetric MCPWM, frequency is half of MCPWM frequency set
	pwm_config.cmpr_a = 0; // duty cycle of PWMxA = 0
	pwm_config.cmpr_b = 0; // duty cycle of PWMxb = 0
	pwm_config.counter_mode = MCPWM_UP_DOWN_COUNTER; // MCPWM_UP_COUNTER;
	pwm_config.duty_mode = l_motors[0].duty_type; // mode 1 - duty cycle proportional to pin low time

	mcpwm_init(l_motors[0].mcpwm_num, l_motors[0].timer_num, &pwm_config);
	mcpwm_init(l_motors[1].mcpwm_num, l_motors[1].timer_num, &pwm_config);

	//motor_brake(0);
	//motor_brake(1);
}

// both pins low
void motor_freewheel(u32 motor) {
	if (l_motor_num_invalid(motor)) return;
	mcpwm_set_signal_low(l_motors[motor].mcpwm_num, l_motors[motor].timer_num, MCPWM_OPR_A);
	mcpwm_set_signal_low(l_motors[motor].mcpwm_num, l_motors[motor].timer_num, MCPWM_OPR_B);
}

// both pins high
void motor_brake(u32 motor) {
	if (l_motor_num_invalid(motor)) return;
	mcpwm_set_signal_high(l_motors[motor].mcpwm_num, l_motors[motor].timer_num, MCPWM_OPR_A);
	mcpwm_set_signal_high(l_motors[motor].mcpwm_num, l_motors[motor].timer_num, MCPWM_OPR_B);
}

// duty -1..+1
void motor_set_duty(u32 motor, float duty) {
	if (l_motor_num_invalid(motor)) return;

	if (duty == 0) {
		motor_brake(motor);
		return;
	}

	duty = duty > 1 ? 1 : duty;
	duty = duty < -1 ? -1 : duty;

	duty *= 100; // convert from -1..+1 to percents. -100..+100 used by esp-idf

	mcpwm_generator_t pwm_gen = MCPWM_OPR_B;
	mcpwm_generator_t static_gen = MCPWM_OPR_A;

	// Select which pin should generate pwm and which should remain high/low depending on motor direction.
	if (duty < 0) {
		static_gen = MCPWM_OPR_A;
		pwm_gen = MCPWM_OPR_B;
		duty = -duty;
	}

	// Set the static pin to high or low depending on duty mode. MODE_1: duty cycle proportional to pin low time.
	if (l_motors[motor].duty_type == MCPWM_DUTY_MODE_1) {
		mcpwm_set_signal_high(l_motors[motor].mcpwm_num, l_motors[motor].timer_num, static_gen);
	} else {
		mcpwm_set_signal_low(l_motors[motor].mcpwm_num, l_motors[motor].timer_num, static_gen);
	}

	mcpwm_set_duty(l_motors[motor].mcpwm_num, l_motors[motor].timer_num, pwm_gen, duty);
	mcpwm_set_duty_type(l_motors[motor].mcpwm_num, l_motors[motor].timer_num, pwm_gen, l_motors[motor].duty_type);
}
