// Elmo Trolla, 2021-01-02
// License: pick one - public domain / UNLICENSE (https://www.unlicense.org) / MIT (https://opensource.org/licenses/MIT).

//
// ps4 controller: https://github.com/aed3/PS4-esp32
// TODO: pairing could be simplified? I used a windows program. but which one?
//       and got the 6-number code from the controller and wrote it below?
//
// TODO: idf.py menuconfig
//       Component config -> Bluetooth [*]
//                        -> Bluetooth controller -> Bluetoot controller mode : BR/EDR Only, or, Bluetooth Dual Mode
//                                                -> The cpu core which bluetooth controller run : Core 1 (APP CPU)
//                        -> Bluedroid Options -> Classic Bluetooth [*]
//                                                                      -> SPP [*]


// the hell! had to erase the flash, because controller disconnected without it! in 5 seconds..
// idf.py -p COM12 erase_flash



/*

PID loops:

    position (not used for manual control)
    wanted robot speed
    wanted angle (if speed is reached, angle should be 0)
    current motor power
    current wheel speed

wanted_angle = clamp( (current_wheel_speed - wanted_robot_speed) * const, 5)



*/


//#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/timer.h"
#include "esp_attr.h"

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "sdkconfig.h"

#include "stdints.h"
#include "quadrenc.h"
#include "raw_imu.h"
#include "motor.h"
#include "sys_globals.h"

#include "PS4Controller.h"


static const char* TAG = "main";

#define MOTOR_TASK_PRIORITY 23 // higher num, higher priority
#define MOTOR_TASK_CPU_CORE 1  // 0, 1, tskNO_AFFINITY

#define MOTOR_DRIVE_FREQ    500
#define WHEEL_DIAMETER      0.07f
#define WHEEL_ENCODER_CPR   768 // counts per wheel revolution

PS4Controller            PS4;
TaskHandle_t             l_motor_task_handle;

[[noreturn]] static void l_motor_task(void *pvParameters);
static bool IRAM_ATTR    l_motor_timer_isr_callback(void *args);


static bool IRAM_ATTR l_motor_timer_isr_callback(void *args) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR(l_motor_task_handle, &xHigherPriorityTaskWoken );
	//if (xHigherPriorityTaskWoken == pdTRUE)
	//	portYIELD_FROM_ISR();

	return xHigherPriorityTaskWoken == pdTRUE; // return whether we need to yield at the end of ISR
}



#define ENCODER_VELOCITY_UPDATE_RATE 50

// https://medium.com/husarion-blog/fresh-look-at-self-balancing-robot-algorithm-d50d41711d58


static float wxaccum;
//static float angle_x;
static float gyro_x_drift;


float get_filtered_angle_x(float wx, float dt)
{
	wx -= gyro_x_drift;
	static float angle;
	angle += wx * dt;

	// move angle slowly to zero..
//	wxaccum += (wx - wxaccum) * 0.001f;
//	aangle_x -= wxaccum * dt;

	return angle;
}

static float aangle_x;

float get_filtered_acc_angle_x(float ay, float az, float dt)
{
	float angle = atan2f(az, ay) * 180.f / (float)M_PI;
	aangle_x = angle;
	return angle;
}


template<uint32_t numelems>
struct Averager {
	float    buf[numelems];
	uint32_t i; // current index
	float    running_accum;

	float get(float new_value) {
		running_accum -= buf[i];
		running_accum += new_value;
		buf[i] = new_value;
		i++; if (i >= numelems) i = 0;
		return running_accum / numelems;
	}
};


float g_wanted_speed; // NOT angular velocity
float g_turn_ofs;

// 500 Hz
static void l_motor_tick() {

	static u32 c;

	if (c % 500 == 0) {
		//ESP_LOGI(TAG, "tick %i", c);
	}

	c++;

	float dt = 1.f / MOTOR_DRIVE_FREQ;

	#define F2I(v) (int((v) < 0 ? (v)*100.f-0.5f : (v)*100+0.5f))

	// read imu

	raw_imu_blocking_read(0);
	raw_imu_readings_t* r = raw_imu_get_readings(0);

	float ax = raw_imu_acc_to_si(r->ax);
	float ay = raw_imu_acc_to_si(r->ay);
	float az = raw_imu_acc_to_si(r->az);
	float wx = raw_imu_gyro_to_si(r->wx);
	float wy = raw_imu_gyro_to_si(r->wy);
	float wz = raw_imu_gyro_to_si(r->wz);
	float temp = raw_imu_temperature_to_si(r->temperature);

	if (c % 100 == 0) {
		//ESP_LOGI(TAG, "------------------enc: ax %.3f wx %.3f temp %.3f", ax, wx, temp);
	}

	static float prev_wheel_pos_left;
	static float prev_wheel_pos_right;

	float wheel_pos_left  = -(float)quadrenc_get_count(1) / WHEEL_ENCODER_CPR * 360.f;
	float wheel_pos_right = -(float)quadrenc_get_count(0) / WHEEL_ENCODER_CPR * 360.f; // this encoder is not working

	float wheel_angvel_left  = g_angledifff(prev_wheel_pos_left, wheel_pos_left) / dt;
	float wheel_angvel_right = g_angledifff(prev_wheel_pos_right, wheel_pos_right) / dt;

	prev_wheel_pos_left = wheel_pos_left;
	prev_wheel_pos_right = wheel_pos_right;

	float robospeed;
	float angleofs;

	if (c % 100 == 0) { // 5 Hz
		//loginfo("ax %05i ay %05i az %05i   wx %05i wy %05i wz %05i   temp %05i  cnt %i", F2I(ax), F2I(ay), F2I(az), F2I(wx), F2I(wy), F2I(wz), F2I(temp), raw_imu_sample_counter);
	}

	// main logic

	const int ST_CALIBRATING_GYRO = 1;
	const int ST_RUNNING = 2;
	static int state = ST_CALIBRATING_GYRO;

	if (state == ST_CALIBRATING_GYRO) {

		static float wx_accum;
		if (c < 1000) { // 2 seconds
			wx_accum += wx;
		} else {
			gyro_x_drift = wx_accum / 1000.f;
			ESP_LOGI(TAG, "gyro calibrated. drift x %05i", F2I(gyro_x_drift));
			state = ST_RUNNING;
		}

	} else if (state == ST_RUNNING) {

		// calculate pid..

		//	wheel_angvel_right = wheel_angvel_left; // remove when the other encoder also works
		//wheel_angvel_left = wheel_angvel_right; // remove when the other encoder also works

		float wanted_angle_x_ofs = 0.f;

		static Averager<50> robot_speed_averager;
		//float robot_speed = M_PI * WHEEL_DIAMETER * (wheel_angvel_left + wheel_angvel_right) / 2.f / 360.f;
		float robot_speed = robot_speed_averager.get(M_PI * WHEEL_DIAMETER * (wheel_angvel_left + wheel_angvel_right) / 2.f / 360.f);
		//float robot_speed = robot_speed_averager.get(M_PI * WHEEL_DIAMETER * (wheel_angvel_left) / 1.f / 360.f);
		robospeed = robot_speed;


		{
			// pid loop above the motor power loop.
			// this pid loop tries to hold wheel speed zero by changing the wanted robot angle.
			// wanted_angle_x_ofs.

			float kp = 12.f; // 0.1 m/s gives 1 deg
			float kd = 0.0001f;
			float ki = 60.f;

			float err = g_wanted_speed - robot_speed; // expected output - actual output

			static float prev_err;
			static float integral_err;

			integral_err = integral_err + err;
//if (c % 500 == 0)
//ESP_LOGI(TAG, "integral_err1 %7.2f lo %7.2f hi %7.2f", integral_err, -150.f / (ki * dt), 150.f / (ki * dt));

			if (ki != 0.f)
				integral_err = g_clampf(integral_err, -150.f / (ki * dt), 150.f / (ki * dt)); // limit so integral wont give larger than amplitude of +/-3.
//if (c % 500 == 0)
//ESP_LOGI(TAG, "integral_err2 %7.2f", integral_err);

			float deriv_err = err - prev_err;

			if (fabsf(kp*err) < 0.1f) kp *= 0.2f;

			wanted_angle_x_ofs = kp * err + (ki * integral_err * dt) + (kd * deriv_err / dt);

			//float sgn = wanted_angle_x_ofs >= 0.f ? 1.f : -1.f;
			//wanted_angle_x_ofs = wanted_angle_x_ofs * wanted_angle_x_ofs;
			//wanted_angle_x_ofs *= sgn;

			wanted_angle_x_ofs = -g_clampf(wanted_angle_x_ofs, -20, +20);

			angleofs = wanted_angle_x_ofs;

			prev_err = err;
		}

		{
			// this is not drift-corrected. so it's just used for quick corrections and drift is eliminated
			// on upper layers with speed feedback from motor encoders. don't know what to call this.. angle_x_integral?
			//float angle_x = get_filtered_angle_x(wx, dt);

			static float _angle_x;
			wx -= gyro_x_drift;
			float angle_acc_x = atan2f(az, ay) * 180.f / (float)M_PI + 0.f;

			{
				// excellent article about complementary filters:
				//     http://d1.amobbs.com/bbs_upload782111/files_44/ourdev_665531S2JZG6.pdf
				//     The Balance Filter
				//     A Simple Solution for Integrating Accelerometer and Gyroscope Measurements for
				//     a Balancing Platform
				//     Shane Colton <scolton@mit.edu>

				const float tau = 1.5f; // amount of time we want the gyro measurement to dominate

				float a = tau / (tau + dt); // amount of gyro we need in relation to (1-a) of accel angle.
				_angle_x = a * (_angle_x + wx * dt) + (1.f - a) * (angle_acc_x);

				// a=tau / (tau + loop time)
				// newAngle = angle measured with atan2 using the accelerometer
				// newRate = angle measured using the gyro
				// looptime = loop time in millis()

//				float tau=0.075;
//				float a=0.0;
//
//				float Complementary(float newAngle, float newRate,int looptime) {
//					float dtC = float(looptime)/1000.0;
//					a=tau/(tau+dtC);
//					x_angleC= a* (x_angleC + newRate * dtC) + (1-a) * (newAngle);
//					return x_angleC;
//				}
			}


			//_angle_x += wx * dt;
			//float angle_acc_x = get_filtered_acc_angle_x(ay, az, dt);

			// move angle_x towards angle_acc_c
			//_angle_x = _angle_x + g_angledifff(_angle_x, angle_acc_x) * 0.001;

			static Averager<2> angle_averager;
			float angle_x = angle_averager.get(_angle_x + wanted_angle_x_ofs);

			//float angle_x = _angle_x + wanted_angle_x_ofs;

			// pid loop

			//float kp = 1. / 5.f; // 5 degrees gives full power
			float kp = 1. / 1.f; // 5 degrees gives full power
			float kd = 0.001;
			float ki = 0 * 1;

			float err = 0 - angle_x; // expected output - actual output

			static float prev_err;
			static float integral_err;

			integral_err = integral_err + err;
			//if (ki != 0.f)
			//	integral_err = g_clampf(integral_err, -1.f / (ki * dt), 1.f / (ki * dt)); // limit so integral wont give larger than amplitude of 1.

			float deriv_err = err - prev_err;

			// motor_power -1..+1
			float motor_power = kp * err + (ki * integral_err * dt) + (kd * deriv_err / dt);
			motor_power = g_clampf(motor_power, -0.99, +0.99);

			prev_err = err;


			//motor_power = -motor_power;

			motor_set_duty(MOTOR_LEFT,  motor_power + g_turn_ofs);
			motor_set_duty(MOTOR_RIGHT, motor_power - g_turn_ofs);


			if (c % 100 == 0) { // 5 Hz
				ESP_LOGI(TAG, "angle_acc_x %7.2f _angle_x %7.2f angle_x %7.2f wanted_angle_x_ofs %7.2f wx %7.2f az %7.2f ay %7.2f  wxaccum %7.2f  wanted_speed %7.2f robot_speed %7.2f integral_err %7.2f motor_power %7.2f", angle_acc_x, _angle_x, angle_x, wanted_angle_x_ofs, wx, az, ay, wxaccum, g_wanted_speed, robot_speed, integral_err, motor_power);
				//ESP_LOGI(TAG, "angle_x %05i  aangle %05i  wxaccum %05i  motor_power %05i", F2I(angle_x), F2I(aangle_x), F2I(wxaccum), F2I(motor_power));
			}
		}
	}

	// report sensor values to uart
//
//	if (c % 2 == 0) { // 250 Hz
//		p_tiltw_sensorval_all_t p;
//		p.fill_header();
//		p.imu_wx = r->wx;
//		p.imu_ay = r->ay;
//		p.imu_az = r->az;
//		p.imu_temp = r->temperature;
//		p.enc_left = quadrenc_get_count(1);
//		p.enc_right = quadrenc_get_count(0);
//		p.enc_left_speed  = g_clampf(robospeed * 100.f, -32768, 32768);
//		p.enc_right_speed = g_clampf(angleofs * 100.f, -32768, 32768);
//		//p.enc_left_speed  = g_clampf(wheel_angvel_left, -32768, 32768);
//		//p.enc_right_speed = g_clampf(wheel_angvel_right, -32768, 32768);
//		//p.enc_right_speed = g_clampf(encoder_enc1_get_angular_velocity(), -32768, 32768);
//		p.sample_id = raw_imu_sample_counter;
//		serialcomm_send_frame_packet((uint8_t*)&p, sizeof(p));
//	}

//	if (c % 5 == 0) { // 100 Hz
//		float a = c / 500.f * 360.f * 0.5f; // 0.5 Hz for full circle
//		float m1 = sin(a * M_PI / 180.f);
//		float m2 = sin(a * 2 * M_PI / 180.f);
//		if (c % 100 == 0) { // 5 Hz
//			//loginfo("angle %05i x %05i  enc0 %05i enc0err %i   enc1 %05i enc1err %i", int(a * 10.f + 0.5f), int(m1 * 100.f + 0.5f), encoder_enc0_pos(), QEIErrorGet(QEI0_BASE), encoder_enc1_pos(), QEIErrorGet(QEI1_BASE));
//		}
//		motor_set_speed(MOTOR_LEFT, m1);
//		motor_set_speed(MOTOR_RIGHT, m2);
//	}

}



[[noreturn]] static void l_motor_task(void *pvParameters) {

	quadrenc_init_encoder(0, 34, 35);
	quadrenc_init_encoder(1, 32, 33);

	raw_imu_init();
	motor_init();

	raw_imu_enable_accelerometer(0, true);

	// start the timer that will wake up this motor drive task at MOTOR_DRIVE_FREQ

	ESP_LOGI(TAG, "motor_task started, imu and motor module initialized.");

	{
		timer_group_t group = TIMER_GROUP_0;
		timer_idx_t timer = TIMER_0;

		const int TIMER_DIVIDER = 16; //  Hardware timer clock divider
		const int TIMER_SCALE   = TIMER_BASE_CLK / TIMER_DIVIDER; // convert counter value to seconds

		timer_config_t config = {
			.alarm_en    = TIMER_ALARM_EN,
			.counter_en  = TIMER_PAUSE,
			.intr_type   = TIMER_INTR_LEVEL,
			.counter_dir = TIMER_COUNT_UP,
			.auto_reload = TIMER_AUTORELOAD_EN,
			.divider     = TIMER_DIVIDER,
		}; // default clock source is APB

		timer_init(group, timer, &config);

		// Timer's counter will initially start from value below. Also, if auto_reload is set, this value
		// will be automatically reload on alarm.
		timer_set_counter_value(group, timer, 0);

		// Configure the alarm value and the interrupt on alarm.
		timer_set_alarm_value(group, timer, TIMER_SCALE / MOTOR_DRIVE_FREQ);
		timer_enable_intr(group, timer);

		timer_isr_callback_add(group, timer, l_motor_timer_isr_callback, nullptr, 0);
		timer_start(group, timer);
	}

	int c = 0;
	while (1) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
//		c++;
//		if (c % 500 == 0) {
//			ESP_LOGI(TAG, "tick %i", c);
//		}
		l_motor_tick();
	}
}


extern "C" [[noreturn]] void app_main(void) {

	// start bluetooth and ps4 stuff
	{
		esp_err_t ret = nvs_flash_init();
		if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
			ESP_ERROR_CHECK(nvs_flash_erase());
			ret = nvs_flash_init();
		}
		ESP_ERROR_CHECK( ret );

		const char* mac = "\x50\xeb\x71\xf8\x9a\x2a";
		PS4.begin(mac);
	}


	xTaskCreatePinnedToCore(l_motor_task, "motor", 4096, nullptr, MOTOR_TASK_PRIORITY, &l_motor_task_handle, MOTOR_TASK_CPU_CORE);

	int c = 0;
	while (1) {
		ESP_LOGI(TAG, "%i conn %i sticks - lx % 4i ly % 4i rx % 4i ry % 4i bat %i", PS4.isConnected(), c, PS4.data.analog.stick.lx, PS4.data.analog.stick.ly, PS4.data.analog.stick.rx, PS4.data.analog.stick.ry, PS4.data.status.battery);
		g_wanted_speed = ((float)PS4.data.analog.stick.ry / 128) * 0.1;
		g_turn_ofs = ((float)PS4.data.analog.stick.rx / 128) * 0.3;
		vTaskDelay(2);
	}

//	while (1) {
//		vTaskDelay(pdMS_TO_TICKS(10));
//		//vTaskDelay(pdMS_TO_TICKS(100));
//		i32 count0 = quadrenc_get_count(0);
//		i32 count1 = quadrenc_get_count(1);
//
//		raw_imu_blocking_read(0);
//		raw_imu_readings_t* r = raw_imu_get_readings(0);
//
//		float ax = raw_imu_acc_to_si(r->ax);
//		float ay = raw_imu_acc_to_si(r->ay);
//		float az = raw_imu_acc_to_si(r->az);
//		float wx = raw_imu_gyro_to_si(r->wx);
//		float wy = raw_imu_gyro_to_si(r->wy);
//		float wz = raw_imu_gyro_to_si(r->wz);
//		float temp = raw_imu_temperature_to_si(r->temperature);
//
//		static float t;
//		t += 0.01;
//		//float duty = sin(t * 90.f * 3.14f / 180.f);
//		float duty = sin(t * 90.f * 3.14f / 180.f);
//
//		motor_set_duty(0, duty);
//		motor_set_duty(1, duty);
//
//		//printf("enc: %i %i ax %.3f wx %.3f temp %.3f\n", count0, count1);
//		ESP_LOGI(TAG, "enc: %i %i ax %.3f wx %.3f temp %.3f duty %.3f\n", count0, count1, ax, wx, temp, duty);
//	}
}


// punane - 3V3
// sinine - GND
// pruun - A3 - CS
// oranz - A5 - MOSI
// kollane - A2 - SCLK
// roheline - A4 - MISO

// PA2, PA4, PA5 - SCLK0, MISO0, MOSI0


// gpio esp 18 - clk,  imu kollane,  saleae 0 must
//      esp 23 - mosi, imu oranz,    saleae 3 oranz
//      esp 19 - miso, imu roheline, saleae 2 punane
//      esp  5 - cs,   imu pruun,    saleae 1 pruun



// motors:
//
//   A6 roheline
//   A7 kollane
//
//   D0 roheline
//   D1 sinine

// 17, 16
// 4, 0




//
// 2 punane MISO
// 0 must   CLCK
// 1 pruun  CS
// 3 oranz  MOSI
//


#if 0


extern "C" void app_main(void)
{
	printf("Hello world!\n");

	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK( ret );


	// Print chip information
	esp_chip_info_t chip_info;
	esp_chip_info(&chip_info);
	printf("This is %s chip with %d CPU cores, WiFi%s%s, ",
			CONFIG_IDF_TARGET,
			chip_info.cores,
			(chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
			(chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

	printf("silicon revision %d, ", chip_info.revision);

	printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
			(chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

	printf("Free heap: %d\n", esp_get_free_heap_size());

	ESP_LOGI(TAG, "1111");

	//esp_bd_addr_t mac = {0x30, 0x30, 0x30, 0x30, 0x30, 0x30};
	//esp_bd_addr_t mac = {0x50, 0xeb, 0x71, 0xf8, 0x9a, 0x2a};
	const char* mac = "\x50\xeb\x71\xf8\x9a\x2a";
	PS4.begin(mac);

	ESP_LOGI(TAG, "2222");

	uint32_t c = 0;
	while (1) {

		// Below has all accessible outputs from the controller
		if(0 && PS4.isConnected()) {
			if ( PS4.data.button.up ) ESP_LOGI(TAG, "Up Button");
			if ( PS4.data.button.down ) ESP_LOGI(TAG, "Down Button");
			if ( PS4.data.button.left ) ESP_LOGI(TAG, "Left Button");
			if ( PS4.data.button.right ) ESP_LOGI(TAG, "Right Button");

			if ( PS4.data.button.upright ) ESP_LOGI(TAG, "Up Right");
			if ( PS4.data.button.upleft ) ESP_LOGI(TAG, "Up Left");
			if ( PS4.data.button.downleft ) ESP_LOGI(TAG, "Down Left");
			if ( PS4.data.button.downright ) ESP_LOGI(TAG, "Down Right");

			if ( PS4.data.button.triangle ) ESP_LOGI(TAG, "Triangle Button");
			if ( PS4.data.button.circle ) ESP_LOGI(TAG, "Circle Button");
			if ( PS4.data.button.cross ) ESP_LOGI(TAG, "Cross Button");
			if ( PS4.data.button.square ) ESP_LOGI(TAG, "Square Button");

			if ( PS4.data.button.l1 ) ESP_LOGI(TAG, "l1 Button");
			if ( PS4.data.button.r1 ) ESP_LOGI(TAG, "r1 Button");

			if ( PS4.data.button.l3 ) ESP_LOGI(TAG, "l3 Button");
			if ( PS4.data.button.r3 ) ESP_LOGI(TAG, "r3 Button");

			if ( PS4.data.button.share ) ESP_LOGI(TAG, "Share Button");
			if ( PS4.data.button.options ) ESP_LOGI(TAG, "Options Button");

			if ( PS4.data.button.ps ) printf("PS Button");
			if ( PS4.data.button.touchpad ) printf("Touch Pad Button");

			if ( PS4.data.button.l2 ) ESP_LOGI(TAG, "l2 button at %i", PS4.data.analog.button.l2);
			if ( PS4.data.button.r2 ) ESP_LOGI(TAG, "r2 button at %i", PS4.data.analog.button.r2);

			// if ( PS4.event.analog_move.stick.lx ) ESP_LOGI(TAG, "Left Stick x at %i", PS4.data.analog.stick.lx);
			// if ( PS4.event.analog_move.stick.ly ) ESP_LOGI(TAG, "Left Stick y at %i", PS4.data.analog.stick.ly);
//			if ( PS4.event.analog_move.stick.rx ) ESP_LOGI(TAG, "Right Stick x at %i", PS4.data.analog.stick.rx);
//			if ( PS4.event.analog_move.stick.ry ) ESP_LOGI(TAG, "Right Stick y at %i", PS4.data.analog.stick.ry);

//			ESP_LOGI(TAG, "sticks - lx % 4i ly % 4i rx % 4i ry % 4i bat %i", PS4.data.analog.stick.lx, PS4.data.analog.stick.ly, PS4.data.analog.stick.rx, PS4.data.analog.stick.ry, PS4.data.status.battery);

			// if ( PS4.event.analog_move.stick.lx ) ESP_LOGI(TAG, "Left Stick x at %i", PS4.data.analog.stick.lx);
			// if ( PS4.event.analog_move.stick.ly ) ESP_LOGI(TAG, "Left Stick y at %i", PS4.data.analog.stick.ly);
//			if ( PS4.event.analog_move.stick.rx ) ESP_LOGI(TAG, "Right Stick x at %i", PS4.data.analog.stick.rx);
//			if ( PS4.event.analog_move.stick.ry ) ESP_LOGI(TAG, "Right Stick y at %i", PS4.data.analog.stick.ry);

			if (PS4.data.status.charging) ESP_LOGI(TAG, "The controller is charging");
			if (PS4.data.status.audio) ESP_LOGI(TAG, "The controller has headphones attached");
			if (PS4.data.status.mic) ESP_LOGI(TAG, "The controller has a mic attached");

//			ESP_LOGI(TAG, "Battey Percent : %i", PS4.data.status.battery);
			//ESP_LOGI(TAG, "");

//			if (PS4.data.status.charging)
//				printf("The controller is charging");
//			if (PS4.data.status.audio)
//				printf("The controller has headphones attached");
//			if (PS4.data.status.mic)
//				printf("The controller has a mic attached");
//
//			Serial.print("Battey Percent : ");
//			printf(PS4.data.status.battery, DEC);
		}

		c++;
//		ESP_LOGI(TAG, "c %i", c);
//		vTaskDelay(100 / portTICK_PERIOD_MS);
		ESP_LOGI(TAG, "%i conn %i sticks - lx % 4i ly % 4i rx % 4i ry % 4i bat %i", PS4.isConnected(), c, PS4.data.analog.stick.lx, PS4.data.analog.stick.ly, PS4.data.analog.stick.rx, PS4.data.analog.stick.ry, PS4.data.status.battery);
		//taskYIELD();
		vTaskDelay(1);
	}


	for (int i = 10; i >= 0; i--) {
		printf("Restarting in %d seconds...\n", i);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
	printf("Restarting now.\n");
	fflush(stdout);
	esp_restart();
}

#endif