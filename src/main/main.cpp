// Elmo Trolla, 2021-01-02
// License: pick one - public domain / UNLICENSE (https://www.unlicense.org) / MIT (https://opensource.org/licenses/MIT).


#include <limits>
#include <cstring>
#include <cstdio>
#include <sys/socket.h> // getsockopt

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

#include "quadrenc.h"
#include "raw_imu.h"


extern "C" [[noreturn]] void app_main(void)
{
	quadrenc_init_encoder(0, 34, 35);
	quadrenc_init_encoder(1, 32, 33);

	raw_imu_init();

	while (1) {
		vTaskDelay(pdMS_TO_TICKS(100));
		i32 count0 = quadrenc_get_count(0);
		i32 count1 = quadrenc_get_count(1);

		raw_imu_blocking_read(0);
		raw_imu_readings_t* r = raw_imu_get_readings(0);

		float ax = raw_imu_acc_to_si(r->ax);
		float ay = raw_imu_acc_to_si(r->ay);
		float az = raw_imu_acc_to_si(r->az);
		float wx = raw_imu_gyro_to_si(r->wx);
		float wy = raw_imu_gyro_to_si(r->wy);
		float wz = raw_imu_gyro_to_si(r->wz);
		float temp = raw_imu_temperature_to_si(r->temperature);

		//printf("enc: %i %i ax %.3f wx %.3f temp %.3f\n", count0, count1);
		printf("enc: %i %i ax %.3f wx %.3f temp %.3f\n", count0, count1, ax, wx, temp);
	}
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



















//
// 2 punane MISO
// 0 must   CLCK
// 1 pruun  CS
// 3 oranz  MOSI
//
