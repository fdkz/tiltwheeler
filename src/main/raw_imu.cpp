// Elmo Trolla, 2021-01-02
// License: pick one - public domain / UNLICENSE (https://www.unlicense.org) / MIT (https://opensource.org/licenses/MIT).

//
// This module should work both for MPU-6000 and ICM-20689.
//
// MPU-6000 doc:
//
//	1. Data is delivered MSB first and LSB last
//	2. Data is latched on the rising edge of SCLK
//	3. Data should be transitioned on the falling edge of SCLK
//	4. The maximum frequency of SCLK is 1MHz
//	5. SPI read and write operations are completed in 16 or more clock cycles (two or more bytes).
//	   The first byte contains the SPI Address, and the following byte(s) contain(s) the SPI data.
//	   The first bit of the first byte contains the Read/Write bit and indicates the Read (1) or
//	   Write (0) operation. The following 7 bits contain the Register Address. In cases of
//	   multiple-byte Read/Writes, data is two or more bytes:
//

#include "raw_imu.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h" // vTaskDelay

#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

static const char* TAG = "imu";

#define L_PIN_MISO    GPIO_NUM_19
#define L_PIN_MOSI    GPIO_NUM_23
#define L_PIN_CLK     GPIO_NUM_18
#define L_PIN_IMU0_CS GPIO_NUM_5

#define RAW_IMU_COUNT 1

// two possibilities, but keep the larger:
//   * 14 bytes of data plus one byte for the first spi request byte.
//     ACCEL_XOUT, ACCEL_YOUT, ACCEL_ZOUT, TEMP_OUT, GYRO_XOUT, GYRO_YOUT, GYRO_ZOUT
//   * 8 bytes of data plus one byte for the first spi request byte. we don't use this 9-byte version, since we have dynamic buf size decisions.
//     TEMP_OUT, GYRO_XOUT, GYRO_YOUT, GYRO_ZOUT
#define IMU_BUF_SIZE 15

typedef struct {
	u8 rx_buf[IMU_BUF_SIZE];
	spi_device_handle_t spi_handle;
	bool acceleration_read_enabled; // in addition to gyros and temperature, also read accelerometer values.
	bool error;
} raw_imu_t;

static void      l_delay_ms(u32 ms);
static void      l_init_imus();
static int       l_imu_blocking_confregister_and_check(int imu_index, u8 registr, u8 value);
static void      l_imu_blocking_transmit(u32 imu_index, const u8* tx, u8* rx, int len);
static void      l_parse_imu(u32 imu_index);

static raw_imu_t l_imu_list[RAW_IMU_COUNT];
static bool      l_initialized;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// public interface
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


raw_imu_readings_t raw_imu_readings[RAW_IMU_COUNT];
u32                raw_imu_sample_counter;


void raw_imu_init() {
	if (l_initialized) return;

	// Currently support only one imu here. just use spi_bus_add_device with spics_io_num to use more imus.
	assert(RAW_IMU_COUNT == 1);

	esp_err_t ret;

	spi_bus_config_t    buscfg = {
		.mosi_io_num     = L_PIN_MOSI,
		.miso_io_num     = L_PIN_MISO,
		.sclk_io_num     = L_PIN_CLK,
		.quadwp_io_num   = -1,
		.quadhd_io_num   = -1,
		.max_transfer_sz = IMU_BUF_SIZE,
		.flags           = 0,
		.intr_flags      = 0,
	};

	ret = spi_bus_initialize(VSPI_HOST, &buscfg, 1);
	ESP_ERROR_CHECK(ret);

	// devcfg.mode:
	//     https://en.m.wikipedia.org/wiki/Serial_Peripheral_Interface#Clock_polarity_and_phase
	//     https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_master.html

	// Init first imu

	spi_device_interface_config_t devcfg = {};
	devcfg.clock_speed_hz = 1*1000*1000;
	devcfg.mode           = 0;
	devcfg.spics_io_num   = L_PIN_IMU0_CS;
	devcfg.queue_size     = 1;          // ? TODO: try 0!

	ret = spi_bus_add_device(VSPI_HOST, &devcfg, &l_imu_list[0].spi_handle);
	ESP_ERROR_CHECK(ret);

	l_init_imus();

	l_initialized = true;
}

// Blocking read data from all imus and fix axes and directions. Use raw_imu_get_readings to get pointers to the data.
void raw_imu_blocking_read(u32 imu_index) {
	// imu datasheet:
	//
	// The data within the gyroscope sensors' internal register set is always updated at the Sample Rate.
	// Meanwhile, the user-facing read register set duplicates the internal register set's data values
	// whenever the serial interface is idle. This guarantees that a burst read of sensor registers will read
	// measurements from the same sampling instant.

	if (imu_index >= RAW_IMU_COUNT) imu_index = RAW_IMU_COUNT - 1;

	//if (l_imu_list[imu_index].error)
	//	return;

	u8 txbuf[IMU_BUF_SIZE] = {};

	int bytes_to_read;
	if (l_imu_list[imu_index].acceleration_read_enabled) {
		txbuf[0] = 0x3B | 0x80; // start burst reading with ACCEL_XOUT_H. later data in tx has no effect.
		bytes_to_read = 15;
	} else {
		txbuf[0] = 0x41 | 0x80; // start burst reading with TEMP_OUT_H. later data in tx has no effect.
		bytes_to_read = 9;
	}

	l_imu_blocking_transmit(imu_index, txbuf, l_imu_list[imu_index].rx_buf, bytes_to_read);

	l_parse_imu(imu_index);
	raw_imu_sample_counter++;
}

raw_imu_readings_t* raw_imu_get_readings(u32 imu_index) {
	if (imu_index >= RAW_IMU_COUNT) imu_index = RAW_IMU_COUNT - 1;
	return &raw_imu_readings[imu_index];
}

void raw_imu_enable_accelerometer(u32 imu_index, bool enable) {
	if (imu_index >= RAW_IMU_COUNT) imu_index = RAW_IMU_COUNT - 1;
	l_imu_list[imu_index].acceleration_read_enabled = enable;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// private functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


static void l_delay_ms(u32 ms) {
	vTaskDelay(pdMS_TO_TICKS(ms));
}

// Reset IMUs, configure clock source and gyro/acceleration sensitivity. takes a little over 0.2 seconds.
static void l_init_imus() {
	u8 rx[4], tx[4]; // logical buf sizes could be 2, but esp32 can read by 4-byte segments if DMA is used.
	int err;

	// This function should work both for MPU-6000 and ICM-20689.

	// From spec (this is not implemented, and not necessary for ICM-20689:
	// When using SPI interface, user should use DEVICE_RESET (register 107) as well as SIGNAL_PATH_RESET
	// (register 104) to ensure the reset is performed properly. The sequence used should be:
	//   1. Set DEVICE_RESET = 1 (register PWR_MGMT_1)
	//   2. Wait 100ms
	//   3. Set GYRO_RESET = ACCEL_RESET = TEMP_RESET = 1 (register SIGNAL_PATH_RESET)
	//   4. Wait 100ms

	// wait for imu wakeup time.
	l_delay_ms(100);

	// step 1: reset all working imus in parallel. needed because while developing, the cpu gets often reset while
	//         the imu continues to work with old settings.

	for (int imu_index = 0; imu_index < RAW_IMU_COUNT; imu_index++) {
		if (l_imu_list[imu_index].error)
			continue;
		tx[0] = 0x6B; // 0x6B - PWR_MGMT_1
		tx[1] = 0x80; // 0x80 - DEVICE_RESET bit
		l_imu_blocking_transmit(imu_index, tx, rx, 2);
	}

	l_delay_ms(100);

	// step 2: turn off i2c

	// From datasheet: The MPU-6000 and MPU-6050 are identical, except that the MPU-6050 supports the I2C serial
	// interface only, and has a separate VLOGIC reference pin.
	//
	// we have MPU-6000 or ICM-20689, and both have i2c and spi interfaces. disable i2c:
	for (int imu_index = 0; imu_index < RAW_IMU_COUNT; imu_index++) {
		tx[0] = 0x6A; // USER_CTRL
		tx[1] = 16; // raise the I2C_IF_DIS bit
		l_imu_blocking_transmit(imu_index, tx, rx, 2);
	}

	// step 3: find out which imus answer at all

	// even if the other imus listen on the spi bus in i2c mode, the WHO_AM_I packet won't confuse them because
	// i2c addresses can be only 01101000 and 01101001, but WHO_AM_I 0x75 is 01110101.
	for (int imu_index = 0; imu_index < RAW_IMU_COUNT; imu_index++) {
		tx[0] = 0x75 | 0x80; // 0x75 - WHO_AM_I. expecting 0x68 for answer.
		tx[1] = 0x00;
		l_imu_blocking_transmit(imu_index, tx, rx, 2);
		if (rx[1] != 0x68 && rx[1] != 0x98) { // 0x68 for MPU-60x0, 0x98 for ICM-20689
			l_imu_list[imu_index].error = true;
			ESP_LOGE(TAG, "imu %i no answer", imu_index);
		}
	}

	// step 4: change imu clock source and wake imu from sleep

	for (int imu_index = 0; imu_index < RAW_IMU_COUNT; imu_index++) {
		if (l_imu_list[imu_index].error)
			continue;

		// select the x axis gyro as a reference clock and wakeup

		// from the MP-6000 register map doc:
		// Upon power up, the MPU-60X0 clock source defaults to the internal oscillator. However, it is highly
		// recommended that the device be configured to use one of the gyroscopes (or an external clock source)
		// as the clock reference for improved stability.

		// 0x6B - PWR_MGMT_1
		// 0x01 - for mpu-6000  : PLL with X axis gyroscope reference
		//        for icm-20689 : Auto selects the best available clock source � PLL if ready, else use the
		//                        Internal oscillator
		err = l_imu_blocking_confregister_and_check(imu_index, 0x6B, 0x01);
		if (err) {
			l_imu_list[imu_index].error = true;
			ESP_LOGE(TAG, "imu %i conf fail a", imu_index);
			continue;
		}
	}

	// wait for 10 milliseconds (5 is in spec) after waking imu. not necessary for MPU-60x0.
	l_delay_ms(10);

	// step 5: decrease gyro and accelerometer sensitivity

	for (int imu_index = 0; imu_index < RAW_IMU_COUNT; imu_index++) {
		if (l_imu_list[imu_index].error)
			continue;

		// raise gyro range from +/- 250 degrees per second to 1000.

		// 0x1B - GYRO_CONFIG
		// 0x10 - +/- 1000 deg/s
		err = l_imu_blocking_confregister_and_check(imu_index, 0x1B, 0x10);
		if (err) {
			l_imu_list[imu_index].error = true;
			ESP_LOGE(TAG, "imu %i conf fail b", imu_index);
			continue;
		}

		// lower accelerometer sensitivity from +/- 2g to +/- 16g

		// 0x1C - ACCEL_CONFIG
		// 0x18 - AFS_SEL 3, +/- 16g.
		// 0x10 - AFS_SEL 2, +/- 8g.
		// 0x08 - AFS_SEL 1, +/- 4g.
		// 0x00 - AFS_SEL 0, +/- 2g.
		err = l_imu_blocking_confregister_and_check(imu_index, 0x1C, 0x18);
		if (err) {
			l_imu_list[imu_index].error = true;
			ESP_LOGE(TAG, "imu %i conf fail c", imu_index);
			continue;
		}
	}
}

// Configure imu register and read back the set value. return 0 if values match, 1 on mismatch.
// Assumes tx contains exactly 2 entries.
static int l_imu_blocking_confregister_and_check(int imu_index, u8 registr, u8 value) {
	// buffers have size 4 because esp32 can read in 4-byte units if DMA is used.
	u8 rx[4];

	// write
	u8 tx[4] = {registr, value, 0, 0};
	l_imu_blocking_transmit(imu_index, tx, rx, 2);

	// read
	u8 tx2[4] = {(u8)(registr | 0x80), 0, 0, 0};
	l_imu_blocking_transmit(imu_index, tx2, rx, 2);

	if (rx[1] != value) return 1;
	return 0;
}

static void l_imu_blocking_transmit(u32 imu_index, const u8* tx, u8* rx, int len) {
	spi_transaction_t t = {};
	t.length    = 8 * len; // len in bits
	t.tx_buffer = tx;
	t.rx_buffer = rx;

	esp_err_t ret = spi_device_polling_transmit(l_imu_list[imu_index].spi_handle, &t);
	assert( ret == ESP_OK );
	ESP_ERROR_CHECK(ret);
}

// Parse the binary blob from imu to raw_imu_readings[imu_index] struct.
static void l_parse_imu(u32 imu_index) {
	if (imu_index >= RAW_IMU_COUNT) return;

	i16 temperature, wx, wy, wz;
	u8* rx = l_imu_list[imu_index].rx_buf;
	raw_imu_readings_t* r = &raw_imu_readings[imu_index];
	i16 ax, ay, az;

	if (l_imu_list[imu_index].acceleration_read_enabled) {
		((u8*)&ax)[0] = rx[2]; ((u8*)&ax)[1] = rx[1];
		((u8*)&ay)[0] = rx[4]; ((u8*)&ay)[1] = rx[3];
		((u8*)&az)[0] = rx[6]; ((u8*)&az)[1] = rx[5];
		rx += 6;
	} else {
		ax = ay = az = 0;
	}

	((u8*)&temperature)[0] = rx[2]; ((u8*)&temperature)[1] = rx[1];
	((u8*)&wx)[0] = rx[4]; ((u8*)&wx)[1] = rx[3];
	((u8*)&wy)[0] = rx[6]; ((u8*)&wy)[1] = rx[5];
	((u8*)&wz)[0] = rx[8]; ((u8*)&wz)[1] = rx[7];

	r->temperature = temperature;
	
	// fix directions/axis/orientation:
	
	if (imu_index == 0) {
		r->ax =  ax; r->ay = -ay; r->az =  az;
		r->wx =  wx; r->wy = -wy; r->wz =  wz;
	}
	
	//static int c = 0;
	//c++;
	//if (c % 1501 == 0) {
	//	// 'rawimu0 |w    24    18   176 |a     9    94    42 |t  2575'
	//	ESP_LOGI(TAG, "imu%i |w %5i %5i %5i |a %5i %5i %5i |t %5i", imu_index,
	//			(int)(r->wx), (int)(r->wy), (int)(r->wz),
	//			(int)(r->ax), (int)(r->ay), (int)(r->az),
	//			(int)(r->temperature));
	//}
}

