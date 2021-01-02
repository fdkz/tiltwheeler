// Elmo Trolla, 2021-01-02
// License: pick one - public domain / UNLICENSE (https://www.unlicense.org) / MIT (https://opensource.org/licenses/MIT).

//
// usage:
//
//    raw_imu_init();
//    raw_imu_blocking_read(0);
//    raw_imu_readings_t* r = raw_imu_get_readings(0);
//
//    float ax = raw_imu_acc_to_si(r->ax);
//    float ay = raw_imu_acc_to_si(r->ay);
//    float az = raw_imu_acc_to_si(r->az);
//    float wx = raw_imu_gyro_to_si(r->wx);
//    float wy = raw_imu_gyro_to_si(r->wy);
//    float wz = raw_imu_gyro_to_si(r->wz);
//    float temp = raw_imu_temperature_to_si(r->temperature);
//

#pragma once

#include "stdints.h"


#pragma pack(push, 1)

	typedef struct {
		int16_t  wx, wy, wz;
		int16_t  ax, ay, az;
		int16_t  temperature;
	} raw_imu_readings_t;

#pragma pack(pop)


#define RAW_IMU_COUNT 1 // number of imus accessible through raw_imu_get_readings


void                raw_imu_init(); // start spi, init imu chips. takes ~0.3 seconds.
void                raw_imu_blocking_read(u32 imu_index); // Call this before raw_imu_get_readings().
raw_imu_readings_t* raw_imu_get_readings(u32 imu_index); // call after raw_imu_tick to get updated values. the given struct will be reused on next imu read.
void                raw_imu_enable_accelerometer(u32 imu_index, bool enable); // by default, only gyro and temperature is read, acc is always 0. enable here to get real values.

// TODO: *_to_si is to degrees or to radians?

inline float        raw_imu_acc_to_normal(i16 acc)      { return (f32)acc / 32768.f; }                   // scale to -1..+1
inline float        raw_imu_acc_to_si(i16 acc)          { return (f32)acc / 32768.f * 16.f * 9.80665f; } // to m/s2
inline float        raw_imu_gyro_to_si(i16 gyro)        { return (f32)gyro / 32.8f; }                    // 131.0 for +/- 250 degrees gyro sensitivity. 65.5 - 500, 32.8 - 1000, 16.4 - 2000

//inline float raw_imu_temperature_to_si(int16_t temp) { return temp / 340.f + 36.53f; } // this is for MPU-6000?
// Invensense ICM-20689 datasheet : TEMP_degC  = ((TEMP_OUT - RoomTemp_Offset)/Temp_Sensitivity) + 25degC
inline float        raw_imu_temperature_to_si(i16 temp) { return ((f32)temp - 25.f) / 326.8f + 25.f; }
