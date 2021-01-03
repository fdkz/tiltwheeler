// Elmo Trolla, 2021-01-03
// License: pick one - public domain / UNLICENSE (https://www.unlicense.org) / MIT (https://opensource.org/licenses/MIT).

#pragma once

#include "stdints.h"


void motor_init();
void motor_freewheel(u32 motor);
void motor_brake(u32 motor);
void motor_set_duty(u32 motor, float duty); // duty -1..+1, 0 - brake.
