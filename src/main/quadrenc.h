// Elmo Trolla, 2021-01-02
// License: pick one - public domain / UNLICENSE (https://www.unlicense.org) / MIT (https://opensource.org/licenses/MIT).

//
// Quadrature encoder decoder, 4x mode (both transitions of both signals of the encoder are counted).
// https://www.esp32.com/viewtopic.php?t=4983
//

#pragma once

#include "stdints.h"


// Change this to the num of encoders you'll use.
#define QUADRENC_NUM_ENCODERS 2 // assert QUADRENC_NUM_ENCODERS <= PCNT_UNIT_MAX


void quadrenc_init_encoder(u32 encoder_num, u8 gpio1, u8 gpio2);
i32 quadrenc_get_count(u32 encoder_num);

