// A-law audio companding algorithm.
// Used to represent a larger dynamic range in 8 bit PCM samples.

#pragma once

#include <common_macros.h>
#include <stdint.h>

uint8_t IRAM alaw_encode(int sample);
int IRAM alaw_decode(uint8_t sample);
