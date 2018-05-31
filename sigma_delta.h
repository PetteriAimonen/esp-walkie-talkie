// Simple sigma-delta modulation with noise-shaping.
// The modulated samples are copied by DMA to the I2S peripheral which outputs
// them as a 1-bit stream that can then be low-pass filtered to give audio output.

#pragma once

#include <common_macros.h>
#include <stdint.h>

// Expands two dac samples to a linear slope between previous and current values.
// DAC values should be in range 0 to 8191.
void IRAM sigma_delta_encode(int dac_prev, int dac_now, volatile uint32_t *output, int oversample_count, int *error_acc);
