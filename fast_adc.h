// Fast ADC reading on ESP8266.
// Most of the information comes from https://github.com/pvvx/esp8266web/blob/2e25559bc489487747205db2ef171d48326b32d4/app/driver/adc.c

#pragma once

#include <common_macros.h>
#include <stdint.h>

void adc_enable();
void IRAM adc_sample();
uint32_t IRAM adc_read_result();
