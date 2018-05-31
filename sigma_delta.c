#include "sigma_delta.h"

// Fast conversion of 5-bit code to a 32-bit word.
// Each word of the array has given number of 1 bits
// evenly distributed within it.
static const uint32_t IRAM_DATA g_sigmadelta_lookup[34] = {
  0x00000000,
  0x00000000, 0x00000001, 0x00010001, 0x00200801, 
  0x01010101, 0x04082041, 0x08210821, 0x08844221, 
  0x11111111, 0x12244891, 0x24492449, 0x24929249, 
  0x29292929, 0x4a5294a5, 0x4aa54aa5, 0x54aaaa55, 
  0x55555555, 0x55aaab55, 0x5ab55ab5, 0x6b5ab5ad, 
  0x6d6d6d6d, 0x6db6db6d, 0xb6dbb6db, 0xb76eeddb, 
  0xbbbbbbbb, 0xbddef77b, 0xdef7def7, 0xdfbefbf7, 
  0xefefefef, 0xf7feffdf, 0xfefffeff, 0xfffeffff,
  0xffffffff
};

void sigma_delta_encode(int dac_prev, int dac_now, volatile uint32_t *output, int oversample_count, int *error_acc)
{
  static uint32_t dither_rnd = 12345678;
  int error = *error_acc;
  int dac_delta = (dac_now - dac_prev) * 256 / oversample_count;
  for (int i = 0; i < oversample_count; i++)
  {
    dither_rnd ^= (dither_rnd << 13);
    dither_rnd ^= (dither_rnd >> 17);
    dither_rnd ^= (dither_rnd << 5);
    
    // This gives approximately triangular distribution for the dither
    int dither_val = (dither_rnd & 255) + ((dither_rnd >> 8) & 255) - 256;
    
    int dac_value = dac_prev + dac_delta * i / 256;
    
    int dithered = dac_value + error + dither_val;
    int quantized = (dithered >> 8);
    error += dac_value - (quantized << 8);
    
    *output = g_sigmadelta_lookup[1 + quantized];
  }
  
  *error_acc = error;
}
