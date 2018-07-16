#include "fast_adc.h"
#include "espressif/esp_common.h"

extern void sdk_rom_i2c_writeReg_Mask(uint32_t block, uint32_t host_id,
        uint32_t reg_add, uint32_t Msb, uint32_t Lsb, uint32_t indata);

#define SAR_BASE ((volatile uint32_t*)0x60000D00)
#define SAR_CFG (SAR_BASE + 20)
#define SAR_TIM1 (SAR_BASE + 21)
#define SAR_TIM2 (SAR_BASE + 22)
#define SAR_CFG1 (SAR_BASE + 23)
#define SAR_CFG2 (SAR_BASE + 24)
#define SAR_DATA (SAR_BASE + 32)
static volatile uint8_t * const tout_dis_txpwr_track = (void*)0x3ffe9674;

void adc_enable()
{
  sdk_system_adc_read();
  
  // Inform the vendor libs that ADC is in use
  *tout_dis_txpwr_track = 1;
  
  // ADC clock speed is 3 MHz, /16/8 means one set of 8 SAR samples takes about 42 us.
  uint32_t clk_div = 16;
  uint32_t win_cnt = 8;
  *SAR_CFG = (*SAR_CFG & 0xFFFF00E3) | ((win_cnt-1) << 2) | (clk_div << 8);
  *SAR_TIM1 = (*SAR_TIM1 & 0xFF000000) | (clk_div * 5 + ((clk_div - 1) << 16) + ((clk_div - 1) << 8) - 1);
  *SAR_TIM2 = (*SAR_TIM2 & 0xFF000000) | (clk_div * 11 + ((clk_div * 3 - 1) << 8) + ((clk_div * 11 - 1) << 16) - 1);
  sdk_rom_i2c_writeReg_Mask (108,2,0,5,5,1);
  *SAR_CFG1 |= 1 << 21;
}

void adc_sample()
{
  // Start reading next sample.
  uint32_t v = *SAR_CFG;
  *SAR_CFG = v & ~2;
  *SAR_CFG = v | 2;
}

// Returns value in range 0-8192.
uint32_t adc_read_result()
{
  uint32_t sum = 0;
  for (int i = 0; i < 8; i++)
  {
      uint32_t x = ~SAR_DATA[i] & 0x7FF;
      x += (x & 0xFF) * 23 / 256; // Improves linearity a bit
      sum += x;
  }
  return sum / 2;
}