#include "alaw.h"

// Implementations are from http://dystopiancode.blogspot.com/2012/02/pcm-law-and-u-law-companding-algorithms.html

uint8_t alaw_encode(int sample)
{
   const uint16_t ALAW_MAX = 0xFFF;
   uint16_t mask = 0x800;
   uint8_t sign = 0;
   uint8_t position = 11;
   uint8_t lsb = 0;
   if (sample < 0)
   {
      sample = -sample;
      sign = 0x80;
   }
   if (sample > ALAW_MAX)
   {
      sample = ALAW_MAX;
   }
   for (; ((sample & mask) != mask && position >= 5); mask >>= 1, position--);
   lsb = (sample >> ((position == 4) ? (1) : (position - 4))) & 0x0f;
   return (sign | ((position - 4) << 4) | lsb) ^ 0x55;
}

int alaw_decode(uint8_t sample)
{
   uint8_t sign = 0x00;
   uint8_t position = 0;
   int decoded = 0;
   sample^=0x55;
   if(sample&0x80)
   {
      sample&=~(1<<7);
      sign = -1;
   }
   position = ((sample & 0xF0) >>4) + 4;
   if(position!=4)
   {
      decoded = ((1<<position)|((sample&0x0F)<<(position-4))
                |(1<<(position-5)));
   }
   else
   {
      decoded = (sample<<1)|1;
   }
   return (sign==0)?(decoded):(-decoded);
}