#ifndef PLATFORM_H_
#define PLATFORM_H_

#include <stdint.h>

#define NUMBER_OF_CHANELS 6
#define HIGH true
#define LOW false

using byte = uint8_t;
uint32_t millis(void);
uint16_t analogRead(byte currentPin);
short map(short x, short in_min, short in_max, short out_min, short out_max);
void delay(uint32_t Delay);

#endif /* PLATFORM_H_ */
