#include "platform.h"
#include "main.h"

uint32_t millis(void){
	return HAL_GetTick();
}

short map(short x, short in_min, short in_max, short out_min, short out_max){
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void delay(uint32_t Delay){
	HAL_Delay(Delay);
}
