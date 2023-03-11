#include "platform.h"
#include "main.h"

uint16_t audioBuffer[NUMBER_OF_CHANELS] = { 0 };

uint32_t millis(void){
	return HAL_GetTick();
}

uint16_t analogRead(byte currentPin){
	// currentPin is offset for audioBuffer in stm massive
	return audioBuffer[currentPin];
}

short map(short x, short in_min, short in_max, short out_min, short out_max){
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void delay(uint32_t Delay){
	HAL_Delay(Delay);
}
