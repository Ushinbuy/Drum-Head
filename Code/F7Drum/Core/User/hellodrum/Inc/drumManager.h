#ifndef DRUMMANAGER_H_
#define DRUMMANAGER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void initHelloDrums(void);
void requestPiezo(void);
void checkHelloDrums(void);
uint16_t analogRead(uint8_t currentPin);
void setLinksDrumCore(ADC_HandleTypeDef *adcGlobal,
		TIM_HandleTypeDef *timGlobalPiezoAsk,
		TIM_HandleTypeDef *timGlobalActiveSense);

void callAudioStreamHandle(void);// todo this function in another plase

#ifdef __cplusplus
}
#endif

#endif /* DRUMMANAGER_H_ */
