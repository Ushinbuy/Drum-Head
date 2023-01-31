#ifndef USER_INC_DRUMCORE_H_
#define USER_INC_DRUMCORE_H_

#include "stm32f7xx_hal.h"
#include "drumidy.h"

#define NUMBER_OF_CHANNELS 6

void requestPiezoAdc(void);
void checkPiezoChannels(void);
void initAndStartDrum(void);


#endif /* USER_INC_DRUMCORE_H_ */
