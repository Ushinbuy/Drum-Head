#ifndef USER_INC_DRUMCORE_H_
#define USER_INC_DRUMCORE_H_

#include "drumidy.h"
#include "main.h"

#define NUMBER_OF_CHANNELS 6

void requestPiezoAdc(void);
void checkPiezoChannels(void);
void initAndStartDrum(void);
void getAuxState(GPIO_PinState *_state);

#endif /* USER_INC_DRUMCORE_H_ */
