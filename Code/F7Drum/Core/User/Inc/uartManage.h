#ifndef USER_INC_UARTMANAGE_H_
#define USER_INC_UARTMANAGE_H_

#include "main.h"

char buffer_out[1000];			// USB Buffers

void initSettingsFromUart(void);
void sendUart(char *_msg);
void getAuxState(GPIO_PinState *_state);
void handleConfigFromUart(void);
void sendDebug(uint8_t _ch, uint8_t _aux);

#endif /* USER_INC_UARTMANAGE_H_ */
