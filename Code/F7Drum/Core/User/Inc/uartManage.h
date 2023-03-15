#ifndef USER_INC_UARTMANAGE_H_
#define USER_INC_UARTMANAGE_H_

#include <stdio.h>
#include "main.h"

void setLinkUart(UART_HandleTypeDef* globalUart);
void sendUart(const char *_msg);

#endif /* USER_INC_UARTMANAGE_H_ */
