#include "uartManage.h"
#include "string.h"

char buffer_out[1000];
static uint8_t buffer_in[64];
static UART_HandleTypeDef* localUart;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == (*localUart).Instance) {
		buffer_in[15] = 1;
	}
}

void setLinkUart(UART_HandleTypeDef* globalUart){
	localUart = globalUart;
}

void sendUart (const char *_msg){
	HAL_UART_Transmit_DMA(localUart, (uint8_t*) _msg, strlen((char const*) _msg));
}
