#include "uartManage.h"
#include "eepromSettings.h"

char buffer_out[1000];
static uint8_t buffer_in[64];

static uint8_t config_Mode[1] = {0};		// flag for activating config over serial

static UART_HandleTypeDef* localUart;
extern DRUM channel[NUMBER_OF_CHANNELS];

static int get_num_from_uart(uint8_t _len);
static uint8_t UartConfigDialog();

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == (*localUart).Instance) {
		buffer_in[15] = 1;
	}
}

void setLinkUart(UART_HandleTypeDef* globalUart){
	localUart = globalUart;
}

void sendUart (char *_msg){
	HAL_UART_Transmit_DMA(localUart, (uint8_t*) _msg, strlen((char const*) _msg));
}

void sendDebug(uint8_t _ch, uint8_t _aux)
{
	uint8_t voice;
	uint8_t volume;
	uint8_t length;

	if (_aux) {
		voice = channel[_ch].aux_voice;

		sprintf(buffer_out, ">>>AUX %d: %X %d [%d %d]\n", _ch, voice,
				channel[_ch].aux_previous_state, channel[_ch].main_peaking,
				channel[_ch].aux_status);
	} else {
		if (channel[_ch].main_rdy_usealt)
			voice = channel[_ch].alt_voice;
		else
			voice = channel[_ch].main_voice;
		volume = channel[_ch].main_ready_volume;
		length = channel[_ch].main_ready_length;
		sprintf(buffer_out,
				">>MAIN %d: voice %X (alt:%d), vol %d (%u/4096) t=%u; AUX = %d\n",
				_ch, voice, channel[_ch].alt_voice, volume,
				channel[_ch].main_ready_height, length, channel[_ch].aux_status);
	}
	sendUart(buffer_out);

	HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
}

void initSettingsFromUart(void) {
	config_Mode[0] = 0;
	HAL_UART_Receive_IT(localUart, &config_Mode[0], 1);
}

static int get_num_from_uart(uint8_t _len){
	uint8_t i;
	int val = 0;
	for (i = 0; i<_len+1; i++)
		buffer_in[i] = 0;


	HAL_UART_Receive_IT (localUart, &buffer_in[0], _len);
	while (buffer_in[0] == 0) {HAL_Delay(1);};
	HAL_Delay(2); // wait for the rest of the message

	val = 0;
	for (i = 0; i<_len; i++){
		if ((buffer_in[i] == 0) || (buffer_in[i] == 10) || (buffer_in[i] == 13)) break;
		if ((buffer_in[0]>='0') && (buffer_in[0]<='9'))
			val = val*10 + (buffer_in[i]-'0');
		else{
			val = -1;
			break;
		}
	}
	HAL_UART_AbortReceive(localUart);
	return val;
}

void handleConfigFromUart(void){
	while (config_Mode[0]) {

		uint8_t rs = UartConfigDialog();

		if ((rs == 1) || (rs == 2)) {
			rs = Save_Setting(0);
			sprintf(buffer_out, "New configuration saved (%X)\n", rs);
			sendUart(buffer_out);
		}
		if (rs == 99) {
			rs = Save_Setting(1);
			sprintf(buffer_out, "Reset to default values, restart the device (%X)\n", rs);
			sendUart(buffer_out);
		}
		config_Mode[0] = 0;
		HAL_UART_Receive_IT(localUart, &config_Mode[0], 1);
	}
}

static uint8_t UartConfigDialog(){

	int val = 0;

	uint8_t rtrn = 0;

	sendUart("\nConfig mode.\nType number of the pad [1..9], or hit the drum (x - reset to default):\n");

	buffer_in[0] = 0;
	HAL_UART_Receive_IT (localUart, &buffer_in[0], 1);

	uint8_t chnl = 10;
	while (chnl == 10){
		  for (uint8_t ch = 0; ch < NUMBER_OF_CHANNELS; ch++)
			  if ((channel[ch].main_rdy)||(channel[ch].aux_ready)){
				  channel[ch].main_rdy = 0;
				  channel[ch].aux_ready = 0;
				  chnl = ch;
				  HAL_UART_AbortReceive(localUart);
			  }
		  if (buffer_in[0]>0){
			  if ((buffer_in[0]>='1') && (buffer_in[0]<='9'))
				  chnl = buffer_in[0]-'1';
			  else
				  chnl = 255;

			  if (buffer_in[0]=='x')
				  // reset to default
				  return 99;
		  }
	}

	if (chnl == 255) {
		HAL_UART_AbortReceive(localUart);
		sendUart("Ciao\n");
		config_Mode[0] = 0;
		HAL_UART_Receive_IT (localUart, &config_Mode[0], 1);
		return 0;
	}

	// got the correct channel.
	// print current values
	sprintf(buffer_out, "Current values CH#%d:\n\tVoices: main %d, aux %d, alt %d\n\tTimings: peak min %d max %d\n\tChannel type: %d,volume norm %d\n",
			chnl+1, channel[chnl].main_voice, channel[chnl].aux_voice, channel[chnl].alt_voice,
			(int)channel[chnl].peak_min_length, (int)channel[chnl].peak_max_length,
			channel[chnl].aux_type, (int)channel[chnl].peak_volume_norm);
	sendUart(buffer_out);
	HAL_Delay(200);

	// Starting to change the values
	// main voicepeak_volume_norm
	sprintf(buffer_out, "\nCH#%d Change main voice from %d:\t",chnl+1, channel[chnl].main_voice);
	sendUart(buffer_out);
	HAL_Delay(200);

	 val = get_num_from_uart(2);
	if ((val>25)&&(val<90)){
		channel[chnl].main_voice = val & 0xFF;
		sprintf(buffer_out, "New main voice: %d\n", channel[chnl].main_voice);
	}else
		sprintf(buffer_out, "Keeping the old value: %d\n", channel[chnl].main_voice);
	sendUart(buffer_out);
	HAL_Delay(200);

	// aux voice
	sprintf(buffer_out, "\nCH#%d Change aux input voice from %d:\t",chnl+1, channel[chnl].aux_voice);
	sendUart(buffer_out);
	HAL_Delay(200);

	 val = get_num_from_uart(2);
	if ((val>25)&&(val<90)){
		channel[chnl].aux_voice = val & 0xFF;
		sprintf(buffer_out, "New aux voice: %d\n", channel[chnl].aux_voice);
	}else
		sprintf(buffer_out, "Keeping the old value: %d\n", channel[chnl].aux_voice);
	sendUart(buffer_out);
	HAL_Delay(200);

	// main alt voice
	sprintf(buffer_out, "\nCH#%d Change main alt voice (when pedal pressed) from %d:\t",chnl+1, channel[chnl].alt_voice);
	sendUart(buffer_out);
	HAL_Delay(200);

	 val = get_num_from_uart(2);
	if ((val>25)&&(val<90)){
		channel[chnl].alt_voice = val & 0xFF;
		sprintf(buffer_out, "New alt voice: %d\n", channel[chnl].alt_voice);
	}else
		sprintf(buffer_out, "Keeping the old value: %d\n", channel[chnl].alt_voice);
	sendUart(buffer_out);
	HAL_Delay(200);

	// channel type
	sprintf(buffer_out, "\nCH#%d Change aux type from %d to:\n\tAUX - auto, MAIN - Mesh(0), Mesh with rim(1), or Cymbal(2),\n\t HiHat(3) with pedal, Cymbal with 2 zones(4), Cymabal with mute button(5)\n", chnl+1,  channel[chnl].chnl_type);
	sendUart(buffer_out);
	HAL_Delay(200);

	val = get_num_from_uart(1);
	if ((val>=0)&&(val<=4)){
		channel[chnl].chnl_type = val & 0xFF;
		sprintf(buffer_out, "New channel type: %d\n", channel[chnl].chnl_type);
	}else
		sprintf(buffer_out, "Keeping the old value: %d\n", channel[chnl].chnl_type);
	sendUart(buffer_out);
	HAL_Delay(200);

	rtrn = 1;
	sprintf(buffer_out, "\nAdjust timing? y - yes, n - save settings and exit\n");
	sendUart(buffer_out);
	HAL_Delay(200);


	buffer_in[0] = 0;
	HAL_UART_Receive_IT (localUart, &buffer_in[0], 1);
	while (buffer_in[0] == 0){HAL_Delay(1);}
	if (buffer_in[0] == 'y'){

		// Peak threshold
		sprintf(buffer_out, "\nCH#%d Volume norm = %d (default 50, 0..255) (full volume point, 100~4096). New:\t",chnl+1,(int) channel[chnl].peak_volume_norm);
		sendUart(buffer_out);
		HAL_Delay(200);

		val = get_num_from_uart(3);
		if ((val>0)&&(val<256)){
			channel[chnl].peak_volume_norm = val;
			sprintf(buffer_out, "New threshold = %d\n", (int)channel[chnl].peak_volume_norm);
		}else
			sprintf(buffer_out, "Keeping the old value: %d\n", (int)channel[chnl].peak_volume_norm);
		sendUart(buffer_out);
		HAL_Delay(200);

		// min peak len
		sprintf(buffer_out, "\nCH#%d Peak min length = %d (default mesh 15, cymbal 4, 1..99) [x0.1ms]. New:\t",chnl+1,(int) channel[chnl].peak_min_length);
		sendUart(buffer_out);
		HAL_Delay(200);

		val = get_num_from_uart(2);
		if ((val>0)&&(val<100)){
			channel[chnl].peak_min_length = val;
			sprintf(buffer_out, "New min length = %d\n", (int)channel[chnl].peak_min_length);
		}else
			sprintf(buffer_out, "Keeping the old value: %d\n", (int)channel[chnl].peak_min_length);
		sendUart(buffer_out);
		HAL_Delay(200);

		// max peak len
		sprintf(buffer_out, "\nCH#%d Peak max length = %d (default 200, 1..255) [x0.1ms]. New:\t",chnl+1, (int)channel[chnl].peak_max_length);
		sendUart(buffer_out);
		HAL_Delay(200);

		val = get_num_from_uart(3);
		if ((val>0)&&(val<256)){
			channel[chnl].peak_max_length = val;
			sprintf(buffer_out, "New max length = %d\n", (int)channel[chnl].peak_max_length);
		}else
			sprintf(buffer_out, "Keeping the old value: %d\n", (int)channel[chnl].peak_max_length);
		sendUart(buffer_out);
		HAL_Delay(200);
		rtrn = 2;
	}
	return rtrn;
}
