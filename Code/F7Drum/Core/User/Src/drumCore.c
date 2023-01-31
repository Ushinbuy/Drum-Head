#include "drumCore.h"

#define OFF_DELAY_MS 200

DRUM channel[NUMBER_OF_CHANNELS];	// array of drums
GPIO_PinState aux_current_state[NUMBER_OF_CHANNELS];
// ADC buffers0
uint32_t adc_buf[NUMBER_OF_CHANNELS];
// channel values
uint16_t adc_val[NUMBER_OF_CHANNELS];

extern ADC_HandleTypeDef hadc3;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;

char ASCIILOGO[] = "\n"\
"  ___                 _    _\n"\
" |   \\ _ _ _  _ _ __ (_)__| |_  _ \n"\
" | |) | '_| || | '  \\| / _` | || |\n"\
" |___/|_|  \\_,_|_|_|_|_\\__,_|\\_, |\n"\
"     .-.,     ,--. ,--.      |__/ \n"\
"    `/|~\\     \\__/T`--'     . \n"\
"    x |`' __   ,-~^~-.___ ==I== \n"\
"      |  |--| /       \\__}  | \n"\
"      |  |  |{   /~\\   }    | \n"\
"     /|\\ \\__/ \\  \\_/  /|   /|\\ \n"\
"    / | \\|  | /`~-_-~'X.\\ //| \\ \n\n"\
"= Send any char for configuration =\n";

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == hadc3.Instance) {
		for (uint8_t i = 0; i < NUMBER_OF_CHANNELS; i++) {
			adc_val[i] = adc_buf[i];
		}

		getAuxState(aux_current_state);

		setStepTime(HAL_GetTick());

		for (uint8_t i = 0; i < NUMBER_OF_CHANNELS; i++) {
			Update_channel(&channel[i], adc_val[i], aux_current_state[i]);
		}
	}
}

void requestPiezoAdc(void){
	HAL_ADC_Start_DMA(&hadc3, (uint32_t*) &adc_buf[0], NUMBER_OF_CHANNELS);
}

void checkPiezoChannels(void){
	int _volume;
	for (uint8_t ch = 0; ch < NUMBER_OF_CHANNELS; ch++) {

		if (channel[ch].main_rdy) {
			channel[ch].main_rdy = 0;

			// custom volume calculation for mesh
			if (channel[ch].chnl_type < 2) {
				_volume = (int) (100.
						* (float) (channel[ch].main_rdy_height - PEAK_THRESHOLD)
						/ 4096. * 100. / (float) channel[ch].peak_volume_norm);
				if ((channel[ch].chnl_type == MESH_RIM_AUTOAUX)
						&& (channel[ch].main_rdy_usealt))
					_volume = _volume * 4;
			} else {
				//volume for cymbals
				_volume = (int) (100.
						* (float) (channel[ch].main_rdy_height - PEAK_THRESHOLD)
						/ 4096. * 100. / (float) channel[ch].peak_volume_norm
						* 2);
			}

			if (_volume > 127)
				_volume = 127;
			if (_volume < 1)
				_volume = 1;
			channel[ch].main_rdy_volume = (uint8_t) _volume;

			uint8_t vc;
			if (channel[ch].main_rdy_usealt)
				vc = channel[ch].alt_voice;	//	sendMidiGEN(channel[ch].alt_voice ,channel[ch].main_rdy_volume);
			else
				vc = channel[ch].main_voice;//	sendMidiGEN(channel[ch].main_voice,channel[ch].main_rdy_volume);

			sendMidi(vc, channel[ch].main_rdy_volume);
			channel[ch].main_last_on_voice = vc;
			channel[ch].main_last_on_time = HAL_GetTick();

			sendDebug(ch, 0);
		}

		if (channel[ch].aux_rdy) {
			channel[ch].aux_rdy = 0;
			sendDebug(ch, 1);

			switch (channel[ch].chnl_type) {
			case CYMBAL_HIHAT:
				if (channel[ch].aux_rdy_state == CHANNEL_PEDAL_PRESSED)
					sendMidiHHPedalOn();
				//				  else
				//					  sendMidiGEN(channel[ch].main_voice, 5);
				break;

			case CYMBAL_MUTE:
				if (channel[ch].aux_rdy_state == CHANNEL_PEDAL_PRESSED)
					sendMidi2(channel[ch].main_voice, 1, channel[ch].main_voice,
							0);
				break;

			case CYMBAL_2_ZONE:
				sendMidi2(channel[ch].main_voice, 1, channel[ch].main_voice, 0);
				break;

				// INDEPENDENT AUX INPUTS
			default:
				if (channel[ch].aux_type == AUX_TYPE_PAD)
					sendMidi2(channel[ch].aux_voice, 100, channel[ch].aux_voice,
							0);
				else { //PEDAL
					   // PEDAL pressed
					if (channel[ch].aux_rdy_state == CHANNEL_PEDAL_PRESSED)
						sendMidi2(channel[ch].aux_voice, 100,
								channel[ch].aux_voice, 0);
					// PEDAL RELEASED... IN CASE
					//					  else
					//						  sendMidi2(channel[ch].aux_voice, 1, channel[ch].aux_voice,0);
				}
			}
		}
		// send off command if needed
		if (channel[ch].main_last_on_voice > 0) {
			if (HAL_GetTick()
					> (channel[ch].main_last_on_time + OFF_DELAY_MS)) {
				sendMidi(channel[ch].main_last_on_voice, 0);
				channel[ch].main_last_on_voice = 0;
			}
		}
	}
}

void initAndStartDrum(void) {
	HAL_ADC_Start(&hadc3);
	getAuxState(aux_current_state);

	initDrum(&channel[0], HHCLOSE, HHCLOSE, MESH_PAD_AUTOAUX, aux_current_state[0]);
	initDrum(&channel[1], TOMF, TOMF, MESH_PAD_AUTOAUX, aux_current_state[1]);
	initDrum(&channel[2], HHPEDAL, HHPEDAL, MESH_PAD_AUTOAUX, aux_current_state[2]);
	initDrum(&channel[3], TOM3, TOM3, MESH_PAD_AUTOAUX, aux_current_state[3]);
	initDrum(&channel[4], HHOPEN, HHOPEN, MESH_PAD_AUTOAUX, aux_current_state[4]);
	initDrum(&channel[5], TOM2, TOM2, MESH_PAD_AUTOAUX, aux_current_state[5]);
	//  initDrum(&channel[6], TOMF , TOMF  	, MESH_PAD_AUTOAUX	, aux_current_state[6]);
	//
	//  // cymbals
	//  initDrum(&channel[7], CRASH, CRASH 	, CYMBAL_MUTE			, aux_current_state[7]);	// CH7 aux disabled
	//  initDrum(&channel[8], RIDE ,  BELL 	, CYMBAL_2_ZONE			, aux_current_state[8]);

	// === Previous Settings ===
	sendUart(ASCIILOGO);
	HAL_Delay(500);

	Load_Setting();

	// start waiting for serial commands
	HAL_Delay(200);
	initSettingsFromUart();

	/// **************************
	/// ******* LETS ROCK! *******
	/// **************************
	HAL_TIM_Base_Start_IT(&htim2); //AS
	HAL_TIM_Base_Start_IT(&htim4); //ADC
}
