#include <hellodrum.hpp>
#include <HelloDrumPads.hpp>
#include "drumManager.h"
#include "midi.h"
#include "eepromManager.h"
#include "drumSound.hpp"

SinglePad * kick;
DoublePad * snare;
// HelloDrum hihat(3);
HiHatPedalPad *hihatPedal;
Cymbal3ZonesPad *ride;

static ADC_HandleTypeDef* adcLocal;
static TIM_HandleTypeDef* timActiveSense;
static TIM_HandleTypeDef* timPiezoAsk;
static uint16_t * adc_val;

typedef enum {
	ADC_IDLE = 0, ADC_READY, ADC_NOT_INIT, ADC_RUN
} ADC_State;
volatile ADC_State adcState = ADC_NOT_INIT;

void initHelloDrums(void) {
	//It is necessary to make the order in exactly the same order as you named the pad first.

	kick = new SinglePad(0);
	snare = new DoublePad(1, 2);
	// HelloDrum hihat(3);
	hihatPedal = new HiHatPedalPad(3, FSR_PEDAL);
	ride = new Cymbal3ZonesPad(4, 5);


	// TODO add automatic variable name
	kick->settingName("KICK");
	snare->settingName("SNARE");
//	hihat.settingName("HIHAT");
	hihatPedal->settingName("HIHAT PEDAL");
	ride->settingName("RIDE");

	//Load settings from EEPROM.
	//It is necessary to make the order in exactly the same order as you named the pad first.
	EepromManager::getInstance()->loadInfoSector();

	HelloDrum::loadPadsSettings();
	DrumSound::initAudioCore();
	HelloDrum::loadPadsSounds();

	adc_val = new uint16_t[HelloDrum::getChannelsAmount()];
	adcState = ADC_IDLE;

	HAL_TIM_Base_Start_IT(timActiveSense); //Active Sense Message
	HAL_TIM_Base_Start_IT(timPiezoAsk); // Piezo Values
}

void requestPiezo(void) {
	 adcState = ADC_RUN;
}

static uint16_t ADC_Read(uint32_t channel) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */

	// TODO maybe this can be optimized by sampling mode (Datasheet)
	sConfig.Channel = channel;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(adcLocal, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	HAL_ADC_Start (adcLocal);
	HAL_ADC_PollForConversion(adcLocal, 1000);
	uint16_t value = HAL_ADC_GetValue(adcLocal);
	HAL_ADC_Stop(adcLocal);

	return value;
}

void checkHelloDrums(void){
	if(adcState == ADC_RUN){
		adc_val[0] = ADC_Read(ADC_CHANNEL_0);
		adc_val[1] = ADC_Read(ADC_CHANNEL_8);
		adc_val[2] = ADC_Read(ADC_CHANNEL_7);
		adc_val[3] = ADC_Read(ADC_CHANNEL_6);
		adc_val[4] = ADC_Read(ADC_CHANNEL_5);
		adc_val[5] = ADC_Read(ADC_CHANNEL_4);
		adcState = ADC_READY;
	}

	if(adcState != ADC_READY)
		return;
	adcState = ADC_IDLE;

	HelloDrum::sensingAllPads();
	HelloDrum::executeAllPads();
}

void setLinksDrumCore(ADC_HandleTypeDef *adcGlobal,
		TIM_HandleTypeDef *timGlobalPiezoAsk,
		TIM_HandleTypeDef *timGlobalActiveSense) {
	adcLocal = adcGlobal;
	timActiveSense = timGlobalActiveSense;
	timPiezoAsk = timGlobalPiezoAsk;
}

uint16_t analogRead(uint8_t currentPin){
	// currentPin is offset for audioBuffer in stm massive
	// TODO check this is can be merge with ADC_Read() with removing TIM4 cycling asking
	return adc_val[currentPin];
}

void callAudioStreamHandle(void){
	DrumSound::handleAudioStream();
}
