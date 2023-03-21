#include <hellodrum.hpp>
#include <HelloDrumPads.hpp>
#include "drumManager.h"
#include "midi.h"
#include "eepromManager.h"
#include "drumSound.hpp"

extern std::vector<HelloDrum*> padsList; // TODO delete this and check if this will work

SinglePad * kick;
DoublePad * snare;
// HelloDrum hihat(3);
 HiHatPedalPad * hihatPedal;
 Cymbal2ZonesPad * ride;

static ADC_HandleTypeDef* adcLocal;
static TIM_HandleTypeDef* timActiveSense;
static TIM_HandleTypeDef* timPiezoAsk;
static uint32_t * adc_buf;
static uint16_t * adc_val;

typedef enum {
	ADC_IDLE = 0, ADC_READY, ADC_NOT_INIT, ADC_RUN
} ADC_State;
volatile ADC_State adcState = ADC_NOT_INIT;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == (*adcLocal).Instance) {
		for (uint8_t i = 0; i < HelloDrum::getChannelsAmount(); i++) {
			adc_val[i] = adc_buf[i];
		}
		adcState = ADC_READY;
	}
}

void initHelloDrums(void) {
	//It is necessary to make the order in exactly the same order as you named the pad first.

	kick = new SinglePad(0);
	snare = new DoublePad(1, 2);
	// HelloDrum hihat(3);
	hihatPedal = new HiHatPedalPad(3, FSR_PEDAL);
	ride = new Cymbal2ZonesPad(4, 5);


	// TODO add automatic variable name
	kick->settingName("KICK");
	snare->settingName("SNARE");
//	hihat.settingName("HIHAT");
	hihatPedal->settingName("HIHAT PEDAL");
	ride->settingName("RIDE");

	//Load settings from EEPROM.
	//It is necessary to make the order in exactly the same order as you named the pad first.
	EepromManager::getInstance()->loadInfoSector();

	HelloDrum::prinListSize();
	HelloDrum::loadPadsSettings();
	DrumSound::initAudioCore();
	HelloDrum::loadPadsSounds();

	adc_buf = new uint32_t[HelloDrum::getChannelsAmount()];
	adc_val = new uint16_t[HelloDrum::getChannelsAmount()];
	adcState = ADC_IDLE;

	HAL_TIM_Base_Start_IT(timActiveSense); //Active Sense Message
	HAL_TIM_Base_Start_IT(timPiezoAsk); // Piezo Values
}

void requestPiezo(void) {
	 HAL_ADC_Start_DMA(adcLocal, (uint32_t*) &adc_buf[0], HelloDrum::getChannelsAmount());
	 adcState = ADC_RUN;
}

void checkHelloDrums(void){
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
	return adc_val[currentPin];
}

void callAudioStreamHandle(void){
	DrumSound::handleAudioStream();
}
