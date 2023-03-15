#include <hellodrum.hpp>
#include "drumManager.h"
#include "midi.h"
#include "eepromManager.h"
#include "drumSound.hpp"

static HelloDrum kick(0);
static HelloDrum snare(1, 2);
static HelloDrum hihat(3);
static HelloDrum hihatPedal(4);
static HelloDrum ride(5);

static ADC_HandleTypeDef* adcLocal;
static TIM_HandleTypeDef* timActiveSense;
static TIM_HandleTypeDef* timPiezoAsk;
static uint32_t * adc_buf;
// channel values
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
	kick.settingName("KICK");
	snare.settingName("SNARE");
	hihat.settingName("HIHAT");
	hihatPedal.settingName("HIHAT PEDAL");
//	ride.settingName("RIDE");

	//Load settings from EEPROM.
	//It is necessary to make the order in exactly the same order as you named the pad first.
	EepromManager *eeprom = EepromManager::getInstance();
	eeprom->loadInfoSector();

	kick.loadMemory();
	snare.loadMemory();
	hihat.loadMemory();
	hihatPedal.loadMemory();
//	ride.loadMemory();

	DrumSound::initAudioCore();

	kick.initSounds();
	snare.initSounds();
	hihat.initSounds();
	hihatPedal.initSounds();
//	ride.initSounds();

	adc_buf = new uint32_t[HelloDrum::getChannelsAmount()];
	adc_val = new uint16_t[HelloDrum::getChannelsAmount()];
	adcState = ADC_IDLE;

	HAL_TIM_Base_Start_IT(timActiveSense); //AS
	HAL_TIM_Base_Start_IT(timPiezoAsk); //ADC
}

void requestPiezo(void) {
	 HAL_ADC_Start_DMA(adcLocal, (uint32_t*) &adc_buf[0], HelloDrum::getChannelsAmount());
	 adcState = ADC_RUN;
}

void checkHelloDrums(void){
	if(adcState != ADC_READY)
		return;
	adcState = ADC_IDLE;

	kick.singlePiezo();
	snare.dualPiezo();
	hihat.HH();
	hihatPedal.hihatControl();
//	ride.cymbal3zone();

	//KICK//
	if (kick.hit == true) {
		sendMidiGEN(kick.settings.note, kick.velocity);
		kick.noteHeadSound->playSound(kick.velocity);
	}

	//SNARE//
	if (snare.hit == true) {
		sendMidiGEN(snare.settings.note, snare.velocity);
		snare.noteHeadSound->playSound(snare.velocity);
	}
	else if(snare.hitRim == true){
		sendMidiGEN(snare.settings.noteRim, snare.velocity);
		snare.noteRimSound->playSound(snare.velocity);
	}

	//HIHAT//
	if (hihat.hit == true) {
		//check open or close
		//1.open
		if (hihatPedal.openHH == true) {
			sendMidiGEN(hihat.noteOpen, hihat.velocity);
			hihat.noteHeadSound->playSound(hihat.velocity);
		}
		//2.close
		else if (hihatPedal.closeHH == true) {
			sendMidiGEN(hihat.noteClose, hihat.velocity);
			hihat.noteRimSound->playSound(hihat.velocity);
		}
	}

	//HIHAT CONTROLLER//
	//when hihat is closed
	if (hihatPedal.hit == true) {
		sendMidiGEN(hihatPedal.settings.note, hihatPedal.velocity);
		hihatPedal.noteHeadSound->playSound(hihatPedal.velocity);
	}

	//sending state of pedal with controll change
	if (hihatPedal.moving == true) {
		sendMidiControlChange(4, hihatPedal.pedalCC);
	}

	//RIDE//
	//1.bow
//	if (ride.hit == true) {
//		sendMidiGEN(ride.settings.note, ride.velocity); //(note, velocity, channel)
//		ride.noteHeadSound->playSound(ride.velocity);
//	}
//
//	//2.edge
//	else if (ride.hitRim == true) {
//		sendMidiGEN(ride.settings.noteRim, ride.velocity); //(note, velocity, channel)
//	}
//
//	//3.cup
//	else if (ride.hitCup == true) {
//		sendMidiGEN(ride.settings.noteCup, ride.velocity); //(note, velocity, channel)
//	}
//
//	//4.choke
//	if (ride.choke == true) {
//		sendMidiAT(ride.settings.note, 127);
//		sendMidiAT(ride.settings.noteRim, 127);
//		sendMidiAT(ride.settings.noteCup, 127);
//		sendMidiAT(ride.settings.note, 0);
//		sendMidiAT(ride.settings.noteRim, 0);
//		sendMidiAT(ride.settings.noteCup, 0);
//	}
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
