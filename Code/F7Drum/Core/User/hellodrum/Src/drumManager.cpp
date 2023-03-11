#include <hellodrum.hpp>
#include "drumManager.h"
#include "midi.h"
#include "eepromManager.h"

static HelloDrum kick(0);
static HelloDrum snare(1);
static HelloDrum hihat(2);
static HelloDrum hihatPedal(3);
static HelloDrum ride(4, 5);

void initHelloDrums(void) {
	//It is necessary to make the order in exactly the same order as you named the pad first.
	kick.settingName("KICK");
	snare.settingName("SNARE");
	hihat.settingName("HIHAT");
	hihatPedal.settingName("HIHAT PEDAL");
	ride.settingName("RIDE");

	//Load settings from EEPROM.
	//It is necessary to make the order in exactly the same order as you named the pad first.
	EepromManager *eeprom = EepromManager::getInstance();
	eeprom->loadInfoSector();

	kick.loadMemory();
	snare.loadMemory();
	hihat.loadMemory();
	hihatPedal.loadMemory();
	ride.loadMemory();
}

void requestPiezo(void) {
	// TODO uncomment string below
	// HAL_ADC_Start_DMA(adcLocal, (uint32_t*) &adc_buf[0], NUMBER_OF_CHANNELS);
}

void checkHelloDrums(void){
	kick.singlePiezo();
	snare.singlePiezo();
	hihat.HH();
	hihatPedal.hihatControl();
	ride.cymbal3zone();

	//KICK//
	if (kick.hit == true) {
		sendMidiGEN(kick.settings.note, kick.velocity);
	}

	//SNARE//
	if (snare.hit == true) {
		sendMidiGEN(snare.settings.note, snare.velocity);
	}

	//HIHAT//
	if (hihat.hit == true) {
		//check open or close
		//1.open
		if (hihatPedal.openHH == true) {
			sendMidiGEN(hihat.noteOpen, hihat.velocity);
		}
		//2.close
		else if (hihatPedal.closeHH == true) {
			sendMidiGEN(hihat.noteClose, hihat.velocity);
		}
	}

	//HIHAT CONTROLLER//
	//when hihat is closed
	if (hihatPedal.hit == true) {
		sendMidiGEN(hihatPedal.settings.note, hihatPedal.velocity);
	}

	//sending state of pedal with controll change
	if (hihatPedal.moving == true) {
		sendMidiControlChange(4, hihatPedal.pedalCC);
	}

	//RIDE//
	//1.bow
	if (ride.hit == true) {
		sendMidiGEN(ride.settings.note, ride.velocity); //(note, velocity, channel)
	}

	//2.edge
	else if (ride.hitRim == true) {
		sendMidiGEN(ride.settings.noteRim, ride.velocity); //(note, velocity, channel)
	}

	//3.cup
	else if (ride.hitCup == true) {
		sendMidiGEN(ride.settings.noteCup, ride.velocity); //(note, velocity, channel)
	}

	//4.choke
	if (ride.choke == true) {
		sendMidiAT(ride.settings.note, 127);
		sendMidiAT(ride.settings.noteRim, 127);
		sendMidiAT(ride.settings.noteCup, 127);
		sendMidiAT(ride.settings.note, 0);
		sendMidiAT(ride.settings.noteRim, 0);
		sendMidiAT(ride.settings.noteCup, 0);
	}
}

