#include <hellodrum.hpp>
#include "drumManager.h"
#include "midi.h"

/**
 * This method for creating space in memory
 */
void firstInitMem(void) {
	// TODO add check if mem not inititalize

	HelloDrum pad_0(0);
	HelloDrum pad_1(1);
	HelloDrum pad_2(2);
	HelloDrum pad_3(3);
	HelloDrum pad_4(4);
	HelloDrum pad_5(5);
	HelloDrum pad_6(6);
	HelloDrum pad_7(7);
	HelloDrum pad_8(8);
	HelloDrum pad_9(9);
	HelloDrum pad_10(10);
	HelloDrum pad_11(11);
	HelloDrum pad_12(12);
	HelloDrum pad_13(13);
	HelloDrum pad_14(14);
	HelloDrum pad_15(15);

	//if you have more pads, just add code like this
	//HelloDrum pad_16(16);
	//HelloDrum pad_17(17);
	//HelloDrum pad_18(18);

	//Initialize
	pad_0.initMemory();
	pad_1.initMemory();
	pad_2.initMemory();
	pad_3.initMemory();
	pad_4.initMemory();
	pad_5.initMemory();
	pad_6.initMemory();
	pad_7.initMemory();
	pad_8.initMemory();
	pad_9.initMemory();
	pad_10.initMemory();
	pad_11.initMemory();
	pad_12.initMemory();
	pad_13.initMemory();
	pad_14.initMemory();
	pad_15.initMemory();

	//if you have more pads, just add code like this
	//pad_16.initMemory();
	//pad_17.initMemory();
	//pad_18.initMemory();
}

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
		sendMidiGEN(kick.note, kick.velocity);
	}

	//SNARE//
	if (snare.hit == true) {
		sendMidiGEN(snare.note, snare.velocity);
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
		sendMidiGEN(hihatPedal.note, hihatPedal.velocity);
	}

	//sending state of pedal with controll change
	if (hihatPedal.moving == true) {
		sendMidiControlChange(4, hihatPedal.pedalCC);
	}

	//RIDE//
	//1.bow
	if (ride.hit == true) {
		sendMidiGEN(ride.note, ride.velocity); //(note, velocity, channel)
	}

	//2.edge
	else if (ride.hitRim == true) {
		sendMidiGEN(ride.noteRim, ride.velocity); //(note, velocity, channel)
	}

	//3.cup
	else if (ride.hitCup == true) {
		sendMidiGEN(ride.noteCup, ride.velocity); //(note, velocity, channel)
	}

	//4.choke
	if (ride.choke == true) {
		sendMidiAT(ride.note, 127);
		sendMidiAT(ride.noteRim, 127);
		sendMidiAT(ride.noteCup, 127);
		sendMidiAT(ride.note, 0);
		sendMidiAT(ride.noteRim, 0);
		sendMidiAT(ride.noteCup, 0);
	}
}
