#include "HelloDrumSinglePad.hpp"

//////////////// SINGLE ///////////////////
SinglePad::SinglePad(byte pin1) : HelloDrum(pin1){
	padType[padNum] = SINGLE_PAD;
}

void SinglePad::sensingPad(){
	singlePiezo();
}

void SinglePad::executePad(){
	if(hit){
		sendMidiGEN(settings.note, velocity);
		noteHeadSound->playSound(velocity);
	}
}

//////////////// DOUBLE ///////////////////

DoublePad::DoublePad(byte pin1, byte pin2) : HelloDrum(pin1, pin2){
	padType[padNum] = DOUBLE_PAD;
}

void DoublePad::sensingPad(){
	dualPiezo();
}

void DoublePad::executePad(){
	if(hit){
		sendMidiGEN(settings.note, velocity);
		noteHeadSound->playSound(velocity);
	}
	if (hitRim) {
		sendMidiGEN(settings.noteRim, velocity);
		noteRimSound->playSound(velocity);
	}
}

//////////////// HI HAT ///////////////////

HiHatPad::HiHatPad(byte pin1, HiHatPedalPad * pedal) : HelloDrum(pin1){
	this->pedal = pedal;
	padType[padNum] = HH_PAD;
}

void HiHatPad::sensingPad(){
	HH();
}

void HiHatPad::executePad(){
	if(hit){
		if(pedal->openHH){
			sendMidiGEN(noteOpen1, velocity);
			noteHeadSound->playSound(velocity);
		}
		else if(pedal->closeHH){
			sendMidiGEN(noteClose1, velocity);
			noteHeadSound->playSound(velocity);
		}
	}
}

void HiHatPad::loadMemory(){
	HelloDrum::loadMemory();
	noteOpen1 = settings.note;
	noteClose1 = settings.noteRim;
}
//////////////// HI HAT 2 ZONE ///////////////////

HiHat2zonePad::HiHat2zonePad(byte pin1, byte pin2, HiHatPedalPad * pedal) : HelloDrum(pin1, pin2){
	// TODO copy noteOpen noteClose
	this->pedal = pedal;
	padType[padNum] = HH2_PAD;
}

void HiHat2zonePad::sensingPad(){
	HH2zone();
}

void HiHat2zonePad::executePad(){
	if (hit) {
		if (pedal->openHH) {
			sendMidiGEN(noteOpen, velocity);
			noteHeadSound->playSound(velocity);
		} else if (pedal->closeHH) {
			sendMidiGEN(noteClose, velocity);
			noteHeadSound->playSound(velocity);
		}
	}
	if (hitRim) {
		if (pedal->openHH) {
			sendMidiGEN(noteOpenEdge, velocity);
			noteHeadSound->playSound(velocity);
		} else if (pedal->closeHH) {
			sendMidiGEN(noteCloseEdge, velocity);
			noteHeadSound->playSound(velocity);
		}
	}
}

void HiHat2zonePad::loadMemory(){
	HelloDrum::loadMemory();
	noteOpen1 = settings.note;
	noteEdge1 = settings.noteRim;
	// TODO notes below must be another
	noteClose1 = settings.noteRim;
	noteOpenEdge = settings.noteRim;
	noteCloseEdge = settings.noteCup;
}
//////////////// HI HAT PEDAL ///////////////////

HiHatPedalPad::HiHatPedalPad(byte pin1, PedalType pedalType) : HelloDrum(pin1){
	this->pedalType = pedalType;
	padType[padNum] = HHC_PAD;
}

void HiHatPedalPad::sensingPad(){
	switch (pedalType){
	case FSR_PEDAL:
		FSR();
		break;
	case TCRT5000_PEDAL:
		TCRT5000();
	}
}

void HiHatPedalPad::executePad(){
	if(hit){
		sendMidiGEN(settings.note, velocity);
		noteHeadSound->playSound(velocity);
	}
	if (moving) {
		sendMidiGEN(settings.noteRim, velocity);
		sendMidiControlChange(4, pedalCC);
	}
}

//////////////// CYMBAL 2 ZONES ///////////////////

Cymbal2ZonesPad::Cymbal2ZonesPad(byte pin1, byte pin2) : HelloDrum(pin1, pin2){
	padType[padNum] = CY2_PAD;
}

void Cymbal2ZonesPad::sensingPad(){
	cymbal2zone();
}

void Cymbal2ZonesPad::executePad(){
	if (hit == true) {
		sendMidiGEN(settings.note, velocity); //(note, velocity, channel)
		noteHeadSound->playSound(velocity);
	}

	//edge
	else if (hitRim == true) {
		sendMidiGEN(settings.noteRim, velocity); //(note, velocity, channel)
		noteRimSound->playSound(velocity);
	}

	//choke
	if (choke == true) {
		sendMidiAT(settings.note, 127);
		sendMidiAT(settings.noteRim, 127);
		sendMidiAT(settings.note, 0);
		sendMidiAT(settings.noteRim, 0);
	}
}

//////////////// CYMBAL 3 ZONES ///////////////////

Cymbal3ZonesPad::Cymbal3ZonesPad(byte pin1, byte pin2) : HelloDrum(pin1, pin2){
	padType[padNum] = CY3_PAD;
}

void Cymbal3ZonesPad::sensingPad(){
	cymbal3zone();
}

void Cymbal3ZonesPad::executePad(){
	//RIDE//
	//1.bow
	if (hit == true) {
		sendMidiGEN(settings.note, velocity); //(note, velocity, channel)
		noteHeadSound->playSound(velocity);
	}

	//2.edge
	else if (hitRim == true) {
		sendMidiGEN(settings.noteRim, velocity); //(note, velocity, channel)
		noteRimSound->playSound(velocity);
	}

	//3.cup
	else if (hitCup == true) {
		sendMidiGEN(settings.noteCup, velocity); //(note, velocity, channel)
		noteCupSound->playSound(velocity);
	}

	//4.choke
	if (choke == true) {
		sendMidiAT(settings.note, 127);
		sendMidiAT(settings.noteRim, 127);
		sendMidiAT(settings.noteCup, 127);
		sendMidiAT(settings.note, 0);
		sendMidiAT(settings.noteRim, 0);
		sendMidiAT(settings.noteCup, 0);
	}
}
