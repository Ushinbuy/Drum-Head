#include "midi.h"

#define TAB 0x09
#define MIDI_CHANNEL 	0x9
#define NOTE_ON			0x90 + MIDI_CHANNEL
#define NOTE_OFF 		0x80 + MIDI_CHANNEL
#define KEY_PRESSURE 	0xA0 + MIDI_CHANNEL // AfterTouch
#define CONTROL_CHANGE	0xB0 + MIDI_CHANNEL

void sendMidiActiveSense(uint8_t* _upd_active_sens){
	if (*_upd_active_sens) {
		*_upd_active_sens = 0;
		uint8_t bff[4] = { 0x0F, 0xFE, 0x00, 0x00 };
		tx_midi((uint8_t*) bff, 4);
	}
}

// MIDI generic ON/OFF message
void sendMidiGEN(uint8_t note, uint8_t vel){
  uint8_t bff[8] = {TAB,  NOTE_ON, 0, 0,
		  	  	  	TAB,  NOTE_OFF,0, 0x00};
  bff[2] = 0x7f & note;
  bff[3] = 0x7f & vel;
  bff[6] = 0x7f & note;
  tx_midi((uint8_t *)bff,8);
}

// MIDI generic ON message
void sendMidi(uint8_t note, uint8_t vel){
  uint8_t bff[4] = {TAB,  NOTE_ON, 0x00, 0x00};
  bff[2] = 0x7f & note;
  bff[3] = 0x7f & vel;
  tx_midi((uint8_t *)bff,4);
}
// MIDI generic OFF message
void sendMidiOFF(uint8_t note){
  uint8_t bff[4] = {TAB,  NOTE_OFF, 0x00, 0x00};
  bff[2] = 0x7f & note;
  bff[3] = 0x7f;
  tx_midi((uint8_t *)bff,4);
}


// aftertouch
void sendMidiAT(uint8_t note, uint8_t vel){
  uint8_t bff[4] = {TAB,  NOTE_ON, 0x00, 0x00};
  bff[2] = 0x7f & note;
  bff[3] = 0x7f & vel;
  tx_midi((uint8_t *)bff,4);
}


// MIDI generic ON message
void sendMidi2(uint8_t note1, uint8_t vel1,uint8_t note2, uint8_t vel2){
  uint8_t bff[8] = {TAB,  NOTE_ON, 0x00, 0x00,
		  	  	  	TAB,  NOTE_ON, 0x00, 0x00};
  bff[2] = 0x7f & note1;
  bff[3] = 0x7f & vel1;
  bff[2+4] = 0x7f & note2;
  bff[3+4] = 0x7f & vel2;
  tx_midi((uint8_t *)bff,8);
}

// MIDI HiHat pedal press message
//void sendMidiHHPedalOn(){
//  uint8_t bff[20] = { TAB,  KEY_PRESSURE, HHOPEN , 0x7F,
//		  	  	  	  TAB,  KEY_PRESSURE, HHCLOSE, 0x7F,
//					  TAB,  KEY_PRESSURE, HHCLOSEPEDAL, 0x7F,
//		  	  	  	  TAB,  NOTE_ON, HHPEDAL, 0x64,
//					  TAB,	NOTE_ON, HHPEDAL, 0x00};
//  tx_midi((uint8_t *)bff, 20);
//}

// pedal aftertouch for hihat
//void sendMidiHHPedalOff(){
//  uint8_t bff[4] = { TAB,  KEY_PRESSURE, HHPEDAL , 0x3F};
//  tx_midi((uint8_t *)bff,4);
//}

void sendMidiControlChange(uint8_t controlNumber, uint8_t controlValue) {
	// TODO check correction of this message. It can be checked from arduino
	uint8_t bff[4] = { TAB, controlNumber, controlValue, MIDI_CHANNEL };
	tx_midi((uint8_t*) bff, 4);
}
