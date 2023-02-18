#ifndef __MIDI_H
#define __MIDI_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

void tx_midi(uint8_t *_buffer, uint16_t len);

void sendMidiActiveSense();
// MIDI generic ON/OFF message
void sendMidiGEN(uint8_t note, uint8_t vel);
// MIDI generic ON message
void sendMidi(uint8_t note, uint8_t vel);
// MIDI generic OFF message
void sendMidiOFF(uint8_t note);
// aftertouch
void sendMidiAT(uint8_t note, uint8_t vel);
// MIDI generic ON message
void sendMidi2(uint8_t note1, uint8_t vel1,uint8_t note2, uint8_t vel2);
// MIDI HiHat pedal press message
void sendMidiHHPedalOn();
// pedal aftertouch for hihat
void sendMidiHHPedalOff();

void sendMidiControlChange(uint8_t controlNumber, uint8_t controlValue);

#ifdef __cplusplus
}
#endif

#endif /* __MIDI_H */
