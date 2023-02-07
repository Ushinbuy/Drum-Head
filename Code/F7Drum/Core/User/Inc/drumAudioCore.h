#ifndef USER_INC_DRUMAUDIOCORE_H_
#define USER_INC_DRUMAUDIOCORE_H_

#define ADDRESS_CRASH	0x90000000
#define ADDRESS_COWBELL	0x90040000
#define ADDRESS_HAT		0x90060000
#define ADDRESS_KICK	0x90080000
#define ADDRESS_SNARE	0x900C0000
#define ADDRESS_TOM		0x900D0000

#include <stdio.h>

typedef enum{
	SOUND_INIT = 0,
	SOUND_IDLE,
	SOUND_PLAY
} SoundStateEnum;

typedef struct {
	uint32_t fileLength;
	uint32_t currentOffset;
	float userVolumeDB;
	float currentVolumeFloat;
	const uint8_t* startAddress;
	volatile SoundStateEnum soundState;
} DrumSoundStruct;

void drumPlayDebugSounds(void);
void initAudioCore(void);
void handleAudioStream(void);
void changeVolume(DrumSoundStruct *drum, float newVolumeDB);
void playSound(DrumSoundStruct *drum, uint8_t velocity);

#endif
