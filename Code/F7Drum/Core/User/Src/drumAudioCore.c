#include "drumAudioCore.h"
#include "drumCore.h"
#include "wm8994.h"
#include "stm32746g_discovery_audio.h"
#include "stm32746g_discovery_qspi.h"
#include "wavFile.h"
#include "audio.h"
#include <string.h>
#include <stdio.h>

extern char buffer_out[1000];

#define AUDIO_BUFFER_SIZE 1024 	// must be equal to 20 ms * 48 kHz
#define NUMBER_OF_AUDIO_CHANNELS NUMBER_OF_CHANNELS + 0 // can be more, because have alternate sounds - rim, ride bell, etc.

typedef enum {
	AUDIO_STATE_IDLE = 0, AUDIO_STATE_INIT, AUDIO_STATE_PLAYING,
} AUDIO_PLAYBACK_StateTypeDef;

typedef enum {
	BUFFER_OFFSET_NONE = 0,
	PLAY_BUFFER_OFFSET_HALF,
	PLAY_BUFFER_OFFSET_FULL,
} BUFFER_StateTypeDef;

typedef enum{
	SOUND_INIT = 0,
	SOUND_IDLE,
	SOUND_PLAY
} SoundStateEnum;

typedef struct {
	uint32_t fileLength;
	uint32_t currentOffset;
	volatile SoundStateEnum soundState;
	const uint8_t* startAddress;
} DrumSoundStruct;

static uint8_t audioBuffer[AUDIO_BUFFER_SIZE] = {0};
static uint8_t *pBufferFirstHalf = &audioBuffer[0];
static uint8_t *pBufferSecondHalf = &audioBuffer[AUDIO_BUFFER_SIZE / 2];
static AUDIO_PLAYBACK_StateTypeDef audioState = AUDIO_STATE_IDLE;
static BUFFER_StateTypeDef audioBufferOffset = BUFFER_OFFSET_NONE;

DrumSoundStruct kick;
DrumSoundStruct crash;
DrumSoundStruct cowbell;
DrumSoundStruct hat;
DrumSoundStruct snare;
DrumSoundStruct tom;
DrumSoundStruct drumSet[NUMBER_OF_AUDIO_CHANNELS];

static void updateBufferFromFile(uint8_t *pBuffer);
static void initSounds(void);

void BSP_AUDIO_OUT_HalfTransfer_CallBack(void) {
	if (audioState != AUDIO_STATE_PLAYING) {
		return;
	}
	audioBufferOffset = PLAY_BUFFER_OFFSET_HALF;
//	BSP_AUDIO_OUT_ChangeBuffer(pBufferSecondHalf, AUDIO_BUFFER_SIZE / 2);
}

void BSP_AUDIO_OUT_TransferComplete_CallBack(void) {
	if (audioState != AUDIO_STATE_PLAYING) {
		return;
	}
	audioBufferOffset = PLAY_BUFFER_OFFSET_FULL;
//	BSP_AUDIO_OUT_ChangeBuffer(pBufferFirstHalf, AUDIO_BUFFER_SIZE / 2);
}

void initAudioCore(void){
	audioState = AUDIO_STATE_INIT;

	uint8_t uwVolume = 15;
	if (BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, uwVolume,
			SAI_AUDIO_FREQUENCY_48K) != AUDIO_OK) {
		return;
	}
	BSP_AUDIO_OUT_SetAudioFrameSlot(CODEC_AUDIOFRAME_SLOT_02);
	if (BSP_AUDIO_OUT_Play((uint16_t*)&audioBuffer, AUDIO_BUFFER_SIZE) != AUDIO_OK) {
		return;
	}

	initSounds();

	audioState = AUDIO_STATE_PLAYING;
}

void initSounds(void){
	WAVE_FormatTypeDef *waveformat = NULL;
	if(BSP_QSPI_Init() != QSPI_OK){
		return;
	}

	BSP_QSPI_MemoryMappedMode();
	WRITE_REG(QUADSPI->LPTR, 0xFFF);


	drumSet[0] = kick;
	drumSet[1] = crash;
	drumSet[2] = cowbell;
	drumSet[3] = hat;
	drumSet[4] = snare;
	drumSet[5] = tom;


	snare.soundState = SOUND_INIT;
	snare.startAddress = (uint8_t*) ADDRESS_CRASH;
	snare.currentOffset = sizeof(WAVE_FormatTypeDef);	// pass WAV header

	memcpy(waveformat, snare.startAddress, sizeof(WAVE_FormatTypeDef));

	snare.fileLength = waveformat->FileSize - 4000;	// TODO 4000 - strange number
	snare.soundState = SOUND_IDLE;

}

void drumPlaySound(void){
	if(snare.soundState == SOUND_IDLE){
		snare.soundState = SOUND_PLAY;
	}
}

static void updateBufferFromFile(uint8_t *pBuffer) {
	uint8_t currentBuffer[AUDIO_BUFFER_SIZE / 2] = {0};
	if (snare.soundState == SOUND_PLAY){
		// TODO create function from this
		if(snare.currentOffset + AUDIO_BUFFER_SIZE / 2 >= snare.fileLength){
			snare.soundState = SOUND_IDLE;
			snare.currentOffset = sizeof(WAVE_FormatTypeDef);
		}
		else{
			for (uint16_t inc = 0; inc < AUDIO_BUFFER_SIZE/2; inc++){
				currentBuffer[inc] += snare.startAddress[snare.currentOffset + inc];
			}
			snare.currentOffset += AUDIO_BUFFER_SIZE /2;
		}
	}
	memcpy(pBuffer, currentBuffer, AUDIO_BUFFER_SIZE/2);
}

void handleAudioStream(void) {
	switch (audioBufferOffset) {
	case PLAY_BUFFER_OFFSET_HALF:
		updateBufferFromFile(pBufferFirstHalf);
		break;
	case PLAY_BUFFER_OFFSET_FULL:
		updateBufferFromFile(pBufferSecondHalf);
		break;
	default:
		return;
	}
	audioBufferOffset = BUFFER_OFFSET_NONE;
}
