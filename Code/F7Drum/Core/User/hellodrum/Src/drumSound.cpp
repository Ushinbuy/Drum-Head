#include "drumSound.hpp"
#include "wavFile.h"
#include "dBinFloat.h"
#include "velocityInDb.h"
#include <string.h>
#include <vector>
#include <algorithm>
#include "wm8994.h"
#include "stm32746g_discovery_audio.h"
#include "stm32746g_discovery_qspi.h"

std::vector< DrumSound *> soundsList;

#define AUDIO_BUFFER_SIZE 128*2 	// must be equal to 20 ms * 48 kHz // TODO minimize this

static void mixingAudio(uint8_t mainBuffer[], const uint8_t addedSound[], float addedVolume);

typedef enum {
	AUDIO_STATE_IDLE = 0, AUDIO_STATE_INIT, AUDIO_STATE_PLAYING,
} AUDIO_PLAYBACK_StateTypeDef;

typedef enum {
	BUFFER_OFFSET_NONE = 0,
	PLAY_BUFFER_OFFSET_HALF,
	PLAY_BUFFER_OFFSET_FULL,
} BUFFER_StateTypeDef;

static uint8_t audioBuffer[AUDIO_BUFFER_SIZE] = {0};
static uint8_t *pBufferFirstHalf = &audioBuffer[0];
static uint8_t *pBufferSecondHalf = &audioBuffer[AUDIO_BUFFER_SIZE / 2];
volatile static AUDIO_PLAYBACK_StateTypeDef audioState = AUDIO_STATE_IDLE;
volatile static BUFFER_StateTypeDef audioBufferOffset = BUFFER_OFFSET_NONE;

DrumSound::DrumSound(uint32_t soundAddress, float soundVolumeDb){
	WAVE_FormatTypeDef *waveformat = NULL;
	soundState = SOUND_INIT;
	startAddress = (uint8_t*) soundAddress;
	currentOffset = sizeof(WAVE_FormatTypeDef);	// pass WAV header
	memcpy(waveformat, startAddress, sizeof(WAVE_FormatTypeDef));

	fileLength = waveformat->FileSize;
	userVolumeDB = soundVolumeDb;
	soundState = SOUND_IDLE;

	soundsList.push_back(this);
}

DrumSound::~DrumSound() {
	soundsList.erase(std::remove(soundsList.begin(), soundsList.end(), this),
			soundsList.end());
}

void DrumSound::setUserVolumeDb(float _newValue){
	userVolumeDB = _newValue;
}

void DrumSound::playSound(uint8_t velocity){
	currentVolumeFloat = convertDbInFloat(velToDb(velocity) + userVolumeDB);
	currentOffset = sizeof(WAVE_FormatTypeDef);
	soundState = SOUND_PLAY;
}

void DrumSound::stopPlaying(void){
	soundState = SOUND_IDLE;
	currentOffset = sizeof(WAVE_FormatTypeDef);
}

void DrumSound::updateBuffer(uint8_t *mainBuffer) {
	if (soundState == SOUND_PLAY) {
		if (currentOffset + AUDIO_BUFFER_SIZE / 2 >= fileLength) {
			stopPlaying();
		} else {
			mixingAudio(mainBuffer, &startAddress[currentOffset],
					currentVolumeFloat);
			currentOffset += AUDIO_BUFFER_SIZE / 2;
		}
	}
}

void mixingAudio(uint8_t mainBuffer[], const uint8_t addedSound[], float addedVolume){
	for (uint16_t inc = 0; inc < AUDIO_BUFFER_SIZE / 2; inc += 2) {
		int16_t mainInt = (mainBuffer[inc+1] << 8) | mainBuffer[inc];
		int16_t addedInt = (addedSound[inc+1] << 8) | addedSound[inc];
		int16_t summ = mainInt + (int16_t)(addedInt*addedVolume);
		mainBuffer[inc] = summ & 0xFF;
		mainBuffer[inc+1] = summ >> 8;
	}
}

void updateAudioBuffer(uint8_t *pBuffer){
	uint8_t currentBuffer[AUDIO_BUFFER_SIZE / 2] = { 0 };

	for(uint8_t i = 0; i < soundsList.size(); i++){
		soundsList[i]->updateBuffer(currentBuffer);
	}
	memcpy(pBuffer, currentBuffer, AUDIO_BUFFER_SIZE / 2);
}

void DrumSound::handleAudioStream(void) {
	if (audioState != AUDIO_STATE_PLAYING)
		return;
	switch (audioBufferOffset) {
	case PLAY_BUFFER_OFFSET_HALF:
		updateAudioBuffer(pBufferFirstHalf);
		break;
	case PLAY_BUFFER_OFFSET_FULL:
		updateAudioBuffer(pBufferSecondHalf);
		break;
	default:
		return;
	}
	audioBufferOffset = BUFFER_OFFSET_NONE;
}

void BSP_AUDIO_OUT_HalfTransfer_CallBack(void) {
	if (audioState != AUDIO_STATE_PLAYING) {
		return;
	}
	audioBufferOffset = PLAY_BUFFER_OFFSET_HALF;
}

void BSP_AUDIO_OUT_TransferComplete_CallBack(void) {
	if (audioState != AUDIO_STATE_PLAYING) {
		return;
	}
	audioBufferOffset = PLAY_BUFFER_OFFSET_FULL;
}

void DrumSound::initAudioCore(void){
	audioState = AUDIO_STATE_INIT;

	uint8_t uwVolume = 50;
	if (BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, uwVolume,
			SAI_AUDIO_FREQUENCY_48K) != AUDIO_OK) {
		return;
	}
	BSP_AUDIO_OUT_SetAudioFrameSlot(CODEC_AUDIOFRAME_SLOT_02);
	if (BSP_AUDIO_OUT_Play((uint16_t*)&audioBuffer, AUDIO_BUFFER_SIZE) != AUDIO_OK) {
		return;
	}

	if(BSP_QSPI_Init() != QSPI_OK){
		return;
	}

	BSP_QSPI_MemoryMappedMode();
	WRITE_REG(QUADSPI->LPTR, 0xFFF);

	audioState = AUDIO_STATE_PLAYING;
}

void stopAudioCore(){
	if(audioState == AUDIO_STATE_IDLE)
		return;
	audioState = AUDIO_STATE_IDLE;
	BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);
	// TODO maybe here need just only pause?
}
