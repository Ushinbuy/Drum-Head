#include "drumAudioCore.h"
#include "drumCore.h"
//#include "wm8994.h"
//#include "stm32746g_discovery_audio.h"
//#include "stm32746g_discovery_qspi.h"
//#include "wavFile.h"
#include "audio.h"
//#include <string.h>
//#include "velocityInDb.h"
//#include "dBinFloat.h"

extern char buffer_out[1000];

//#define AUDIO_BUFFER_SIZE 128 	// must be equal to 20 ms * 48 kHz
//#define NUMBER_OF_AUDIO_CHANNELS NUMBER_OF_CHANNELS + 0 // can be more, because have alternate sounds - rim, ride bell, etc.

//typedef enum {
//	AUDIO_STATE_IDLE = 0, AUDIO_STATE_INIT, AUDIO_STATE_PLAYING,
//} AUDIO_PLAYBACK_StateTypeDef;
//
//typedef enum {
//	BUFFER_OFFSET_NONE = 0,
//	PLAY_BUFFER_OFFSET_HALF,
//	PLAY_BUFFER_OFFSET_FULL,
//} BUFFER_StateTypeDef;

//static uint8_t audioBuffer[AUDIO_BUFFER_SIZE] = {0};
//static uint8_t *pBufferFirstHalf = &audioBuffer[0];
//static uint8_t *pBufferSecondHalf = &audioBuffer[AUDIO_BUFFER_SIZE / 2];
//static AUDIO_PLAYBACK_StateTypeDef audioState = AUDIO_STATE_IDLE;
//static BUFFER_StateTypeDef audioBufferOffset = BUFFER_OFFSET_NONE;

//DrumSoundStruct kick;
//DrumSoundStruct crash;
//DrumSoundStruct cowbell;
//DrumSoundStruct hat;
//DrumSoundStruct snare;
//DrumSoundStruct tom;
//DrumSoundStruct drumSet[NUMBER_OF_AUDIO_CHANNELS];

//static void updateBufferFromFile(uint8_t *pBuffer);
//static void stopPlaying(DrumSoundStruct *drum);
//static void mixingAudio(uint8_t mainBuffer[], const uint8_t addedSound[], float addedVolume);
//static void initSounds(void);

//void BSP_AUDIO_OUT_HalfTransfer_CallBack(void) {
//	if (audioState != AUDIO_STATE_PLAYING) {
//		return;
//	}
//	audioBufferOffset = PLAY_BUFFER_OFFSET_HALF;
////	BSP_AUDIO_OUT_ChangeBuffer(pBufferSecondHalf, AUDIO_BUFFER_SIZE / 2);
//}
//
//void BSP_AUDIO_OUT_TransferComplete_CallBack(void) {
//	if (audioState != AUDIO_STATE_PLAYING) {
//		return;
//	}
//	audioBufferOffset = PLAY_BUFFER_OFFSET_FULL;
////	BSP_AUDIO_OUT_ChangeBuffer(pBufferFirstHalf, AUDIO_BUFFER_SIZE / 2);
//}

//void initAudioCore(void){
//	audioState = AUDIO_STATE_INIT;
//
//	uint8_t uwVolume = 15;
//	if (BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, uwVolume,
//			SAI_AUDIO_FREQUENCY_48K) != AUDIO_OK) {
//		return;
//	}
//	BSP_AUDIO_OUT_SetAudioFrameSlot(CODEC_AUDIOFRAME_SLOT_02);
//	if (BSP_AUDIO_OUT_Play((uint16_t*)&audioBuffer, AUDIO_BUFFER_SIZE) != AUDIO_OK) {
//		return;
//	}
//
////	initSounds();
//	if(BSP_QSPI_Init() != QSPI_OK){
//		return;
//	}
//
//	BSP_QSPI_MemoryMappedMode();
//	WRITE_REG(QUADSPI->LPTR, 0xFFF);
//
//	audioState = AUDIO_STATE_PLAYING;
//}
//
//void stopAudioCore(){
//	audioState = AUDIO_STATE_IDLE;
//	BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);
//	// TODO maybe here need just only pause?
//}

//void initSounds(void){
//	WAVE_FormatTypeDef *waveformat = NULL;
//	if(BSP_QSPI_Init() != QSPI_OK){
//		return;
//	}
//
//	BSP_QSPI_MemoryMappedMode();
//	WRITE_REG(QUADSPI->LPTR, 0xFFF);


//	drumSet[0] = kick;
//	drumSet[1] = crash;
//	drumSet[2] = cowbell;
//	drumSet[3] = hat;
//	drumSet[4] = snare;
//	drumSet[5] = tom;


//	snare.soundState = SOUND_INIT;
//	snare.startAddress = (uint8_t*) ADDRESS_SNARE;
//	snare.currentOffset = sizeof(WAVE_FormatTypeDef);	// pass WAV header
//	memcpy(waveformat, snare.startAddress, sizeof(WAVE_FormatTypeDef));
//
//	snare.fileLength = waveformat->FileSize;
//	snare.userVolumeDB = -6.0f;
//	snare.soundState = SOUND_IDLE;
//
//
//	kick.soundState = SOUND_INIT;
//	kick.startAddress = (uint8_t*) ADDRESS_KICK;
//	kick.currentOffset = sizeof(WAVE_FormatTypeDef);	// pass WAV header
//	memcpy(waveformat, kick.startAddress, sizeof(WAVE_FormatTypeDef));
//
//	kick.fileLength = waveformat->FileSize;
//	kick.userVolumeDB = 3.0f;
//	kick.soundState = SOUND_IDLE;
//
//
//	crash.soundState = SOUND_INIT;
//	crash.startAddress = (uint8_t*) ADDRESS_CRASH;
//	crash.currentOffset = sizeof(WAVE_FormatTypeDef);	// pass WAV header
//	memcpy(waveformat, crash.startAddress, sizeof(WAVE_FormatTypeDef));
//
//	crash.fileLength = waveformat->FileSize;
//	crash.userVolumeDB = -3.0f;
//	crash.soundState = SOUND_IDLE;
//}

//void drumPlayDebugSounds(void){
//	playSound(&snare, 100);
//	playSound(&kick, 120);
//	playSound(&crash, 50);
//}
//
//void playSound(DrumSoundStruct *drum, uint8_t velocity){
//	drum->currentVolumeFloat = convertDbInFloat(velToDb(velocity) + drum->userVolumeDB);
//	drum->currentOffset = sizeof(WAVE_FormatTypeDef);
//	drum->soundState = SOUND_PLAY;
//}

//static void updateBufferFromFile(uint8_t *pBuffer) {
//	uint8_t currentBuffer[AUDIO_BUFFER_SIZE / 2] = { 0 };
//	if (snare.soundState == SOUND_PLAY) {
//		if (snare.currentOffset + AUDIO_BUFFER_SIZE / 2 >= snare.fileLength) {
//			stopPlaying(&snare);
//		}
//		else {
//			mixingAudio(currentBuffer, &snare.startAddress[snare.currentOffset], snare.currentVolumeFloat);
//			snare.currentOffset += AUDIO_BUFFER_SIZE / 2;
//		}
//	}
//
//	if (kick.soundState == SOUND_PLAY) {
//		if (kick.currentOffset + AUDIO_BUFFER_SIZE / 2 >= kick.fileLength) {
//			stopPlaying(&kick);
//		}
//		else {
//			mixingAudio(currentBuffer, &kick.startAddress[kick.currentOffset], kick.currentVolumeFloat);
//			kick.currentOffset += AUDIO_BUFFER_SIZE / 2;
//		}
//	}
//
//	if (crash.soundState == SOUND_PLAY) {
//		if (crash.currentOffset + AUDIO_BUFFER_SIZE / 2 >= crash.fileLength) {
//			stopPlaying(&crash);
//		}
//		else {
//			mixingAudio(currentBuffer, &crash.startAddress[crash.currentOffset], crash.currentVolumeFloat);
//			crash.currentOffset += AUDIO_BUFFER_SIZE / 2;
//		}
//	}
//
//	memcpy(pBuffer, currentBuffer, AUDIO_BUFFER_SIZE / 2);
//}

//void mixingAudio(uint8_t mainBuffer[], const uint8_t addedSound[], float addedVolume){
//	for (uint16_t inc = 0; inc < AUDIO_BUFFER_SIZE / 2; inc += 2) {
//		int16_t mainInt = (mainBuffer[inc+1] << 8) | mainBuffer[inc];
//		int16_t addedInt = (addedSound[inc+1] << 8) | addedSound[inc];
//		int16_t summ = mainInt + (int16_t)(addedInt*addedVolume);
//		mainBuffer[inc] = summ & 0xFF;
//		mainBuffer[inc+1] = summ >> 8;
//	}
//}

//void stopPlaying(DrumSoundStruct *drum){
//	drum->soundState = SOUND_IDLE;
//	drum->currentOffset = sizeof(WAVE_FormatTypeDef);
//}
//
//void changeVolume(DrumSoundStruct *drum, float newVolumeDB){
//	drum->userVolumeDB = newVolumeDB;
//}

//void handleAudioStream(void) {
//	if (audioState != AUDIO_STATE_PLAYING)
//		return;
//	switch (audioBufferOffset) {
//	case PLAY_BUFFER_OFFSET_HALF:
//		updateBufferFromFile(pBufferFirstHalf);
//		break;
//	case PLAY_BUFFER_OFFSET_FULL:
//		updateBufferFromFile(pBufferSecondHalf);
//		break;
//	default:
//		return;
//	}
//	audioBufferOffset = BUFFER_OFFSET_NONE;
//}
