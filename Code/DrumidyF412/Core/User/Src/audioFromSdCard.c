#include "audioFromSdCard.h"
#include <stdio.h>

#define AUDIO_BUFFER_SIZE 2048*16 	// must be equal to 20 ms * 48 kHz

uint16_t audioBuffer[AUDIO_BUFFER_SIZE];
static uint32_t fileSize;
static uint32_t offset;
static FIL fil;

// TODO to know difference between uint16_t and uint8_t for playing in SAI and i2S
static uint16_t *pBufferFirstHalf = (uint16_t*) &audioBuffer[0];
static uint16_t *pBufferSecondHalf = (uint16_t*) &audioBuffer[AUDIO_BUFFER_SIZE / 2];

uint16_t updateBufferFromFile(uint16_t *pBuffer);
void playAudioSd(FILINFO fno);
void stopPlaying(void);

typedef enum {
	AUDIO_STATE_IDLE = 0, AUDIO_STATE_INIT, AUDIO_STATE_PLAYING,
} AUDIO_PLAYBACK_StateTypeDef;

typedef enum {
	BUFFER_OFFSET_NONE = 0,
	PLAY_BUFFER_OFFSET_HALF,
	PLAY_BUFFER_OFFSET_FULL,
} BUFFER_StateTypeDef;

char buffer_out[1000];			// USB Buffers

BUFFER_StateTypeDef audioBufferOffset = BUFFER_OFFSET_NONE;
AUDIO_PLAYBACK_StateTypeDef audioState = AUDIO_STATE_IDLE;

void BSP_AUDIO_OUT_HalfTransfer_CallBack(void) {
	if (audioState != AUDIO_STATE_PLAYING) {
		return;
	}
	audioBufferOffset = PLAY_BUFFER_OFFSET_HALF;
	BSP_AUDIO_OUT_ChangeBuffer(pBufferSecondHalf, AUDIO_BUFFER_SIZE / 2);
}

void BSP_AUDIO_OUT_TransferComplete_CallBack(void) {
	if (audioState != AUDIO_STATE_PLAYING) {
		return;
	}
	audioBufferOffset = PLAY_BUFFER_OFFSET_FULL;
	BSP_AUDIO_OUT_ChangeBuffer(pBufferFirstHalf, AUDIO_BUFFER_SIZE / 2);
}

/**
 * Place this method in main stream
 */
void handleAudioStream(void) {
	uint16_t amountWasRead;
	switch (audioBufferOffset) {
	case PLAY_BUFFER_OFFSET_HALF:
		amountWasRead = updateBufferFromFile(pBufferFirstHalf);
		sendUart("\r\n1st half callback");
		break;
	case PLAY_BUFFER_OFFSET_FULL:
		amountWasRead = updateBufferFromFile(pBufferSecondHalf);
		sendUart("\r\n2nd half callback");
		break;
	default:
		return;
	}
	audioBufferOffset = BUFFER_OFFSET_NONE;
	if (amountWasRead < AUDIO_BUFFER_SIZE / 2) {
		stopPlaying();
	}
}

extern char SDPath[4]; /* SD logical drive path */
extern FATFS SDFatFS;

void sdCardTextExample(void) {
	FRESULT res;
	DIR dir;
	FILINFO fno;

	res = f_mount(&SDFatFS, SDPath, 0);
	if (res != FR_OK) {
		return;
	}

	res = f_opendir(&dir, SDPath);
	if (res != FR_OK) {
		sendUart("SD CARD NOT DETECTED");
		return;
	}

	while (1) {
		res = f_readdir(&dir, &fno);
		if (res != FR_OK || fno.fname[0] == 0)
			return;

		char *filename = fno.fname;

		if (strstr(filename, ".WAV") || (strstr(filename, ".wav")) != 0) {
			playAudioSd(fno);
			break;
		}
	}
	f_closedir(&dir);
}

void playAudioSd(FILINFO fno) {
	FRESULT res;
	WAVE_FormatTypeDef header;
	UINT count = 0;

	// TODO optimize this section it must be separated
	res = f_open(&fil, fno.fname, FA_READ);
	if (res != FR_OK)
		return;

	res = f_read(&fil, &header, sizeof(struct WAVE_FormatTypeDef), &count);
	if (res != FR_OK) {
		sendUart("CAN'T READ AUDIOFILE");
		return;
	}

	fileSize = header.FileSize;
	audioState = AUDIO_STATE_INIT;

	uint8_t uwVolume = 15;
	if (BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, uwVolume,
			header.SampleRate) != AUDIO_OK) {
		return;
	}

	UINT bytesWasRead;

	if (f_read(&fil, pBufferFirstHalf, AUDIO_BUFFER_SIZE, &bytesWasRead)
			!= FR_OK) {
		sendUart("CAN'T READ FILE AT START");
		return;
	}

	audioState = AUDIO_STATE_PLAYING;
	if (BSP_AUDIO_OUT_Play((uint16_t*)&audioBuffer, AUDIO_BUFFER_SIZE) != AUDIO_OK) {
		sendUart("ERROR START TO PLAY AUDIO");
		return;
	}
	offset = sizeof(struct WAVE_FormatTypeDef) + AUDIO_BUFFER_SIZE;
	sendUart("Header is read correctly \n\r");
}

uint16_t updateBufferFromFile(uint16_t *pBuffer) {
	UINT bytesWasRead;
	if (f_read(&fil, pBuffer, AUDIO_BUFFER_SIZE / 2, &bytesWasRead) != FR_OK) {
		stopPlaying();
		sendUart("CAN'T READ FILE AT START");
	}
	if (bytesWasRead == 0){
		stopPlaying();
	}
	offset = fil.fptr;
	return bytesWasRead;
}

void stopPlaying(void) {
	if (audioState != AUDIO_STATE_IDLE) {
		audioState = AUDIO_STATE_IDLE;
		BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);
		f_mount(&SDFatFS, (TCHAR const*) NULL, 0);
	}
}
