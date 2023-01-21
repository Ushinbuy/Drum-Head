#include "audioFromSdCard.h"

#define AUDIO_BUFFER_SIZE 960 	// must be equal to 20 ms * 48 kHz

static uint16_t audioBuffer[AUDIO_BUFFER_SIZE];
static uint32_t fileSize;
static uint32_t offset;
static FIL fil;

static uint16_t* pBufferFirstHalf = &audioBuffer[0];
static uint16_t* pBufferSecondHalf = &audioBuffer[AUDIO_BUFFER_SIZE/2];

uint16_t updateBufferFromFile(uint16_t* pBuffer);
void playAudioSd(FILINFO fno);
void stopPlaying(void);

void BSP_AUDIO_OUT_HalfTransfer_CallBack(void) {
	if (offset < fileSize) {
		uint16_t amountToRead = updateBufferFromFile(pBufferFirstHalf);
		BSP_AUDIO_OUT_ChangeBuffer(pBufferFirstHalf, amountToRead);
	}
}

void BSP_AUDIO_OUT_TransferComplete_CallBack(void) {
	if(offset < fileSize){
		uint16_t amountToRead = updateBufferFromFile(pBufferSecondHalf);
		BSP_AUDIO_OUT_ChangeBuffer(pBufferSecondHalf, amountToRead);
	}
	else{
		stopPlaying();
	}
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

	UINT bytesWasRead;
	uint16_t amountToRead = fileSize - sizeof(struct WAVE_FormatTypeDef);
	if (amountToRead > AUDIO_BUFFER_SIZE) {
		amountToRead = AUDIO_BUFFER_SIZE;
	}
	offset = sizeof(struct WAVE_FormatTypeDef);
	if (f_read(&fil, audioBuffer, amountToRead, &bytesWasRead) != FR_OK) {
		sendUart("CAN'T READ FILE AT START");
	}
//	currentPosition = sizeof(struct WAVE_FormatTypeDef) + bytesWasRead;
	uint8_t uwVolume = 25;
	if (BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, uwVolume, I2S_AUDIOFREQ_48K)
			!= AUDIO_OK) {
		return;
	}

	if (BSP_AUDIO_OUT_Play(audioBuffer, amountToRead) != AUDIO_OK) {
		sendUart("ERROR START TO PLAY AUDIO");
		return;
	}
	sendUart("Header is read correctly \n\r");
}

void sdCardTextExample(void) {
	FRESULT res;
	DIR dir;
	FILINFO fno;

	res = f_mount(&SDFatFS, "", 0);
	if (res != FR_OK) {
		sendUart("SD CARD NOT DETECTED");
		return;
	}

	res = f_opendir(&dir, "");
	if (res != FR_OK)
		return; // EXIT_FAILURE;

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

uint16_t updateBufferFromFile(uint16_t* pBuffer){
	UINT bytesWasRead;
	uint16_t amountToRead = fileSize - sizeof(struct WAVE_FormatTypeDef);
	if (amountToRead > AUDIO_BUFFER_SIZE / 2) {
		amountToRead = AUDIO_BUFFER_SIZE / 2;
	}
	if (f_read(&fil, pBuffer, amountToRead, &bytesWasRead) != FR_OK) {
		sendUart("CAN'T READ FILE AT START");
	}
	offset += amountToRead;
	return amountToRead;
}

void stopPlaying(void) {
	BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);
	f_mount(&SDFatFS, (TCHAR const*) NULL, 0);
}
