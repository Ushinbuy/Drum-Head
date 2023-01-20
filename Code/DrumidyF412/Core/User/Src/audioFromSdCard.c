#include "audioFromSdCard.h"

#define AUDIO_BUFFER_SIZE 960 	// must be equal to 20 ms * 48 kHz

static uint16_t audioBuffer[AUDIO_BUFFER_SIZE];
static uint32_t fileSize;
static uint32_t currentPosition;
FIL fil;

void BSP_AUDIO_OUT_Error_CallBack(void) {
	sendUart("Error in playing audio from sd card");
	Error_Handler();
}

uint16_t getBytesFromBuffer(void);

void BSP_AUDIO_OUT_HalfTransfer_CallBack(void) {
	uint16_t amountToRead = getBytesFromBuffer();
	BSP_AUDIO_OUT_ChangeBuffer(&audioBuffer[0], amountToRead);
}

void BSP_AUDIO_OUT_TransferComplete_CallBack(void) {
	uint16_t amountToRead = getBytesFromBuffer();
	BSP_AUDIO_OUT_ChangeBuffer(&audioBuffer[AUDIO_BUFFER_SIZE/2], amountToRead);
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
	if (f_read(&fil, audioBuffer, amountToRead, &bytesWasRead) != FR_OK) {
		sendUart("CAN'T READ FILE AT START");
	}
//	currentPosition = sizeof(struct WAVE_FormatTypeDef) + bytesWasRead;
	uint8_t uwVolume = 35;
	if (BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, uwVolume, I2S_AUDIOFREQ_8K)
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

uint16_t getBytesFromBuffer(void){
	UINT bytesWasRead;
	uint16_t amountToRead = fileSize - sizeof(struct WAVE_FormatTypeDef);
	if (amountToRead > AUDIO_BUFFER_SIZE / 2) {
		amountToRead = AUDIO_BUFFER_SIZE / 2;
	}
	if (f_read(&fil, audioBuffer, amountToRead, &bytesWasRead) != FR_OK) {
		sendUart("CAN'T READ FILE AT START");
	}
	currentPosition = sizeof(struct WAVE_FormatTypeDef) + bytesWasRead;
	return amountToRead;
}

void stopPlaying(void) {
	f_mount(&SDFatFS, (TCHAR const*) NULL, 0);
}