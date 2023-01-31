#include "eepromSettings.h"
#include "main.h"
#include "drumCore.h"
#include "uartManage.h"

#define FLASH_USER_START_ADDR 	0x08040000	//0x0804 0000 		//0x0801 F800
const volatile uint32_t *userConfig=(const volatile uint32_t *)FLASH_USER_START_ADDR;

static uint32_t saved_config[64];
extern DRUM channel[NUMBER_OF_CHANNELS];

uint8_t Save_Setting(uint8_t _rst)
{
	uint32_t SavingBuff[64];
	uint8_t i;
	uint32_t error = 0;
	uint64_t val = 0;

	FLASH_EraseInitTypeDef FLASH_EraseInitStruct = {
			.TypeErase = FLASH_TYPEERASE_SECTORS,
//			.Banks = FLASH_BANK_1,
			.Sector = 6,
			.NbSectors = 1
	};

	for (i = 0; i < 64; i++)
		SavingBuff[i] = 0;
	if (_rst == 0)
		SavingBuff[0] = 0xC4C0FFEE; // load settings marker
	else
		SavingBuff[0] = 0xFFFFFFFF; // do not load marker
	SavingBuff[1] = 0xBB;

	for (i = 1; i < 10; i++) {
		// channel configuration settings
		SavingBuff[2 * i] = (channel[i - 1].main_voice & 0xFF) * 0x01000000;
		SavingBuff[2 * i] += (channel[i - 1].aux_voice & 0xFF) * 0x00010000;
		SavingBuff[2 * i] += (channel[i - 1].alt_voice & 0xFF) * 0x00000100;
		SavingBuff[2 * i] += (channel[i - 1].chnl_type & 0xFF);
		// channel parameter settings
		SavingBuff[2 * i + 1] = (channel[i - 1].peak_volume_norm & 0xFF) * 0x01000000;
		SavingBuff[2 * i + 1] += (channel[i - 1].peak_min_length & 0xFF) * 0x00010000;
		SavingBuff[2 * i + 1] += (channel[i - 1].peak_max_length & 0xFF) * 0x00000100;
//		SavingBuff[2*i + 1] += (channel[i-1].peak2peak  & 0xFF);
	}

	HAL_StatusTypeDef err;
	uint8_t st = 0;
	err = HAL_FLASH_Unlock();
	if (err != HAL_OK)
		st += 0b10000000;
	__HAL_FLASH_CLEAR_FLAG (FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR
							| FLASH_FLAG_PGAERR | FLASH_SR_ERSERR);

	err = HAL_FLASHEx_Erase(&FLASH_EraseInitStruct, &error);
	if (err != HAL_OK)
		st += 0b01000000;

	for (i = 0; i < 32; i++) {
		val = (((uint64_t) SavingBuff[i * 2 + 1]) << 32) + SavingBuff[i * 2];
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
				FLASH_USER_START_ADDR + 8 * i, val) != HAL_OK)
			st += 1;
	}
	if (HAL_FLASH_Lock() != HAL_OK)
		st += 0b00100000;

	for (i = 0; i < 64; i++)
		saved_config[0] = 0;

	return st;
}

uint8_t Load_Setting()
{
	uint8_t i;

	for (i=0;i<64;i++){
		saved_config[i] = *(userConfig+i);
	}

	if (saved_config[0] != 0xC4C0FFEE) return 0;

	for (i = 1; i < 10; i++){
		channel[i-1].main_voice = 0xff & (uint8_t)(saved_config[2*i]>>24);
		channel[i-1].aux_voice 	= 0xff & (uint8_t)(saved_config[2*i]>>16);
		channel[i-1].alt_voice 	= 0xff & (uint8_t)(saved_config[2*i]>>8);
		channel[i-1].chnl_type 	= 0xff & (uint8_t)(saved_config[2*i]);

		//		channel[i-1].peak_threshold 	= 0xff & (uint8_t)(saved_config[2*i+1]>>24);
		channel[i-1].peak_volume_norm 	= 0xff & (uint8_t)(saved_config[2*i+1]>>24);
		channel[i-1].peak_min_length 	= 0xff & (uint8_t)(saved_config[2*i+1]>>16);
		channel[i-1].peak_max_length 	= 0xff & (uint8_t)(saved_config[2*i+1]>>8);
//		channel[i-1].time_between_peaks = 0xff & (uint8_t)(saved_config[2*i+1]);
	}

	sprintf(buffer_out, "........ Previous settings: .......\n%08lX %08lX %08lX %08lX\n%08lX %08lX %08lX %08lX\n%08lX %08lX %08lX %08lX\n%08lX %08lX %08lX %08lX\n%08lX %08lX %08lX %08lX\n",
	  saved_config[0] ,saved_config[1] ,saved_config[2] ,saved_config[3] ,
	  saved_config[4] ,saved_config[5] ,saved_config[6] ,saved_config[7] ,
	  saved_config[8] ,saved_config[9] ,saved_config[10],saved_config[11],
	  saved_config[12],saved_config[13],saved_config[14],saved_config[15],
	  saved_config[16],saved_config[17],saved_config[18],saved_config[19]);

		sendUart(buffer_out);
	  HAL_Delay(500);

	return 1;
}
