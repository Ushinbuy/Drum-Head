#include "memmoryManager.h"

#include <string.h>

#define SECTOR_SIZE 0x10000
#define PAD_ADDRESS_START 0x0

typedef uint8_t byte;

uint8_t pageAddress[SECTOR_SIZE];
#define PADS_NUMBER 6

extern "C" void	*memset(void *__b, int __c, size_t __len);

static void writeToExternalFlash(uint8_t* pData, uint32_t WriteAddr, uint32_t Size);

struct sPadMemory{
public:
	  byte sensitivity = 100;   //0
	  byte threshold1 = 10;     //1
	  byte scantime = 10;       //2
	  byte masktime = 30;       //3
	  byte rimSensitivity = 20; //4 edgeThreshold
	  byte rimThreshold = 3;    //5 cupThreshold
	  byte curvetype = 0;       //6
	  byte note = 38;           //7
	  byte noteRim = 39;        //8
	  byte noteCup = 40;        //9
	  uint32_t noteSoundAddress;	//false TODO check the struct fields 10-13
};

struct sMemorySystem{
	uint8_t numberOfTotalPads = PADS_NUMBER;
	uint64_t sPadOffset[PADS_NUMBER];
};

void eraseSector(void) {
	memset(pageAddress, 0xFF, SECTOR_SIZE);

//	printf("\nmassive is filled 0x%X, 0x%X, 0x%X, 0x%X", pageAddress[0],
//			pageAddress[1], pageAddress[2], pageAddress[3]);
}

void writeToExternalFlash(uint8_t* pData, uint32_t WriteAddr, uint32_t Size){
	// this emulate working of flash memory
	for(uint32_t i = 0; i < Size; i++){
		pageAddress[WriteAddr + i] &= pData[i];
	}

	printf("\n value of noteCup %d", pageAddress[WriteAddr]);
	printf("\n description before address is 0x%X 0x%X",
			pageAddress[WriteAddr + 1], pageAddress[WriteAddr + 2]);
	printf("\n massive is copy 0x%X, 0x%X, 0x%X, 0x%X",
			pageAddress[WriteAddr + 3], pageAddress[WriteAddr + 4],
			pageAddress[WriteAddr + 5], pageAddress[WriteAddr + 6]);
//
//	uint32_t currentAddress = (pageAddress[WriteAddr + 15] << 8 * 3)
//			+ (pageAddress[WriteAddr + 14] << 8 * 2)
//			+ (pageAddress[WriteAddr + 13] << 8)
//			+ pageAddress[WriteAddr + 12];
//	printf("\n with address 0x%X", currentAddress);
//
//	printf("\n values after struct 0x%X 0x%X",
//				pageAddress[WriteAddr + 16], pageAddress[WriteAddr + 17]);
}

void setPadsNumber(uint8_t padsNumbers){
	TestClass test;
	writeToExternalFlash((uint8_t *) &test, PAD_ADDRESS_START, sizeof(TestClass));
}

void initMemory(uint8_t padNum)
{
  //Write initial value to EEPROM.

//	sPadMemory currentPad;
//	currentPad.noteSoundAddress = 0x87654321;
//	printf("\nPad number %d is initialize", padNum);
//
//	uint32_t currentPadAddress = PAD_ADDRESS_START + padNum * sizeof(sPadMemory);
//	writeToExternalFlash((uint8_t *) &currentPad, currentPadAddress, sizeof(sPadMemory));
}

void changeValueInFlash(void){
	TestClass test;
	writeToExternalFlash((uint8_t *) &test, PAD_ADDRESS_START, sizeof(TestClass));
}
