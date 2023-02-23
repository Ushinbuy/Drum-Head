#include "memmoryManager.h"

#include <string.h>

#define SECTOR_SIZE 0x10000
#define PAD_ADDRESS_START 0x0
#define ADDRESS_SYSTEM_START 0x0

typedef uint8_t byte;

uint8_t pageAddress[SECTOR_SIZE];
#define PADS_NUMBER 6
#define MAX_SOUNDS 256

static void writeToExternalFlash(uint8_t* pData, uint32_t WriteAddr, uint32_t Size);
void filFFvalues(uint64_t *pData, uint32_t size);

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
	  byte soundAddressItem;	//10 this field show which item from soundsAdresses
};

struct MemorySystem{
	uint8_t numberOfTotalPads = PADS_NUMBER;
	uint8_t soundsNumber;
	uint64_t padSavingsOffsets[PADS_NUMBER];
	uint32_t soundsAdresses[MAX_SOUNDS];
};

void eraseSector(void) {
	memset(pageAddress, 0xFF, SECTOR_SIZE);
}

void writeToExternalFlash(uint8_t* pData, uint32_t WriteAddr, uint32_t Size){
	// this emulate working of flash memory
	for(uint32_t i = 0; i < Size; i++){
		pageAddress[WriteAddr + i] &= pData[i];
	}

	for(int i = 2; i < 100; i++){
		printf ("\n%3d - 0x%X", i, pageAddress[WriteAddr + i]);
	}
}

void initMemmorySystem(void){
	MemorySystem _memSyst;
	_memSyst.numberOfTotalPads = PADS_NUMBER;
	_memSyst.soundsNumber = 2;

	filFFvalues(_memSyst.padSavingsOffsets, PADS_NUMBER);

	_memSyst.soundsAdresses[0] = 0x10;
	_memSyst.soundsAdresses[1] = 0x20;
	_memSyst.soundsAdresses[2] = 0x30;
	_memSyst.soundsAdresses[3] = 0x44;
	_memSyst.soundsAdresses[4] = 0x55;
	_memSyst.soundsAdresses[5] = 0x66;

	writeToExternalFlash((uint8_t *) & _memSyst, ADDRESS_SYSTEM_START, sizeof(MemorySystem));
}

void filFFvalues(uint64_t *pData, uint32_t size){
	for(uint32_t i = 0; i < size; i++){
		pData[i] = 0xFFFFFFFFFFFFFFFF;
	}
}

void mainWork(void)
{
	eraseSector();
	initMemmorySystem();
}
