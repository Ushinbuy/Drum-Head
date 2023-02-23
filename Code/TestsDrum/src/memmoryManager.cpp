#include "memmoryManager.h"

#include <string.h>

#define SECTOR_SIZE 0x10000
#define PAD_ADDRESS_START 0x0
#define ADDRESS_SYSTEM_START 0x0

typedef uint8_t byte;

static uint8_t pageAddress[SECTOR_SIZE];
#define PADS_NUMBER 6
#define MAX_SOUNDS 256

static void writeToExternalFlash(uint8_t* pData, uint32_t WriteAddr, uint32_t Size);
static void filFFvalues(uint64_t *pData, uint32_t size);
static void readMemorySystem(void);
static void writePadInfo(uint8_t padNum);
uint32_t calcPadAddress(uint8_t padNum);

// this structure do not be used in main project
struct PadMemory{
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

static MemorySystem mainMemSyst;

void eraseSector(void) {
	// this imitate function
	memset(pageAddress, 0xFF, SECTOR_SIZE);
}

void writeToExternalFlash(uint8_t* pData, uint32_t WriteAddr, uint32_t Size){
	// change this function for QSPI_write
	for(uint32_t i = 0; i < Size; i++){
		pageAddress[WriteAddr + i] &= pData[i];
	}

//	for(int i = 2; i < 100; i++){
//		printf ("\n%3d - 0x%X", i, pageAddress[WriteAddr + i]);
//	}
}

void readFromExternalFlash(uint8_t* pData, uint32_t ReadAddr, uint32_t Size){
	// change this function for QSPI_read
	memcpy(pData, &pageAddress[ReadAddr], Size);
}

void initMemmorySystem(void){
	// this imitate function
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

void writePadInfo(uint8_t padNum){
	PadMemory pad;
	pad.curvetype = 2;
	pad.soundAddressItem = 2;

	writeToExternalFlash((uint8_t*)&pad, calcPadAddress(padNum), sizeof(PadMemory));
}

uint32_t calcPadAddress(uint8_t padNum){
	uint64_t offsetInBytes = mainMemSyst.padSavingsOffsets[padNum];
//	offsetInBytes >>= 10;
	uint8_t numOfOverwrites = 0;
	while(offsetInBytes){
		numOfOverwrites++;
		offsetInBytes >>= 1;
	}
	numOfOverwrites = 64 - numOfOverwrites;

	return ADDRESS_SYSTEM_START + sizeof(MemorySystem) + padNum * sizeof(PadMemory) * numOfOverwrites;
}

void filFFvalues(uint64_t *pData, uint32_t size){
	// this imitate function
	for(uint32_t i = 0; i < size; i++){
		pData[i] = 0xFFFFFFFFFFFFFFFF;
	}
}

void readMemorySystem(void){
	readFromExternalFlash((uint8_t *)&mainMemSyst, ADDRESS_SYSTEM_START, sizeof(MemorySystem));
}

void overwritePad(PadMemory _pad, uint8_t padNum){
	mainMemSyst.padSavingsOffsets[padNum] >>= 1;
	writeToExternalFlash((uint8_t *) &mainMemSyst, ADDRESS_SYSTEM_START, sizeof(MemorySystem));
	writeToExternalFlash((uint8_t*)&_pad, calcPadAddress(padNum), sizeof(PadMemory));
}

void mainWork(void)
{
	eraseSector();
	initMemmorySystem();
	readMemorySystem();

	// check memory system
	printf ("CHECK 0x%X", mainMemSyst.soundsAdresses[4]);

	writePadInfo(0);
	for(int i = calcPadAddress(0); i < calcPadAddress(0) + 11; i++){
		printf ("\naddress %d has value %d", i, pageAddress[i]);
	}

	PadMemory tempPad;
	readFromExternalFlash((uint8_t *)&tempPad, calcPadAddress(0), sizeof(tempPad));

	printf ("\nCHECK %d", tempPad.curvetype);

	tempPad.masktime = 99;

	overwritePad(tempPad, 0);

	PadMemory updatedPad;
	readFromExternalFlash((uint8_t *)&updatedPad, calcPadAddress(0), sizeof(tempPad));
	printf ("\nCHECK %d", updatedPad.masktime);
}
