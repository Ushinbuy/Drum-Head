#include <MemmorySystem.h>
#include <string.h>

#define SECTOR_SIZE 0x10000
#define PAD_ADDRESS_START 0x0
#define ADDRESS_SYSTEM_START 0x0

typedef uint8_t byte;

static uint8_t pageAddress[SECTOR_SIZE];
#define PADS_NUMBER 3		// max num of pads. It can be shared
#define MAX_SOUNDS 0x20 //256 - 1	// minus 1 mean that one sector of flash reserved by MemorySystem

static void writeToExternalFlash(uint8_t* pData, uint32_t WriteAddr, uint32_t Size);
static void filFFvalues(uint64_t *pData, uint32_t size);
static void readMemorySystem(void);
static void writePadInfo(uint8_t padNum);
static uint32_t calcPadAddress(uint8_t padNum);
static void overrideSector(void);

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

PadMemory readPadInfo(uint8_t padNum);

struct MemmorySpace{
	// TODO change in some cases PADS_NUMBER to numberOfTotalPads
	uint8_t numberOfTotalPads = PADS_NUMBER;
	uint8_t soundsNumber;
	uint64_t padSavingsOffsets[PADS_NUMBER];
	uint32_t soundsAdresses[MAX_SOUNDS];
};

void MemmorySystem::test(){
	printf("Singleton test");
}

MemmorySystem::MemmorySystem() { }

static MemmorySpace mainMemSpace;

void eraseSector(void) {
	// this imitate function
	memset(pageAddress, 0xFF, SECTOR_SIZE);
}

void writeToExternalFlash(uint8_t* pData, uint32_t WriteAddr, uint32_t Size){
	// change this function for QSPI_write
	for(uint32_t i = 0; i < Size; i++){
		pageAddress[WriteAddr + i] &= pData[i];
	}
}

void readFromExternalFlash(uint8_t* pData, uint32_t ReadAddr, uint32_t Size){
	// change this function for QSPI_read
	memcpy(pData, &pageAddress[ReadAddr], Size);
}

void initMemmorySystem(void){
	// this imitate function
	MemmorySpace _memSyst;
	_memSyst.numberOfTotalPads = PADS_NUMBER;
	_memSyst.soundsNumber = 2;

	filFFvalues(_memSyst.padSavingsOffsets, PADS_NUMBER);

	_memSyst.soundsAdresses[0] = 0x10;
	_memSyst.soundsAdresses[1] = 0x20;
	_memSyst.soundsAdresses[2] = 0x30;
	_memSyst.soundsAdresses[3] = 0x44;
	_memSyst.soundsAdresses[4] = 0x55;
	_memSyst.soundsAdresses[5] = 0x66;

	writeToExternalFlash((uint8_t *) & _memSyst, ADDRESS_SYSTEM_START, sizeof(MemmorySpace));
}

void writePadInfo(uint8_t padNum){
	PadMemory pad;
	pad.curvetype = 2;
	pad.soundAddressItem = 2;

	writeToExternalFlash((uint8_t*)&pad, calcPadAddress(padNum), sizeof(PadMemory));
}

PadMemory readPadInfo(uint8_t padNum){
	PadMemory pad;

	readFromExternalFlash((uint8_t *) &pad, calcPadAddress(padNum), sizeof(PadMemory));
	return pad;
}

uint32_t calcPadAddress(uint8_t padNum){
	uint64_t offsetInBytes = mainMemSpace.padSavingsOffsets[padNum];
//	offsetInBytes >>= 10;
	uint8_t numOfOverwrites = 0;
	while(offsetInBytes){
		numOfOverwrites++;
		offsetInBytes >>= 1;
	}
	numOfOverwrites = 64 - numOfOverwrites;

	return ADDRESS_SYSTEM_START + sizeof(MemmorySpace) + (padNum + numOfOverwrites * PADS_NUMBER) * sizeof(PadMemory);
}

void filFFvalues(uint64_t *pData, uint32_t size){
	// this imitate function
	for(uint32_t i = 0; i < size; i++){
		pData[i] = 0xFFFFFFFFFFFFFFFF;
	}
}

void readMemorySystem(void){
	readFromExternalFlash((uint8_t *)&mainMemSpace, ADDRESS_SYSTEM_START, sizeof(MemmorySpace));
}

void overwritePad(PadMemory _pad, uint8_t padNum){
	uint32_t address = calcPadAddress(padNum);
	printf("\n old address for pad is %d", address);


	mainMemSpace.padSavingsOffsets[padNum] >>= 1;
	if(mainMemSpace.padSavingsOffsets[padNum] == 0){
		overrideSector();
	}
	writeToExternalFlash((uint8_t *) &mainMemSpace, ADDRESS_SYSTEM_START, sizeof(MemmorySpace));

	address = calcPadAddress(padNum);
	printf("\n new address for pad is %d", address);

	writeToExternalFlash((uint8_t*)&_pad, address, sizeof(PadMemory));
}

void overrideSector(void){
	// TODO create var massive PadMemory
	filFFvalues(mainMemSpace.padSavingsOffsets, PADS_NUMBER);
	PadMemory tempBuffer[PADS_NUMBER];
	for(int i = 0; i < PADS_NUMBER; i++){
		tempBuffer[i] = readPadInfo(i);
	}

	eraseSector();
	writeToExternalFlash((uint8_t *) & mainMemSpace, ADDRESS_SYSTEM_START, sizeof(MemmorySpace));
	for(int i = 0; i < PADS_NUMBER; i++){
		overwritePad(tempBuffer[i], i);
	}
}

void mainWork(void)
{
	eraseSector();
	initMemmorySystem();
	readMemorySystem();

	// check memory system
	printf ("CHECK 0x%X", mainMemSpace.soundsAdresses[4]);

	writePadInfo(0);
	for(int i = calcPadAddress(0); i < calcPadAddress(0) + 11; i++){
		printf ("\naddress %d has value %d", i, pageAddress[i]);
	}

	PadMemory tempPad;
	readFromExternalFlash((uint8_t *)&tempPad, calcPadAddress(0), sizeof(PadMemory));

	printf ("\nCHECK %d", tempPad.curvetype);

	tempPad.masktime = 99;

	overwritePad(tempPad, 0);

//	PadMemory updatedPad;
	readFromExternalFlash((uint8_t *)&tempPad, calcPadAddress(0), sizeof(PadMemory));
	printf ("\nCHECK %d", tempPad.masktime);

	tempPad.note = 98;
	overwritePad(tempPad, 0);

	readFromExternalFlash((uint8_t *)&tempPad, calcPadAddress(0), sizeof(PadMemory));
	printf ("\nCHECK %d", tempPad.note);

	PadMemory tempPad1;
	for(int i = 1; i < PADS_NUMBER; i++){
		overwritePad(tempPad1, i);
	}

	printf("Try to override 64 times");

	printf("\n\nLet's look for memory before overriding\n");

	for(int i = 0; i< 1500; i++){
		if(i%16 == 15){
			printf("\n");
		}
		else{
			printf("0x%2X ", pageAddress[i]);
		}
	}

	for (uint8_t i = 0; i < 65; i++){
		overwritePad(tempPad, 2);
	}

	printf("\n\nLet's look for memory after overriding\n");

	for(int i = 0; i< 1500; i++){
		if(i%16 == 15){
			printf("\n");
		}
		else{
			printf("0x%2X ", pageAddress[i]);
		}
	}

	readFromExternalFlash((uint8_t *)&tempPad, calcPadAddress(0), sizeof(PadMemory));
	printf ("\nCHECK %d", tempPad.note);
}
