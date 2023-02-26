#ifndef MEMMORYSYSTEM_H_
#define MEMMORYSYSTEM_H_

#include <stdint.h>
#include <stdio.h>

void mainWork(void); // remove this in release

typedef uint8_t byte;

#define SECTOR_SIZE 0x10000 // this must be taken from header QSPI

#define PADS_NUMBER 3		// max num of pads. It can be shared
#define MAX_SOUNDS 0x10 //256 - 1	// minus 1 mean that one sector of flash reserved by MemorySystem

// this struct must be taken from Hellodrum
struct PadMemory{
public:
	  byte sensitivity = 0x11;   //0
	  byte threshold1 = 0x12;     //1
	  byte scantime = 0x13;       //2
	  byte masktime = 0x14;       //3
	  byte rimSensitivity = 0x15; //4 edgeThreshold
	  byte rimThreshold = 0x16;    //5 cupThreshold
	  byte curvetype = 0x17;       //6
	  byte note = 0x18;           //7
	  byte noteRim = 0x19;        //8
	  byte noteCup = 0x20;        //9
	  byte soundAddressId = 0x21;	//10 this field show which item from soundsAdresses
};

struct MemmorySpace{
	// TODO change in some cases PADS_NUMBER to numberOfTotalPads
	uint8_t numberOfTotalPads = PADS_NUMBER;
	uint8_t soundsNumber;
	uint64_t padSavingsOffsets[PADS_NUMBER];
	uint32_t soundsAdresses[MAX_SOUNDS];
};

class MemmorySystem {
public:
	static MemmorySystem* getInstance() {
		static MemmorySystem instance;
		return &instance;
	}

    void eraseSector(void);
    void getPadInfo(PadMemory* padToWhereRead, uint8_t padNum);
    void getMemorySystem(void);
    void overridePad(PadMemory _pad, uint8_t padNum);
    void writePadFirstTime(PadMemory pad, uint8_t padNum);
    void showPageAddress(uint32_t startAddress, uint32_t stopAddress);
    void initMemmorySystem(void);
private:
    uint8_t pageAddress[SECTOR_SIZE];
    MemmorySpace mainMemSpace;

    MemmorySystem();

    uint32_t calcPadAddress(uint8_t padNum);
    void filFFvalues(uint64_t *pData, uint32_t size);
    void overrideMainMemSector(void);
    void writeToExternalFlash(uint8_t *pData, uint32_t WriteAddr,
			uint32_t Size);
	void readFromExternalFlash(uint8_t *pData, uint32_t ReadAddr,
			uint32_t Size);
};

#endif /* MEMMORYSYSTEM_H_ */
