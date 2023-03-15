#ifndef EEPROMFORPAD_H_
#define EEPROMFORPAD_H_

//#include "MemmorySystem.h"
#include <stdint.h>
#include <stdio.h>

void eeprom_work(void); // remove this in release

#define MAX_SOUNDS 63 //256 - 1	// minus 1 mean that one sector of flash reserved by MemorySystem
#define SECTOR_SIZE 0x1000 // this must be taken from header QSPI
#define PADS_NUMBER 4

typedef uint8_t byte;

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
	  byte soundHeadAddressId = 0x21;	//10 this field show which item from soundsAdresses
	  byte soundRimAddressId = 0x22;	//11
	  byte soundCupAddressId = 0x23;	//12
	  float soundHeadVolumeDb = -3.0;
	  float soundRimVolumeDb = -3.0;
	  float soundCupVolumeDb = -3.0;
};

struct PadInEeprom{
	byte id;
	PadMemory pad;
};

struct InfoSector{
	uint8_t numberOfTotalPads;
	uint8_t soundsNumber;
	uint32_t soundsAdresses[MAX_SOUNDS];
};

class EepromManager {
public:
	static EepromManager* getInstance() {
		static EepromManager instance;
		return &instance;
	}

	void init(void);

	void loadInfoSector(void);
	void showPageAddress(uint32_t startAddress, uint32_t stopAddress);
	void writePad(PadInEeprom _pad);
	PadInEeprom readPad(uint8_t idPad);

private:
	EepromManager();

	InfoSector infoSector;
	uint8_t pageAddress[SECTOR_SIZE]; // THIS MUST BE CHANGE

	void erasePage(void);
	uint32_t calcPadAddress(uint8_t padNum);
	void writeToExternalFlash(uint8_t *pData, uint32_t WriteAddr, uint32_t Size);
	void readFromExternalFlash(uint8_t *pData, uint32_t ReadAddr, uint32_t Size);
	bool isAddressEmpty(uint8_t* bytesToCheck, uint32_t size);
	void writeInfoSector(void);
	PadInEeprom * findAllPads(void);
	void overrideSector(PadInEeprom lastUpdatedPad);
};


#endif /* EEPROMFORPAD_H_ */
