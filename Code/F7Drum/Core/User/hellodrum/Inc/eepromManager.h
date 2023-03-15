#ifndef EEPROMFORPAD_H_
#define EEPROMFORPAD_H_

#include "hellodrum.hpp"
#include <stdint.h>
#include <stdio.h>
#include "n25q128a.h"

void eeprom_work(void); // remove this in release

#define MAX_SOUNDS 63 // this value optimize for 0x10
#define SECTOR_SIZE N25Q128A_SECTOR_SIZE // this must be taken from header QSPI

struct InfoSector {
	uint8_t numberOfTotalPads;
	uint8_t soundsNumber;
	uint32_t soundsAdresses[MAX_SOUNDS];
};

struct PadInEeprom {
	uint8_t id;
	PadMemory pad;
};

class EepromManager {
public:
	static EepromManager* getInstance() {
		static EepromManager instance;
		return &instance;
	}

//	void init(void);

	void loadInfoSector(void);
//	void showPageAddress(uint32_t startAddress, uint32_t stopAddress);
	void writePad(PadInEeprom _pad);
	PadInEeprom readPad(uint8_t idPad);
	InfoSector infoSector;

private:
	EepromManager();

//	uint8_t pageAddress[SECTOR_SIZE]; // THIS MUST BE CHANGE

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
