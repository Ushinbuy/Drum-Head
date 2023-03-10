#include <eepromManager.h>
#include <string.h>
#include "stm32746g_discovery_qspi.h"

#define ADDRESS_SYSTEM_START 0x0

EepromManager::EepromManager(){}

void EepromManager::erasePage(void) {
	// this imitate function
//	memset(pageAddress, 0xFF, SECTOR_SIZE);
	BSP_QSPI_Erase_Block(0);
}

//void EepromManager::showPageAddress(uint32_t startAddress, uint32_t stopAddress){
//	for (uint32_t i = startAddress; i < stopAddress; i++) {
//		if (i % 0x10 == 0) {
//			printf("\n0x%03x : ", i);
//		}
//		printf("0x%02X ", pageAddress[i]);
//	}
//	printf("\n");
//}

//void EepromManager::init(void){
//	// this imitate function
//// /*
//	InfoSector _memSyst;
//
//	_memSyst.numberOfTotalPads = PADS_NUMBER;
//	_memSyst.soundsNumber = 2;
//
//	erasePage();
//
//	_memSyst.soundsAdresses[0] = 0x10;
//	_memSyst.soundsAdresses[1] = 0x20;
//	_memSyst.soundsAdresses[2] = 0x30;
//	_memSyst.soundsAdresses[3] = 0x44;
//	_memSyst.soundsAdresses[4] = 0x55;
//	_memSyst.soundsAdresses[5] = 0x66;
//
//	writeToExternalFlash((uint8_t *) & _memSyst, ADDRESS_SYSTEM_START, sizeof(InfoSector));
//// */
//
//}

void EepromManager::loadInfoSector(void){
	readFromExternalFlash((uint8_t *)&infoSector, ADDRESS_SYSTEM_START, sizeof(InfoSector));
}

void EepromManager::writeToExternalFlash(uint8_t* pData, uint32_t WriteAddr, uint32_t Size){
//	for(uint32_t i = 0; i < Size; i++){
//		pageAddress[WriteAddr + i] &= pData[i];
//	}
	BSP_QSPI_Write(pData, WriteAddr, Size);
}

void EepromManager::writePad(PadInEeprom _pad){
	PadInEeprom tempPad;
	uint8_t padReadedCounter = 0;
	bool isEmptyField = false;
	const uint32_t startAddress = ADDRESS_SYSTEM_START + sizeof(InfoSector);
	uint32_t currentAddress;
	do{
		currentAddress =  startAddress + padReadedCounter * sizeof(PadInEeprom);
		if(currentAddress + sizeof(PadInEeprom) < ADDRESS_SYSTEM_START + SECTOR_SIZE){
			readFromExternalFlash((uint8_t *)&tempPad, currentAddress, sizeof(PadInEeprom));
			isEmptyField = isAddressEmpty((uint8_t *)&tempPad, sizeof(PadInEeprom));
			padReadedCounter++;
		}
		else{
			overrideSector(_pad);
			currentAddress = startAddress;
			isEmptyField = true;
		}
	} while(!isEmptyField);
	writeToExternalFlash((uint8_t *) & _pad, currentAddress, sizeof(PadInEeprom));
}

void EepromManager::overrideSector(PadInEeprom lastUpdatedPad){
	PadInEeprom* padsArray = findAllPads();
	erasePage();
	writeInfoSector();
	for(uint8_t i = 0; i < infoSector.numberOfTotalPads; i++){
		if(lastUpdatedPad.id == padsArray[i].id){
			padsArray[i] = lastUpdatedPad;
		}
		writePad(padsArray[i]);
	}
}

/**
 * @param idPad is not in memory than it write default value
 * @retval return pad info from memory
 */
PadInEeprom EepromManager::readPad(uint8_t idPad) {
	PadInEeprom tempPad;
	PadInEeprom lastIdenticalPad;
	uint8_t padReadedCounter = 0;
	uint8_t counterOfFinded = 0;
	bool isEmptyField = false;
	const uint32_t startAddress = ADDRESS_SYSTEM_START + sizeof(InfoSector);
	uint32_t currentAddress;
	do {
		currentAddress = startAddress + padReadedCounter * sizeof(PadInEeprom);
		if((currentAddress + sizeof(PadInEeprom)) > (ADDRESS_SYSTEM_START + SECTOR_SIZE))
			break;
		readFromExternalFlash((uint8_t*) &tempPad, currentAddress, sizeof(PadInEeprom));
		if(tempPad.id == idPad){
			lastIdenticalPad = tempPad;
			counterOfFinded++;
		}
		isEmptyField = isAddressEmpty((uint8_t*) &tempPad, sizeof(PadInEeprom));
		padReadedCounter++;
	} while (!isEmptyField);
	if(counterOfFinded == 0){
		PadInEeprom newPad;
		newPad.id = idPad;
		writePad(newPad);
		return newPad;
	}
	return lastIdenticalPad;
}

/**
 * This function write InfoSector only after erasePage()
 */
void EepromManager::writeInfoSector(void){
	InfoSector mustBeeEmpty;
	readFromExternalFlash((uint8_t *)&mustBeeEmpty, ADDRESS_SYSTEM_START, sizeof(InfoSector));
	if(isAddressEmpty((uint8_t *)&mustBeeEmpty, sizeof(InfoSector))){
		writeToExternalFlash((uint8_t *)&infoSector, ADDRESS_SYSTEM_START, sizeof(InfoSector));
	}
}

bool EepromManager::isAddressEmpty(uint8_t* bytesToCheck, uint32_t size){
	bool result = true;
	for(uint32_t i = 0; i < size; i++){
		result &= (0xFF && bytesToCheck[i]);
	}
	return result;
}

void EepromManager::readFromExternalFlash(uint8_t* pData, uint32_t ReadAddr, uint32_t Size){
//	memcpy(pData, &pageAddress[ReadAddr], Size);
	BSP_QSPI_Read(pData, ReadAddr, Size);
}

PadInEeprom * EepromManager::findAllPads(void){
	PadInEeprom * padsArray = new PadInEeprom[infoSector.numberOfTotalPads];
	memset(padsArray, 0xff, sizeof(PadInEeprom) * infoSector.numberOfTotalPads);
	uint8_t padReadedCounter = 0;

	PadInEeprom tempPad;
	bool isEmptyField = false;
	const uint32_t startAddress = ADDRESS_SYSTEM_START + sizeof(InfoSector);
	uint32_t currentAddress;
	do {
		currentAddress = startAddress + padReadedCounter * sizeof(PadInEeprom);
		if((currentAddress + sizeof(PadInEeprom)) > (ADDRESS_SYSTEM_START + SECTOR_SIZE))
			break;
		readFromExternalFlash((uint8_t*) &tempPad, currentAddress, sizeof(PadInEeprom));
		isEmptyField = isAddressEmpty((uint8_t*) &tempPad, sizeof(PadInEeprom));
		if(isEmptyField){
			break;
		}
		for (int i = 0; i < infoSector.numberOfTotalPads; i++){
			if((padsArray[i].id == 0xFF)||(tempPad.id == padsArray[i].id)){
				padsArray[i] = tempPad;
				break;
			}
		}
		padReadedCounter++;
	} while (!isEmptyField);
	return padsArray;
}

//void eeprom_work(void){
//	EepromManager *eeprom = EepromManager::getInstance();
//	eeprom->init();
//	eeprom->loadInfoSector();
//	PadInEeprom newPad;
//
//	newPad.id = 0;
//	newPad.pad.curvetype = 0x5;
//	eeprom->writePad(newPad);
//
//	PadInEeprom secondPad;
//	secondPad.id = 1;
//	secondPad.pad.curvetype = 0x6;
//	eeprom->writePad(secondPad);
//
//	PadInEeprom checkNewPad;
//
//	checkNewPad = eeprom->readPad(0);
//	PadInEeprom thirdPad;
//	thirdPad = eeprom->readPad(4);
//
//
//	PadInEeprom renewPad;
//	renewPad.id = 0;
//	renewPad.pad.curvetype = 0x10;
//	eeprom->writePad(renewPad);
//
//	for(int i = 0; i < 0x15; i++){
//		printf("\nCURRENT OVERRIDE IS %d", i);
//		newPad.pad.note = i + 0x20;
//		eeprom->writePad(newPad);
//		eeprom->showPageAddress(0, 0x100);
//	}
//}
