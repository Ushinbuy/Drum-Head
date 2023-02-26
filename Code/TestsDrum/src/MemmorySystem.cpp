#include <MemmorySystem.h>
#include <string.h>

#define SECTOR_SIZE 0x10000
#define PAD_ADDRESS_START 0x0
#define ADDRESS_SYSTEM_START 0x0

MemmorySystem::MemmorySystem() { }

void MemmorySystem::eraseSector(void) {
	// this imitate function
	memset(pageAddress, 0xFF, SECTOR_SIZE);
}

void MemmorySystem::writeToExternalFlash(uint8_t* pData, uint32_t WriteAddr, uint32_t Size){
	// change this function for QSPI_write
	for(uint32_t i = 0; i < Size; i++){
		pageAddress[WriteAddr + i] &= pData[i];
	}
}

void MemmorySystem::readFromExternalFlash(uint8_t* pData, uint32_t ReadAddr, uint32_t Size){
	// change this function for QSPI_read
	memcpy(pData, &pageAddress[ReadAddr], Size);
}

void MemmorySystem::initMemmorySystem(void){
	// this imitate function
	MemmorySpace _memSyst;
	_memSyst.numberOfTotalPads = PADS_NUMBER;
	_memSyst.soundsNumber = 2;

	MemmorySystem::getInstance()->filFFvalues(_memSyst.padSavingsOffsets, PADS_NUMBER);

	_memSyst.soundsAdresses[0] = 0x10;
	_memSyst.soundsAdresses[1] = 0x20;
	_memSyst.soundsAdresses[2] = 0x30;
	_memSyst.soundsAdresses[3] = 0x44;
	_memSyst.soundsAdresses[4] = 0x55;
	_memSyst.soundsAdresses[5] = 0x66;

	MemmorySystem::getInstance()->writeToExternalFlash((uint8_t *) & _memSyst, ADDRESS_SYSTEM_START, sizeof(MemmorySpace));
}

void MemmorySystem::writePadFirstTime(PadMemory pad, uint8_t padNum){
	if(mainMemSpace.padSavingsOffsets[padNum] == (uint64_t) -1){
		writeToExternalFlash((uint8_t*)&pad, calcPadAddress(padNum), sizeof(PadMemory));
	}
	else{
		overridePad(pad, padNum);
	}
}

void MemmorySystem::getPadInfo(PadMemory* padToWhereRead, uint8_t padNum){
	readFromExternalFlash((uint8_t *) padToWhereRead, calcPadAddress(padNum), sizeof(PadMemory));
}

uint32_t MemmorySystem::calcPadAddress(uint8_t padNum){
	uint64_t offsetInBytes = mainMemSpace.padSavingsOffsets[padNum];
//	offsetInBytes >>= 10;
	uint8_t numOfOverwrites = 0;
	while(offsetInBytes){
		numOfOverwrites++;
		offsetInBytes >>= 1;
	}
	numOfOverwrites = 64 - numOfOverwrites;

	return ADDRESS_SYSTEM_START + sizeof(MemmorySpace) + (padNum + numOfOverwrites * PADS_NUMBER) * sizeof(PadMemory) +1;
}

void MemmorySystem::filFFvalues(uint64_t *pData, uint32_t size){
	// this imitate function
	for(uint32_t i = 0; i < size; i++){
		pData[i] = 0xFFFFFFFFFFFFFFFF;
	}
}

void MemmorySystem::getMemorySystem(void){
	readFromExternalFlash((uint8_t *)&mainMemSpace, ADDRESS_SYSTEM_START, sizeof(MemmorySpace));
}

void MemmorySystem::overridePad(PadMemory _pad, uint8_t padNum){
	if (mainMemSpace.padSavingsOffsets[padNum] != 0) {
		mainMemSpace.padSavingsOffsets[padNum] >>= 1;
		writeToExternalFlash((uint8_t*) &mainMemSpace, ADDRESS_SYSTEM_START, sizeof(MemmorySpace));
		writeToExternalFlash((uint8_t*) &_pad, calcPadAddress(padNum), sizeof(PadMemory));
	} else {
		PadMemory tempBuffer[PADS_NUMBER];
		for (int i = 0; i < PADS_NUMBER; i++) {
			if(i == padNum){
				tempBuffer[i] = _pad;
			}
			else{
				MemmorySystem::getInstance()->getPadInfo(&tempBuffer[i], i);
			}
		}
		printf("\nStart erasing sector\n");
		overrideMainMemSector();
		for (int i = 0; i < PADS_NUMBER; i++) {
			writePadFirstTime(tempBuffer[i], i);
		}
	}
}

void MemmorySystem::overrideMainMemSector(void) {
	eraseSector();
	filFFvalues(mainMemSpace.padSavingsOffsets, PADS_NUMBER);
	writeToExternalFlash((uint8_t*) &mainMemSpace, ADDRESS_SYSTEM_START, sizeof(MemmorySpace));
}

void MemmorySystem::showPageAddress(uint32_t startAddress, uint32_t stopAddress){
	for (uint32_t i = startAddress; i < stopAddress; i++) {
		if (i % 0x10 == 0) {
			printf("\n0x%03x : ", i);
		} else {
			printf("0x%02X ", pageAddress[i]);
		}
	}
}

void showPadInfo(PadMemory pad){
	printf("\n sensitivity = 0x%X", pad.sensitivity);   //0
	printf("\n threshold1 = 0x%X", pad.threshold1);     //1
	printf("\n scantime  = 0x%X", pad.scantime);       //2
	printf("\n masktime = 0x%X", pad.masktime);       //3
	printf("\n rimSensitivity = 0x%X", pad.rimSensitivity); //4 edgeThreshold
	printf("\n rimThreshold = 0x%X", pad.rimThreshold);    //5 cupThreshold
	printf("\n curvetype = 0x%X", pad.curvetype);       //6
	printf("\n note = 0x%X", pad.note);           //7
	printf("\n noteRim = 0x%X", pad.noteRim);        //8
	printf("\n noteCup = 0x%X", pad.noteCup);        //9
	printf("\n soundAddressId = 0x%X", pad.soundAddressId);	//10 this field show which item from soundsAdresses
}

void mainWork(void)
{
	MemmorySystem::getInstance()->eraseSector();
	MemmorySystem::getInstance()->initMemmorySystem();
	MemmorySystem::getInstance()->getMemorySystem();

	printf("Address of Pad(0) is 0x%02x\n\n", (uint32_t) sizeof(MemmorySpace));

	PadMemory defaultPad;
	for(uint8_t i = 0; i < PADS_NUMBER; i++){
		MemmorySystem::getInstance()->writePadFirstTime(defaultPad, i);
	}

	PadMemory tempPad;
	MemmorySystem::getInstance()->getPadInfo(&tempPad, 0);

	tempPad.masktime = 0x22;

	MemmorySystem::getInstance()->overridePad(tempPad, 0);

	printf("Try to override 64 times");

	printf("\n\nLet's look for memory before overriding\n");

	MemmorySystem::getInstance()->showPageAddress(0, 0xc0);

	for (uint8_t i = 0; i < 64; i++) {
		tempPad.masktime = i;
		MemmorySystem::getInstance()->overridePad(tempPad, 0);
	}

	printf("\n\nLet's look for memory after overriding 0x%x\n", tempPad.masktime);

	MemmorySystem::getInstance()->showPageAddress(0, 0xc0);

	PadMemory newPad;
	MemmorySystem::getInstance()->getPadInfo(&newPad, 0);
	showPadInfo(newPad);
}
