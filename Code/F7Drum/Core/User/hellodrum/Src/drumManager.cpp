#include <hellodrum.hpp>
#include "drumManager.h"
#include "midi.h"
#include "stm32746g_discovery_qspi.h"

void Error_Handler(void);
void sendUart(const char *_msg);

/**
 * This method for creating space in memory
 */
void firstInitMem(void) {
	uint8_t numBytesForCheck = 8;
	uint8_t readBuff[numBytesForCheck] = { 0 };

	BSP_QSPI_Init();

	bool isMemoryInitBefore = true;
	if (BSP_QSPI_Read(readBuff, 0x0, numBytesForCheck) != QSPI_OK) {
		sendUart("qspi can't read");
		Error_Handler();
	}
	for(uint8_t i = 0; i < numBytesForCheck; i++){
		isMemoryInitBefore &= (readBuff[i] == 0xFF);
	}
	if(isMemoryInitBefore){
		BSP_QSPI_DeInit();
		return;
	}

	for(uint8_t i = 0; i < NUMBER_OF_PADS; i++){
		HelloDrum pad(i);
		pad.initMemory();
	}

	BSP_QSPI_DeInit();
	// reboot mcu
	NVIC_SystemReset();
}

static HelloDrum kick(0);
static HelloDrum snare(1);
static HelloDrum hihat(2);
static HelloDrum hihatPedal(3);
static HelloDrum ride(4, 5);

void initHelloDrums(void) {
	//It is necessary to make the order in exactly the same order as you named the pad first.
	kick.settingName("KICK");
	snare.settingName("SNARE");
	hihat.settingName("HIHAT");
	hihatPedal.settingName("HIHAT PEDAL");
	ride.settingName("RIDE");

	//Load settings from EEPROM.
	//It is necessary to make the order in exactly the same order as you named the pad first.
	kick.loadMemory();
	snare.loadMemory();
	hihat.loadMemory();
	hihatPedal.loadMemory();
	ride.loadMemory();
}

void requestPiezo(void) {
	// TODO uncomment string below
	// HAL_ADC_Start_DMA(adcLocal, (uint32_t*) &adc_buf[0], NUMBER_OF_CHANNELS);
}

void checkHelloDrums(void){
	kick.singlePiezo();
	snare.singlePiezo();
	hihat.HH();
	hihatPedal.hihatControl();
	ride.cymbal3zone();

	//KICK//
	if (kick.hit == true) {
		sendMidiGEN(kick.note, kick.velocity);
	}

	//SNARE//
	if (snare.hit == true) {
		sendMidiGEN(snare.note, snare.velocity);
	}

	//HIHAT//
	if (hihat.hit == true) {
		//check open or close
		//1.open
		if (hihatPedal.openHH == true) {
			sendMidiGEN(hihat.noteOpen, hihat.velocity);
		}
		//2.close
		else if (hihatPedal.closeHH == true) {
			sendMidiGEN(hihat.noteClose, hihat.velocity);
		}
	}

	//HIHAT CONTROLLER//
	//when hihat is closed
	if (hihatPedal.hit == true) {
		sendMidiGEN(hihatPedal.note, hihatPedal.velocity);
	}

	//sending state of pedal with controll change
	if (hihatPedal.moving == true) {
		sendMidiControlChange(4, hihatPedal.pedalCC);
	}

	//RIDE//
	//1.bow
	if (ride.hit == true) {
		sendMidiGEN(ride.note, ride.velocity); //(note, velocity, channel)
	}

	//2.edge
	else if (ride.hitRim == true) {
		sendMidiGEN(ride.noteRim, ride.velocity); //(note, velocity, channel)
	}

	//3.cup
	else if (ride.hitCup == true) {
		sendMidiGEN(ride.noteCup, ride.velocity); //(note, velocity, channel)
	}

	//4.choke
	if (ride.choke == true) {
		sendMidiAT(ride.note, 127);
		sendMidiAT(ride.noteRim, 127);
		sendMidiAT(ride.noteCup, 127);
		sendMidiAT(ride.note, 0);
		sendMidiAT(ride.noteRim, 0);
		sendMidiAT(ride.noteCup, 0);
	}
}

void writeToExternalFlash(uint8_t* pData, uint32_t WriteAddr, uint32_t Size){
	if(BSP_QSPI_Write(pData, WriteAddr, Size) != QSPI_OK){
		sendUart("QSPI can't writeToExternalFlash");
		Error_Handler();
	}
}

void readFromExternalFlash(uint8_t* pData, uint32_t ReadAddr, uint32_t Size){
	if(BSP_QSPI_Read(pData, ReadAddr, Size) != QSPI_OK){
		sendUart("QSPI can't writeToExternalFlash");
		Error_Handler();
	}
}

