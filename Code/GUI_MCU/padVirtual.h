#ifndef PADVIRTUAL_H_
#define PADVIRTUAL_H_

#ifdef SIMULATION
typedef struct {
	uint8_t sensitivity;   //0
	uint8_t threshold1;     //1
	uint8_t scantime;       //2
	uint8_t masktime;       //3
	uint8_t rimSensitivity; //4
	uint8_t rimThreshold;    //5
	uint8_t curvetype;       //6
	uint8_t note;           //7
	uint8_t noteRim;        //8
	uint8_t noteCup;        //9
	uint8_t soundHeadAddressId;	//10 this field show which item from soundsAdresses
	uint8_t soundRimAddressId;	//11
	uint8_t soundCupAddressId;	//12
	float soundHeadVolumeDb;
	float soundRimVolumeDb;
	float soundCupVolumeDb;
} PadMemory;

typedef enum {
	PAD_NOT_CHANGED,
	PAD_WAS_CHANGED
} PadState;

typedef struct {
	PadMemory pad;
	PadState padState;
} PadGuiState;

#endif

#endif /* PADVIRTUAL_H_ */
