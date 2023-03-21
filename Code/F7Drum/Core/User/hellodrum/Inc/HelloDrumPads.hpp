#ifndef USER_HELLODRUM_INC_HELLODRUMPADS_HPP_
#define USER_HELLODRUM_INC_HELLODRUMPADS_HPP_

#include "hellodrum.hpp"

class SinglePad : public HelloDrum{
//	using HelloDrum::HelloDrum; // TODO remove this at the end
public:
	SinglePad(byte pin1);
	void sensingPad(void);
	void executePad(void);
};

class DoublePad : public HelloDrum{
public:
	DoublePad(byte pin1, byte pin2);
	void sensingPad(void);
	void executePad(void);
	void dualPiezoSensing(byte sens, byte thre, byte scanTime, byte maskTime, byte rimSens, byte rimThre);
};

typedef enum{
	TCRT5000_PEDAL,
	FSR_PEDAL,
//	FSR_INVERSE TODO add this variant
} PedalType;

class HiHatPedalPad : public HelloDrum{
public:
	bool openHH = false;
	bool closeHH = false;
	bool moving;

	HiHatPedalPad(byte pin1, PedalType pedalType);
	void sensingPad(void);
	void executePad(void);
	void TCRT5000Sensing(byte sens, byte thre1, byte thre2, byte scanTime);
	void FSRSensing(byte sens, byte thre, byte scanStart, byte scanEnd, byte pedalSens);
private:
	byte exTCRT = 0;
	byte exFSR = 0;
	bool pedalVelocityFlag = false;
	bool pedalFlag = true;
	int TCRT;
	int fsr;
	unsigned long time_hit_pedal_1;
	unsigned long time_hit_pedal_2;

	PedalType pedalType;
};

class HiHatPad : public HelloDrum{
public:
	HiHatPad(byte pin1, HiHatPedalPad * pedal);
	void sensingPad(void);
	void executePad(void);
	void loadMemory();
private:
	HiHatPedalPad *pedal;
	// TODO fix noteOpen and noteClose
	byte noteOpen1;
	byte noteClose1;
};

class HiHat2zonePad : public HelloDrum{
public:
	HiHat2zonePad(byte pin1, byte pin2, HiHatPedalPad * pedal);
	void sensingPad(void);
	void executePad(void);
	void loadMemory();
private:
	HiHatPedalPad *pedal;
	// TODO fix notes below
	byte noteOpen1;
	byte noteEdge1;
	byte noteClose1;
	byte noteOpenEdge1;
	byte noteCloseEdge1;
};

class Cymbal2ZonesPad : public HelloDrum{
public:
	Cymbal2ZonesPad(byte pin1, byte pin2);
	void sensingPad(void);
	void executePad(void);
};

class Cymbal3ZonesPad : public HelloDrum{
public:
	Cymbal3ZonesPad(byte pin1, byte pin2);
	void sensingPad(void);
	void executePad(void);
	void cymbal3zoneSensing(byte sens, byte thre, byte scanTime, byte maskTime, byte edgeThre, byte cupThre);
};

#endif /* USER_HELLODRUM_INC_HELLODRUMPADS_HPP_ */
