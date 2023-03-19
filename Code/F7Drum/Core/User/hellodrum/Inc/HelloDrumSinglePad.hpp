#ifndef USER_HELLODRUM_INC_HELLODRUMSINGLEPAD_HPP_
#define USER_HELLODRUM_INC_HELLODRUMSINGLEPAD_HPP_

#include "hellodrum.hpp"

class SinglePad : HelloDrum{
//	using HelloDrum::HelloDrum; // TODO remove this
public:
	SinglePad(byte pin1);
	void sensingPad(void);
	void executePad(void);
};

class DoublePad : HelloDrum{
public:
	DoublePad(byte pin1, byte pin2);
	void sensingPad(void);
	void executePad(void);
};

typedef enum{
	TCRT5000_PEDAL,
	FSR_PEDAL,
//	FSR_INVERSE TODO add this variant
} PedalType;

class HiHatPedalPad : public HelloDrum{
public:
	HiHatPedalPad(byte pin1, PedalType pedalType);
	void sensingPad(void);
	void executePad(void);
private:
	PedalType pedalType;
};

class HiHatPad : HelloDrum{
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

class HiHat2zonePad : HelloDrum{
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

class Cymbal2ZonesPad : HelloDrum{
public:
	Cymbal2ZonesPad(byte pin1, byte pin2);
	void sensingPad(void);
	void executePad(void);
};

class Cymbal3ZonesPad : HelloDrum{
public:
	Cymbal3ZonesPad(byte pin1, byte pin2);
	void sensingPad(void);
	void executePad(void);
};

#endif /* USER_HELLODRUM_INC_HELLODRUMSINGLEPAD_HPP_ */
