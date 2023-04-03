/*
  " HELLO DRUM LIBRARY" Ver.0.7.7
  
  by Ryo Kosaka

  GitHub : https://github.com/RyoKosaka/HelloDrum-arduino-Library
  Blog : https://open-e-drums.tumblr.com/
*/

#ifndef HelloDrum_h
#define HelloDrum_h

#include "platform.h"
#include "drumSound.hpp"
#include "midi.h"
#include "drumManager.h"
#include <math.h>
#include <vector>

#define UNCORRECT_SOUND_ID 0xFF
#define DEFUAULT_SOUND_ID UNCORRECT_SOUND_ID

const static char *item[] = {
    "SENSITIVITY", //0 0
    "THRESHOLD",   //1 1
    "SCAN TIME",   //2 2
    "MASK TIME",   //3 3
    "CURVE TYPE",  //6 4
    "NOTE",        //7 5
};

const static char *itemD[] = {
    "SENSITIVITY", //0
    "THRESHOLD",   //1
    "SCAN TIME",   //2
    "MASK TIME",   //3
    "RIM SENS",    //4
    "RIM THRE",    //5
    "CURVE TYPE",  //6
    "NOTE HEAD",   //7
    "NOTE RIM",    //8
    "NOTE CROSS",  //9
};

const static char *itemCY2[] = {
    "SENSITIVITY", //0 0
    "THRESHOLD",   //1 1
    "SCAN TIME",   //2 2
    "MASK TIME",   //3 3
    "EDGE THRE",   //4 4
    "CURVE TYPE",  //6 5
    "NOTE BOW",    //7 6
    "NOTE EDGE",   //8 7
};

const static char *itemCY3[] = {
    "SENSITIVITY", //0
    "THRESHOLD",   //1
    "SCAN TIME",   //2
    "MASK TIME",   //3
    "EDGE THRE",   //4
    "CUP THRE",    //5
    "CURVE TYPE",  //6
    "NOTE BOW",    //7
    "NOTE EDGE",   //8
    "NOTE CUP",    //9
};

const static char *itemHH[] = {
    "SENSITIVITY", //0 0
    "THRESHOLD",   //1 1
    "SCAN TIME",   //2 2
    "MASK TIME",   //3 3
    "CURVE TYPE",  //6 4
    "NOTE OPEN",   //7 5
    "NOTE CLOSE",  //8 6
};

const static char *itemHH2[] = {
    "SENSITIVITY", //0 0
    "THRESHOLD",   //1 1
    "SCAN TIME",   //2 2
    "MASK TIME",   //3 3
    "EDGE THRE",   //4 4
    "CURVE TYPE",  //6 5
    "NOTE OPEN",   //7 6
    "NOTE CLOSE",  //8 7
};

const static char *itemHHC[] = {
    "SENSITIVITY",  //0 0
    "THRESHOLD",    //1 1
    "SCAN START",   //2 2
    "SCAN END",     //3 3
    "PEDAL SENS",   //4 4
    "CURVE TYPE",   //6 5
    "NOTE PEDAL",   //7 6
    "NOTE OPEN E",  //8 7
    "NOTE CLOSE E", //9 8
};

const static char *showInstrument[] = {
    "Pad 1",
    "Pad 2",
    "Pad 3",
    "Pad 4",
    "Pad 5",
    "Pad 6",
    "Pad 7",
    "Pad 8",
    "Pad 9",
    "Pad 10",
    "Pad 11",
    "Pad 12",
    "Pad 13",
    "Pad 14",
    "Pad 15",
    "Pad 16",
};

static bool push;
static bool showLCD;
static bool showFlag;

static byte showVelocity;
static byte nameIndex;
static byte nameIndexMax;
static byte showValue = 0;
static byte padIndex = 0;
static byte channelsAmount = 0;

typedef enum{
	SINGLE_PAD = 0,
	DOUBLE_PAD,
	CY2_PAD,
	CY3_PAD,
	HH_PAD,
	HH2_PAD,
	HHC_PAD
} PadType;
static PadType padType[16]; //if you use more pad, add number

static bool edit;
static bool editCheck;
static bool editdone;
static bool change;
static byte itemNumber;
static byte itemNumberShow;
static bool buttonState;
static bool buttonState_set;
static bool button_set;
static bool button_up;
static bool button_down;
static bool button_next;
static bool button_back;
static byte UP[10] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

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
	  byte soundHeadAddressId = DEFUAULT_SOUND_ID;	//10 this field show which item from soundsAdresses
	  byte soundRimAddressId = DEFUAULT_SOUND_ID;	//11
	  byte soundCupAddressId = DEFUAULT_SOUND_ID;	//12
	  float soundHeadVolumeDb = -3.0;
	  float soundRimVolumeDb = -3.0;
	  float soundCupVolumeDb = -3.0;
};

class HelloDrum {
public:
	HelloDrum(byte pin1, byte pin2);
	HelloDrum(byte pin1);
	~HelloDrum();

	virtual void loadMemory();
	virtual void sensingPad(void) = 0;
	virtual void executePad(void) = 0;
	void loadSounds();

	static uint8_t getChannelsAmount(void);
	static void init(void);
	static void loadPadsSettings(void);
	static void loadPadsSounds(void);
	static void sensingAllPads(void);
	static void executeAllPads(void);

	void setCurve(byte curveType);

	void settingName(const char *instrumentName);
	void settingEnable();

	void initMemory();

	int velocity;
	int velocityRim;
	int velocityCup;
	byte pedalCC;

	bool hit;
	bool hitRim;
	bool hitCup;
	bool choke;

	char* GetItem(byte i);

	byte value;
	byte padNum;

	PadMemory settings;
	DrumSound *noteHeadSound = NULL;
	DrumSound *noteRimSound = NULL;
	DrumSound *noteCupSound = NULL;

	// TODO remove duplicates
	byte noteEdge;
	byte noteOpen;
	byte noteClose;
	byte noteOpenEdge;
	byte noteCloseEdge;
	byte noteCross; // this maybe note for stick on rim

protected:

	byte pin_1;
	byte pin_2;
	int piezoValue;
	int RimPiezoValue;
	int sensorValue;
	int firstSensorValue;
	int lastSensorValue;
	int loopTimes = 0;
	unsigned long time_hit;
	unsigned long time_end;

	void singlePiezoSensing(byte sens, byte thre, byte scanTime, byte maskTime);
	void cymbal2zoneSensing(byte sens, byte thre, byte scanTime, byte maskTime, byte edgeThre);

	int curve(int velocityRaw, int threshold, int sensRaw, byte curveType);
};

#endif
