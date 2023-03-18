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
	  byte soundHeadAddressId = 0xFF;	//10 this field show which item from soundsAdresses
	  byte soundRimAddressId = 0xFF;	//11
	  byte soundCupAddressId = 0xFF;	//12
	  float soundHeadVolumeDb = -3.0;
	  float soundRimVolumeDb = -3.0;
	  float soundCupVolumeDb = -3.0;
};

class HelloDrum
{
public:
  HelloDrum(byte pin1, byte pin2);
  HelloDrum(byte pin1);
  static uint8_t getChannelsAmount(void);

  void singlePiezo(byte sens, byte thre, byte scan, byte mask);
  void singlePiezo();
  void dualPiezo(byte sens, byte thre, byte scan, byte mask, byte rimSens, byte rimThre);
  void dualPiezo();
  void HH(byte sens, byte thre, byte scan, byte mask);
  void HH();
  void HH2zone(byte sens, byte thre, byte scan, byte mask, byte edgeThre);
  void HH2zone();
  void cymbal3zone(byte sens, byte thre, byte scan, byte mask, byte edgeThre, byte cupThre);
  void cymbal3zone();
  void cymbal2zone(byte sens, byte thre, byte scan, byte mask, byte edgeThre);
  void cymbal2zone();
  void TCRT5000(byte sens, byte thre1, byte thre2, byte scan);
  void TCRT5000();
  void FSR(byte sens, byte thre, byte scanStart, byte scanEnd, byte pedalSens);
  void FSR();
  void hihatControl(byte sens, byte thre, byte scanStart, byte scanEnd, byte pedalSens);
  void hihatControl();

  void setCurve(byte curveType);

  void settingName(const char *instrumentName);
  void settingEnable();

  void loadMemory();
  void initMemory();
  void initSounds();

  int velocity;
  int velocityRim;
  int velocityCup;
  byte pedalCC;

  //  int exValue;
  byte exTCRT = 0;
  byte exFSR = 0;
  bool hit;
  bool openHH = false;
  bool closeHH = false;
  bool hitRim;
  bool hitCup;
  bool choke;
  bool sensorFlag;
  bool moving;
  bool pedalVelocityFlag = false;
  bool pedalFlag = true;
  bool settingHHC = false;
  bool chokeFlag;

  char *GetItem(byte i);

  byte value;
  byte padNum;

  PadMemory settings;
  DrumSound* noteHeadSound;
  DrumSound* noteRimSound;
  DrumSound* noteCupSound;

  // TODO remove duplicates
  byte noteEdge;
  byte noteOpen;
  byte noteClose;
  byte noteOpenEdge;
  byte noteCloseEdge;
  byte noteCross;

private:
  byte pin_1;
  byte pin_2;
  int piezoValue;
  int RimPiezoValue;
  int sensorValue;
  int TCRT;
  int fsr;
  int firstSensorValue;
  int lastSensorValue;
  int loopTimes = 0;
  unsigned long time_hit;
  unsigned long time_end;
  unsigned long time_choke;
  unsigned long time_hit_pedal_1;
  unsigned long time_hit_pedal_2;

  void singlePiezoSensing(byte sens, byte thre, byte scanTime, byte maskTime);
  void dualPiezoSensing(byte sens, byte thre, byte scanTime, byte maskTime, byte rimSens, byte rimThre);
  void cymbal2zoneSensing(byte sens, byte thre, byte scanTime, byte maskTime, byte edgeThre);
  void cymbal3zoneSensing(byte sens, byte thre, byte scanTime, byte maskTime, byte edgeThre, byte cupThre);
  void TCRT5000Sensing(byte sens, byte thre1, byte thre2, byte scanTime);
  void FSRSensing(byte sens, byte thre, byte scanStart, byte scanEnd, byte pedalSens);
  int curve(int velocityRaw, int threshold, int sensRaw, byte curveType);
};

#endif
