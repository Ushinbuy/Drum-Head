/*
  "HELLO DRUM LIBRARY" Ver.0.7.7
  
  by Ryo Kosaka

  GitHub : https://github.com/RyoKosaka/HelloDrum-arduino-Library
  Blog : https://open-e-drums.tumblr.com/
*/

//#define DEBUG_DRUM //<-- uncomment this line to enable debug mode with Serial.

#include <hellodrum.hpp>

#include <stdio.h>
#include "eepromManager.h"
#include "HelloDrumKnob.hpp"
#include <vector>

extern "C" void sendUart(const char *_msg);
extern char buffer_out[1000];

//#define DEBUG_DRUM
//#define DEBUG_DRUM_VOLTAGE

static std::vector<HelloDrum*> padsList;

HelloDrum::~HelloDrum(){
	sendUart("Pad was deleted");
}

//Pad with a sensor.
HelloDrum::HelloDrum(byte pin1)
{
	for (uint8_t i = 0; i < padsList.size(); i++) {
		if(pin1 == padsList[i]->pin_1){
			sendUart("\nPad was init before");
			Error_Handler();
		}
	}
  pin_1 = pin1;

  //Give the instance a pad number.
  padNum = padIndex;
  padIndex++;
  channelsAmount++;

  padsList.push_back(this);
}

//Pad with 2 sensors.
HelloDrum::HelloDrum(byte pin1, byte pin2)
{
	for (uint8_t i = 0; i < padsList.size(); i++) {
		if ((pin1 == padsList[i]->pin_1) || (pin1 == padsList[i]->pin_2)
				|| ((pin2 == padsList[i]->pin_1) || (pin2 == padsList[i]->pin_2))) {
			sendUart("\nPad was init before");
			Error_Handler();
		}
	}
  pin_1 = pin1;
  pin_2 = pin2;

  //Give the instance a pad number.
  padNum = padIndex;
  padIndex++;
  channelsAmount += 2;

  padsList.push_back(this);
}

uint8_t HelloDrum::getChannelsAmount(void){
	return channelsAmount;
}

///////////////////// 1. SENSING  ///////////////////////

void HelloDrum::singlePiezoSensing(byte sens, byte thre, byte scanTime, byte maskTime)
{
  int Threshold = thre * 10;
  int Sensitivity = sens * 10;

  hit = false;

  //when the value > threshold
  if (piezoValue > Threshold && loopTimes == 0)
  {
    //Start the scan time
    time_hit = millis(); //check the time pad hitted

    //compare time to cancel retrigger
    if (time_hit - time_end < maskTime)
    {
      return; //Ignore the scan
    }
    else
    {
      velocity = piezoValue; //first peak
      loopTimes = 1;         //start scan trigger
    }
  }

  //peak scan start
	if (loopTimes > 0) {

#ifdef DEBUG_DRUM_VOLTAGE
	  sprintf(buffer_out, "%d\n", piezoValue);
	  sendUart(buffer_out);
#endif
    if (piezoValue > velocity)
    {
      velocity = piezoValue;
    }
    loopTimes++;

    //scan end
    //if (loopTimes == scanTime)
    if (millis() - time_hit >= scanTime)
    {

#ifdef DEBUG_DRUM
      int prevVel = velocity;
#endif

      velocity = curve(velocity, Threshold, Sensitivity, settings.curvetype); //apply the curve at the velocity
      hit = true;                                                    //mark as hit
      time_end = millis();

      //Those 3 lines are for "Eeprom version"
      //I think that are ignore in simple mode, but otw we can pass a flag and skip that.
      showVelocity = velocity;
      showLCD = true;
      padIndex = padNum;

#ifdef DEBUG_DRUM
      sprintf(buffer_out, "\n[Hit] velocity : %d (raw value : %d), loopTimes : %d, ScanTime(ms) : %ld",
    		  velocity,
			  prevVel,
			  loopTimes,
			  (time_end - time_hit));
      sendUart(buffer_out);
#endif

      loopTimes = 0; //reset loopTimes (ready for next sensing)
    }
  }
}

void HelloDrum::cymbal2zoneSensing(byte sens, byte thre, byte scanTime, byte maskTime, byte edgeThre)
{
  int Threshold = thre * 10;
  int Sensitivity = sens * 10;
  int edgeThreshold = edgeThre * 10;

  hit = false;
  hitRim = false;
  choke = false;

  //when the value > threshold
  if ((piezoValue > Threshold && loopTimes == 0) || (sensorValue > edgeThreshold && loopTimes == 0))
  {
    time_hit = millis(); //check the time pad hitted

    if (time_hit - time_end < maskTime)
    {
      return;
    }
    else
    {
      velocity = abs(piezoValue - sensorValue); //first peak //ここコメント化ありうる。
      firstSensorValue = sensorValue;
      loopTimes = 1;
    }
  }

  //peak scan start
  if (loopTimes > 0)
  {
#ifdef DEBUG_DRUM_VOLTAGE
	sprintf(buffer_out, "%d, %d, %d",
			piezoValue,
			sensorValue,
			abs(piezoValue - sensorValue));
	sendUart(buffer_out);
#endif
    if (abs(piezoValue - sensorValue) > velocity)
    {
      velocity = abs(piezoValue - sensorValue);
    }
    if (sensorValue > firstSensorValue && loopTimes < 5)
    {
      firstSensorValue = sensorValue;
    }
    loopTimes++;

    //scan end
    if (millis() - time_hit >= scanTime)
    {
      velocity = velocity - firstSensorValue;

#ifdef DEBUG_DRUM
      int prevVel = velocity;
#endif

      time_end = millis();

      lastSensorValue = sensorValue;

      //bow
      if (firstSensorValue < edgeThreshold && lastSensorValue < edgeThreshold)
      {
        velocity = curve(velocity, Threshold, Sensitivity, settings.curvetype);

#ifdef DEBUG_DRUM
		sprintf(buffer_out, "\n[Hit Bow] velocity : %d, (raw value : %d, firstSensorValue : %d, lastSensorValue : %d), loopTimes : %d, ScanTime(ms) : %ld",
			  velocity,
			  prevVel,
			  firstSensorValue,
			  lastSensorValue,
			  loopTimes,
			  (time_end - time_hit));
		sendUart(buffer_out);
#endif

        hit = true;
        showVelocity = velocity;
        showLCD = true;
        padIndex = padNum;
      }

      //edge
      else if (velocity > Threshold && firstSensorValue > edgeThreshold && firstSensorValue > lastSensorValue)
      {
        velocity = curve(velocity, Threshold, Sensitivity, settings.curvetype);

#ifdef DEBUG_DRUM
		sprintf(buffer_out, "\n[Hit Edge] velocity : %d, (raw value : %d, firstSensorValue : %d, lastSensorValue : %d), loopTimes : %d, ScanTime(ms) : %ld",
			  velocity,
			  prevVel,
			  firstSensorValue,
			  lastSensorValue,
			  loopTimes,
			  (time_end - time_hit));
		sendUart(buffer_out);
#endif

        hitRim = true;
        showVelocity = velocity;
        showLCD = true;
        padIndex = padNum;
      }

      //choke
      else if (lastSensorValue > edgeThreshold && firstSensorValue > edgeThreshold && lastSensorValue >= firstSensorValue)
      {

#ifdef DEBUG_DRUM
		sprintf(buffer_out, "\n[Choke] firstSensorValue : %d, lastSensorValue : %d, loopTimes : %d, ScanTime(ms) : %ld",
			  firstSensorValue,
			  lastSensorValue,
			  loopTimes,
			  (time_end - time_hit));
		sendUart(buffer_out);
#endif

        choke = true;
      }

      loopTimes = 0;
    }
  }
}



int HelloDrum::curve(int velocityRaw, int threshold, int sensRaw, byte curveType)
{
	  //Curve Type 0 : Linear
	  if (curveType == 0)
	  {
	    int res = map(velocityRaw, threshold, sensRaw, 1, 127); //map the value in linear range 1/127

	    if (res <= 1) //initial velocity cannot be lower than thre1Raw so probably velocity here cannot be lower than 1
	    {
	      res = 1;
	    }

	    if (res > 127) //ok, velocity can be greather than 127 if I set a sensRaw too low and pass a initial velocity higher than this value
	    {
	      res = 127;
	    }
	    return res;
	  }

	  //Curve Type 1 : exp 1
	  else if (curveType == 1)
	  {
	    float resF = map(velocityRaw, threshold, sensRaw, 1, 127);

	    if (resF <= 1)
	    {
	      resF = 1;
	    }

	    if (resF > 127)
	    {
	      resF = 127;
	    }

	    resF = (126 / (pow(1.02, 126) - 1)) * (pow(1.02, resF - 1) - 1) + 1; // 1.02

	    byte res;
	    res = (byte)round(resF);
	    return res;
	  }

	  //Curve Type 2 : exp 2
	  else if (curveType == 2)
	  {
	    float resF = map(velocityRaw, threshold, sensRaw, 1, 127);

	    if (resF <= 1)
	    {
	      resF = 1;
	    }

	    if (resF > 127)
	    {
	      resF = 127;
	    }

	    resF = (126 / (pow(1.05, 126) - 1)) * (pow(1.05, resF - 1) - 1) + 1; // 1.05

	    byte res;
	    res = (byte)round(resF);
	    return res;
	  }

	  //Curve Type 3 : log 1
	  else if (curveType == 3)
	  {
	    float resF = map(velocityRaw, threshold, sensRaw, 1, 127);

	    if (resF <= 1)
	    {
	      resF = 1;
	    }

	    if (resF > 127)
	    {
	      resF = 127;
	    }

	    resF = (126 / (pow(0.98, 126) - 1)) * (pow(0.98, resF - 1) - 1) + 1; // 0.98

	    byte res;
	    res = (byte)round(resF);
	    return res;
	  }

	  //Curve Type 4 : log 2
	  else if (curveType == 4)
	  {
	    float resF = map(velocityRaw, threshold, sensRaw, 1, 127);

	    if (resF <= 1)
	    {
	      resF = 1;
	    }

	    if (resF > 127)
	    {
	      resF = 127;
	    }

	    resF = (126 / (pow(0.95, 126) - 1)) * (pow(0.95, resF - 1) - 1) + 1; // 0.95

	    byte res;
	    res = (byte)round(resF);
	    return res;
	  }

	  else
	  {
	    return 0;
	  }
	}

void HelloDrum::setCurve(byte curveType)
{
	settings.curvetype = curveType;
}

//////////////////////////// 6. EEPROM SETTING  //////////////////////////////

void HelloDrum::settingEnable()
{
	// TODO Override memory just only by click on SAVE button
  //When EDIT button pushed
  if (padNum == nameIndex)
  {

    //When UP button pushed
    if (button_up == LOW && buttonState == true && editCheck == true)
    {

      switch (itemNumber)
      {
      case 0:
    	  settings.sensitivity = settings.sensitivity + UP[itemNumber];
        if (settings.sensitivity > 100)
        {
        	settings.sensitivity = 1;
        }
//        EEPROM.write(padNum * 8, sensitivity);
        break;

      case 1:
    	  settings.threshold1 = settings.threshold1 + UP[itemNumber];
        if (settings.threshold1 > 100)
        {
        	settings.threshold1 = 1;
        }
//        EEPROM.write((padNum * 10) + 1, threshold1);
        break;

      case 2:
    	  settings.scantime = settings.scantime + UP[itemNumber];
        if (settings.scantime > 100)
        {
        	settings.scantime = 1;
        }
//        EEPROM.write((padNum * 10) + 2, scantime);
        break;

      case 3:
    	  settings.masktime = settings.masktime + UP[itemNumber];
        if (settings.masktime > 100)
        {
        	settings.masktime = 1;
        }
//        EEPROM.write((padNum * 10) + 3, masktime);
        break;

      case 4:
    	  settings.rimSensitivity = settings.rimSensitivity + UP[itemNumber];
        if (settings.rimSensitivity > 100)
        {
        	settings.rimSensitivity = 1;
        }
//        EEPROM.write((padNum * 10) + 4, rimSensitivity);
        break;

      case 5:
    	  settings.rimThreshold = settings.rimThreshold + UP[itemNumber];
        if (settings.rimThreshold > 100)
        {
        	settings.rimThreshold = 1;
        }
//        EEPROM.write((padNum * 10) + 5, rimThreshold);
        break;

      case 6:
    	  settings.curvetype = settings.curvetype + UP[itemNumber];
        if (settings.curvetype > 4)
        {
        	settings.curvetype = 0;
        }
//        EEPROM.write((padNum * 10) + 6, curvetype);
        break;

      case 7:
    	  settings.note = settings.note + UP[itemNumber];
        if (settings.note > 127)
        {
        	settings.note = 0;
        }
//        EEPROM.write((padNum * 10) + 7, note);
        noteOpen = settings.note;
        break;

      case 8:
    	  settings.noteRim = settings.noteRim + UP[itemNumber];
        if (settings.noteRim > 127)
        {
        	settings.noteRim = 0;
        }
//        EEPROM.write((padNum * 10) + 8, noteRim);
        noteEdge = settings.noteRim;
        noteClose = settings.noteRim;
        noteOpenEdge = settings.noteRim;
        break;

      case 9:
    	  settings.noteCup = settings.noteCup + UP[itemNumber];
        if (settings.noteCup > 127)
        {
        	settings.noteCup = 0;
        }
//        EEPROM.write((padNum * 10) + 9, noteCup);
        noteCloseEdge = settings.noteCup;
        noteCross = settings.noteCup;
        break;
      }
      change = true;
      push = true;
      buttonState = false;
      delay(30);
    }

    //When Down button pushed
    if (button_down == LOW && buttonState == true && editCheck == true)
    {

      switch (itemNumber)
      {
      case 0:
    	  settings.sensitivity = settings.sensitivity - UP[itemNumber];
        if (settings.sensitivity < 1)
        {
        	settings.sensitivity = 100;
        }
//        EEPROM.write(padNum * 10, sensitivity);
        break;

      case 1:
    	  settings.threshold1 = settings.threshold1 - UP[itemNumber];
        if (settings.threshold1 < 1)
        {
        	settings.threshold1 = 100;
        }
//        EEPROM.write((padNum * 10) + 1, threshold1);
        break;

      case 2:
    	  settings.scantime = settings.scantime - UP[itemNumber];
        if (settings.scantime < 1)
        {
        	settings.scantime = 100;
        }
//        EEPROM.write((padNum * 10) + 2, scantime);
        break;

      case 3:
    	  settings.masktime = settings.masktime - UP[itemNumber];
        if (settings.masktime < 1)
        {
        	settings.masktime = 100;
        }
//        EEPROM.write((padNum * 10) + 3, masktime);
        break;

      case 4:
    	  settings.rimSensitivity = settings.rimSensitivity - UP[itemNumber];
        if (settings.rimSensitivity < 1)
        {
        	settings.rimSensitivity = 100;
        }
//        EEPROM.write((padNum * 10) + 4, rimSensitivity);
        break;

      case 5:
    	  settings.rimThreshold = settings.rimThreshold - UP[itemNumber];
        if (settings.rimThreshold < 1)
        {
        	settings.rimThreshold = 100;
        }
//        EEPROM.write((padNum * 10) + 5, rimThreshold);
        break;

      case 6:
    	  settings.curvetype = settings.curvetype - UP[itemNumber];
        if (settings.curvetype == 255)
        {
        	settings.curvetype = 4;
        }
//        EEPROM.write((padNum * 10) + 6, curvetype);
        break;

      case 7:
    	  settings.note = settings.note - UP[itemNumber];
        if (settings.note == 255)
        {
        	settings.note = 127;
        }
//        EEPROM.write((padNum * 10) + 7, note);
        noteOpen = settings.note;
        break;

      case 8:
    	  settings.noteRim = settings.noteRim - UP[itemNumber];
        if (settings.noteRim == 255)
        {
        	settings.noteRim = 127;
        }
//        EEPROM.write((padNum * 10) + 8, noteRim);
        noteEdge = settings.noteRim;
        noteClose = settings.noteRim;
        noteOpenEdge = settings.noteRim;
        break;

      case 9:
    	  settings.noteCup = settings.noteCup - UP[itemNumber];
        if (settings.noteCup == 255)
        {
        	settings.noteCup = 127;
        }
//        EEPROM.write((padNum * 10) + 9, noteCup);
        noteCloseEdge = settings.noteCup;
        noteCross = settings.noteCup;
        break;
      }
      change = true;
      push = true;
      buttonState = false;
      delay(30);
    }

    if (itemNumber == 0)
    {
      value = settings.sensitivity;
    }

    else if (itemNumber == 1)
    {
      value = settings.threshold1;
    }

    else if (itemNumber == 2)
    {
      value = settings.scantime;
    }

    else if (itemNumber == 3)
    {
      value = settings.masktime;
    }

    else if (itemNumber == 4)
    {
      value = settings.rimSensitivity;
    }

    else if (itemNumber == 5)
    {
      value = settings.rimThreshold;
    }

    else if (itemNumber == 6)
    {
      value = settings.curvetype;
    }

    else if (itemNumber == 7)
    {
      value = settings.note;
    }

    else if (itemNumber == 8)
    {
      value = settings.noteRim;
    }

    else if (itemNumber == 9)
    {
      value = settings.noteCup;
    }

    showValue = value;
  }
}

void HelloDrum::settingName(const char *instrumentName)
{
  //Store the name of the pad in the array.
  showInstrument[nameIndex] = instrumentName;
  nameIndexMax = nameIndex;
  nameIndex++;
}

void HelloDrum::loadPadsSettings(){
	for (HelloDrum* pad : padsList) {
		pad->loadMemory();
	}
}

void HelloDrum::loadMemory()
{
  //Read values from EEPROM.
	PadInEeprom tempStruct = EepromManager::getInstance()->readPad(padNum);
	settings = tempStruct.pad;

	noteOpen = 			settings.note;
	noteEdge = 			settings.noteRim;
	noteClose = 		settings.noteRim;
	noteOpenEdge = 		settings.noteRim;
	noteCloseEdge = 	settings.noteCup;
	noteCross = 		settings.noteCup;

	// TODO Remove duplicates of parameters.
}

void HelloDrum::loadPadsSounds(){
	for (HelloDrum* pad : padsList) {
		pad->loadSounds();
	}
}

void HelloDrum::loadSounds(){
	uint32_t address = 0;
	float volumeDb = 0.0;
	if(settings.soundHeadAddressId != UNCORRECT_SOUND_ID){
		address = EepromManager::getInstance()->infoSector.soundsAdresses[settings.soundHeadAddressId];
		noteHeadSound = new DrumSound(address, settings.soundHeadVolumeDb);
	}
	if(settings.soundRimAddressId != UNCORRECT_SOUND_ID){
		address = EepromManager::getInstance()->infoSector.soundsAdresses[settings.soundRimAddressId];
		noteRimSound = new DrumSound(address, settings.soundRimVolumeDb);
	}
	if(settings.soundCupAddressId != UNCORRECT_SOUND_ID){
		address = EepromManager::getInstance()->infoSector.soundsAdresses[settings.soundCupAddressId];
		noteCupSound = new DrumSound(address, settings.soundCupVolumeDb);
	}
}


void HelloDrum::sensingAllPads(){
	for (HelloDrum* pad : padsList) {
		pad->sensingPad();
	}
}

void HelloDrum::executeAllPads(){
	for (HelloDrum* pad : padsList) {
		pad->executePad();
	}
}
