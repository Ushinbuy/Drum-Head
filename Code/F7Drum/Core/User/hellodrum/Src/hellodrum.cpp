/*
  "HELLO DRUM LIBRARY" Ver.0.7.7
  
  by Ryo Kosaka

  GitHub : https://github.com/RyoKosaka/HelloDrum-arduino-Library
  Blog : https://open-e-drums.tumblr.com/
*/

//#define DEBUG_DRUM //<-- uncomment this line to enable debug mode with Serial.

#include <hellodrum.hpp>
#include <math.h>
#include <stdio.h>
#include "drumManager.h"
#include "eepromManager.h"

extern "C" void sendUart(const char *_msg);
extern char buffer_out[1000];

#define DEBUG_DRUM

//#include "Arduino.h"
//#include "EEPROM.h"

//Pad with a sensor.
HelloDrum::HelloDrum(byte pin1)
{
  pin_1 = pin1;

//  //initial EEPROM value
//  settings.sensitivity = 100;   //0
//  settings.threshold1 = 10;     //1
//  settings.scantime = 10;       //2
//  param.masktime = 30;       //3
//  param.rimSensitivity = 20; //4 edgeThreshold
//  param.rimThreshold = 3;    //5 cupThreshold
//  param.curvetype = 0;       //6
//  param.note = 38;           //7
//  param.noteRim = 39;        //8
//  param.noteCup = 40;        //9

  //Give the instance a pad number.
  padNum = padIndex;
  padIndex++;
  channelsAmount++;
}

//Pad with 2 sensors.
HelloDrum::HelloDrum(byte pin1, byte pin2)
{
  pin_1 = pin1;
  pin_2 = pin2;

//  //initial value
//  sensitivity = 100;   //0
//  threshold1 = 10;     //1
//  scantime = 10;       //2
//  masktime = 30;       //3
//  rimSensitivity = 20; //4 edgeThreshold
//  rimThreshold = 3;    //5 cupThreshold
//  curvetype = 0;       //6
//  note = 38;           //7
//  noteRim = 39;        //8
//  noteCup = 40;        //9

  //Give the instance a pad number.
  padNum = padIndex;
  padIndex++;
  channelsAmount += 2;
}

////MUX(4051) pin define
//HelloDrumMUX_4051::HelloDrumMUX_4051(byte pin1, byte pin2, byte pin3, byte pinA) //s0,s1,s2, analogPin
//{
//  pin_1 = pin1;
//  pin_2 = pin2;
//  pin_3 = pin3;
//  pin_A = pinA;
//  selectPins[0] = pin_1;
//  selectPins[1] = pin_2;
//  selectPins[2] = pin_3;
//
//  for (byte i = 0; i < 3; i++)
//  {
//    pinMode(selectPins[i], OUTPUT);
//    digitalWrite(selectPins[i], HIGH);
//  }
//  muxNum = muxIndex;
//  muxIndex++;
//}
//
////MUX(4067) pin define
//HelloDrumMUX_4067::HelloDrumMUX_4067(byte pin1, byte pin2, byte pin3, byte pin4, byte pinA) //s0,s1,s2,s3,analogPin
//{
//  pin_1 = pin1;
//  pin_2 = pin2;
//  pin_3 = pin3;
//  pin_4 = pin4;
//  pin_A = pinA;
//  selectPins[0] = pin_1;
//  selectPins[1] = pin_2;
//  selectPins[2] = pin_3;
//  selectPins[3] = pin_4;
//
//  for (byte i = 0; i < 4; i++)
//  {
//    pinMode(selectPins[i], OUTPUT);
//    digitalWrite(selectPins[i], HIGH);
//  }
//  muxNum = muxIndex;
//  muxIndex++;
//}

//control button
HelloDrumButton::HelloDrumButton(byte pin1, byte pin2, byte pin3, byte pin4, byte pin5)
{
  pin_1 = pin1; //EDIT
  pin_2 = pin2; //UP
  pin_3 = pin3; //DOWN
  pin_4 = pin4; //NEXT
  pin_5 = pin5; //BACK
}

uint8_t HelloDrum::getChannelsAmount(void){
	return channelsAmount;
}

//control button
HelloDrumButtonLcdShield::HelloDrumButtonLcdShield(byte pin1)
{
  pin_1 = pin1; //button's analog pin
}

//Knob
HelloDrumKnob::HelloDrumKnob(byte pin1)
{
  pin_1 = pin1;
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

#ifdef DEBUG_DRUM
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
      sprintf(buffer_out, "[Hit] velocity : %d (raw value : %d), loopTimes : %d, ScanTime(ms) : %ld",
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

void HelloDrum::dualPiezoSensing(byte sens, byte thre, byte scanTime, byte maskTime, byte rimSens, byte rimThre)
{
  int Threshold = thre * 10;
  int Sensitivity = sens * 10;
  int RimThreshold = rimThre * 10;
  int RimSensitivity = rimSens * 10;

  hit = false;
  hitRim = false;

  //when the value > threshold
  if ((piezoValue > Threshold && loopTimes == 0) || (RimPiezoValue > Threshold && loopTimes == 0))
  {
    time_hit = millis();

    if (time_hit - time_end < maskTime)
    {
      return;
    }
    else
    {
      velocity = piezoValue; //first peak
      velocityRim = RimPiezoValue;
      loopTimes = 1;
    }
  }

  //peak scan start
  if (loopTimes > 0)
  {
#ifdef DEBUG_DRUM
	  sprintf(buffer_out, "%d, %d\n",
			  piezoValue,
			  RimPiezoValue);
	  	  sendUart(buffer_out);
#endif
    if (piezoValue > velocity)
    {
      velocity = piezoValue;
    }
    if (RimPiezoValue > velocityRim)
    {
      velocityRim = RimPiezoValue;
    }
    loopTimes++;

    //scan end
    if (millis() - time_hit >= scanTime)
    {

#ifdef DEBUG_DRUM
      int prevVel = velocity;
      int prevVelR = velocityRim;
#endif

      time_end = millis();

      if ((velocity - velocityRim < RimSensitivity) && (velocityRim > RimThreshold))
      {

        velocity = curve(velocity, Threshold, Sensitivity, settings.curvetype);
        velocityRim = curve(velocityRim, Threshold, Sensitivity, settings.curvetype);

#ifdef DEBUG_DRUM
        sprintf(buffer_out, "[HitRim] velocity : %d , velocity rim : %d (raw value : %d, %d, head - rim : %d), loopTimes : %d, ScanTime(ms) : %ld",
			  velocity,
			  velocityRim,
			  prevVel,
			  prevVelR,
			  (prevVel - prevVelR),
			  loopTimes,
			  (time_end - time_hit));
	  sendUart(buffer_out);
#endif
        velocity = velocityRim;
        hitRim = true;
      }

      else
      {

        velocity = curve(velocity, Threshold, Sensitivity, settings.curvetype);
        velocityRim = curve(velocityRim, Threshold, Sensitivity, settings.curvetype);

#ifdef DEBUG_DRUM
        sprintf(buffer_out, "[HitRim] velocity : %d , velocity rim : %d (raw value : %d, %d, d : %d), loopTimes : %d, ScanTime(ms) : %ld",
			  velocity,
			  velocityRim,
			  prevVel,
			  prevVelR,
			  (prevVel - prevVelR),
			  loopTimes,
			  (time_end - time_hit));
	  sendUart(buffer_out);
#endif
        hit = true;
      }

      //Those 3 lines are for "Eeprom version"
      //I think that are ignore in simple mode, but otw we can pass a flag and skip that.
      showVelocity = velocity;
      showLCD = true;
      padIndex = padNum;

      loopTimes = 0;
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
#ifdef DEBUG_DRUM
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
		sprintf(buffer_out, "[Hit Bow] velocity : %d, (raw value : %d, firstSensorValue : %d, lastSensorValue : %d), loopTimes : %d, ScanTime(ms) : %ld",
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
		sprintf(buffer_out, "[Hit Edge] velocity : %d, (raw value : %d, firstSensorValue : %d, lastSensorValue : %d), loopTimes : %d, ScanTime(ms) : %ld",
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
		sprintf(buffer_out, "[Choke] firstSensorValue : %d, lastSensorValue : %d, loopTimes : %d, ScanTime(ms) : %ld",
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

void HelloDrum::cymbal3zoneSensing(byte sens, byte thre, byte scanTime, byte maskTime, byte edgeThre, byte cupThre)
{
  int Threshold = thre * 10;
  int Sensitivity = sens * 10;
  int edgeThreshold = edgeThre * 10;
  int cupThreshold = cupThre * 10;

  hit = false;
  hitRim = false;
  hitCup = false;
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
#ifdef DEBUG_DRUM
	sprintf(buffer_out, "%d, %d, %d",
			piezoValue,
			sensorValue,
			abs(piezoValue - sensorValue));
	sendUart(buffer_out);
#endif
    if (abs(piezoValue - sensorValue) > velocity)
    {
      velocity = abs(piezoValue - sensorValue);
      velocityCup = velocity;
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
      if (velocity > Threshold && firstSensorValue < edgeThreshold && lastSensorValue < edgeThreshold)
      {
        velocity = curve(velocity, Threshold, Sensitivity, settings.curvetype);
#ifdef DEBUG_DRUM
		sprintf(buffer_out, "[Hit Bow] velocity : %d, (raw value : %d, firstSensorValue : %d, lastSensorValue : %d), loopTimes : %d, ScanTime(ms) : %ld",
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
      else if (velocity > Threshold && firstSensorValue > edgeThreshold && firstSensorValue < cupThreshold && firstSensorValue > lastSensorValue)
      {
        velocity = curve(velocity, Threshold, Sensitivity, settings.curvetype);
#ifdef DEBUG_DRUM
		sprintf(buffer_out, "[Hit Edge] velocity : %d, (raw value : %d, firstSensorValue : %d, lastSensorValue : %d), loopTimes : %d, ScanTime(ms) : %ld",
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

      //cup
      else if (velocity > Threshold && firstSensorValue > cupThreshold && lastSensorValue < edgeThreshold)
      {
        velocity = curve(velocity, Threshold, Sensitivity, settings.curvetype);
#ifdef DEBUG_DRUM
		sprintf(buffer_out, "[Hit Cup] velocity : %d, (raw value : %d, firstSensorValue : %d, lastSensorValue : %d), loopTimes : %d, ScanTime(ms) : %ld",
			  velocity,
			  prevVel,
			  firstSensorValue,
			  lastSensorValue,
			  loopTimes,
			  (time_end - time_hit));
		sendUart(buffer_out);
#endif
        hitCup = true;
        showVelocity = velocity;
        showLCD = true;
        padIndex = padNum;
      }

      //choke
      else if (firstSensorValue > edgeThreshold && lastSensorValue > edgeThreshold && lastSensorValue >= firstSensorValue)
      {
#ifdef DEBUG_DRUM
  		sprintf(buffer_out, "[Choke] firstSensorValue : %d, lastSensorValue : %d, loopTimes : %d, ScanTime(ms) : %ld",
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

//This is OLD CODE!
void HelloDrum::TCRT5000Sensing(byte sens, byte thre1, byte thre2, byte scanTime)
{
	// TODO remove ESP it comment can be use for understanding
#ifdef ESP32
  int thre1Raw = thre1 * 40;
  int thre2Raw = thre2 * 40;
  int sensRaw = sens * 40;
  TCRT = 4096 - TCRT;
#else
  int thre1Raw = thre1 * 10;
  int thre2Raw = thre2 * 10;
  int sensRaw = sens * 10;
  TCRT = 1024 - TCRT;
#endif

  velocity = 0;
  openHH = false;
  closeHH = false;

  //thre2 : first trigger (start to close)
  if (TCRT > thre2Raw && closeHH == false && pedalVelocityFlag == false && pedalFlag == false)
  {
    time_hit_pedal_1 = millis();
    pedalVelocityFlag = true;
  }

  //sensitivity : second trigger (close)
  else if (TCRT > sensRaw && pedalFlag == false)
  {
    time_hit_pedal_2 = millis();

    velocity = time_hit_pedal_2 - time_hit_pedal_1;
    velocity = map(velocity, scanTime * 100, 0, 1, 127); //?

    if (velocity <= 1)
    {
      velocity = 1;
    }

    if (velocity > 127)
    {
      velocity = 127;
    }

    closeHH = true;
    openHH = false;
    pedalFlag = true;
    pedalVelocityFlag = false;
  }

  if (TCRT < thre2Raw && pedalFlag == true)
  {
    pedalFlag = false;
    closeHH = false;
    openHH = true;
  }

  //Pedal CC
  //TCRT = map(TCRT, thre1Raw, sensRaw, 0, 127);
  TCRT = curve(TCRT, thre1Raw, sensRaw, settings.curvetype);

  if (TCRT < 20)
  {
    TCRT = 0;
  }

  else if (TCRT >= 20 && TCRT < 40)
  {
    TCRT = 20;
  }

  else if (TCRT >= 40 && TCRT < 60)
  {
    TCRT = 40;
  }

  else if (TCRT >= 60 && TCRT < 80)
  {
    TCRT = 60;
  }

  else if (TCRT >= 80 && TCRT < 100)
  {
    TCRT = 80;
  }

  else if (TCRT >= 100 && TCRT < 120)
  {
    TCRT = 100;
  }

  else if (TCRT >= 120)
  {
    TCRT = 127;
  }

  if (exTCRT != TCRT)
  {
    pedalCC = TCRT;
    moving = true;
    exTCRT = TCRT;
  }

  else
  {
    moving = false;
  }
}

void HelloDrum::FSRSensing(byte sens, byte thre, byte scanStart, byte scanEnd, byte pedalSens)
{
  int sensRaw = sens * 10;
  int thre1Raw = thre * 10;
  int ScanStart = scanStart * 10;
  int ScanEnd = scanEnd * 10;

  hit = false;
  velocity = 0;
  //openHH = false;
  //closeHH = false;

  //scan start
  if (fsr > ScanStart && closeHH == false && pedalVelocityFlag == false && pedalFlag == false)
  {
    time_hit_pedal_1 = millis();
    pedalVelocityFlag = true;
  }

  //scan end
  else if (fsr > ScanEnd && pedalFlag == false)
  {
    time_hit_pedal_2 = millis();

    velocity = time_hit_pedal_2 - time_hit_pedal_1;

#ifdef DEBUG_DRUM
    int prevVel = velocity;
#endif

    velocity = map(velocity, pedalSens * 100, 0, 1, 127);

    if (velocity <= 1)
    {
      velocity = 1;
    }

    if (velocity > 127)
    {
      velocity = 127;
    }

#ifdef DEBUG_DRUM
	sprintf(buffer_out, "[Close] velocity : %d, ScanTime(ms) : %d",
		  velocity,
		  prevVel);
	sendUart(buffer_out);
#endif

    hit = true;
    closeHH = true;
    openHH = false;
    pedalFlag = true;
    pedalVelocityFlag = false;

    showVelocity = velocity;
    showLCD = true;
    padIndex = padNum;
  }

#ifdef DEBUG_DRUM
  int prevFsr = fsr;
#endif
  //
  if (fsr < ScanEnd && pedalFlag == true)
  {
#ifdef DEBUG_DRUM
	sprintf(buffer_out, "[Open] sensorValue :  %d", fsr);
	sendUart(buffer_out);
#endif
    pedalFlag = false;
    closeHH = false;
    openHH = true;
  }

  //Pedal CC
  fsr = curve(fsr, thre1Raw, sensRaw, settings.curvetype);

  if (fsr < 20)
  {
    fsr = 0;
  }

  else if (fsr >= 20 && fsr < 40)
  {
    fsr = 20;
  }

  else if (fsr >= 40 && fsr < 60)
  {
    fsr = 40;
  }

  else if (fsr >= 60 && fsr < 80)
  {
    fsr = 60;
  }

  else if (fsr >= 80 && fsr < 100)
  {
    fsr = 80;
  }

  else if (fsr >= 100 && fsr < 120)
  {
    fsr = 100;
  }

  else if (fsr >= 120)
  {
    fsr = 127;
  }

  if (exFSR != fsr)
  {
    pedalCC = fsr;
    moving = true;
    exFSR = fsr;

#ifdef DEBUG_DRUM
	sprintf(buffer_out, "[Move] sensorValue : %d, (raw value :  %d), openHH : %d, closeHH : %d",
		fsr,
		prevFsr,
		openHH,
		closeHH);
	sendUart(buffer_out);
#endif
  }

  else
  {
    moving = false;
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

///////////////////// 2. PAD without EEPROM //////////////////////////

void HelloDrum::singlePiezo(byte sens, byte thre, byte scan, byte mask)
{
  padType[padNum] = Snum;
  piezoValue = analogRead(pin_1);
  singlePiezoSensing(sens, thre, scan, mask);
}

void HelloDrum::dualPiezo(byte sens, byte thre, byte scan, byte mask, byte rimSens, byte rimThre)
{
  padType[padNum] = Dnum;
  piezoValue = analogRead(pin_1);
  RimPiezoValue = analogRead(pin_2);
  dualPiezoSensing(sens, thre, scan, mask, rimSens, rimThre);
}

void HelloDrum::HH(byte sens, byte thre, byte scan, byte mask)
{
  padType[padNum] = HHnum;
  piezoValue = analogRead(pin_1);
  singlePiezoSensing(sens, thre, scan, mask);
}

void HelloDrum::HH2zone(byte sens, byte thre, byte scan, byte mask, byte edgeThre)
{
  padType[padNum] = HH2num;
  piezoValue = analogRead(pin_1);
  sensorValue = analogRead(pin_2);
  cymbal2zoneSensing(sens, thre, scan, mask, edgeThre);
}

void HelloDrum::cymbal2zone(byte sens, byte thre, byte scan, byte mask, byte edgeThre)
{
  padType[padNum] = CY2num;
  piezoValue = analogRead(pin_1);
  sensorValue = analogRead(pin_2);
  cymbal2zoneSensing(sens, thre, scan, mask, edgeThre);
}

void HelloDrum::cymbal3zone(byte sens, byte thre, byte scan, byte mask, byte edgeThre, byte cupThre)
{
  padType[padNum] = CY3num;
  piezoValue = analogRead(pin_1);
  sensorValue = analogRead(pin_2);
  cymbal3zoneSensing(sens, thre, scan, mask, edgeThre, cupThre);
}

void HelloDrum::TCRT5000(byte sens, byte thre1, byte thre2, byte scan)
{
  padType[padNum] = HHCnum;
  TCRT = analogRead(pin_1);
  TCRT5000Sensing(sens, thre1, thre2, scan);
}

void HelloDrum::FSR(byte sens, byte thre, byte scanStart, byte scanEnd, byte pedalSens)
{
  padType[padNum] = HHCnum;
  fsr = analogRead(pin_1);
  FSRSensing(sens, thre, scanStart, scanEnd, pedalSens);
}

void HelloDrum::hihatControl(byte sens, byte thre, byte scanStart, byte scanEnd, byte pedalSens)
{
  padType[padNum] = HHCnum;
  fsr = analogRead(pin_1);
  FSRSensing(sens, thre, scanStart, scanEnd, pedalSens);
}

////////////////// 3. PAD WITH LCD & EEPROM //////////////////////

void HelloDrum::singlePiezo()
{
  padType[padNum] = Snum;
  piezoValue = analogRead(pin_1);
  singlePiezoSensing(settings.sensitivity, settings.threshold1, settings.scantime, settings.masktime);
}

void HelloDrum::dualPiezo()
{
  padType[padNum] = Dnum;
  piezoValue = analogRead(pin_1);
  RimPiezoValue = analogRead(pin_2);
  dualPiezoSensing(settings.sensitivity, settings.threshold1, settings.scantime, settings.masktime, settings.rimSensitivity, settings.rimThreshold);
}

void HelloDrum::HH()
{
  padType[padNum] = HHnum;
  piezoValue = analogRead(pin_1);
  singlePiezoSensing(settings.sensitivity, settings.threshold1, settings.scantime, settings.masktime);
}

void HelloDrum::HH2zone()
{
  padType[padNum] = HH2num;
  piezoValue = analogRead(pin_1);
  sensorValue = analogRead(pin_2);
  cymbal2zoneSensing(settings.sensitivity, settings.threshold1, settings.scantime, settings.masktime, settings.rimSensitivity);
}

void HelloDrum::cymbal2zone()
{
  padType[padNum] = CY2num;
  piezoValue = analogRead(pin_1);
  sensorValue = analogRead(pin_2);
  cymbal2zoneSensing(settings.sensitivity, settings.threshold1, settings.scantime, settings.masktime, settings.rimSensitivity);
}

void HelloDrum::cymbal3zone()
{
  padType[padNum] = CY3num;
  piezoValue = analogRead(pin_1);
  sensorValue = analogRead(pin_2);
  cymbal3zoneSensing(settings.sensitivity, settings.threshold1, settings.scantime, settings.masktime, settings.rimSensitivity, settings.rimThreshold);
}

void HelloDrum::TCRT5000()
{
  padType[padNum] = HHCnum;
  TCRT = analogRead(pin_1);
  TCRT5000Sensing(settings.sensitivity, settings.threshold1, settings.masktime, settings.scantime);
}

void HelloDrum::FSR()
{
  padType[padNum] = HHCnum;
  fsr = analogRead(pin_1);
  FSRSensing(settings.sensitivity, settings.threshold1, settings.scantime, settings.masktime, settings.rimSensitivity);
}

void HelloDrum::hihatControl()
{
  padType[padNum] = HHCnum;
  fsr = analogRead(pin_1);
  FSRSensing(settings.sensitivity, settings.threshold1, settings.scantime, settings.masktime, settings.rimSensitivity);
}

//////////////////////////// 6. EEPROM SETTING  //////////////////////////////

void HelloDrum::settingEnable()
{
	// TODO Override this function to save one time when you get out
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

void HelloDrum::initSounds(){
	uint32_t address = 0;
	float volumeDb = 0.0;
	if(settings.soundHeadAddressId != 0xff){
		address = EepromManager::getInstance()->infoSector.soundsAdresses[settings.soundHeadAddressId];
		noteHeadSound = new DrumSound(address, settings.soundHeadVolumeDb);
	}
	if(settings.soundRimAddressId != 0xff){
		address = EepromManager::getInstance()->infoSector.soundsAdresses[settings.soundRimAddressId];
		noteRimSound = new DrumSound(address, settings.soundRimVolumeDb);
	}
	if(settings.soundCupAddressId != 0xff){
		address = EepromManager::getInstance()->infoSector.soundsAdresses[settings.soundCupAddressId];
		noteCupSound = new DrumSound(address, settings.soundCupVolumeDb);
	}
}

void HelloDrum::initMemory()
{
	// this function is deprecated

  //Write initial value to EEPROM.
}

///////////////////// 7. BUTONN //////////////////////////

void HelloDrumButton::readButton(bool button_set, bool button_up, bool button_down, bool button_next, bool button_back)
{
  ////////////////////////////// EDIT START////////////////////////////////

  if (nameIndex > nameIndexMax) // Reset nameIndex
  {
    nameIndex = 0;
  }

  if (button_set == LOW && buttonState == true && editCheck == false)
  {
    editCheck = true;
    edit = true;
    buttonState = false;
    buttonState_set = false;
    delay(30);
  }

  if (button_set == LOW && buttonState == true && editCheck == true)
  {
    editCheck = false;
    editdone = true;
    buttonState = false;
    buttonState_set = true;
    delay(30);
  }

  if (button_set == HIGH)
  {
    edit = false;
    editdone = false;
  }

  /////////////////////////// UP DOWN ///////////////////////////////////////

  if (button_up == LOW && buttonState == true && editCheck == false)
  {
    UPDOWN++;

    if (UPDOWN < 0)
    {
      UPDOWN = nameIndexMax;
    }
    if (UPDOWN > nameIndexMax)
    {
      UPDOWN = 0;
    }

    nameIndex = UPDOWN;
    itemNumber = 0;
    itemNumberShow = 0;
    NEXTBACK = 0;

    push = true;
    buttonState = false;

#ifdef DEBUG_DRUM
	sprintf(buffer_out, "itemNumber : %d, itemNumberShow : %d, nameIndex : %d, padType : %d, settingItem : %s",
			itemNumber,
			itemNumberShow,
			nameIndex,
			padType[nameIndex],
			GetSettingItem());
	sendUart(buffer_out);
#endif

    delay(30);
  }

  if (button_down == LOW && buttonState == true && editCheck == false)
  {
    UPDOWN--;

    if (UPDOWN < 0)
    {
      UPDOWN = nameIndexMax;
    }
    if (UPDOWN > nameIndexMax)
    {
      UPDOWN = 0;
    }
    nameIndex = UPDOWN;
    itemNumber = 0;
    itemNumberShow = 0;
    NEXTBACK = 0;

    push = true;
    buttonState = false;

#ifdef DEBUG_DRUM
	sprintf(buffer_out, "itemNumber : %d, itemNumberShow : %d, nameIndex : %d, padType : %d, settingItem : %s",
			itemNumber,
			itemNumberShow,
			nameIndex,
			padType[nameIndex],
			GetSettingItem());
	sendUart(buffer_out);
#endif

    delay(30);
  }

  ///////////////////////////// NEXT BACK ////////////////////////////////

  if (button_next == LOW && buttonState == true && editCheck == false)
  {
    //NEXTBACK++;
    //itemNumberShow = NEXTBACK;
    //itemNumber = NEXTBACK;

    itemNumberShow++;
    itemNumber++;

    //itemNumber : for EEPROM

    if (itemNumber == 4)
    {
      if (padType[nameIndex] == HHnum || padType[nameIndex] == Snum)
      {
        itemNumber = 6;
      }
    }

    else if (itemNumber == 5)
    {
      if (padType[nameIndex] == CY2num || padType[nameIndex] == HH2num || padType[nameIndex] == HHCnum)
      {
        itemNumber = 6;
      }
    }

    else if (itemNumber == 8)
    {
      if (padType[nameIndex] == Snum)
      {
        itemNumber = 0;
      }
    }

    else if (itemNumber == 9)
    {
      if (padType[nameIndex] == CY2num || padType[nameIndex] == HHnum || padType[nameIndex] == HH2num)
      {
        itemNumber = 0;
      }
    }

    else if (itemNumber == 10)
    {
      itemNumber = 0;
    }

    //itemNumberShow : for LCD or OLED

    if (itemNumberShow == 6)
    {
      if (padType[nameIndex] == Snum)
      {
        itemNumberShow = 0;
      }
    }

    else if (itemNumberShow == 7)
    {
      if (padType[nameIndex] == HHnum)
      {
        itemNumberShow = 0;
      }
    }

    else if (itemNumberShow == 8)
    {
      if (padType[nameIndex] == HH2num || padType[nameIndex] == CY2num)
      {
        itemNumberShow = 0;
      }
    }

    else if (itemNumberShow == 9)
    {
      if (padType[nameIndex] == HHCnum)
      {
        itemNumberShow = 0;
      }
    }

    else if (itemNumberShow == 10)
    {
      if (padType[nameIndex] == Dnum || padType[nameIndex] == CY3num)
      {
        itemNumberShow = 0;
      }
    }

    nameIndex = UPDOWN;
    push = true;
    buttonState = false;

#ifdef DEBUG_DRUM
	sprintf(buffer_out, "itemNumber : %d, itemNumberShow : %d, nameIndex : %d, padType : %d, settingItem : %s",
			itemNumber,
			itemNumberShow,
			nameIndex,
			padType[nameIndex],
			GetSettingItem());
	sendUart(buffer_out);
#endif

    delay(30);
  }

  if (button_back == LOW && buttonState == true && editCheck == false)
  {
    //NEXTBACK--;

    itemNumberShow--;
    itemNumber--;

    //itemNumber : for EEPROM

    if (itemNumber == 255)
    {
      if (padType[nameIndex] == Snum)
      {
        itemNumber = 7;
      }
      else if (padType[nameIndex] == CY2num || padType[nameIndex] == HHnum || padType[nameIndex] == HH2num)
      {
        itemNumber = 8;
      }
      else if (padType[nameIndex] == Dnum || padType[nameIndex] == CY3num || padType[nameIndex] == HHCnum)
      {
        itemNumber = 9;
      }
    }

    else if (itemNumber == 5)
    {
      if (padType[nameIndex] == HHnum || padType[nameIndex] == Snum)
      {
        itemNumber = 3;
      }

      else if (padType[nameIndex] == CY2num || padType[nameIndex] == HHCnum || padType[nameIndex] == HH2num)
      {
        itemNumber = 4;
      }
    }

    else if (itemNumber == 8)
    {
      if (padType[nameIndex] == Snum)
      {
        itemNumber = 7;
      }
    }

    else if (itemNumber == 9)
    {
      if (padType[nameIndex] == Snum)
      {
        itemNumber = 7;
      }

      else if (padType[nameIndex] == CY2num || padType[nameIndex] == HHnum || padType[nameIndex] == HHCnum || padType[nameIndex] == HH2num)
      {
        itemNumber = 8;
      }
    }

    //itemNumberShow : for LCD or OLED

    if (itemNumberShow == 255)
    {
      if (padType[nameIndex] == Snum)
      {
        itemNumberShow = 5;
      }
      else if (padType[nameIndex] == HHnum)
      {
        itemNumberShow = 6;
      }
      else if (padType[nameIndex] == CY2num || padType[nameIndex] == HH2num)
      {
        itemNumberShow = 7;
      }
      else if (padType[nameIndex] == HHCnum)
      {
        itemNumberShow = 8;
      }
      else if (padType[nameIndex] == Dnum || padType[nameIndex] == CY3num)
      {
        itemNumberShow = 9;
      }
    }

    nameIndex = UPDOWN;
    push = true;
    buttonState = false;

#ifdef DEBUG_DRUM
	sprintf(buffer_out, "itemNumber : %d, itemNumberShow : %d, nameIndex : %d, padType : %d, settingItem : %s",
			itemNumber,
			itemNumberShow,
			nameIndex,
			padType[nameIndex],
			GetSettingItem());
	sendUart(buffer_out);
#endif

    delay(30);
  }

  //When you take your hand off the button
  if (buttonState == false && button_up == HIGH && button_down == HIGH && button_next == HIGH && button_back == HIGH && button_set == HIGH)
  {
    push = false;
    buttonState = true;
    change = false;
  }
}

void HelloDrumButton::readButtonState()
{

//  pinMode(pin_1, INPUT_PULLUP);
//  pinMode(pin_2, INPUT_PULLUP);
//  pinMode(pin_3, INPUT_PULLUP);
//  pinMode(pin_4, INPUT_PULLUP);
//  pinMode(pin_5, INPUT_PULLUP);
//
//  button_set = digitalRead(pin_1);
//  button_up = digitalRead(pin_2);
//  button_down = digitalRead(pin_3);
//  button_next = digitalRead(pin_4);
//  button_back = digitalRead(pin_5);
//
//  readButton(button_set, button_up, button_down, button_next, button_back);
}

/////////////////// 7-2. BUTTON (LCD KEYPAD SHIELD) ////////////////////////

void HelloDrumButtonLcdShield::readButton(bool button_set, bool button_up, bool button_down, bool button_next, bool button_back)
{
  ////////////////////////////// EDIT START////////////////////////////////

  if (nameIndex > nameIndexMax) // Reset nameIndex
  {
    nameIndex = 0;
  }

  if (button_set == LOW && buttonState == true && editCheck == false)
  {
    editCheck = true;
    edit = true;
    buttonState = false;
    buttonState_set = false;
    delay(30);
  }

  if (button_set == LOW && buttonState == true && editCheck == true)
  {
    editCheck = false;
    editdone = true;
    buttonState = false;
    buttonState_set = true;
    delay(30);
  }

  if (button_set == HIGH)
  {
    edit = false;
    editdone = false;
  }

  /////////////////////////// UP DOWN ///////////////////////////////////////

  if (button_up == LOW && buttonState == true && editCheck == false)
  {
    UPDOWN++;

    if (UPDOWN < 0)
    {
      UPDOWN = nameIndexMax;
    }
    if (UPDOWN > nameIndexMax)
    {
      UPDOWN = 0;
    }

    nameIndex = UPDOWN;
    itemNumber = 0;
    itemNumberShow = 0;
    NEXTBACK = 0;

    push = true;
    buttonState = false;

#ifdef DEBUG_DRUM
	sprintf(buffer_out, "itemNumber : %d, itemNumberShow : %d, nameIndex : %d, padType : %d, settingItem : %s",
			itemNumber,
			itemNumberShow,
			nameIndex,
			padType[nameIndex],
			GetSettingItem());
	sendUart(buffer_out);
#endif

    delay(30);
  }

  if (button_down == LOW && buttonState == true && editCheck == false)
  {
    UPDOWN--;

    if (UPDOWN < 0)
    {
      UPDOWN = nameIndexMax;
    }
    if (UPDOWN > nameIndexMax)
    {
      UPDOWN = 0;
    }
    nameIndex = UPDOWN;
    itemNumber = 0;
    itemNumberShow = 0;
    NEXTBACK = 0;

    push = true;
    buttonState = false;

#ifdef DEBUG_DRUM
	sprintf(buffer_out, "itemNumber : %d, itemNumberShow : %d, nameIndex : %d, padType : %d, settingItem : %s",
			itemNumber,
			itemNumberShow,
			nameIndex,
			padType[nameIndex],
			GetSettingItem());
	sendUart(buffer_out);
#endif

    delay(30);
  }

  ///////////////////////////// NEXT BACK ////////////////////////////////

  if (button_next == LOW && buttonState == true && editCheck == false)
  {
    //NEXTBACK++;
    //itemNumberShow = NEXTBACK;
    //itemNumber = NEXTBACK;

    itemNumberShow++;
    itemNumber++;

    //itemNumber : for EEPROM

    if (itemNumber == 4)
    {
      if (padType[nameIndex] == HHnum || padType[nameIndex] == Snum)
      {
        itemNumber = 6;
      }
    }

    else if (itemNumber == 5)
    {
      if (padType[nameIndex] == CY2num || padType[nameIndex] == HH2num || padType[nameIndex] == HHCnum)
      {
        itemNumber = 6;
      }
    }

    else if (itemNumber == 8)
    {
      if (padType[nameIndex] == Snum)
      {
        itemNumber = 0;
      }
    }

    else if (itemNumber == 9)
    {
      if (padType[nameIndex] == CY2num || padType[nameIndex] == HHnum || padType[nameIndex] == HH2num)
      {
        itemNumber = 0;
      }
    }

    else if (itemNumber == 10)
    {
      itemNumber = 0;
    }

    //itemNumberShow : for LCD or OLED

    if (itemNumberShow == 6)
    {
      if (padType[nameIndex] == Snum)
      {
        itemNumberShow = 0;
      }
    }

    else if (itemNumberShow == 7)
    {
      if (padType[nameIndex] == HHnum)
      {
        itemNumberShow = 0;
      }
    }

    else if (itemNumberShow == 8)
    {
      if (padType[nameIndex] == HH2num || padType[nameIndex] == CY2num)
      {
        itemNumberShow = 0;
      }
    }

    else if (itemNumberShow == 9)
    {
      if (padType[nameIndex] == HHCnum)
      {
        itemNumberShow = 0;
      }
    }

    else if (itemNumberShow == 10)
    {
      if (padType[nameIndex] == Dnum || padType[nameIndex] == CY3num)
      {
        itemNumberShow = 0;
      }
    }

    nameIndex = UPDOWN;
    push = true;
    buttonState = false;

#ifdef DEBUG_DRUM
	sprintf(buffer_out, "itemNumber : %d, itemNumberShow : %d, nameIndex : %d, padType : %d, settingItem : %s",
			itemNumber,
			itemNumberShow,
			nameIndex,
			padType[nameIndex],
			GetSettingItem());
	sendUart(buffer_out);
#endif

    delay(30);
  }

  if (button_back == LOW && buttonState == true && editCheck == false)
  {
    //NEXTBACK--;

    itemNumberShow--;
    itemNumber--;

    //itemNumber : for EEPROM

    if (itemNumber == 255)
    {
      if (padType[nameIndex] == Snum)
      {
        itemNumber = 7;
      }
      else if (padType[nameIndex] == CY2num || padType[nameIndex] == HHnum || padType[nameIndex] == HH2num)
      {
        itemNumber = 8;
      }
      else if (padType[nameIndex] == Dnum || padType[nameIndex] == CY3num || padType[nameIndex] == HHCnum)
      {
        itemNumber = 9;
      }
    }

    else if (itemNumber == 5)
    {
      if (padType[nameIndex] == HHnum || padType[nameIndex] == Snum)
      {
        itemNumber = 3;
      }

      else if (padType[nameIndex] == CY2num || padType[nameIndex] == HHCnum || padType[nameIndex] == HH2num)
      {
        itemNumber = 4;
      }
    }

    else if (itemNumber == 8)
    {
      if (padType[nameIndex] == Snum)
      {
        itemNumber = 7;
      }
    }

    else if (itemNumber == 9)
    {
      if (padType[nameIndex] == Snum)
      {
        itemNumber = 7;
      }

      else if (padType[nameIndex] == CY2num || padType[nameIndex] == HHnum || padType[nameIndex] == HHCnum || padType[nameIndex] == HH2num)
      {
        itemNumber = 8;
      }
    }

    //itemNumberShow : for LCD or OLED

    if (itemNumberShow == 255)
    {
      if (padType[nameIndex] == Snum)
      {
        itemNumberShow = 5;
      }
      else if (padType[nameIndex] == HHnum)
      {
        itemNumberShow = 6;
      }
      else if (padType[nameIndex] == CY2num || padType[nameIndex] == HH2num)
      {
        itemNumberShow = 7;
      }
      else if (padType[nameIndex] == HHCnum)
      {
        itemNumberShow = 8;
      }
      else if (padType[nameIndex] == Dnum || padType[nameIndex] == CY3num)
      {
        itemNumberShow = 9;
      }
    }

    nameIndex = UPDOWN;
    push = true;
    buttonState = false;

#ifdef DEBUG_DRUM
	sprintf(buffer_out, "itemNumber : %d, itemNumberShow : %d, nameIndex : %d, padType : %d, settingItem : %s",
			itemNumber,
			itemNumberShow,
			nameIndex,
			padType[nameIndex],
			GetSettingItem());
	sendUart(buffer_out);
#endif

    delay(30);
  }

  //When you take your hand off the button
  if (buttonState == false && button_up == HIGH && button_down == HIGH && button_next == HIGH && button_back == HIGH && button_set == HIGH)
  {
    push = false;
    buttonState = true;
    change = false;
  }
}

void HelloDrumButtonLcdShield::readButtonState()
{

//  pinMode(pin_1, INPUT);
  buttonValue = analogRead(pin_1);

  //read buttons analog value
  if (buttonValue > 1000)
  {
    button_set = HIGH;
    button_up = HIGH;
    button_down = HIGH;
    button_next = HIGH;
    button_back = HIGH;
  }
  else if (buttonValue >= 700 && buttonValue < 1000)
  {
    button_set = LOW;
  }
  else if (buttonValue >= 500 && buttonValue < 700)
  {
    button_back = LOW;
  }
  else if (buttonValue >= 300 && buttonValue < 500)
  {
    button_down = LOW;
  }
  else if (buttonValue >= 50 && buttonValue < 300)
  {
    button_up = LOW;
  }
  else if (buttonValue < 50)
  {
    button_next = LOW;
  }

  readButton(button_set, button_up, button_down, button_next, button_back);
}

//////////////////////  KNOB  /////////////////////////

void HelloDrumKnob::read()
{
  knobValue = analogRead(pin_1);
}

//void HelloDrumKnob::readMUX()
//{
//  knobValue = rawValue[pin_1];
//}

//////////////////////////////////////////////////////////

//For Display

byte HelloDrumButton::GetVelocity()
{
  return showVelocity;
}
byte HelloDrumButton::GetSettingValue()
{
  return showValue;
}
bool HelloDrumButton::GetEditState()
{
  return edit;
}
bool HelloDrumButton::GetDisplayState()
{
  if (showLCD == true)
  {
    showLCD = false;
    showFlag = true;
  }
  else
  {
    showFlag = false;
  }
  return showFlag;
}
bool HelloDrumButton::GetEditdoneState()
{
  return editdone;
}
bool HelloDrumButton::GetPushState()
{
  return push;
}
const char *HelloDrumButton::GetPadName()
{
  return showInstrument[nameIndex];
}
const char *HelloDrumButton::GetSettingItem()
{
  if (padType[nameIndex] == Dnum)
  {
    return itemD[itemNumberShow];
  }
  else if (padType[nameIndex] == CY2num)
  {
    return itemCY2[itemNumberShow];
  }
  else if (padType[nameIndex] == CY3num)
  {
    return itemCY3[itemNumberShow];
  }
  else if (padType[nameIndex] == HH2num)
  {
    return itemHH2[itemNumberShow];
  }
  else if (padType[nameIndex] == HHnum)
  {
    return itemHH[itemNumberShow];
  }
  else if (padType[nameIndex] == HHCnum)
  {
    return itemHHC[itemNumberShow];
  }
  else if (padType[nameIndex] == Snum)
  {
    return item[itemNumberShow];
  }
  else{
	  return "INCORRECT";
  }
}
const char *HelloDrumButton::GetHitPad()
{
  return showInstrument[padIndex];
}

/////////////// For Display Keypad ////////////////////

byte HelloDrumButtonLcdShield::GetVelocity()
{
  return showVelocity;
}
byte HelloDrumButtonLcdShield::GetSettingValue()
{
  return showValue;
}
bool HelloDrumButtonLcdShield::GetEditState()
{
  return edit;
}
bool HelloDrumButtonLcdShield::GetDisplayState()
{
  if (showLCD == true)
  {
    showLCD = false;
    showFlag = true;
  }
  else
  {
    showFlag = false;
  }
  return showFlag;
}
bool HelloDrumButtonLcdShield::GetEditdoneState()
{
  return editdone;
}
bool HelloDrumButtonLcdShield::GetPushState()
{
  return push;
}
const char *HelloDrumButtonLcdShield::GetPadName()
{
  return showInstrument[nameIndex];
}
const char *HelloDrumButtonLcdShield::GetSettingItem()
{
  if (padType[nameIndex] == Dnum)
  {
    return itemD[itemNumberShow];
  }
  else if (padType[nameIndex] == CY2num)
  {
    return itemCY2[itemNumberShow];
  }
  else if (padType[nameIndex] == CY3num)
  {
    return itemCY3[itemNumberShow];
  }
  else if (padType[nameIndex] == HH2num)
  {
    return itemHH2[itemNumberShow];
  }
  else if (padType[nameIndex] == HHnum)
  {
    return itemHH[itemNumberShow];
  }
  else if (padType[nameIndex] == HHCnum)
  {
    return itemHHC[itemNumberShow];
  }
  else if (padType[nameIndex] == Snum)
  {
    return item[itemNumberShow];
  }
  else{
	  return "INCORRECT";
  }
}
const char *HelloDrumButtonLcdShield::GetHitPad()
{
  return showInstrument[padIndex];
}

/*
byte HelloDrumButton::GetItemNumber()
{
  return itemNumber;
}
bool HelloDrumButton::GetChangeState()
{
  return change;
}
 */
