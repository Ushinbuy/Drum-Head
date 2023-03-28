#include <HelloDrumPads.hpp>

#define DEBUG_DRUM //<-- uncomment this line to enable debug mode with Serial.
//#define DEBUG_DRUM_VOLTAGE
extern "C" void sendUart(const char *_msg);
extern char buffer_out[1000];


#ifdef DEBUG_DRUM
extern "C" void sendUart(const char *_msg);
extern char buffer_out[1000];
#endif

//////////////// SINGLE ///////////////////
SinglePad::SinglePad(byte pin1) : HelloDrum(pin1){
	padType[padNum] = SINGLE_PAD;
//	padsList.push_back(this);
}

void SinglePad::sensingPad(){
	piezoValue = analogRead(pin_1);
	singlePiezoSensing(settings.sensitivity, settings.threshold1, settings.scantime, settings.masktime);
}

void SinglePad::executePad(){
	if(hit){
		sendMidiGEN(settings.note, velocity);
		if (noteHeadSound != NULL) {
			noteHeadSound->playSound(velocity);
		}
	}
}

//////////////// DOUBLE ///////////////////

DoublePad::DoublePad(byte pin1, byte pin2) : HelloDrum(pin1, pin2){
	padType[padNum] = DOUBLE_PAD;
//	padsList.push_back(this);
}

void DoublePad::sensingPad(){
	piezoValue = analogRead(pin_1);
	RimPiezoValue = analogRead(pin_2);
	dualPiezoSensing(settings.sensitivity, settings.threshold1, settings.scantime, settings.masktime, settings.rimSensitivity, settings.rimThreshold);
}

void DoublePad::executePad(){
	if(hit){
		sendMidiGEN(settings.note, velocity);
		if (noteHeadSound != NULL) {
			noteHeadSound->playSound(velocity);
		}
	}
	if (hitRim) {
		sendMidiGEN(settings.noteRim, velocity);
		if (noteRimSound != NULL) {
			noteRimSound->playSound(velocity);
		}
	}
}

void DoublePad::dualPiezoSensing(byte sens, byte thre, byte scanTime, byte maskTime, byte rimSens, byte rimThre)
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
#ifdef DEBUG_DRUM_VOLTAGE
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
        sprintf(buffer_out, "\n[HitRim] velocity : %d , velocity rim : %d (raw value : %d, %d, head - rim : %d), loopTimes : %d, ScanTime(ms) : %ld",
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
        sprintf(buffer_out, "\n[HitRim] velocity : %d , velocity rim : %d (raw value : %d, %d, d : %d), loopTimes : %d, ScanTime(ms) : %ld",
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

//////////////// HI HAT ///////////////////

HiHatPad::HiHatPad(byte pin1, HiHatPedalPad * pedal) : HelloDrum(pin1){
	this->pedal = pedal;
	padType[padNum] = HH_PAD;
//	padsList.push_back(this);
}

void HiHatPad::sensingPad(){
	piezoValue = analogRead(pin_1);
	singlePiezoSensing(settings.sensitivity, settings.threshold1, settings.scantime, settings.masktime);
}

void HiHatPad::executePad(){
	if(hit){ // TODO here must be another sounds
		if(pedal->openHH){
			sendMidiGEN(noteOpen1, velocity);
			if (noteHeadSound != NULL) {
				noteHeadSound->playSound(velocity);
			}
		}
		else if(pedal->closeHH){
			sendMidiGEN(noteClose1, velocity);
			if (noteHeadSound != NULL) {
				noteHeadSound->playSound(velocity);
			}
		}
	}
}

void HiHatPad::loadMemory(){
	HelloDrum::loadMemory();
	noteOpen1 = settings.note;
	noteClose1 = settings.noteRim;
}
//////////////// HI HAT 2 ZONE ///////////////////

HiHat2zonePad::HiHat2zonePad(byte pin1, byte pin2, HiHatPedalPad * pedal) : HelloDrum(pin1, pin2){
	// TODO copy noteOpen noteClose
	this->pedal = pedal;
	padType[padNum] = HH2_PAD;
//	padsList.push_back(this);
}

void HiHat2zonePad::sensingPad(){
	piezoValue = analogRead(pin_1);
	sensorValue = analogRead(pin_2);
	cymbal2zoneSensing(settings.sensitivity, settings.threshold1,
			settings.scantime, settings.masktime, settings.rimSensitivity);
}

void HiHat2zonePad::executePad(){
	if (hit) {
		if (pedal->openHH) { // TODO here must be another sounds
			sendMidiGEN(noteOpen, velocity);
			if(noteHeadSound != NULL){
				noteHeadSound->playSound(velocity);
			}
		} else if (pedal->closeHH) {
			sendMidiGEN(noteClose, velocity);
			if(noteHeadSound != NULL){
				noteHeadSound->playSound(velocity);
			}
		}
	}
	if (hitRim) {	// TODO here must be another sounds
		if (pedal->openHH) {
			sendMidiGEN(noteOpenEdge, velocity);
			if(noteHeadSound != NULL){
				noteHeadSound->playSound(velocity);
			}
		} else if (pedal->closeHH) {
			sendMidiGEN(noteCloseEdge, velocity);
			if (noteHeadSound != NULL) {
				noteHeadSound->playSound(velocity);
			}
		}
	}
}

void HiHat2zonePad::loadMemory(){
	HelloDrum::loadMemory();
	noteOpen1 = settings.note;
	noteEdge1 = settings.noteRim;
	// TODO notes below must be another
	noteClose1 = settings.noteRim;
	noteOpenEdge = settings.noteRim;
	noteCloseEdge = settings.noteCup;
}
//////////////// HI HAT PEDAL ///////////////////

HiHatPedalPad::HiHatPedalPad(byte pin1, PedalType pedalType) : HelloDrum(pin1){
	this->pedalType = pedalType;
	padType[padNum] = HHC_PAD;
//	padsList.push_back(this);
}

void HiHatPedalPad::sensingPad(){
	switch (pedalType){
	case FSR_PEDAL:
		fsr = analogRead(pin_1);
		FSRSensing(settings.sensitivity, settings.threshold1, settings.scantime,
				settings.masktime, settings.rimSensitivity);
		break;
	case TCRT5000_PEDAL:
		TCRT = analogRead(pin_1);
		TCRT5000Sensing(settings.sensitivity, settings.threshold1,
				settings.masktime, settings.scantime);
	}
}

void HiHatPedalPad::executePad(){
	if(hit){
		sendMidiGEN(settings.note, velocity);
		if(noteHeadSound != NULL){
			noteHeadSound->playSound(velocity);
		}
	}
	if (moving) {
		sendMidiGEN(settings.noteRim, velocity);
		sendMidiControlChange(4, pedalCC);
	}
}


//This is OLD CODE!
void HiHatPedalPad::TCRT5000Sensing(byte sens, byte thre1, byte thre2, byte scanTime)
{
  int thre1Raw = thre1 * 10;
  int thre2Raw = thre2 * 10;
  int sensRaw = sens * 10;
  TCRT = 1024 - TCRT;


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

void HiHatPedalPad::FSRSensing(byte sens, byte thre, byte scanStart, byte scanEnd, byte pedalSens)
{
	// TODO add reverse function
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
	sprintf(buffer_out, "\n[Close] velocity : %d, ScanTime(ms) : %d",
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
	sprintf(buffer_out, "\n[Open] sensorValue :  %d", fsr);
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
	sprintf(buffer_out, "\n[Move] sensorValue : %d, (raw value :  %d), openHH : %d, closeHH : %d",
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

//////////////// CYMBAL 2 ZONES ///////////////////

Cymbal2ZonesPad::Cymbal2ZonesPad(byte pin1, byte pin2) : HelloDrum(pin1, pin2){
	padType[padNum] = CY2_PAD;
//	padsList.push_back(this);
}

void Cymbal2ZonesPad::sensingPad(){
	piezoValue = analogRead(pin_1);
	sensorValue = analogRead(pin_2);
	cymbal2zoneSensing(settings.sensitivity, settings.threshold1, settings.scantime, settings.masktime, settings.rimSensitivity);
}

void Cymbal2ZonesPad::executePad(){
	if (hit == true) {
		sendMidiGEN(settings.note, velocity); //(note, velocity, channel)
		if(noteHeadSound != NULL){
			noteHeadSound->playSound(velocity);
		}
	}

	//edge
	else if (hitRim == true) {
		sendMidiGEN(settings.noteRim, velocity); //(note, velocity, channel)
		if (noteRimSound != NULL) {
			noteRimSound->playSound(velocity);
		}
	}

	//choke
	if (choke == true) {
		sendMidiAT(settings.note, 127);
		sendMidiAT(settings.noteRim, 127);
		sendMidiAT(settings.note, 0);
		sendMidiAT(settings.noteRim, 0);
	}
}

//////////////// CYMBAL 3 ZONES ///////////////////

Cymbal3ZonesPad::Cymbal3ZonesPad(byte pin1, byte pin2) : HelloDrum(pin1, pin2){
	padType[padNum] = CY3_PAD;
//	padsList.push_back(this);
}

void Cymbal3ZonesPad::sensingPad(){
	piezoValue = analogRead(pin_1);
	sensorValue = analogRead(pin_2);
	cymbal3zoneSensing(settings.sensitivity, settings.threshold1,
			settings.scantime, settings.masktime, settings.rimSensitivity,
			settings.rimThreshold);
}

void Cymbal3ZonesPad::executePad(){
	//RIDE//
	//1.bow
	if (hit == true) {
		sendMidiGEN(settings.note, velocity); //(note, velocity, channel)
		if(noteHeadSound != NULL){
			noteHeadSound->playSound(velocity);
		}
	}

	//2.edge
	else if (hitRim == true) {
		sendMidiGEN(settings.noteRim, velocity); //(note, velocity, channel)
		if (noteRimSound != NULL) {
			noteRimSound->playSound(velocity);
		}
	}

	//3.cup
	else if (hitCup == true) {
		sendMidiGEN(settings.noteCup, velocity); //(note, velocity, channel)
		if (noteCupSound != NULL) {
			noteCupSound->playSound(velocity);
		}
	}

	//4.choke
	if (choke == true) {
		sendMidiAT(settings.note, 127);
		sendMidiAT(settings.noteRim, 127);
		sendMidiAT(settings.noteCup, 127);
		sendMidiAT(settings.note, 0);
		sendMidiAT(settings.noteRim, 0);
		sendMidiAT(settings.noteCup, 0);
	}
}


void Cymbal3ZonesPad::cymbal3zoneSensing(byte sens, byte thre, byte scanTime, byte maskTime, byte edgeThre, byte cupThre)
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

      //cup
      else if (velocity > Threshold && firstSensorValue > cupThreshold && lastSensorValue < edgeThreshold)
      {
        velocity = curve(velocity, Threshold, Sensitivity, settings.curvetype);
#ifdef DEBUG_DRUM
		sprintf(buffer_out, "\n[Hit Cup] velocity : %d, (raw value : %d, firstSensorValue : %d, lastSensorValue : %d), loopTimes : %d, ScanTime(ms) : %ld",
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
