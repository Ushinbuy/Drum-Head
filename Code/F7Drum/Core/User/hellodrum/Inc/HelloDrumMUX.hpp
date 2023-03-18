#ifndef USER_HELLODRUM_INC_HELLODRUMMUX_HPP_
#define USER_HELLODRUM_INC_HELLODRUMMUX_HPP_

//static byte muxIndex = 0;


//class HelloDrumMUX_4051
//{
//public:
//  HelloDrumMUX_4051(byte pin1, byte pin2, byte pin3, byte pinA);
//  void scan();
//  byte selectPins[3];
//  byte muxNum;
//
//private:
//  byte pin_1;
//  byte pin_2;
//  byte pin_3;
//  byte pin_A;
//};



//class HelloDrumMUX_4067
//{
//public:
//  HelloDrumMUX_4067(byte pin1, byte pin2, byte pin3, byte pin4, byte pinA);
//  void scan();
//  byte selectPins[4];
//  byte muxNum;
//
//private:
//  byte pin_1;
//  byte pin_2;
//  byte pin_3;
//  byte pin_4;
//  byte pin_A;
//};


// -------------------- ANOTHER FUNCTIONS ----------------
//  void singlePiezoMUX(byte sens, byte thre, byte scan, byte mask);
//  void singlePiezoMUX();
//  void dualPiezoMUX(byte sens, byte thre, byte scan, byte mask, byte rimSens, byte rimThre);
//  void dualPiezoMUX();
//  void HHMUX(byte sens, byte thre, byte scan, byte mask);
//  void HHMUX();
//  void HH2zoneMUX(byte sens, byte thre, byte scan, byte mask, byte edgeThre);
//  void HH2zoneMUX();
//  void cymbal3zoneMUX(byte sens, byte thre, byte scan, byte mask, byte edgeThre, byte cupThre);
//  void cymbal3zoneMUX();
//  void cymbal2zoneMUX(byte sens, byte thre, byte scan, byte mask, byte edgeThre);
//  void cymbal2zoneMUX();
//  void TCRT5000MUX(byte sens, byte thre1, byte thre2, byte scan);
//  void TCRT5000MUX();
//  void FSRMUX(byte sens, byte thre, byte scanStart, byte scanEnd, byte pedalSens);
//  void FSRMUX();
//  void hihatControlMUX(byte sens, byte thre, byte scanStart, byte scanEnd, byte pedalSens);
//  void hihatControlMUX();



#endif /* USER_HELLODRUM_INC_HELLODRUMMUX_HPP_ */



// in cpp file!!!


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

