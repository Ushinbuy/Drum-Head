#ifndef USER_HELLODRUM_INC_HELLODRUMBUTTON_HPP_
#define USER_HELLODRUM_INC_HELLODRUMBUTTON_HPP_

#include "hellodrum.hpp"

class HelloDrumButton
{
public:
  HelloDrumButton(byte pin1, byte pin2, byte pin3, byte pin4, byte pin5);

  void readButtonState();
  void readButton(bool button_set, bool button_up, bool button_down, bool button_next, bool button_back);

  byte GetSettingValue();
  byte GetVelocity();
  bool GetEditState();
  bool GetEditdoneState();
  bool GetPushState();
  bool GetDisplayState();
  const char *GetPadName();
  const char *GetSettingItem();
  const char *GetHitPad();

  int UPDOWN;
  int NEXTBACK;

private:
  byte pin_1;
  byte pin_2;
  byte pin_3;
  byte pin_4;
  byte pin_5;
};

#endif /* USER_HELLODRUM_INC_HELLODRUMBUTTON_HPP_ */
