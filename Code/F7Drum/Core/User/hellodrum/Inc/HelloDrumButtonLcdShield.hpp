#ifndef USER_HELLODRUM_INC_HELLODRUMBUTTONLCDSHIELD_HPP_
#define USER_HELLODRUM_INC_HELLODRUMBUTTONLCDSHIELD_HPP_

#include "hellodrum.hpp"

class HelloDrumButtonLcdShield
{
public:
  HelloDrumButtonLcdShield(byte pin1);

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
  int buttonValue;
  byte pin_1;
};

#endif /* USER_HELLODRUM_INC_HELLODRUMBUTTONLCDSHIELD_HPP_ */
