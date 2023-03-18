#include "hellodrum.hpp"
#include "drumManager.h"

/////////////// For Display Keypad ////////////////////
/////////////////// 7-2. BUTTON (LCD KEYPAD SHIELD) ////////////////////////

//control button
HelloDrumButtonLcdShield::HelloDrumButtonLcdShield(byte pin1)
{
  pin_1 = pin1; //button's analog pin
}

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
  if (padType[nameIndex] == DOUBLE_PAD)
  {
    return itemD[itemNumberShow];
  }
  else if (padType[nameIndex] == CY2_PAD)
  {
    return itemCY2[itemNumberShow];
  }
  else if (padType[nameIndex] == CY3_PAD)
  {
    return itemCY3[itemNumberShow];
  }
  else if (padType[nameIndex] == HH2_PAD)
  {
    return itemHH2[itemNumberShow];
  }
  else if (padType[nameIndex] == HH_PAD)
  {
    return itemHH[itemNumberShow];
  }
  else if (padType[nameIndex] == HHC_PAD)
  {
    return itemHHC[itemNumberShow];
  }
  else if (padType[nameIndex] == SINGLE_PAD)
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
      if (padType[nameIndex] == HH_PAD || padType[nameIndex] == SINGLE_PAD)
      {
        itemNumber = 6;
      }
    }

    else if (itemNumber == 5)
    {
      if (padType[nameIndex] == CY2_PAD || padType[nameIndex] == HH2_PAD || padType[nameIndex] == HHC_PAD)
      {
        itemNumber = 6;
      }
    }

    else if (itemNumber == 8)
    {
      if (padType[nameIndex] == SINGLE_PAD)
      {
        itemNumber = 0;
      }
    }

    else if (itemNumber == 9)
    {
      if (padType[nameIndex] == CY2_PAD || padType[nameIndex] == HH_PAD || padType[nameIndex] == HH2_PAD)
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
      if (padType[nameIndex] == SINGLE_PAD)
      {
        itemNumberShow = 0;
      }
    }

    else if (itemNumberShow == 7)
    {
      if (padType[nameIndex] == HH_PAD)
      {
        itemNumberShow = 0;
      }
    }

    else if (itemNumberShow == 8)
    {
      if (padType[nameIndex] == HH2_PAD || padType[nameIndex] == CY2_PAD)
      {
        itemNumberShow = 0;
      }
    }

    else if (itemNumberShow == 9)
    {
      if (padType[nameIndex] == HHC_PAD)
      {
        itemNumberShow = 0;
      }
    }

    else if (itemNumberShow == 10)
    {
      if (padType[nameIndex] == DOUBLE_PAD || padType[nameIndex] == CY3_PAD)
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
      if (padType[nameIndex] == SINGLE_PAD)
      {
        itemNumber = 7;
      }
      else if (padType[nameIndex] == CY2_PAD || padType[nameIndex] == HH_PAD || padType[nameIndex] == HH2_PAD)
      {
        itemNumber = 8;
      }
      else if (padType[nameIndex] == DOUBLE_PAD || padType[nameIndex] == CY3_PAD || padType[nameIndex] == HHC_PAD)
      {
        itemNumber = 9;
      }
    }

    else if (itemNumber == 5)
    {
      if (padType[nameIndex] == HH_PAD || padType[nameIndex] == SINGLE_PAD)
      {
        itemNumber = 3;
      }

      else if (padType[nameIndex] == CY2_PAD || padType[nameIndex] == HHC_PAD || padType[nameIndex] == HH2_PAD)
      {
        itemNumber = 4;
      }
    }

    else if (itemNumber == 8)
    {
      if (padType[nameIndex] == SINGLE_PAD)
      {
        itemNumber = 7;
      }
    }

    else if (itemNumber == 9)
    {
      if (padType[nameIndex] == SINGLE_PAD)
      {
        itemNumber = 7;
      }

      else if (padType[nameIndex] == CY2_PAD || padType[nameIndex] == HH_PAD || padType[nameIndex] == HHC_PAD || padType[nameIndex] == HH2_PAD)
      {
        itemNumber = 8;
      }
    }

    //itemNumberShow : for LCD or OLED

    if (itemNumberShow == 255)
    {
      if (padType[nameIndex] == SINGLE_PAD)
      {
        itemNumberShow = 5;
      }
      else if (padType[nameIndex] == HH_PAD)
      {
        itemNumberShow = 6;
      }
      else if (padType[nameIndex] == CY2_PAD || padType[nameIndex] == HH2_PAD)
      {
        itemNumberShow = 7;
      }
      else if (padType[nameIndex] == HHC_PAD)
      {
        itemNumberShow = 8;
      }
      else if (padType[nameIndex] == DOUBLE_PAD || padType[nameIndex] == CY3_PAD)
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
