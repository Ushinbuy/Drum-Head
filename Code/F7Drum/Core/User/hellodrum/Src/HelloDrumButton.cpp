#include "HelloDrumButton.hpp"

///////////////////// 7. BUTONN //////////////////////////

//control button
HelloDrumButton::HelloDrumButton(byte pin1, byte pin2, byte pin3, byte pin4, byte pin5)
{
  pin_1 = pin1; //EDIT
  pin_2 = pin2; //UP
  pin_3 = pin3; //DOWN
  pin_4 = pin4; //NEXT
  pin_5 = pin5; //BACK
}

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
const char *HelloDrumButton::GetHitPad()
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
