#ifndef USER_INC_EEPROMSETTINGS_H_
#define USER_INC_EEPROMSETTINGS_H_

#include <stdio.h>
#include "drumCore.h"

uint8_t Save_Setting(uint8_t _rst);
uint8_t Load_Setting();
void setLinksEeprom(DRUM extChannel[]);

#endif /* USER_INC_EEPROMSETTINGS_H_ */
