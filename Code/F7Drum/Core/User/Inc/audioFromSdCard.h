#ifndef USER_INC_AUDIOFROMSDCARD_H_
#define USER_INC_AUDIOFROMSDCARD_H_

#include "wm8994.h"
#include "stm32746g_discovery_audio.h"
#include "wavFile.h"
#include "audio.h"
#include "fatfs.h"
#include <string.h>

void sendUart(char *_msg); // interface. This need to release in youre platform

void sdCardTextExample(void);

#endif /* USER_INC_AUDIOFROMSDCARD_H_ */
