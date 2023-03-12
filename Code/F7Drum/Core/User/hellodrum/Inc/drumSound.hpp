#ifndef USER_HELLODRUM_INC_DRUMSOUND_HPP_
#define USER_HELLODRUM_INC_DRUMSOUND_HPP_

#include "stdio.h"

typedef enum{
	SOUND_INIT = 0,
	SOUND_IDLE,
	SOUND_PLAY
} SoundStateEnum;

class DrumSound{
public:
	DrumSound(uint32_t soundAddress, float soundVolumeDb);
	~DrumSound();

	const uint8_t *startAddress;
	volatile SoundStateEnum soundState;

	void setUserVolumeDb(float _newValue);
	void playSound(uint8_t velocity);
	void stopPlaying(void);
	void updateBuffer(uint8_t *mainBuffer);

private:
	float currentVolumeFloat;
	float userVolumeDB;
	uint32_t currentOffset;
	uint32_t fileLength;
};

#endif /* USER_HELLODRUM_INC_DRUMSOUND_HPP_ */
