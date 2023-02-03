#ifndef USER_INC_AUDIOEXAMPLE_H_
#define USER_INC_AUDIOEXAMPLE_H_

/**
 * If you wan't to use audio example,
 * include audioExample.h
 * and start playAudioExample();
 */

typedef enum {
	EXAMPLE_ERROR,
	EXAMPLE_OK
} AudioExampleState;

AudioExampleState playAudioExample(void);

#endif /* USER_INC_AUDIOEXAMPLE_H_ */
