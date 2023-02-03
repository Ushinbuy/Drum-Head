#include "audioExample.h"
#include "audio_sample.h"
#include "wm8994.h"
//#include "stm32412g_discovery_audio.h"
#include "wavFile.h"
#include "audio.h"

#define AUDIO_START_OFFSET_ADDRESS		sizeof(WAVE_FormatTypeDef)
#define AUDIO_FILE_ADDRESS				&AUDIO_SAMPLE[0]
#define AUDIO_FILE_SIZE					sizeof(AUDIO_SAMPLE)

static AUDIO_PLAYBACK_StateTypeDef  audio_state;
__IO uint32_t uwCommand = AUDIO_PAUSE;
__IO uint32_t uwVolume = 35;
  uint8_t Volume_string[20] = {0};

uint32_t AudioTotalSize = 0xFFFF;  /* This variable holds the total size of the audio file */
uint32_t AudioRemSize   = 0xFFFF;  /* This variable holds the remaining data in audio file */
uint16_t* CurrentPos;              /* This variable holds the current position address of audio data */


void BSP_AUDIO_OUT_Error_CallBack(void)
{
  /* Display message on the LCD screen */
//  sendUart("Error while playing audio");
  /* Stop the program with an infinite loop */
  while (1)
  {
  }
}

void BSP_AUDIO_OUT_HalfTransfer_CallBack(void){
	// TODO release this method
}

void BSP_AUDIO_OUT_TransferComplete_CallBack(void)
{
  if (audio_state == AUDIO_STATE_PLAYING)
  {
    /* Calculate the remaining audio data in the file and the new size
    for the DMA transfer. If the Audio files size is less than the DMA max
    data transfer size, so there is no calculation to be done, just restart
    from the beginning of the file ... */
    /* Check if the end of file has been reached */
    if(AudioRemSize > 0)
    {
      /* Replay from the current position */
      BSP_AUDIO_OUT_ChangeBuffer((uint16_t*)CurrentPos, DMA_MAX(AudioRemSize));

      /* Update the current pointer position */
      CurrentPos += DMA_MAX(AudioRemSize);

      /* Update the remaining number of data to be played */
      AudioRemSize -= DMA_MAX(AudioRemSize);
    }
    else
    {
      /* Set the current audio pointer position */
      CurrentPos = (uint16_t*)(AUDIO_FILE_ADDRESS + AUDIO_START_OFFSET_ADDRESS);
      /* Replay from the beginning */
      BSP_AUDIO_OUT_Play((uint16_t*)CurrentPos,  (uint32_t)(AUDIO_FILE_SIZE - AUDIO_START_OFFSET_ADDRESS));
      /* Update the remaining number of data to be played */
      AudioRemSize = AudioTotalSize - DMA_MAX(AudioTotalSize);
      /* Update the current audio pointer position */
      CurrentPos += DMA_MAX(AudioTotalSize);
    }
  }
}


AudioExampleState playAudioExample(void) {
	WAVE_FormatTypeDef *waveformat = NULL;
	if (BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, uwVolume, I2S_AUDIOFREQ_8K)
			!= AUDIO_OK) {
		return EXAMPLE_ERROR;
	}

	audio_state = AUDIO_STATE_PLAYING;
	waveformat = (WAVE_FormatTypeDef*) AUDIO_FILE_ADDRESS;

	AudioTotalSize = (AUDIO_FILE_SIZE - AUDIO_START_OFFSET_ADDRESS)
			/ (waveformat->NbrChannels);
	/* Set the current audio pointer position */
	CurrentPos = (uint16_t*) (AUDIO_FILE_ADDRESS + AUDIO_START_OFFSET_ADDRESS);
	/* Start the audio player */
	if (BSP_AUDIO_OUT_Play((uint16_t*) CurrentPos,
			(uint32_t) (AUDIO_FILE_SIZE - AUDIO_START_OFFSET_ADDRESS))
			!= AUDIO_OK) {
		return EXAMPLE_ERROR;
	}
	return EXAMPLE_OK;
}
