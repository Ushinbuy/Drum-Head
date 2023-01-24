/**
  ******************************************************************************
  * @file    stm32412g_discovery_audio.c
  * @author  MCD Application Team
  * @brief   This file provides the Audio driver for the STM32412G-DISCOVERY board.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/*==============================================================================
                                 User NOTES
                                 
How To use this driver:
-----------------------
   + This driver supports STM32F4xx devices on STM32412G-DISCOVERY boards.
   + Call the function BSP_AUDIO_OUT_Init(
                                    OutputDevice: physical output mode (OUTPUT_DEVICE_SPEAKER, 
                                                  OUTPUT_DEVICE_HEADPHONE or OUTPUT_DEVICE_BOTH)
                                    Volume      : Initial volume to be set (0 is min (mute), 100 is max (100%)
                                    AudioFreq   : Audio frequency in Hz (8000, 16000, 22500, 32000...)
                                                  this parameter is relative to the audio file/stream type.
                                   )
      This function configures all the hardware required for the audio application (codec, I2C, I2S, 
      GPIOs, DMA and interrupt if needed). This function returns AUDIO_OK if configuration is OK.
      If the returned value is different from AUDIO_OK or the function is stuck then the communication with
      the codec has failed (try to un-plug the power or reset device in this case).
      - OUTPUT_DEVICE_SPEAKER  : only speaker will be set as output for the audio stream.
      - OUTPUT_DEVICE_HEADPHONE: only headphones will be set as output for the audio stream.
      - OUTPUT_DEVICE_BOTH     : both Speaker and Headphone are used as outputs for the audio stream
                                 at the same time.
   + Call the function BSP_AUDIO_OUT_Play(
                                  pBuffer: pointer to the audio data file address
                                  Size   : size of the buffer to be sent in Bytes
                                 )
      to start playing (for the first time) from the audio file/stream.
   + Call the function BSP_AUDIO_OUT_Pause() to pause playing   
   + Call the function BSP_AUDIO_OUT_Resume() to resume playing.
       Note. After calling BSP_AUDIO_OUT_Pause() function for pause, only BSP_AUDIO_OUT_Resume() should be called
          for resume (it is not allowed to call BSP_AUDIO_OUT_Play() in this case).
       Note. This function should be called only when the audio file is played or paused (not stopped).
   + For each mode, you may need to implement the relative callback functions into your code.
      The Callback functions are named AUDIO_OUT_XXX_CallBack() and only their prototypes are declared in 
      the stm32412g_discovery_audio.h file. (refer to the example for more details on the callbacks implementations)
   + To Stop playing, to modify the volume level, the frequency, use the functions: BSP_AUDIO_OUT_SetVolume(), 
      AUDIO_OUT_SetFrequency(), BSP_AUDIO_OUT_SetOutputMode(), BSP_AUDIO_OUT_SetMute() and BSP_AUDIO_OUT_Stop().
   + The driver API and the callback functions are at the end of the stm32412g_discovery_audio.h file.
 

Driver architecture:
--------------------
   + This driver provides the High Audio Layer: consists of the function API exported in the stm32412g_discovery_audio.h file
     (BSP_AUDIO_OUT_Init(), BSP_AUDIO_OUT_Play() ...)
   + This driver provide also the Media Access Layer (MAL): which consists of functions allowing to access the media containing/
     providing the audio file/stream. These functions are also included as local functions into
     the stm32412g_discovery_audio.c file (I2Sx_Out_Init(), I2Sx_Out_DeInit(), I2Sx_In_Init() and I2Sx_In_DeInit())

Known Limitations:
------------------
   1- If the TDM Format used to play in parallel 2 audio Stream (the first Stream is configured in codec SLOT0 and second 
      Stream in SLOT1) the Pause/Resume, volume and mute feature will control the both streams.
   2- Parsing of audio file is not implemented (in order to determine audio file properties: Mono/Stereo, Data size, 
      File size, Audio Frequency, Audio Data header size ...). The configuration is fixed for the given audio file.
   3- Supports only Stereo audio streaming.
   4- Supports only 16-bits audio data size.
==============================================================================*/

/* Includes ------------------------------------------------------------------*/
#include "stm32412g_discovery_audio.h"


#define AUDIO_I2C_ADDRESS                ((uint16_t)0x34)

  
AUDIO_DrvTypeDef                *audio_drv;
I2S_HandleTypeDef               haudio_i2s;       /* for Audio_OUT and Audio_IN_analog mic */


/* Application Buffer Trigger */
__IO uint32_t                   AppBuffTrigger          = 0;
__IO uint32_t                   AppBuffHalf             = 0;


static void I2Sx_Out_Init(uint32_t AudioFreq);
static void I2Sx_Out_DeInit(void);

/**
  * @}
  */ 

/** @defgroup STM32412G_DISCOVERY_AUDIO_OUT_Private_Functions STM32412G Discovery Audio Out Private Functions
  * @{
  */ 

/**
  * @brief  Configures the audio peripherals.
  * @param  OutputDevice: OUTPUT_DEVICE_SPEAKER, OUTPUT_DEVICE_HEADPHONE,
  *                       or OUTPUT_DEVICE_BOTH.
  * @param  Volume: Initial volume level (from 0 (Mute) to 100 (Max))
  * @param  AudioFreq: Audio frequency used to play the audio stream.
  * @note   The I2S PLL input clock must be done in the user application.  
  * @retval AUDIO_OK if correct communication, else wrong communication
  */
uint8_t BSP_AUDIO_OUT_Init(uint16_t OutputDevice, uint8_t Volume, uint32_t AudioFreq)
{ 
  uint8_t  ret = AUDIO_ERROR;
  uint32_t deviceid = 0x00;
  uint16_t buffer_fake[16] = {0x00};

  I2Sx_Out_DeInit();
  AUDIO_IO_DeInit();
  
  /* PLL clock is set depending on the AudioFreq (44.1 kHz vs 48kHz groups) */
  BSP_AUDIO_OUT_ClockConfig(&haudio_i2s, AudioFreq, NULL);
  
  /* Configure the I2S peripheral */
  haudio_i2s.Instance = AUDIO_OUT_I2Sx;
  if(HAL_I2S_GetState(&haudio_i2s) == HAL_I2S_STATE_RESET)
  {
    /* Initialize the I2S Msp: this __weak function can be rewritten by the application */
    BSP_AUDIO_OUT_MspInit(&haudio_i2s, NULL);
  }
  I2Sx_Out_Init(AudioFreq);

  AUDIO_IO_Init();
  
  /* wm8994 codec initialization */
  deviceid = wm8994_drv.ReadID(AUDIO_I2C_ADDRESS);

  if(deviceid == WM8994_ID)
  {
    /* Reset the Codec Registers */
    wm8994_drv.Reset(AUDIO_I2C_ADDRESS);
    /* Initialize the audio driver structure */
    audio_drv = &wm8994_drv;
    ret = AUDIO_OK;
  }
  else
  {
    ret = AUDIO_ERROR;
  }

  if(ret == AUDIO_OK)
  {
    /* Send fake I2S data in order to generate MCLK needed by WM8994 to set its registers
     * MCLK is generated only when a data stream is sent on I2S */
    HAL_I2S_Transmit_DMA(&haudio_i2s, buffer_fake, 16);
    /* Initialize the codec internal registers */
    audio_drv->Init(AUDIO_I2C_ADDRESS, OutputDevice, Volume, AudioFreq);
    /* Stop sending fake I2S data */
    HAL_I2S_DMAStop(&haudio_i2s);
  }

  return ret;
}

/**
  * @brief  Starts playing audio stream from a data buffer for a determined size. 
  * @param  pBuffer: Pointer to the buffer 
  * @param  Size: Number of audio data BYTES.
  * @retval AUDIO_OK if correct communication, else wrong communication
  */
uint8_t BSP_AUDIO_OUT_Play(uint16_t* pBuffer, uint32_t Size)
{
  /* Call the audio Codec Play function */
  if(audio_drv->Play(AUDIO_I2C_ADDRESS, pBuffer, Size) != 0)
  {  
    return AUDIO_ERROR;
  }
  else
  {
    /* Update the Media layer and enable it for play */  
    HAL_I2S_Transmit_DMA(&haudio_i2s, pBuffer, DMA_MAX(Size / AUDIODATA_SIZE));
    
    return AUDIO_OK;
  }
}

/**
  * @brief  Sends n-Bytes on the I2S interface.
  * @param  pData: pointer on data address 
  * @param  Size: number of data to be written
  */
void BSP_AUDIO_OUT_ChangeBuffer(uint16_t *pData, uint16_t Size)
{
   HAL_I2S_Transmit_DMA(&haudio_i2s, pData, Size);
}

/**
  * @brief  This function Pauses the audio file stream. In case
  *         of using DMA, the DMA Pause feature is used.
  * @note   When calling BSP_AUDIO_OUT_Pause() function for pause, only
  *          BSP_AUDIO_OUT_Resume() function should be called for resume (use of BSP_AUDIO_OUT_Play() 
  *          function for resume could lead to unexpected behavior).
  * @retval AUDIO_OK if correct communication, else wrong communication
  */
uint8_t BSP_AUDIO_OUT_Pause(void)
{    
  /* Call the Audio Codec Pause/Resume function */
  if(audio_drv->Pause(AUDIO_I2C_ADDRESS) != 0)
  {
    return AUDIO_ERROR;
  }
  else
  {
    /* Call the Media layer pause function */
    HAL_I2S_DMAPause(&haudio_i2s);
    
    /* Return AUDIO_OK when all operations are correctly done */
    return AUDIO_OK;
  }
}

/**
  * @brief  This function  Resumes the audio file stream.  
  * @note   When calling BSP_AUDIO_OUT_Pause() function for pause, only
  *          BSP_AUDIO_OUT_Resume() function should be called for resume (use of BSP_AUDIO_OUT_Play() 
  *          function for resume could lead to unexpected behavior).
  * @retval AUDIO_OK if correct communication, else wrong communication
  */
uint8_t BSP_AUDIO_OUT_Resume(void)
{    
  /* Call the Media layer pause/resume function */
  /* DMA stream resumed before accessing WM8994 register as WM8994 needs the MCLK to be generated to access its registers
   * MCLK is generated only when a data stream is sent on I2S */
  HAL_I2S_DMAResume(&haudio_i2s);

  /* Call the Audio Codec Pause/Resume function */
  if(audio_drv->Resume(AUDIO_I2C_ADDRESS) != 0)
  {
    return AUDIO_ERROR;
  }
  else
  {
    /* Return AUDIO_OK when all operations are correctly done */
    return AUDIO_OK;
  }
}

/**
  * @brief  Stops audio playing and Power down the Audio Codec. 
  * @param  Option: could be one of the following parameters 
  *           - CODEC_PDWN_SW: for software power off (by writing registers). 
  *                            Then no need to reconfigure the Codec after power on.
  *           - CODEC_PDWN_HW: completely shut down the codec (physically). 
  *                            Then need to reconfigure the Codec after power on.  
  * @retval AUDIO_OK if correct communication, else wrong communication
  */
uint8_t BSP_AUDIO_OUT_Stop(uint32_t Option)
{
  /* Call the Media layer stop function */
  HAL_I2S_DMAStop(&haudio_i2s);

  /* Call Audio Codec Stop function */
  if(audio_drv->Stop(AUDIO_I2C_ADDRESS, Option) != 0)
  {
    return AUDIO_ERROR;
  }
  else
  {
    if(Option == CODEC_PDWN_HW)
    { 
      /* Wait at least 100us */
      HAL_Delay(1);
    }
    /* Return AUDIO_OK when all operations are correctly done */
    return AUDIO_OK;
  }
}

/**
  * @brief  Controls the current audio volume level. 
  * @param  Volume: Volume level to be set in percentage from 0% to 100% (0 for 
  *         Mute and 100 for Max volume level).
  * @retval AUDIO_OK if correct communication, else wrong communication
  */
uint8_t BSP_AUDIO_OUT_SetVolume(uint8_t Volume)
{
  /* Call the codec volume control function with converted volume value */
  if(audio_drv->SetVolume(AUDIO_I2C_ADDRESS, Volume) != 0)
  {
    return AUDIO_ERROR;
  }
  else
  {
    /* Return AUDIO_OK when all operations are correctly done */
    return AUDIO_OK;
  }
}

/**
  * @brief  Enables or disables the MUTE mode by software 
  * @param  Cmd: Could be AUDIO_MUTE_ON to mute sound or AUDIO_MUTE_OFF to 
  *         unmute the codec and restore previous volume level.
  * @retval AUDIO_OK if correct communication, else wrong communication
  */
uint8_t BSP_AUDIO_OUT_SetMute(uint32_t Cmd)
{ 
  /* Call the Codec Mute function */
  if(audio_drv->SetMute(AUDIO_I2C_ADDRESS, Cmd) != 0)
  {
    return AUDIO_ERROR;
  }
  else
  {
    /* Return AUDIO_OK when all operations are correctly done */
    return AUDIO_OK;
  }
}

/**
  * @brief  Switch dynamically (while audio file is played) the output target 
  *         (speaker or headphone).
  * @param  Output: The audio output target: OUTPUT_DEVICE_SPEAKER,
  *         OUTPUT_DEVICE_HEADPHONE or OUTPUT_DEVICE_BOTH
  * @retval AUDIO_OK if correct communication, else wrong communication
  */
uint8_t BSP_AUDIO_OUT_SetOutputMode(uint8_t Output)
{
  /* Call the Codec output device function */
  if(audio_drv->SetOutputMode(AUDIO_I2C_ADDRESS, Output) != 0)
  {
    return AUDIO_ERROR;
  }
  else
  {
    /* Return AUDIO_OK when all operations are correctly done */
    return AUDIO_OK;
  }
}

/**
  * @brief  Updates the audio frequency.
  * @param  AudioFreq: Audio frequency used to play the audio stream.
  * @note   This API should be called after the BSP_AUDIO_OUT_Init() to adjust the
  *         audio frequency.
  */
void BSP_AUDIO_OUT_SetFrequency(uint32_t AudioFreq)
{ 
  /* PLL clock is set depending by the AudioFreq (44.1khz vs 48khz groups) */ 
  BSP_AUDIO_OUT_ClockConfig(&haudio_i2s, AudioFreq, NULL);

  /* Disable I2S peripheral to allow access to I2S internal registers */
  __HAL_I2S_DISABLE(&haudio_i2s);
    
  /* Update the I2S audio frequency configuration */
  haudio_i2s.Init.AudioFreq = AudioFreq;
  HAL_I2S_Init(&haudio_i2s);
  
  /* Enable I2S peripheral to generate MCLK */
  __HAL_I2S_ENABLE(&haudio_i2s);
}

/**
  * @brief  Deinit the audio peripherals.
  */
void BSP_AUDIO_OUT_DeInit(void)
{
  I2Sx_Out_DeInit();
  /* DeInit the I2S MSP : this __weak function can be rewritten by the application */
  BSP_AUDIO_OUT_MspDeInit(&haudio_i2s, NULL);
}

/**
  * @brief  Tx Transfer completed callbacks.
  * @param  hi2s: I2S handle
  */
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  /* Manage the remaining file size and new address offset: This function 
     should be coded by user (its prototype is already declared in stm32412g_discovery_audio.h) */
  BSP_AUDIO_OUT_TransferComplete_CallBack();
}

/**
  * @brief  Tx Half Transfer completed callbacks.
  * @param  hi2s: I2S handle
  */
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
  /* Manage the remaining file size and new address offset: This function 
     should be coded by user (its prototype is already declared in stm32412g_discovery_audio.h) */
  BSP_AUDIO_OUT_HalfTransfer_CallBack();
}

/**
  * @brief  I2S error callbacks.
  * @param  hi2s: I2S handle
  */
void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s)
{
  BSP_AUDIO_OUT_Error_CallBack();
}

/**
  * @brief  Manages the DMA full Transfer complete event.
  */
__weak void BSP_AUDIO_OUT_TransferComplete_CallBack(void)
{
}

/**
  * @brief  Manages the DMA Half Transfer complete event.
  */
__weak void BSP_AUDIO_OUT_HalfTransfer_CallBack(void)
{ 
}

/**
  * @brief  Manages the DMA FIFO error event.
  */
__weak void BSP_AUDIO_OUT_Error_CallBack(void)
{
}

/**
  * @brief  Initializes BSP_AUDIO_OUT MSP.
  * @param  hi2s: I2S handle
  * @param  Params : pointer on additional configuration parameters, can be NULL.
  */
__weak void BSP_AUDIO_OUT_MspInit(I2S_HandleTypeDef *hi2s, void *Params)
{ 
  static DMA_HandleTypeDef hdma_i2s_tx;
  GPIO_InitTypeDef  gpio_init_structure;

  /* Enable I2S clock */
  AUDIO_OUT_I2Sx_CLK_ENABLE();

  /* Enable MCK, SCK, WS, SD and CODEC_INT GPIO clock */
  AUDIO_OUT_I2Sx_MCK_GPIO_CLK_ENABLE();
  AUDIO_OUT_I2Sx_SCK_GPIO_CLK_ENABLE();
  AUDIO_OUT_I2Sx_SD_GPIO_CLK_ENABLE();
  AUDIO_OUT_I2Sx_WS_GPIO_CLK_ENABLE();

  /* CODEC_I2S pins configuration: MCK, SCK, WS and SD pins */
  gpio_init_structure.Pin = AUDIO_OUT_I2Sx_MCK_PIN;
  gpio_init_structure.Mode = GPIO_MODE_AF_PP;
  gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_FAST;
  gpio_init_structure.Alternate = AUDIO_OUT_I2Sx_MCK_AF;
  HAL_GPIO_Init(AUDIO_OUT_I2Sx_MCK_GPIO_PORT, &gpio_init_structure);

  gpio_init_structure.Pin = AUDIO_OUT_I2Sx_SCK_PIN;
  gpio_init_structure.Alternate = AUDIO_OUT_I2Sx_SCK_AF;
  HAL_GPIO_Init(AUDIO_OUT_I2Sx_SCK_GPIO_PORT, &gpio_init_structure);

  gpio_init_structure.Pin = AUDIO_OUT_I2Sx_WS_PIN;
  gpio_init_structure.Alternate = AUDIO_OUT_I2Sx_WS_AF;
  HAL_GPIO_Init(AUDIO_OUT_I2Sx_WS_GPIO_PORT, &gpio_init_structure);

  gpio_init_structure.Pin = AUDIO_OUT_I2Sx_SD_PIN;
  gpio_init_structure.Alternate = AUDIO_OUT_I2Sx_SD_AF;
  HAL_GPIO_Init(AUDIO_OUT_I2Sx_SD_GPIO_PORT, &gpio_init_structure);

  /* Enable the DMA clock */
  AUDIO_OUT_I2Sx_DMAx_CLK_ENABLE();

  if(hi2s->Instance == AUDIO_OUT_I2Sx)
  {
    /* Configure the hdma_i2s_rx handle parameters */
    hdma_i2s_tx.Init.Channel             = AUDIO_OUT_I2Sx_DMAx_CHANNEL;
    hdma_i2s_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_i2s_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_i2s_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_i2s_tx.Init.PeriphDataAlignment = AUDIO_OUT_I2Sx_DMAx_PERIPH_DATA_SIZE;
    hdma_i2s_tx.Init.MemDataAlignment    = AUDIO_OUT_I2Sx_DMAx_MEM_DATA_SIZE;
    hdma_i2s_tx.Init.Mode                = DMA_CIRCULAR;
    hdma_i2s_tx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_i2s_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdma_i2s_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_i2s_tx.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_i2s_tx.Init.PeriphBurst         = DMA_MBURST_SINGLE;

    hdma_i2s_tx.Instance = AUDIO_OUT_I2Sx_DMAx_STREAM;

    /* Associate the DMA handle */
    __HAL_LINKDMA(hi2s, hdmatx, hdma_i2s_tx);

    /* Deinitialize the Stream for new transfer */
    HAL_DMA_DeInit(&hdma_i2s_tx);

    /* Configure the DMA Stream */
    HAL_DMA_Init(&hdma_i2s_tx);
  }
  
  /* Enable and set I2Sx Interrupt to a lower priority */
  HAL_NVIC_SetPriority(SPI3_IRQn, 0x0F, 0x00);
  HAL_NVIC_EnableIRQ(SPI3_IRQn);

  /* I2S DMA IRQ Channel configuration */
  HAL_NVIC_SetPriority(AUDIO_OUT_I2Sx_DMAx_IRQ, AUDIO_OUT_IRQ_PREPRIO, 0);
  HAL_NVIC_EnableIRQ(AUDIO_OUT_I2Sx_DMAx_IRQ);
}

/**
  * @brief  Deinitializes I2S MSP.
  * @param  Params : pointer on additional configuration parameters, can be NULL.
  * @param  hi2s: I2S handle
  */
__weak void BSP_AUDIO_OUT_MspDeInit(I2S_HandleTypeDef *hi2s, void *Params)
{
  GPIO_InitTypeDef  gpio_init_structure;
  
  /* I2S DMA IRQ Channel deactivation */
  HAL_NVIC_DisableIRQ(AUDIO_OUT_I2Sx_DMAx_IRQ);
  
  if(hi2s->Instance == AUDIO_OUT_I2Sx)
  {
    /* Deinitialize the DMA stream */
    HAL_DMA_DeInit(hi2s->hdmatx);
  }
  
  /* Disable I2S peripheral */
  __HAL_I2S_DISABLE(hi2s);
  
  /* Deactives CODEC_I2S pins MCK, SCK, WS and SD by putting them in input mode */
  gpio_init_structure.Pin = AUDIO_OUT_I2Sx_MCK_PIN;
  HAL_GPIO_DeInit(AUDIO_OUT_I2Sx_MCK_GPIO_PORT, gpio_init_structure.Pin);
  
  gpio_init_structure.Pin = AUDIO_OUT_I2Sx_SCK_PIN;
  HAL_GPIO_DeInit(AUDIO_OUT_I2Sx_SCK_GPIO_PORT, gpio_init_structure.Pin);
  
  gpio_init_structure.Pin = AUDIO_OUT_I2Sx_WS_PIN;
  HAL_GPIO_DeInit(AUDIO_OUT_I2Sx_WS_GPIO_PORT, gpio_init_structure.Pin);
  
  gpio_init_structure.Pin = AUDIO_OUT_I2Sx_SD_PIN;
  HAL_GPIO_DeInit(AUDIO_OUT_I2Sx_SD_GPIO_PORT, gpio_init_structure.Pin);
  
  /* Disable I2S clock */
  AUDIO_OUT_I2Sx_CLK_DISABLE();
  
  /* GPIO pins clock and DMA clock can be shut down in the application 
  by surcharging this __weak function */ 
}

/**
  * @brief  Clock Config.
  * @param  hi2s: might be required to set audio peripheral predivider if any.
  * @param  AudioFreq: Audio frequency used to play the audio stream.
  * @param  Params : pointer on additional configuration parameters, can be NULL.
  * @note   This API is called by BSP_AUDIO_OUT_Init() and BSP_AUDIO_OUT_SetFrequency()
  *         Being __weak it can be overwritten by the application     
  */
__weak void BSP_AUDIO_OUT_ClockConfig(I2S_HandleTypeDef *hi2s, uint32_t AudioFreq, void *Params)
{ 
  RCC_PeriphCLKInitTypeDef rcc_ex_clk_init_struct;

  HAL_RCCEx_GetPeriphCLKConfig(&rcc_ex_clk_init_struct);

  /* Set the PLL configuration according to the audio frequency */
  if((AudioFreq == AUDIO_FREQUENCY_11K) || (AudioFreq == AUDIO_FREQUENCY_22K) || (AudioFreq == AUDIO_FREQUENCY_44K))
  {
    /* Configure PLLI2S prescalers */
    rcc_ex_clk_init_struct.PeriphClockSelection = (RCC_PERIPHCLK_I2S_APB1  | RCC_PERIPHCLK_PLLI2S);
    rcc_ex_clk_init_struct.I2sApb1ClockSelection = RCC_I2SAPB1CLKSOURCE_PLLI2S;
    rcc_ex_clk_init_struct.PLLI2SSelection = RCC_PLLI2SCLKSOURCE_PLLSRC;    
    rcc_ex_clk_init_struct.PLLI2S.PLLI2SM = 8;
    rcc_ex_clk_init_struct.PLLI2S.PLLI2SN = 271;
    rcc_ex_clk_init_struct.PLLI2S.PLLI2SR = 2;
    
    HAL_RCCEx_PeriphCLKConfig(&rcc_ex_clk_init_struct);
  }
  else if(AudioFreq == AUDIO_FREQUENCY_96K) /* AUDIO_FREQUENCY_96K */
  {
    /* I2S clock config */
    rcc_ex_clk_init_struct.PeriphClockSelection = (RCC_PERIPHCLK_I2S_APB1 | RCC_PERIPHCLK_PLLI2S);
    rcc_ex_clk_init_struct.I2sApb1ClockSelection = RCC_I2SAPB1CLKSOURCE_PLLI2S;
    rcc_ex_clk_init_struct.PLLI2SSelection = RCC_PLLI2SCLKSOURCE_PLLSRC;    
    rcc_ex_clk_init_struct.PLLI2S.PLLI2SM = 8;
    rcc_ex_clk_init_struct.PLLI2S.PLLI2SN = 344;
    rcc_ex_clk_init_struct.PLLI2S.PLLI2SR = 2;

    HAL_RCCEx_PeriphCLKConfig(&rcc_ex_clk_init_struct);    
  }  
  else /* AUDIO_FREQUENCY_8K, AUDIO_FREQUENCY_16K, AUDIO_FREQUENCY_48K */
  {
    /* I2S clock config
    PLLI2S_VCO: VCO_344M
    I2S_CLK(first level) = PLLI2S_VCO/PLLI2SR = 344/7 = 49.142 Mhz
    I2S_CLK_x = I2S_CLK(first level)/PLLI2SDIVR = 49.142/1 = 49.142 Mhz */
    rcc_ex_clk_init_struct.PeriphClockSelection = RCC_PERIPHCLK_I2S_APB1 | RCC_PERIPHCLK_PLLI2S;
    rcc_ex_clk_init_struct.I2sApb1ClockSelection = RCC_I2SAPB1CLKSOURCE_PLLI2S;
    rcc_ex_clk_init_struct.PLLI2SSelection = RCC_PLLI2SCLKSOURCE_PLLSRC;
    rcc_ex_clk_init_struct.PLLI2S.PLLI2SM = 8;
    rcc_ex_clk_init_struct.PLLI2S.PLLI2SN = 344;
    rcc_ex_clk_init_struct.PLLI2S.PLLI2SR = 7;
    
    HAL_RCCEx_PeriphCLKConfig(&rcc_ex_clk_init_struct);
  }
}

/*******************************************************************************
                            Static Functions
*******************************************************************************/

/**
  * @brief  Initializes the Audio Codec audio interface (I2S)
  * @note   This function assumes that the I2S input clock
  *         is already configured and ready to be used.
  * @param  AudioFreq: Audio frequency to be configured for the I2S peripheral.
  */
static void I2Sx_Out_Init(uint32_t AudioFreq)
{
  /* Initialize the hAudioInI2s Instance parameter */
  haudio_i2s.Instance = AUDIO_OUT_I2Sx;

 /* Disable I2S block */
  __HAL_I2S_DISABLE(&haudio_i2s);
  
  /* I2S peripheral configuration */
  haudio_i2s.Init.AudioFreq = AudioFreq;
  haudio_i2s.Init.ClockSource = I2S_CLOCK_PLL;
  haudio_i2s.Init.CPOL = I2S_CPOL_LOW;
  haudio_i2s.Init.DataFormat = I2S_DATAFORMAT_16B;
  haudio_i2s.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  haudio_i2s.Init.Mode = I2S_MODE_MASTER_TX;
  haudio_i2s.Init.Standard =  I2S_STANDARD_PHILIPS;
  haudio_i2s.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  
  /* Init the I2S */
  HAL_I2S_Init(&haudio_i2s);

 /* Enable I2S block */
  __HAL_I2S_ENABLE(&haudio_i2s);
}

/**
  * @brief  Deinitializes the Audio Codec audio interface (I2S).
  */
static void I2Sx_Out_DeInit(void)
{
  /* Initialize the hAudioInI2s Instance parameter */
  haudio_i2s.Instance = AUDIO_OUT_I2Sx;

 /* Disable I2S block */
  __HAL_I2S_DISABLE(&haudio_i2s);

  /* DeInit the I2S */
  HAL_I2S_DeInit(&haudio_i2s);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
