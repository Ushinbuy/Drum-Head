/**
  ******************************************************************************
  * @file    stm32412g_discovery_audio.h
  * @author  MCD Application Team
  * @brief   This file contains the common defines and functions prototypes for
  *          the stm32412g_discovery_audio.c driver.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32412G_DISCOVERY_AUDIO_H
#define __STM32412G_DISCOVERY_AUDIO_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
/* Include audio component Driver */
#include "wm8994.h"
#include "stm32f4xx_hal.h"
//#include "stm32412g_discovery.h"

/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup STM32412G_DISCOVERY
  * @{
  */
    
/** @addtogroup STM32412G_DISCOVERY_AUDIO
  * @{
  */

/** @defgroup STM32412G_DISCOVERY_AUDIO_Exported_Types STM32412G DISCOVERY Audio Exported Types
  * @{
  */  
typedef struct
{
  uint32_t               Frequency;      /* Record Frequency */
  uint32_t               BitResolution;  /* Record bit resolution */
  uint32_t               ChannelNbr;     /* Record Channel Number */
  uint16_t               *pRecBuf;       /* Pointer to record user buffer */
  uint32_t               RecSize;        /* Size to record in mono, double size to record in stereo */
  uint32_t               InputDevice;    /* Audio Input Device */
  uint32_t               MultiBuffMode;  /* Multi buffer mode selection */
}AUDIOIN_ContextTypeDef;

/**
  * @}
  */ 

/** @defgroup STM32412G_DISCOVERY_AUDIO_Exported_Constants  STM32412G DISCOVERY Audio Exported Constants
  * @{
  */

/*------------------------------------------------------------------------------
                        AUDIO OUT CONFIGURATION
------------------------------------------------------------------------------*/
/* SPI Configuration defines */
#define AUDIO_OUT_I2Sx                           SPI3
#define AUDIO_OUT_I2Sx_CLK_ENABLE()              __HAL_RCC_SPI3_CLK_ENABLE()
#define AUDIO_OUT_I2Sx_CLK_DISABLE()             __HAL_RCC_SPI3_CLK_DISABLE()

#define AUDIO_OUT_I2Sx_MCK_PIN                   GPIO_PIN_7
#define AUDIO_OUT_I2Sx_MCK_GPIO_PORT             GPIOC
#define AUDIO_OUT_I2Sx_MCK_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOC_CLK_ENABLE()
#define AUDIO_OUT_I2Sx_MCK_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOC_CLK_DISABLE()
#define AUDIO_OUT_I2Sx_MCK_AF                    GPIO_AF6_SPI3

#define AUDIO_OUT_I2Sx_SCK_PIN                   GPIO_PIN_12
#define AUDIO_OUT_I2Sx_SCK_GPIO_PORT             GPIOB
#define AUDIO_OUT_I2Sx_SCK_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOB_CLK_ENABLE()
#define AUDIO_OUT_I2Sx_SCK_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOB_CLK_DISABLE()
#define AUDIO_OUT_I2Sx_SCK_AF                    GPIO_AF7_SPI3

#define AUDIO_OUT_I2Sx_WS_PIN                    GPIO_PIN_4
#define AUDIO_OUT_I2Sx_WS_GPIO_PORT              GPIOA
#define AUDIO_OUT_I2Sx_WS_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define AUDIO_OUT_I2Sx_WS_GPIO_CLK_DISABLE()     __HAL_RCC_GPIOA_CLK_DISABLE()
#define AUDIO_OUT_I2Sx_WS_AF                     GPIO_AF6_SPI3

#define AUDIO_OUT_I2Sx_SD_PIN                    GPIO_PIN_5
#define AUDIO_OUT_I2Sx_SD_GPIO_PORT              GPIOB
#define AUDIO_OUT_I2Sx_SD_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define AUDIO_OUT_I2Sx_SD_GPIO_CLK_DISABLE()     __HAL_RCC_GPIOB_CLK_DISABLE()
#define AUDIO_OUT_I2Sx_SD_AF                     GPIO_AF6_SPI3

/* I2S DMA Stream Tx definitions */
#define AUDIO_OUT_I2Sx_DMAx_CLK_ENABLE()         __HAL_RCC_DMA1_CLK_ENABLE()
#define AUDIO_OUT_I2Sx_DMAx_CLK_DISABLE()        __HAL_RCC_DMA1_CLK_DISABLE()
#define AUDIO_OUT_I2Sx_DMAx_STREAM               DMA1_Stream5
#define AUDIO_OUT_I2Sx_DMAx_CHANNEL              DMA_CHANNEL_0
#define AUDIO_OUT_I2Sx_DMAx_IRQ                  DMA1_Stream5_IRQn
#define AUDIO_OUT_I2Sx_DMAx_PERIPH_DATA_SIZE     DMA_PDATAALIGN_HALFWORD
#define AUDIO_OUT_I2Sx_DMAx_MEM_DATA_SIZE        DMA_MDATAALIGN_HALFWORD
#define DMA_MAX_SIZE                             0xFFFF
   
#define AUDIO_OUT_I2Sx_DMAx_IRQHandler           DMA1_Stream5_IRQHandler
  
   /* Select the interrupt preemption priority for the DMA interrupt */
#define AUDIO_OUT_IRQ_PREPRIO                    ((uint32_t)0x0E)   /* Select the preemption priority level(0 is the highest) */

/* Two channels are used:
   - one channel as input which is connected to I2S SCK in stereo mode 
   - one channel as output which divides the frequency on the input
*/

#define AUDIO_TIMx_CLK_ENABLE()             __HAL_RCC_TIM4_CLK_ENABLE()
#define AUDIO_TIMx_CLK_DISABLE()            __HAL_RCC_TIM4_CLK_DISABLE()
#define AUDIO_TIMx                          TIM4
#define AUDIO_TIMx_IN_CHANNEL               TIM_CHANNEL_1
#define AUDIO_TIMx_OUT_CHANNEL              TIM_CHANNEL_2 /* Select channel 2 as output */
#define AUDIO_TIMx_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOB_CLK_ENABLE()
#define AUDIO_TIMx_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOB_CLK_DISABLE()
#define AUDIO_TIMx_GPIO_PORT                GPIOB
#define AUDIO_TIMx_IN_GPIO_PIN              GPIO_PIN_6
#define AUDIO_TIMx_OUT_GPIO_PIN             GPIO_PIN_7
#define AUDIO_TIMx_AF                       GPIO_AF2_TIM4

/*------------------------------------------------------------------------------
             CONFIGURATION: Audio Driver Configuration parameters
------------------------------------------------------------------------------*/

#define AUDIODATA_SIZE                      2   /* 16-bits audio data size */

/* Audio status definition */     
#define AUDIO_OK                            ((uint8_t)0)
#define AUDIO_ERROR                         ((uint8_t)1)
#define AUDIO_TIMEOUT                       ((uint8_t)2)

/* Audio out parameters */
#define DEFAULT_AUDIO_OUT_FREQ              I2S_AUDIOFREQ_48K
#define DEFAULT_AUDIO_OUT_BIT_RESOLUTION    ((uint8_t)16)
#define DEFAULT_AUDIO_OUT_CHANNEL_NBR       ((uint8_t)2) /* Mono = 1, Stereo = 2 */
#define DEFAULT_AUDIO_OUT_VOLUME            ((uint16_t)64)

/*------------------------------------------------------------------------------
                            OUTPUT DEVICES definition
------------------------------------------------------------------------------*/

/* Alias on existing output devices to adapt for 2 headphones output */
#define OUTPUT_DEVICE_HEADPHONE1 OUTPUT_DEVICE_HEADPHONE
#define OUTPUT_DEVICE_HEADPHONE2 OUTPUT_DEVICE_SPEAKER /* Headphone2 is connected to Speaker output of the wm8994 */

 /**
  * @}
  */
   
/** @defgroup STM32412G_DISCOVERY_AUDIO_Exported_Macros  STM32412G DISCOVERY Audio Exported Macros
  * @{
  */
#define DMA_MAX(x)           (((x) <= DMA_MAX_SIZE)? (x):DMA_MAX_SIZE)
#define POS_VAL(VAL)         (POSITION_VAL(VAL) - 4) 
/**
  * @}
  */ 

/** @defgroup STM32412G_DISCOVERY_AUDIO_OUT_Exported_Functions  STM32412G DISCOVERY AUDIO OUT Exported Functions
  * @{
  */
uint8_t BSP_AUDIO_OUT_Init(uint16_t OutputDevice, uint8_t Volume, uint32_t AudioFreq);
uint8_t BSP_AUDIO_OUT_Play(uint16_t* pBuffer, uint32_t Size);
void    BSP_AUDIO_OUT_ChangeBuffer(uint16_t *pData, uint16_t Size);
uint8_t BSP_AUDIO_OUT_Pause(void);
uint8_t BSP_AUDIO_OUT_Resume(void);
uint8_t BSP_AUDIO_OUT_Stop(uint32_t Option);
uint8_t BSP_AUDIO_OUT_SetVolume(uint8_t Volume);
void    BSP_AUDIO_OUT_SetFrequency(uint32_t AudioFreq);
uint8_t BSP_AUDIO_OUT_SetMute(uint32_t Cmd);
uint8_t BSP_AUDIO_OUT_SetOutputMode(uint8_t Output);
void    BSP_AUDIO_OUT_DeInit(void);

/* User Callbacks: user has to implement these functions in his code if they are needed. */
/* This function is called when the requested data has been completely transferred.*/
void    BSP_AUDIO_OUT_TransferComplete_CallBack(void);

/* This function is called when half of the requested buffer has been transferred. */
void    BSP_AUDIO_OUT_HalfTransfer_CallBack(void);

/* This function is called when an Interrupt due to transfer error on or peripheral
   error occurs. */
void    BSP_AUDIO_OUT_Error_CallBack(void);

/* These function can be modified in case the current settings (e.g. DMA stream)
   need to be changed for specific application needs */
void  BSP_AUDIO_OUT_ClockConfig(I2S_HandleTypeDef *hi2s, uint32_t AudioFreq, void *Params);
void  BSP_AUDIO_OUT_MspInit(I2S_HandleTypeDef *hi2s, void *Params);
void  BSP_AUDIO_OUT_MspDeInit(I2S_HandleTypeDef *hi2s, void *Params);

#ifdef __cplusplus
}
#endif

#endif /* __STM32412G_DISCOVERY_AUDIO_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
