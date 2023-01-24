#ifndef USER_INC_AUDIO_IO_H_
#define USER_INC_AUDIO_IO_H_

// This codec for i2c part of wm8994

#include "stm32f7xx_hal.h"

/* Exported constant IO ------------------------------------------------------*/

#define AUDIO_I2C_ADDRESS                ((uint16_t)0x34)

/* User can use this section to tailor I2Cx/I2Cx instance used and associated
   resources */
/* Definition for AUDIO I2Cx resources */
#define DISCOVERY_AUDIO_I2Cx                             I2C1
#define DISCOVERY_AUDIO_I2Cx_CLK_ENABLE()                __HAL_RCC_I2C1_CLK_ENABLE()
#define DISCOVERY_AUDIO_DMAx_CLK_ENABLE()                __HAL_RCC_DMA1_CLK_ENABLE()
#define DISCOVERY_AUDIO_I2Cx_SCL_SDA_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()

#define DISCOVERY_AUDIO_I2Cx_FORCE_RESET()               __HAL_RCC_I2C1_FORCE_RESET()
#define DISCOVERY_AUDIO_I2Cx_RELEASE_RESET()             __HAL_RCC_I2C1_RELEASE_RESET()

/* Definition for I2Cx Pins */
#define DISCOVERY_AUDIO_I2Cx_SCL_PIN                     GPIO_PIN_6
#define DISCOVERY_AUDIO_I2Cx_SCL_SDA_GPIO_PORT           GPIOB
#define DISCOVERY_AUDIO_I2Cx_SCL_SDA_AF                  GPIO_AF4_I2C1
#define DISCOVERY_AUDIO_I2Cx_SDA_PIN                     GPIO_PIN_7

/* I2C interrupt requests */
#define DISCOVERY_AUDIO_I2Cx_EV_IRQn                     I2C1_EV_IRQn
#define DISCOVERY_AUDIO_I2Cx_ER_IRQn                     I2C1_ER_IRQn

/* Definition for external, camera and Arduino connector I2Cx resources */
#define DISCOVERY_EXT_I2Cx                               I2C2
#define DISCOVERY_EXT_I2Cx_CLK_ENABLE()                  __HAL_RCC_I2C2_CLK_ENABLE()
#define DISCOVERY_EXT_DMAx_CLK_ENABLE()                  __HAL_RCC_DMA1_CLK_ENABLE()
#define DISCOVERY_EXT_I2Cx_SCL_SDA_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOB_CLK_ENABLE()

#define DISCOVERY_EXT_I2Cx_FORCE_RESET()                 __HAL_RCC_I2C2_FORCE_RESET()
#define DISCOVERY_EXT_I2Cx_RELEASE_RESET()               __HAL_RCC_I2C2_RELEASE_RESET()

/* Definition for I2Cx Pins */
#define DISCOVERY_EXT_I2Cx_SCL_PIN                       GPIO_PIN_10
#define DISCOVERY_EXT_I2Cx_SCL_SDA_GPIO_PORT             GPIOB
#define DISCOVERY_EXT_I2Cx_SCL_AF                        GPIO_AF4_I2C2
#define DISCOVERY_EXT_I2Cx_SDA_AF                        GPIO_AF9_I2C2
#define DISCOVERY_EXT_I2Cx_SDA_PIN                       GPIO_PIN_9

/* I2C interrupt requests */
#define DISCOVERY_EXT_I2Cx_EV_IRQn                       I2C2_EV_IRQn
#define DISCOVERY_EXT_I2Cx_ER_IRQn                       I2C2_ER_IRQn

/* I2C clock speed configuration (in Hz)
  WARNING:
   Make sure that this define is not already declared in other files.
   It can be used in parallel by other modules. */
#ifndef DISCOVERY_I2C_SPEED
 #define DISCOVERY_I2C_SPEED                             100000
#endif /* DISCOVERY_I2C_SPEED */


void I2Cx_Error(I2C_HandleTypeDef *i2c_handler, uint8_t Addr);

void AUDIO_IO_Init();
void AUDIO_IO_DeInit();
void AUDIO_IO_Delay(uint32_t Delay);
uint16_t AUDIO_IO_Read(uint8_t Addr, uint16_t Reg);
void AUDIO_IO_Write(uint8_t Addr, uint16_t Reg, uint16_t Value);

#endif /* USER_INC_AUDIO_IO_H_ */
