/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <usbd_cdc.h>
#include "midi.h"
#include "drumidy.h"
#include "wm8994.h"
#include "audio_sample.h"
#include "stm32412g_discovery_audio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define OFF_DELAY_MS 200
#define FLASH_USER_START_ADDR 	0x08040000	//0x0804 0000 		//0x0801 F800
const volatile uint32_t *userConfig=(const volatile uint32_t *)FLASH_USER_START_ADDR;

typedef struct
{
  uint32_t   ChunkID;       /* 0 */
  uint32_t   FileSize;      /* 4 */
  uint32_t   FileFormat;    /* 8 */
  uint32_t   SubChunk1ID;   /* 12 */
  uint32_t   SubChunk1Size; /* 16*/
  uint16_t   AudioFormat;   /* 20 */
  uint16_t   NbrChannels;   /* 22 */
  uint32_t   SampleRate;    /* 24 */

  uint32_t   ByteRate;      /* 28 */
  uint16_t   BlockAlign;    /* 32 */
  uint16_t   BitPerSample;  /* 34 */
  uint32_t   SubChunk2ID;   /* 36 */
  uint32_t   SubChunk2Size; /* 40 */

}WAVE_FormatTypeDef;

typedef enum
{
  AUDIO_STATE_IDLE = 0,
  AUDIO_STATE_INIT,
  AUDIO_STATE_PLAYING,
}AUDIO_PLAYBACK_StateTypeDef;


#define NUMBER_OF_CHANNELS 6

#define AUDIO_BUFFER_SIZE 128 	// must be equal to 20 ms

#define AUDIO_START_OFFSET_ADDRESS		44
#define AUDIO_FILE_ADDRESS				&AUDIO_SAMPLE[0]
#define AUDIO_FILE_SIZE					sizeof(AUDIO_SAMPLE)

char ASCIILOGO[] = "\n"\
"  ___                 _    _\n"\
" |   \\ _ _ _  _ _ __ (_)__| |_  _ \n"\
" | |) | '_| || | '  \\| / _` | || |\n"\
" |___/|_|  \\_,_|_|_|_|_\\__,_|\\_, |\n"\
"     .-.,     ,--. ,--.      |__/ \n"\
"    `/|~\\     \\__/T`--'     . \n"\
"    x |`' __   ,-~^~-.___ ==I== \n"\
"      |  |--| /       \\__}  | \n"\
"      |  |  |{   /~\\   }    | \n"\
"     /|\\ \\__/ \\  \\_/  /|   /|\\ \n"\
"    / | \\|  | /`~-_-~'X.\\ //| \\ \n\n"\
"= Send any char for configuration =\n";

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

__IO uint32_t uwCommand = AUDIO_PAUSE;
__IO uint32_t uwVolume = 70;
  uint8_t Volume_string[20] = {0};

uint32_t AudioTotalSize = 0xFFFF;  /* This variable holds the total size of the audio file */
uint32_t AudioRemSize   = 0xFFFF;  /* This variable holds the remaining data in audio file */
uint16_t* CurrentPos;              /* This variable holds the current position address of audio data */
static AUDIO_PLAYBACK_StateTypeDef  audio_state;

// --------------------------



DRUM channel[NUMBER_OF_CHANNELS];	// array of drums

int cnt;

// ADC buffers0
uint32_t adc_buf[NUMBER_OF_CHANNELS];
// channel values
uint16_t adc_val[NUMBER_OF_CHANNELS];

GPIO_PinState aux_current_state[NUMBER_OF_CHANNELS];

uint32_t saved_config[64];

char buffer_out[1000];			// USB Buffers
uint8_t buffer_in[64];

// MIDI operation
uint8_t upd_active_sens = 0;	//flag for active sense, triggered every 300ms
uint8_t config_Mode[1] = {0};		// flag for activating config over serial
uint32_t custom_timer = 0;


//SDCARD defines

FRESULT res;
uint32_t bytesWritten, bytesRead;
uint8_t wText[] = "STM32 FATFS works great";
uint8_t rText[_MAX_SS];

//int16_t dacData[AUDIO_BUFFER_SIZE];
//static volatile int16_t *outBufPtr = &dacData[0];
uint8_t dataReadyFlag; // must be renamed
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_tx;

SD_HandleTypeDef hsd;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
uint8_t CDC_Receive_FS(uint8_t* Buf, uint16_t Len);

void sendUart(char *_msg);

void getAuxState(GPIO_PinState *_state);
void checkPiezoChannels();
void handleConfigFromUart(void);

void sendDebug (uint8_t _ch, uint8_t _aux);
uint8_t Save_Setting(uint8_t _rst);
uint8_t Load_Setting();
int get_num_from_uart(uint8_t _len);
uint8_t UartConfigDialog();

uint32_t dataReceivedSize = 0;
volatile char flag_New_Settings;

void sdCardTextExample();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	// 10kHz trigger, 0.1ms
	if (htim->Instance == htim4.Instance) {
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &adc_buf[0], NUMBER_OF_CHANNELS);
		cnt++;
	}

	// 3.33Hz active sensing, 300ms
	if (htim->Instance == htim2.Instance) {
		upd_active_sens = 1;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == huart2.Instance) {
		buffer_in[15] = 1;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == hadc1.Instance) {
		for (uint8_t i = 0; i < NUMBER_OF_CHANNELS; i++) {
			adc_val[i] = adc_buf[i];
		}

		getAuxState(aux_current_state);

		setStepTime(HAL_GetTick());

		for (uint8_t i = 0; i < NUMBER_OF_CHANNELS; i++) {
			Update_channel(&channel[i], adc_val[i], aux_current_state[i]);
		}
	}
}
//
//void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
//	outBufPtr = &dacData[0];
//
//	dataReadyFlag = 1;
//}
//
//void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s) {
//	outBufPtr = &dacData[AUDIO_BUFFER_SIZE / 2];
//
//	dataReadyFlag = 1;
//}

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

/**
  * @brief  Manages the DMA FIFO error event.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_OUT_Error_CallBack(void)
{
  /* Display message on the LCD screen */
  sendUart("Error while playing audio");
  /* Stop the program with an infinite loop */
  while (1)
  {
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2S3_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */


	HAL_ADC_Start(&hadc1);
	HAL_Delay(200);

	/// **************************
	/// ******* Defaul CFG *******
	/// **************************
	getAuxState(aux_current_state);

	initDrum(&channel[0], HHCLOSE, HHCLOSE, MESH_PAD_AUTOAUX, aux_current_state[0]);
	initDrum(&channel[1], TOMF, TOMF, MESH_PAD_AUTOAUX, aux_current_state[1]);
	initDrum(&channel[2], HHPEDAL, HHPEDAL, MESH_PAD_AUTOAUX, aux_current_state[2]);
	initDrum(&channel[3], TOM3, TOM3, MESH_PAD_AUTOAUX, aux_current_state[3]);
	initDrum(&channel[4], HHOPEN, HHOPEN, MESH_PAD_AUTOAUX, aux_current_state[4]);
	initDrum(&channel[5], TOM2, TOM2, MESH_PAD_AUTOAUX, aux_current_state[5]);
//  initDrum(&channel[6], TOMF , TOMF  	, MESH_PAD_AUTOAUX	, aux_current_state[6]);
//
//  // cymbals
//  initDrum(&channel[7], CRASH, CRASH 	, CYMBAL_MUTE			, aux_current_state[7]);	// CH7 aux disabled
//  initDrum(&channel[8], RIDE ,  BELL 	, CYMBAL_2_ZONE			, aux_current_state[8]);

	// === Previous Settings ===
	sendUart(ASCIILOGO);
	HAL_Delay(500);

	Load_Setting();

	// start waiting for serial commands
	HAL_Delay(200);
	config_Mode[0] = 0;
	HAL_UART_Receive_IT (&huart2, &config_Mode[0], 1);

	/// **************************
	/// ******* LETS ROCK! *******
	/// **************************
	HAL_TIM_Base_Start_IT(&htim2); //AS
	HAL_TIM_Base_Start_IT(&htim4); //ADC

	WAVE_FormatTypeDef *waveformat = NULL;
	if (BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, uwVolume,
			I2S_AUDIOFREQ_8K) != AUDIO_OK) {
		/* Initialization Error */
		sendUart("Initialization problem");
		Error_Handler();
	} else {
		sendUart("Audio Codec Ready");
	}

	audio_state = AUDIO_STATE_PLAYING;
	waveformat = (WAVE_FormatTypeDef*) AUDIO_FILE_ADDRESS;

	AudioTotalSize = (AUDIO_FILE_SIZE - AUDIO_START_OFFSET_ADDRESS)
			/ (waveformat->NbrChannels);
	/* Set the current audio pointer position */
	CurrentPos = (uint16_t*) (AUDIO_FILE_ADDRESS + AUDIO_START_OFFSET_ADDRESS);
	/* Start the audio player */
	if (BSP_AUDIO_OUT_Play((uint16_t*) CurrentPos,
			(uint32_t) (AUDIO_FILE_SIZE - AUDIO_START_OFFSET_ADDRESS)) != AUDIO_OK) {
		sendUart("Can't play audio");
		Error_Handler();
	} else {
		sendUart("Start playing audio");
	}

//	sdCardTextExample();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		handleConfigFromUart();
		sendMidiActiveSense(&upd_active_sens);
		checkPiezoChannels();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 30000000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 8400;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED3_Pin|LED4_Pin|LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, LCD_BLCTRL_Pin|EXT_RESET_Pin|CTP_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_OTGFS_PPWR_EN_GPIO_Port, USB_OTGFS_PPWR_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LED3_Pin LED4_Pin LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED3_Pin|LED4_Pin|LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : DD3_Pin DD5_Pin */
  GPIO_InitStruct.Pin = DD3_Pin|DD5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_BLCTRL_Pin EXT_RESET_Pin CTP_RST_Pin */
  GPIO_InitStruct.Pin = LCD_BLCTRL_Pin|EXT_RESET_Pin|CTP_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : JOY_SEL_Pin */
  GPIO_InitStruct.Pin = JOY_SEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(JOY_SEL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : JOY_RIGHT_Pin JOY_LEFT_Pin */
  GPIO_InitStruct.Pin = JOY_RIGHT_Pin|JOY_LEFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : JOY_UP_Pin JOY_DOWN_Pin LCD_TE_Pin USB_OTGFS_OVRCR_Pin */
  GPIO_InitStruct.Pin = JOY_UP_Pin|JOY_DOWN_Pin|LCD_TE_Pin|USB_OTGFS_OVRCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : I2C2_SCL_Pin */
  GPIO_InitStruct.Pin = I2C2_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(I2C2_SCL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : M2_CKIN_Pin */
  GPIO_InitStruct.Pin = M2_CKIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(M2_CKIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_RESET_Pin */
  GPIO_InitStruct.Pin = LCD_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CODEC_INT_Pin CTP_INT_Pin */
  GPIO_InitStruct.Pin = CODEC_INT_Pin|CTP_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTGFS_PPWR_EN_Pin */
  GPIO_InitStruct.Pin = USB_OTGFS_PPWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_OTGFS_PPWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : M2_CKINA8_Pin */
  GPIO_InitStruct.Pin = M2_CKINA8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(M2_CKINA8_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : uSD_DETECT_Pin */
  GPIO_InitStruct.Pin = uSD_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(uSD_DETECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DD0_Pin DD4_Pin DD2_Pin DD1_Pin */
  GPIO_InitStruct.Pin = DD0_Pin|DD4_Pin|DD2_Pin|DD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : I2C2_SDA_Pin */
  GPIO_InitStruct.Pin = I2C2_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_I2C2;
  HAL_GPIO_Init(I2C2_SDA_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void sendUart (char *_msg){
	HAL_UART_Transmit(&huart2, (uint8_t*) _msg, strlen((char const*) _msg), 50);
}

void handleConfigFromUart(void){
	while (config_Mode[0]) {

		uint8_t rs = UartConfigDialog();

		if ((rs == 1) || (rs == 2)) {
			rs = Save_Setting(0);
			sprintf(buffer_out, "New configuration saved (%X)\n", rs);
			sendUart(buffer_out);
		}
		if (rs == 99) {
			rs = Save_Setting(1);
			sprintf(buffer_out, "Reset to default values, restart the device (%X)\n", rs);
			sendUart(buffer_out);
		}
		config_Mode[0] = 0;
		HAL_UART_Receive_IT(&huart2, &config_Mode[0], 1);
	}
}

void checkPiezoChannels(){
	int _volume;
	for (uint8_t ch = 0; ch < NUMBER_OF_CHANNELS; ch++) {

		if (channel[ch].main_rdy) {
			channel[ch].main_rdy = 0;

			// custom volume calculation for mesh
			if (channel[ch].chnl_type < 2) {
				_volume = (int) (100.
						* (float) (channel[ch].main_rdy_height - PEAK_THRESHOLD)
						/ 4096. * 100. / (float) channel[ch].peak_volume_norm);
				if ((channel[ch].chnl_type == MESH_RIM_AUTOAUX)
						&& (channel[ch].main_rdy_usealt))
					_volume = _volume * 4;
			} else {
				//volume for cymbals
				_volume = (int) (100.
						* (float) (channel[ch].main_rdy_height - PEAK_THRESHOLD)
						/ 4096. * 100. / (float) channel[ch].peak_volume_norm
						* 2);
			}

			if (_volume > 127)
				_volume = 127;
			if (_volume < 1)
				_volume = 1;
			channel[ch].main_rdy_volume = (uint8_t) _volume;

			uint8_t vc;
			if (channel[ch].main_rdy_usealt)
				vc = channel[ch].alt_voice;	//	sendMidiGEN(channel[ch].alt_voice ,channel[ch].main_rdy_volume);
			else
				vc = channel[ch].main_voice;//	sendMidiGEN(channel[ch].main_voice,channel[ch].main_rdy_volume);

//#define DEBUG_ADC
#ifdef DEBUG_ADC
			  sprintf(buffer_out, "\r\n 0- %d 1 - %d 2 - %d 3 - %d 4 - %d 5 - %d", adc_val[0], adc_val[1], adc_val[2], adc_val[3], adc_val[4], adc_val[5]);
			  sendUart(buffer_out);
#endif
			sendMidi(vc, channel[ch].main_rdy_volume);
			channel[ch].main_last_on_voice = vc;
			channel[ch].main_last_on_time = HAL_GetTick();

			sendDebug(ch, 0);
		}

		if (channel[ch].aux_rdy) {
			channel[ch].aux_rdy = 0;
			sendDebug(ch, 1);

			switch (channel[ch].chnl_type) {
			case CYMBAL_HIHAT:
				if (channel[ch].aux_rdy_state == CHANNEL_PEDAL_PRESSED)
					sendMidiHHPedalOn();
				//				  else
				//					  sendMidiGEN(channel[ch].main_voice, 5);
				break;

			case CYMBAL_MUTE:
				if (channel[ch].aux_rdy_state == CHANNEL_PEDAL_PRESSED)
					sendMidi2(channel[ch].main_voice, 1, channel[ch].main_voice,
							0);
				break;

			case CYMBAL_2_ZONE:
				sendMidi2(channel[ch].main_voice, 1, channel[ch].main_voice, 0);
				break;

				// INDEPENDENT AUX INPUTS
			default:
				if (channel[ch].aux_type == AUX_TYPE_PAD)
					sendMidi2(channel[ch].aux_voice, 100, channel[ch].aux_voice,
							0);
				else { //PEDAL
					   // PEDAL pressed
					if (channel[ch].aux_rdy_state == CHANNEL_PEDAL_PRESSED)
						sendMidi2(channel[ch].aux_voice, 100,
								channel[ch].aux_voice, 0);
					// PEDAL RELEASED... IN CASE
					//					  else
					//						  sendMidi2(channel[ch].aux_voice, 1, channel[ch].aux_voice,0);
				}
			}
		}
		// send off command if needed
		if (channel[ch].main_last_on_voice > 0) {
			if (HAL_GetTick()
					> (channel[ch].main_last_on_time + OFF_DELAY_MS)) {
				sendMidi(channel[ch].main_last_on_voice, 0);
				channel[ch].main_last_on_voice = 0;
			}
		}
	}
}



void tx_midi(uint8_t *_buffer, uint16_t len) {
	uint8_t rt = USBD_BUSY;

	while (rt == USBD_BUSY) {
		rt = CDC_Transmit_FS(_buffer, len);
	};

	TIM2->CNT = 0; // restart active sense timer
}

void sendDebug(uint8_t _ch, uint8_t _aux)
{
	uint8_t voice;
	uint8_t volume;
	uint8_t length;

	if (_aux) {
		voice = channel[_ch].aux_voice;

		sprintf(buffer_out, ">>>AUX %d: %X %d [%d %d]\n", _ch, voice,
				channel[_ch].aux_rdy_state, channel[_ch].main_peaking,
				channel[_ch].aux_status);
	} else {
		if (channel[_ch].main_rdy_usealt)
			voice = channel[_ch].alt_voice;
		else
			voice = channel[_ch].main_voice;
		volume = channel[_ch].main_rdy_volume;
		length = channel[_ch].main_rdy_length;
		sprintf(buffer_out,
				">>MAIN %d: voice %X (alt:%d), vol %d (%u/4096) t=%u; AUX = %d\n",
				_ch, voice, channel[_ch].alt_voice, volume,
				channel[_ch].main_rdy_height, length, channel[_ch].aux_status);
	}
	sendUart(buffer_out);

	HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
}


// READ Diginal state of aux channels
void getAuxState (GPIO_PinState *_state){
	_state[0] = HAL_GPIO_ReadPin(DD0_GPIO_Port, DD0_Pin);
	_state[1] = HAL_GPIO_ReadPin(DD1_GPIO_Port, DD1_Pin);
	_state[2] = HAL_GPIO_ReadPin(DD2_GPIO_Port, DD2_Pin);
	_state[3] = HAL_GPIO_ReadPin(DD3_GPIO_Port, DD3_Pin);

	_state[4] = HAL_GPIO_ReadPin(DD4_GPIO_Port, DD4_Pin);
	_state[5] = HAL_GPIO_ReadPin(DD5_GPIO_Port, DD5_Pin);
//	_state[6] = HAL_GPIO_ReadPin(DIG_IN7_GPIO_Port, DIG_IN7_Pin);
//	_state[7] = 0; //HAL_GPIO_ReadPin(DIG_IN8_GPIO_Port, DIG_IN8_Pin);

//	_state[8] = HAL_GPIO_ReadPin(DIG_IN9_GPIO_Port, DIG_IN9_Pin);
//	_state[9] = 0;
}

uint8_t Save_Setting(uint8_t _rst)
{
	uint32_t SavingBuff[64];
	uint8_t i;
	uint32_t error = 0;
	uint64_t val = 0;

	FLASH_EraseInitTypeDef FLASH_EraseInitStruct = {
			.TypeErase = FLASH_TYPEERASE_SECTORS,
			.Banks = FLASH_BANK_1,
			.Sector = 6,
			.NbSectors = 1
	};

	for (i = 0; i < 64; i++)
		SavingBuff[i] = 0;
//112233445566778899 AABBCCDDEEFF
	if (_rst == 0)
		SavingBuff[0] = 0xC4C0FFEE; // load settings marker
	else
		SavingBuff[0] = 0xFFFFFFFF; // do not load marker
	SavingBuff[1] = 0xBB;

	// 0x11223344
	for (i = 1; i < 10; i++) {
		// channel configuration settings
		SavingBuff[2 * i] = (channel[i - 1].main_voice & 0xFF) * 0x01000000;
		SavingBuff[2 * i] += (channel[i - 1].aux_voice & 0xFF) * 0x00010000;
		SavingBuff[2 * i] += (channel[i - 1].alt_voice & 0xFF) * 0x00000100;
		SavingBuff[2 * i] += (channel[i - 1].chnl_type & 0xFF);
		// channel parameter settings
		SavingBuff[2 * i + 1] = (channel[i - 1].peak_volume_norm & 0xFF) * 0x01000000;
		SavingBuff[2 * i + 1] += (channel[i - 1].peak_min_length & 0xFF) * 0x00010000;
		SavingBuff[2 * i + 1] += (channel[i - 1].peak_max_length & 0xFF) * 0x00000100;
//		SavingBuff[2*i + 1] += (channel[i-1].peak2peak  & 0xFF);
	}

	HAL_StatusTypeDef err;
	uint8_t st = 0;
	err = HAL_FLASH_Unlock();
	if (err != HAL_OK)
		st += 0b10000000;
	__HAL_FLASH_CLEAR_FLAG (FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR
							| FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR);

	err = HAL_FLASHEx_Erase(&FLASH_EraseInitStruct, &error);
	if (err != HAL_OK)
		st += 0b01000000;

	for (i = 0; i < 32; i++) {
		val = (((uint64_t) SavingBuff[i * 2 + 1]) << 32) + SavingBuff[i * 2];
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
				FLASH_USER_START_ADDR + 8 * i, val) != HAL_OK)
			st += 1;
	}
	if (HAL_FLASH_Lock() != HAL_OK)
		st += 0b00100000;

	for (i = 0; i < 64; i++)
		saved_config[0] = 0;

	return st;
}


uint8_t Load_Setting()
{
	uint8_t i;
//	uint32_t LoadingBuff[64];

	for (i=0;i<64;i++){
		saved_config[i] = *(userConfig+i);
	}

	if (saved_config[0] != 0xC4C0FFEE) return 0;

	for (i = 1; i < 10; i++){
		channel[i-1].main_voice = 0xff & (uint8_t)(saved_config[2*i]>>24);
		channel[i-1].aux_voice 	= 0xff & (uint8_t)(saved_config[2*i]>>16);
		channel[i-1].alt_voice 	= 0xff & (uint8_t)(saved_config[2*i]>>8);
		channel[i-1].chnl_type 	= 0xff & (uint8_t)(saved_config[2*i]);

		//		channel[i-1].peak_threshold 	= 0xff & (uint8_t)(saved_config[2*i+1]>>24);
		channel[i-1].peak_volume_norm 	= 0xff & (uint8_t)(saved_config[2*i+1]>>24);
		channel[i-1].peak_min_length 	= 0xff & (uint8_t)(saved_config[2*i+1]>>16);
		channel[i-1].peak_max_length 	= 0xff & (uint8_t)(saved_config[2*i+1]>>8);
//		channel[i-1].time_between_peaks = 0xff & (uint8_t)(saved_config[2*i+1]);
	}

	sprintf(buffer_out, "........ Previous settings: .......\n%08lX %08lX %08lX %08lX\n%08lX %08lX %08lX %08lX\n%08lX %08lX %08lX %08lX\n%08lX %08lX %08lX %08lX\n%08lX %08lX %08lX %08lX\n",
	  saved_config[0] ,saved_config[1] ,saved_config[2] ,saved_config[3] ,
	  saved_config[4] ,saved_config[5] ,saved_config[6] ,saved_config[7] ,
	  saved_config[8] ,saved_config[9] ,saved_config[10],saved_config[11],
	  saved_config[12],saved_config[13],saved_config[14],saved_config[15],
	  saved_config[16],saved_config[17],saved_config[18],saved_config[19]);

		sendUart(buffer_out);
	  HAL_Delay(500);

	return 1;
}

//receive number from serial or a given max length
void sdCardTextExample(){
	if(f_mount(&SDFatFS, (TCHAR const*)SDPath, 0) != FR_OK){
			sendUart("SD CARD NOT DETECTED");
		}
		else{
			if(f_mkfs((TCHAR const*)SDPath, FM_ANY, 0, rText, sizeof(rText)) != FR_OK){
				sendUart("SD CARD FS NOT ACCESS");
			}
			else{
				if(f_open(&SDFile, "STM32", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK){
					sendUart("SD CARD FS NOT ACCESS");
				}
				else{
					res = f_write(&SDFile, wText, strlen((char *)wText), (void *)&bytesWritten);
					if((bytesWritten == 0) || (res != FR_OK)){
						sendUart("SD CARD CAN't WRITE FILE");
					}
					else{
						f_close(&SDFile);
						sendUart("SD CARD write is done");
					}
				}
			}
			f_mount(&SDFatFS, (TCHAR const*)NULL, 0);
		}
}

int get_num_from_uart(uint8_t _len){
	uint8_t i;
	int val = 0;
	for (i = 0; i<_len+1; i++)
		buffer_in[i] = 0;


	HAL_UART_Receive_IT (&huart2, &buffer_in[0], _len);
	while (buffer_in[0] == 0) {HAL_Delay(1);};
	HAL_Delay(2); // wait for the rest of the message

	val = 0;
	for (i = 0; i<_len; i++){
		if ((buffer_in[i] == 0) || (buffer_in[i] == 10) || (buffer_in[i] == 13)) break;
		if ((buffer_in[0]>='0') && (buffer_in[0]<='9'))
			val = val*10 + (buffer_in[i]-'0');
		else{
			val = -1;
			break;
		}
	}
	HAL_UART_AbortReceive(&huart2);
	return val;
}

uint8_t UartConfigDialog(){

	int val = 0;

	uint8_t rtrn = 0;

	sendUart("\nConfig mode.\nType number of the pad [1..9], or hit the drum (x - reset to default):\n");

	buffer_in[0] = 0;
	HAL_UART_Receive_IT (&huart2, &buffer_in[0], 1);

	uint8_t chnl = 10;
	while (chnl == 10){
		  for (uint8_t ch = 0; ch < NUMBER_OF_CHANNELS; ch++)
			  if ((channel[ch].main_rdy)||(channel[ch].aux_rdy)){
				  channel[ch].main_rdy = 0;
				  channel[ch].aux_rdy = 0;
				  chnl = ch;
				  HAL_UART_AbortReceive(&huart2);
			  }
		  if (buffer_in[0]>0){
			  if ((buffer_in[0]>='1') && (buffer_in[0]<='9'))
				  chnl = buffer_in[0]-'1';
			  else
				  chnl = 255;

			  if (buffer_in[0]=='x')
				  // reset to default
				  return 99;
		  }
	}

	if (chnl == 255) {
		HAL_UART_AbortReceive(&huart2);
		sendUart("Ciao\n");
		config_Mode[0] = 0;
		HAL_UART_Receive_IT (&huart2, &config_Mode[0], 1);
		return 0;
	}

	// got the correct channel.
	// print current values
	sprintf(buffer_out, "Current values CH#%d:\n\tVoices: main %d, aux %d, alt %d\n\tTimings: peak min %d max %d\n\tChannel type: %d,volume norm %d\n",
			chnl+1, channel[chnl].main_voice, channel[chnl].aux_voice, channel[chnl].alt_voice,
			(int)channel[chnl].peak_min_length, (int)channel[chnl].peak_max_length,
			channel[chnl].aux_type, (int)channel[chnl].peak_volume_norm);
	sendUart(buffer_out);
	HAL_Delay(200);

	// Starting to change the values
	// main voicepeak_volume_norm
	sprintf(buffer_out, "\nCH#%d Change main voice from %d:\t",chnl+1, channel[chnl].main_voice);
	sendUart(buffer_out);
	HAL_Delay(200);

	 val = get_num_from_uart(2);
	if ((val>25)&&(val<90)){
		channel[chnl].main_voice = val & 0xFF;
		sprintf(buffer_out, "New main voice: %d\n", channel[chnl].main_voice);
	}else
		sprintf(buffer_out, "Keeping the old value: %d\n", channel[chnl].main_voice);
	sendUart(buffer_out);
	HAL_Delay(200);

	// aux voice
	sprintf(buffer_out, "\nCH#%d Change aux input voice from %d:\t",chnl+1, channel[chnl].aux_voice);
	sendUart(buffer_out);
	HAL_Delay(200);

	 val = get_num_from_uart(2);
	if ((val>25)&&(val<90)){
		channel[chnl].aux_voice = val & 0xFF;
		sprintf(buffer_out, "New aux voice: %d\n", channel[chnl].aux_voice);
	}else
		sprintf(buffer_out, "Keeping the old value: %d\n", channel[chnl].aux_voice);
	sendUart(buffer_out);
	HAL_Delay(200);

	// main alt voice
	sprintf(buffer_out, "\nCH#%d Change main alt voice (when pedal pressed) from %d:\t",chnl+1, channel[chnl].alt_voice);
	sendUart(buffer_out);
	HAL_Delay(200);

	 val = get_num_from_uart(2);
	if ((val>25)&&(val<90)){
		channel[chnl].alt_voice = val & 0xFF;
		sprintf(buffer_out, "New alt voice: %d\n", channel[chnl].alt_voice);
	}else
		sprintf(buffer_out, "Keeping the old value: %d\n", channel[chnl].alt_voice);
	sendUart(buffer_out);
	HAL_Delay(200);

	// channel type
	sprintf(buffer_out, "\nCH#%d Change aux type from %d to:\n\tAUX - auto, MAIN - Mesh(0), Mesh with rim(1), or Cymbal(2),\n\t HiHat(3) with pedal, Cymbal with 2 zones(4), Cymabal with mute button(5)\n", chnl+1,  channel[chnl].chnl_type);
	sendUart(buffer_out);
	HAL_Delay(200);

	val = get_num_from_uart(1);
	if ((val>=0)&&(val<=4)){
		channel[chnl].chnl_type = val & 0xFF;
		sprintf(buffer_out, "New channel type: %d\n", channel[chnl].chnl_type);
	}else
		sprintf(buffer_out, "Keeping the old value: %d\n", channel[chnl].chnl_type);
	sendUart(buffer_out);
	HAL_Delay(200);

	rtrn = 1;
	sprintf(buffer_out, "\nAdjust timing? y - yes, n - save settings and exit\n");
	sendUart(buffer_out);
	HAL_Delay(200);


	buffer_in[0] = 0;
	HAL_UART_Receive_IT (&huart2, &buffer_in[0], 1);
	while (buffer_in[0] == 0){HAL_Delay(1);}
	if (buffer_in[0] == 'y'){

		// Peak threshold
		sprintf(buffer_out, "\nCH#%d Volume norm = %d (default 50, 0..255) (full volume point, 100~4096). New:\t",chnl+1,(int) channel[chnl].peak_volume_norm);
		sendUart(buffer_out);
		HAL_Delay(200);

		val = get_num_from_uart(3);
		if ((val>0)&&(val<256)){
			channel[chnl].peak_volume_norm = val;
			sprintf(buffer_out, "New threshold = %d\n", (int)channel[chnl].peak_volume_norm);
		}else
			sprintf(buffer_out, "Keeping the old value: %d\n", (int)channel[chnl].peak_volume_norm);
		sendUart(buffer_out);
		HAL_Delay(200);

		// min peak len
		sprintf(buffer_out, "\nCH#%d Peak min length = %d (default mesh 15, cymbal 4, 1..99) [x0.1ms]. New:\t",chnl+1,(int) channel[chnl].peak_min_length);
		sendUart(buffer_out);
		HAL_Delay(200);

		val = get_num_from_uart(2);
		if ((val>0)&&(val<100)){
			channel[chnl].peak_min_length = val;
			sprintf(buffer_out, "New min length = %d\n", (int)channel[chnl].peak_min_length);
		}else
			sprintf(buffer_out, "Keeping the old value: %d\n", (int)channel[chnl].peak_min_length);
		sendUart(buffer_out);
		HAL_Delay(200);

		// max peak len
		sprintf(buffer_out, "\nCH#%d Peak max length = %d (default 200, 1..255) [x0.1ms]. New:\t",chnl+1, (int)channel[chnl].peak_max_length);
		sendUart(buffer_out);
		HAL_Delay(200);

		val = get_num_from_uart(3);
		if ((val>0)&&(val<256)){
			channel[chnl].peak_max_length = val;
			sprintf(buffer_out, "New max length = %d\n", (int)channel[chnl].peak_max_length);
		}else
			sprintf(buffer_out, "Keeping the old value: %d\n", (int)channel[chnl].peak_max_length);
		sendUart(buffer_out);
		HAL_Delay(200);
		rtrn = 2;
	}
	return rtrn;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

