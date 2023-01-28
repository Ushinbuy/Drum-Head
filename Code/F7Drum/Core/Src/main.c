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
#include "wavFile.h"
//#include "audioExample.h"
#include "audioFromSdCard.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define OFF_DELAY_MS 200
#define FLASH_USER_START_ADDR 	0x08040000	//0x0804 0000 		//0x0801 F800
const volatile uint32_t *userConfig=(const volatile uint32_t *)FLASH_USER_START_ADDR;

#define NUMBER_OF_CHANNELS 6

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
DRUM channel[NUMBER_OF_CHANNELS];	// array of drums

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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;

I2C_HandleTypeDef hi2c3;

QSPI_HandleTypeDef hqspi;

SAI_HandleTypeDef hsai_BlockA2;
DMA_HandleTypeDef hdma_sai2_a;

SD_HandleTypeDef hsd1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC3_Init(void);
static void MX_FMC_Init(void);
static void MX_I2C3_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_SAI2_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

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
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == huart1.Instance) {
		buffer_in[15] = 1;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == hadc3.Instance) {
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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */
  MX_DMA_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC3_Init();
  MX_FMC_Init();
  MX_I2C3_Init();
  MX_QUADSPI_Init();
  MX_SAI2_Init();
  MX_SDMMC1_SD_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_FATFS_Init();
  MX_USB_DEVICE_Init();
  MX_DMA_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM6_STOP;		// shutdown TIM6 on debug
	HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, GPIO_PIN_RESET);	// shutdown display



	HAL_ADC_Start(&hadc3);
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
	HAL_UART_Receive_IT (&huart1, &config_Mode[0], 1);

	/// **************************
	/// ******* LETS ROCK! *******
	/// **************************
	HAL_TIM_Base_Start_IT(&htim2); //AS
	HAL_TIM_Base_Start_IT(&htim4); //ADC

	sdCardTextExample();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		handleConfigFromUart();
		if(isUsbConfigured()){
			sendMidiActiveSense(&upd_active_sens);
		}
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SDMMC1|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 5;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_8;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
  PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 6;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00C0EAFF;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 1;
  hqspi.Init.FifoThreshold = 4;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 24;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_6_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief SAI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI2_Init(void)
{

  /* USER CODE BEGIN SAI2_Init 0 */

  /* USER CODE END SAI2_Init 0 */

  /* USER CODE BEGIN SAI2_Init 1 */

  /* USER CODE END SAI2_Init 1 */
  hsai_BlockA2.Instance = SAI2_Block_A;
  hsai_BlockA2.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockA2.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA2.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA2.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA2.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_48K;
  hsai_BlockA2.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA2.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA2.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA2.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  if (HAL_SAI_InitProtocol(&hsai_BlockA2, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI2_Init 2 */

  /* USER CODE END SAI2_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 8;
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

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
  htim2.Init.Period = 66000000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  htim4.Init.Period = 20000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK1;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_ENABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 7;
  SdramTiming.SelfRefreshTime = 4;
  SdramTiming.RowCycleDelay = 7;
  SdramTiming.WriteRecoveryTime = 3;
  SdramTiming.RPDelay = 2;
  SdramTiming.RCDDelay = 2;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
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
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, ARDUINO_D7_Pin|ARDUINO_D8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_DISP_GPIO_Port, LCD_DISP_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DCMI_PWR_EN_GPIO_Port, DCMI_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, ARDUINO_D4_Pin|ARDUINO_D2_Pin|EXT_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LCD_B0_Pin */
  GPIO_InitStruct.Pin = LCD_B0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
  HAL_GPIO_Init(LCD_B0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_HS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_HS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_HS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TXD1_Pin RMII_TXD0_Pin RMII_TX_EN_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin|RMII_TXD0_Pin|RMII_TX_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_SCL_D15_Pin ARDUINO_SDA_D14_Pin */
  GPIO_InitStruct.Pin = ARDUINO_SCL_D15_Pin|ARDUINO_SDA_D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_D7_Pin ULPI_D6_Pin ULPI_D5_Pin ULPI_D3_Pin
                           ULPI_D2_Pin ULPI_D1_Pin ULPI_D4_Pin */
  GPIO_InitStruct.Pin = ULPI_D7_Pin|ULPI_D6_Pin|ULPI_D5_Pin|ULPI_D3_Pin
                          |ULPI_D2_Pin|ULPI_D1_Pin|ULPI_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_PWM_D3_Pin */
  GPIO_InitStruct.Pin = ARDUINO_PWM_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(ARDUINO_PWM_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPDIF_RX0_Pin */
  GPIO_InitStruct.Pin = SPDIF_RX0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF8_SPDIFRX;
  HAL_GPIO_Init(SPDIF_RX0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DCMI_D6_Pin DCMI_D7_Pin */
  GPIO_InitStruct.Pin = DCMI_D6_Pin|DCMI_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_B1_Pin LCD_B2_Pin LCD_B3_Pin LCD_G4_Pin
                           LCD_G1_Pin LCD_G3_Pin LCD_G0_Pin LCD_G2_Pin
                           LCD_R7_Pin LCD_R5_Pin LCD_R6_Pin LCD_R4_Pin
                           LCD_R3_Pin LCD_R1_Pin LCD_R2_Pin */
  GPIO_InitStruct.Pin = LCD_B1_Pin|LCD_B2_Pin|LCD_B3_Pin|LCD_G4_Pin
                          |LCD_G1_Pin|LCD_G3_Pin|LCD_G0_Pin|LCD_G2_Pin
                          |LCD_R7_Pin|LCD_R5_Pin|LCD_R6_Pin|LCD_R4_Pin
                          |LCD_R3_Pin|LCD_R1_Pin|LCD_R2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
  HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_VBUS_Pin */
  GPIO_InitStruct.Pin = OTG_FS_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Audio_INT_Pin */
  GPIO_InitStruct.Pin = Audio_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Audio_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_DE_Pin LCD_B7_Pin LCD_B6_Pin LCD_B5_Pin
                           LCD_G6_Pin LCD_G7_Pin LCD_G5_Pin */
  GPIO_InitStruct.Pin = LCD_DE_Pin|LCD_B7_Pin|LCD_B6_Pin|LCD_B5_Pin
                          |LCD_G6_Pin|LCD_G7_Pin|LCD_G5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
  HAL_GPIO_Init(GPIOK, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_B4_Pin */
  GPIO_InitStruct.Pin = LCD_B4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF9_LTDC;
  HAL_GPIO_Init(LCD_B4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DCMI_D5_Pin */
  GPIO_InitStruct.Pin = DCMI_D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(DCMI_D5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_D7_Pin ARDUINO_D8_Pin LCD_DISP_Pin */
  GPIO_InitStruct.Pin = ARDUINO_D7_Pin|ARDUINO_D8_Pin|LCD_DISP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : uSD_Detect_Pin */
  GPIO_InitStruct.Pin = uSD_Detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(uSD_Detect_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_HSYNC_Pin LCD_VSYNC_Pin LCD_R0_Pin LCD_CLK_Pin */
  GPIO_InitStruct.Pin = LCD_HSYNC_Pin|LCD_VSYNC_Pin|LCD_R0_Pin|LCD_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_BL_CTRL_Pin */
  GPIO_InitStruct.Pin = LCD_BL_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_BL_CTRL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DCMI_VSYNC_Pin */
  GPIO_InitStruct.Pin = DCMI_VSYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(DCMI_VSYNC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TP3_Pin NC2_Pin */
  GPIO_InitStruct.Pin = TP3_Pin|NC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : LD1_Pin */
  GPIO_InitStruct.Pin = LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(LD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DCMI_PWR_EN_Pin */
  GPIO_InitStruct.Pin = DCMI_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DCMI_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DCMI_D4_Pin DCMI_D3_Pin DCMI_D0_Pin DCMI_D2_Pin
                           DCMI_D1_Pin */
  GPIO_InitStruct.Pin = DCMI_D4_Pin|DCMI_D3_Pin|DCMI_D0_Pin|DCMI_D2_Pin
                          |DCMI_D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_PWM_CS_D5_Pin */
  GPIO_InitStruct.Pin = ARDUINO_PWM_CS_D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
  HAL_GPIO_Init(ARDUINO_PWM_CS_D5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_PWM_D10_Pin */
  GPIO_InitStruct.Pin = ARDUINO_PWM_D10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(ARDUINO_PWM_D10_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_INT_Pin */
  GPIO_InitStruct.Pin = LCD_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LCD_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_RX_D0_Pin ARDUINO_TX_D1_Pin */
  GPIO_InitStruct.Pin = ARDUINO_RX_D0_Pin|ARDUINO_TX_D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ULPI_NXT_Pin */
  GPIO_InitStruct.Pin = ULPI_NXT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(ULPI_NXT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_D4_Pin ARDUINO_D2_Pin EXT_RST_Pin */
  GPIO_InitStruct.Pin = ARDUINO_D4_Pin|ARDUINO_D2_Pin|EXT_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_STP_Pin ULPI_DIR_Pin */
  GPIO_InitStruct.Pin = ULPI_STP_Pin|ULPI_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_RXER_Pin */
  GPIO_InitStruct.Pin = RMII_RXER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RMII_RXER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DCMI_HSYNC_Pin PA6 */
  GPIO_InitStruct.Pin = DCMI_HSYNC_Pin|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_CLK_Pin ULPI_D0_Pin */
  GPIO_InitStruct.Pin = ULPI_CLK_Pin|ULPI_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_PWM_D6_Pin */
  GPIO_InitStruct.Pin = ARDUINO_PWM_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF9_TIM12;
  HAL_GPIO_Init(ARDUINO_PWM_D6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_MISO_D12_Pin ARDUINO_MOSI_PWM_D11_Pin */
  GPIO_InitStruct.Pin = ARDUINO_MISO_D12_Pin|ARDUINO_MOSI_PWM_D11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void sendUart (char *_msg){
	HAL_UART_Transmit(&huart1, (uint8_t*) _msg, strlen((char const*) _msg), 50);
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
		HAL_UART_Receive_IT(&huart1, &config_Mode[0], 1);
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
	if(!isUsbConfigured()){
		return;
	}
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

	HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
}


// READ Diginal state of aux channels
void getAuxState (GPIO_PinState *_state){
	_state[0] = HAL_GPIO_ReadPin(ARDUINO_RX_D0_GPIO_Port, ARDUINO_RX_D0_Pin);
	_state[1] = HAL_GPIO_ReadPin(ARDUINO_TX_D1_GPIO_Port, ARDUINO_TX_D1_Pin);
	_state[2] = HAL_GPIO_ReadPin(ARDUINO_D2_GPIO_Port, ARDUINO_D2_Pin);
	_state[3] = HAL_GPIO_ReadPin(ARDUINO_PWM_D3_GPIO_Port, ARDUINO_PWM_D3_Pin);

	_state[4] = HAL_GPIO_ReadPin(ARDUINO_D4_GPIO_Port, ARDUINO_D4_Pin);
	_state[5] = HAL_GPIO_ReadPin(ARDUINO_PWM_CS_D5_GPIO_Port, ARDUINO_PWM_CS_D5_Pin);
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
//			.Banks = FLASH_BANK_1,
			.Sector = 6,
			.NbSectors = 1
	};

	for (i = 0; i < 64; i++)
		SavingBuff[i] = 0;
	if (_rst == 0)
		SavingBuff[0] = 0xC4C0FFEE; // load settings marker
	else
		SavingBuff[0] = 0xFFFFFFFF; // do not load marker
	SavingBuff[1] = 0xBB;

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
							| FLASH_FLAG_PGAERR | FLASH_SR_ERSERR);

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


int get_num_from_uart(uint8_t _len){
	uint8_t i;
	int val = 0;
	for (i = 0; i<_len+1; i++)
		buffer_in[i] = 0;


	HAL_UART_Receive_IT (&huart1, &buffer_in[0], _len);
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
	HAL_UART_AbortReceive(&huart1);
	return val;
}

uint8_t UartConfigDialog(){

	int val = 0;

	uint8_t rtrn = 0;

	sendUart("\nConfig mode.\nType number of the pad [1..9], or hit the drum (x - reset to default):\n");

	buffer_in[0] = 0;
	HAL_UART_Receive_IT (&huart1, &buffer_in[0], 1);

	uint8_t chnl = 10;
	while (chnl == 10){
		  for (uint8_t ch = 0; ch < NUMBER_OF_CHANNELS; ch++)
			  if ((channel[ch].main_rdy)||(channel[ch].aux_rdy)){
				  channel[ch].main_rdy = 0;
				  channel[ch].aux_rdy = 0;
				  chnl = ch;
				  HAL_UART_AbortReceive(&huart1);
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
		HAL_UART_AbortReceive(&huart1);
		sendUart("Ciao\n");
		config_Mode[0] = 0;
		HAL_UART_Receive_IT (&huart1, &config_Mode[0], 1);
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
	HAL_UART_Receive_IT (&huart1, &buffer_in[0], 1);
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
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	// 10kHz trigger, 0.1ms
	if (htim->Instance == htim4.Instance) {
		HAL_ADC_Start_DMA(&hadc3, (uint32_t*) &adc_buf[0], NUMBER_OF_CHANNELS);
	}

	// 3.33Hz active sensing, 300ms
	if (htim->Instance == htim2.Instance) {
		upd_active_sens = 1;
	}
  /* USER CODE END Callback 1 */
}

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

