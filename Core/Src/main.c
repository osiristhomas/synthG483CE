/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
 */
/* Osiris Thomas
 * STM32 Synthesizer
 * Last edited: 12/7/2021
 */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "notes.h"

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;

DAC_HandleTypeDef hdac1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;


// Global midi note variables
unsigned char midi_msg[NUM_MIDI_BYTES];
unsigned char midi_tmp[NUM_MIDI_BYTES];

// Voices
struct voice voices[MAX_NOTES];

// Current number of notes on
uint8_t notes_on = 0;

// A-to-D result for wave select
uint16_t AD_wave_sel = 0;

// A-to-D results for ADSR values
uint16_t AD_ADSR[4];

// Waveform multiplier
float multiplier = 1.0;

// Sine LUT - generated with https://www.daycounter.com/Calculators/Sine-Generator-Calculator.phtml
uint16_t sin_lut[NUM_PTS] = {683,716,749,783,816,848,881,912,944,974,1004,1033,1062,1089,1115,1141,1165,1188,1210,1231,1250,1268,1284,1299,1313,1325,1336,1345,1352,1358,1362,1364,1365,1364,1362,1358,1352,1345,1336,1325,1313,1299,1284,1268,1250,1231,1210,1188,1165,1141,1115,1089,1062,1033,1004,974,944,912,881,848,816,783,749,716,683,649,616,582,549,517,484,453,421,391,361,332,303,276,250,224,200,177,155,134,115,97,81,66,52,40,29,20,13,7,3,1,0,1,3,7,13,20,29,40,52,66,81,97,115,134,155,177,200,224,250,276,303,332,361,391,421,453,484,517,549,582,616,649};

// Triangle LUT - generated with https://www.daycounter.com/Calculators/Triangle-Wave-Generator-Calculator2.phtml
uint16_t tri_lut[NUM_PTS] = {21,43,64,85,107,128,149,171,192,213,235,256,277,299,320,341,363,384,405,427,448,469,491,512,533,555,576,597,619,640,661,683,704,725,746,768,789,810,832,853,874,896,917,938,960,981,1002,1024,1045,1066,1088,1109,1130,1152,1173,1194,1216,1237,1258,1280,1301,1322,1344,1365,1344,1322,1301,1280,1258,1237,1216,1194,1173,1152,1130,1109,1088,1066,1045,1024,1002,981,960,938,917,896,874,853,832,810,789,768,746,725,704,683,661,640,619,597,576,555,533,512,491,469,448,427,405,384,363,341,320,299,277,256,235,213,192,171,149,128,107,85,64,43,21,0};

// Square LUT
uint16_t sqr_lut[NUM_PTS] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365,1365};

// Sawtooth LUT - generated with MATLAB
uint16_t saw_lut[NUM_PTS] = {0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230,240,250,260,270,280,290,300,310,320,330,340,350,360,370,380,390,400,410,420,430,440,450,460,470,480,490,500,510,520,530,540,550,560,570,580,590,600,610,620,630,640,650,660,670,680,690,700,710,720,730,740,750,760,770,780,790,800,810,820,830,840,850,860,870,880,890,900,910,920,930,940,950,960,970,980,990,1000,1010,1020,1030,1040,1050,1060,1070,1080,1090,1100,1110,1120,1130,1140,1150,1160,1170,1180,1190,1200,1210,1220,1230,1240,1250,1260,1270};

// Look up table used in wavetable synthesis
uint16_t *lut = sin_lut;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM8_Init(void);
static inline void LED_Handler(void);
static void Update_Env_Mult(void);
static void Update_Wave_Shape(void);
static inline void Init_Voices(void);
static void Update_ADSR_Param(uint32_t, uint8_t);

int main(void)
{
	// Reset of all peripherals, Initializes the Flash interface and the Systick.
	HAL_Init();

	// Configure the system clock
	SystemClock_Config();

	// Initialize all voices to their reset value
	Init_Voices();

	// Initialize all configured peripherals
	MX_GPIO_Init();
	MX_DAC1_Init();
	MX_USART1_UART_Init();
	MX_ADC1_Init();
	MX_ADC3_Init();
	MX_TIM2_Init();
	MX_TIM6_Init();
	MX_TIM7_Init();
	MX_TIM8_Init();

	// Calibrate ADCs
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);

	// Enable DAC
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);

	// Used for ADC timing
	HAL_TIM_Base_Start_IT(&htim2);

	// Used for voices
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_Base_Start_IT(&htim8);

	// Get initial MIDI message
 	HAL_UART_Receive_IT(&huart1, midi_tmp, NUM_MIDI_BYTES);

 	// Program hangs here while waiting for an interrupt from timers or UART
 	while (1);

}

static inline void Init_Voices(void)
{
	uint8_t i;
	for (i = 0; i < MAX_NOTES; i++) {
		// Initialize each voice gate to OFF
		voices[i].gate = OFF;

		// Initialize each voice status to OFF
		voices[i].status = OFF;

		// Initialize each envelope value to 0
		voices[i].env_val = 0.0;

		// Initialize each voice index to start at beginning of lookup table
		voices[i].lut_index = 0;
	}
}

// Select lookup table based on POT ADC value
static void Update_Wave_Shape(void)
{

	HAL_ADC_Start(&hadc3);
	AD_wave_sel = HAL_ADC_GetValue(&hadc3);
	HAL_ADC_Stop(&hadc3);

	// Change wave shape based on shape POT
	if (AD_wave_sel >= 0 && AD_wave_sel < 1024) {
	    lut = sin_lut;
	}
	else if (AD_wave_sel >= 1024 && AD_wave_sel < 2048) {
	 	lut = tri_lut;
	}
	else if (AD_wave_sel >= 2048 && AD_wave_sel < 3072) {
	 	lut = saw_lut;
	}
	else if (AD_wave_sel >= 3072 && AD_wave_sel < 4096) {
		lut = sqr_lut;
	}
}

// Configure an ADC1 channel and store a new ADSR parameter
void Update_ADSR_Param(uint32_t channel, uint8_t param)
 {
	 ADC_ChannelConfTypeDef sConfig = {0};
	 sConfig.Channel = channel;
	 sConfig.Rank = ADC_REGULAR_RANK_1;
	 sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	 sConfig.SingleDiff = ADC_SINGLE_ENDED;
	 sConfig.OffsetNumber = ADC_OFFSET_NONE;
	 sConfig.Offset = 0;
	 if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	 {
	  	Error_Handler();
	 }

	 HAL_ADC_Start(&hadc1);
	 HAL_ADC_PollForConversion(&hadc1, 1);
	 AD_ADSR[param] = HAL_ADC_GetValue(&hadc1);
	 HAL_ADC_Stop(&hadc1);
 }

// Get new envelope multiplier
static void Update_Env_Mult(void)
{
	uint8_t i = 0;

	Update_ADSR_Param(ADC_CHANNEL_1, ATTACK);
	Update_ADSR_Param(ADC_CHANNEL_2, DECAY);
	Update_ADSR_Param(ADC_CHANNEL_3, SUSTAIN);
	Update_ADSR_Param(ADC_CHANNEL_4, RELEASE);

	for (i = 0; i < 3; i++) {
		switch(voices[i].state) {
		// Attack - Increase envelope value until 1.0 is reached
		case ATTACK:
			voices[i].rate = 1.0 / ATTACK_VAL;
			// Check if index had reached end of attack phase
			if (voices[i].env_val >= 1.0) {
				voices[i].state = DECAY;
				voices[i].env_val = 1.0;
			}
			else {
				voices[i].env_val += voices[i].rate;
			}
			break;
		// Decay - Decrease envelope value until sustain value is reached
		case DECAY:
			voices[i].rate = (DECAY_NORM - 1.0) / DECAY_VAL;
			if (voices[i].env_val <= SUSTAIN_NORM) {
				voices[i].state = SUSTAIN;
				// Return sustain level as it is last value of decay phase
				voices[i].env_val = SUSTAIN_NORM;
			}
			else {
				voices[i].env_val += voices[i].rate;
			}
			break;
		// Sustain - Hold sustain value until key is released
		case SUSTAIN:
			if (voices[i].gate == OFF) {
				voices[i].state = RELEASE;
			}
			// Remain here until gate has been turned off by key release
			voices[i].env_val = SUSTAIN_NORM;
			break;

		// Release - Decrease envelope value until value has reached 0 - turn off note
		case RELEASE:
			voices[i].rate = SUSTAIN_NORM / RELEASE_VAL;
			if (voices[i].env_val <= 0.0) {
				voices[i].status = OFF;
				voices[i].env_val = 0.0;
			}
			else {
				voices[i].env_val -= voices[i].rate;
			}
			break;
		}
	}
}

// Turn on LED while a key is pressed
static inline void LED_Handler(void)
{
	if (notes_on > 0) {
		MIDI_IN_LED_ON;
	}
	else {
		MIDI_IN_LED_OFF;
	}
}

 // When timer overflows, put corresponding signal on DAC
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
 {

	if (htim == &htim2) {
		Update_Wave_Shape();
 		Update_Env_Mult();
	}

	else if (htim == &htim6) {
 		PUT_TO_DAC(VOICE_SUM);
 		voices[0].lut_index++;
 		RST_INDEX(0);
 	}
 	else if (htim == &htim7) {
 		PUT_TO_DAC(VOICE_SUM);
 		voices[1].lut_index++;
 		RST_INDEX(1);
 	}
 	else if (htim == &htim8) {
 		PUT_TO_DAC(VOICE_SUM);
 		voices[2].lut_index++;
 		RST_INDEX(2);
 	}

 }

 // When UART message recieved, only valid if starts with 0x80 or 0x90
 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
 {

	// Only allow a note to turn on if there are <= 2 notes being played already
 	if ((midi_tmp[0] == 0x90) && (notes_on <= 2)) {
 		uint8_t i;
 		//
 		for (i = 0; i < 3; i++) {
 			midi_msg[i] = midi_tmp[i];
 		}
 		voices[notes_on].env_val = 0;
 		voices[notes_on].gate = ON;
 		voices[notes_on].status = ON;
 		voices[notes_on].state = ATTACK;
 		voices[notes_on].lut_index = 0;
 		voices[notes_on].note = NOTE;
 		// Increment notes_on after since array of voices starts at index 0, while notes_on can range from 0-3
 		notes_on++;
 	}



 	// Only turn a note off if <=3 notes are on at once
 	else if (midi_tmp[0] == 0x80) {
 		uint8_t i;
 		// transfer midi data over to semi-permenant array
 		for (i = 0; i < 3; i++) {
 			midi_msg[i] = midi_tmp[i];
 		}

 		// Scan for which key was released and turn the gate of that voice off
 		for (i = 0; i < 3; i++) {
 			if (voices[i].note == NOTE) {
 				voices[i].gate = OFF;
 				notes_on--;
 			}
 		}


 	}

 	// Change frequency of timers based on desired note frequencies
 	switch (STATUS_SUM) {
 	case 1:
 		TIM6->ARR = ARR_VAL(voices[0].note);
 		multiplier = 3.0;
 		break;
 	case 2:
 		TIM6->ARR = ARR_VAL(voices[0].note);
 		TIM7->ARR = ARR_VAL(voices[1].note);
 		multiplier = 1.5;
 		break;
 	case 3:
 		TIM6->ARR = ARR_VAL(voices[0].note);
 		TIM7->ARR = ARR_VAL(voices[1].note);
 		TIM8->ARR = ARR_VAL(voices[2].note);
 		multiplier = 1.0;
 		break;
 	default:
 		multiplier = 1.0;
 	}

 	// Update MIDI IN light if more than one key is pressed
 	LED_Handler();

 	// Get another MIDI message
 	HAL_UART_Receive_IT(&huart1, midi_tmp, 3);
 }


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_ADC12
                              |RCC_PERIPHCLK_ADC345;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  PeriphClkInit.Adc345ClockSelection = RCC_ADC345CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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

  ADC_MultiModeTypeDef multimode = {0};
  //ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.GainCompensation = 0;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_BOTH;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

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
  htim2.Init.Prescaler = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 16999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

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
  huart1.Init.BaudRate = 31250;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
