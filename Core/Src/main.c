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
 * Last edited: 12/12/2021
 */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "notes.h"
#include "wavetables.h"

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;

DAC_HandleTypeDef hdac1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;

// Flags for interrupts
volatile uint8_t flag_midi_ready = 0;
volatile uint8_t flag_tim_adcs = 0;
volatile uint8_t flag_tim_voice1 = 0;
volatile uint8_t flag_tim_voice2 = 0;
volatile uint8_t flag_tim_voice3 = 0;

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
static inline void LED_Handler(uint8_t);
static void Update_Env_Mult(struct voice *, uint16_t *);
static void Update_Wave_Shape(const uint16_t **);
static inline void Init_Voices(struct voice *);
static uint16_t Update_ADSR_Param(uint32_t, uint8_t);

int main(void)
{
	// counting variable
	uint8_t i;

	// 3-byte midi message
	unsigned char midi_msg[NUM_MIDI_BYTES];
	unsigned char midi_tmp[NUM_MIDI_BYTES];

	// Voices
	struct voice voices[MAX_NOTES];

	// total number of notes on
	uint8_t gate_sum = 0;

	// A-to-D results for ADSR values
	uint16_t AD_ADSR[4];

	// Waveform multiplier
	float multiplier = 1.0;

	// Look up table used in wavetable synthesis
	const uint16_t * lut = sin_lut;

	// Reset of all peripherals, Initializes the Flash interface and the Systick.
	HAL_Init();

	// Configure the system clock
	SystemClock_Config();

	// Initialize all voices to their reset value
	Init_Voices(voices);

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
 	while (1) {

 		// -----------------------------------------
 		// MIDI Interrupt
 		// -----------------------------------------
 		if (flag_midi_ready == 1) {

 			flag_midi_ready = 0;

 			// Only allow a note to turn on if there are <= 2 notes being played already
 			if (GLOBAL_MIDI_NOTE_ON && (gate_sum < MAX_NOTES)) {
 			//
 				for (i = 0; i < NUM_MIDI_BYTES; i++) {
 					midi_msg[i] = midi_tmp[i];
 				}
 				voices[gate_sum].env_val = 0;
 				voices[gate_sum].status = ON;
 				voices[gate_sum].state = ATTACK;
 				voices[gate_sum].lut_index = 0;
 				voices[gate_sum].note = NOTE;
 				voices[gate_sum].gate = ON;

 			}

 			// Care about every note off because it may contain the note needed to turn off
 			else if (GLOBAL_MIDI_NOTE_OFF) {
 				// transfer midi data over to semi-permenant array
 				for (i = 0; i < NUM_MIDI_BYTES; i++) {
 					midi_msg[i] = midi_tmp[i];
 				}

 				// Scan for which key was released and turn the gate of that voice off
 				for (i = 0; i < MAX_NOTES; i++) {
 					if (voices[i].note == NOTE) {
 						voices[i].gate = OFF;
 					}
 				}
 			}

 			// Update the number of gates that are currently on
 			gate_sum = GATE_SUM;

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
 				 multiplier = 0.0;
 			 }

 			 // Update MIDI IN light if more than one key is pressed
 			 LED_Handler(gate_sum);

 			 // Get another MIDI message
 			 HAL_UART_Receive_IT(&huart1, midi_tmp, NUM_MIDI_BYTES);
 		}


 		// -----------------------------------
 		// Timer Interrupts
 		// -----------------------------------

 		if (flag_tim_adcs == 1) {
 			flag_tim_adcs = 0;
 			Update_Wave_Shape(&lut);
 		    Update_Env_Mult(voices, AD_ADSR);
 		}

 		if (flag_tim_voice1 == 1) {
 			flag_tim_voice1 = 0;
 			PUT_TO_DAC(VOICE_SUM);
 			voices[0].lut_index++;
 			RST_INDEX(0);

 		}

 		if (flag_tim_voice2 == 1) {
 			flag_tim_voice2 = 0;
 			PUT_TO_DAC(VOICE_SUM);
 			voices[1].lut_index++;
 			RST_INDEX(1);
 		}

 		if (flag_tim_voice3 == 1) {
 			flag_tim_voice3 = 0;
 		 	PUT_TO_DAC(VOICE_SUM);
 		 	voices[2].lut_index++;
 		 	RST_INDEX(2);
 		}

 	}

}

static inline void Init_Voices(struct voice * v)
{
	uint8_t i;
	for (i = 0; i < MAX_NOTES; i++) {
		// Initialize each voice gate to OFF
		v[i].gate = OFF;

		// Initialize each voice status to OFF
		v[i].status = OFF;

		// Initialize each envelope value to 0
		v[i].env_val = 0.0;

		// Initialize each voice index to start at beginning of lookup table
		v[i].lut_index = 0;
	}
}

// Turn on LED while a key is pressed
static inline void LED_Handler(uint8_t sum)
{
	if (sum > 0) {
		MIDI_IN_LED_ON;
	}
	else {
		MIDI_IN_LED_OFF;
	}
}

// Select lookup table based on POT ADC value
static void Update_Wave_Shape(const uint16_t ** lut)
{
	uint16_t shape_adc_val;
	HAL_ADC_Start(&hadc3);
	//HAL_ADC_PollForConversion(&hadc3, 1);
	shape_adc_val = HAL_ADC_GetValue(&hadc3);
	HAL_ADC_Stop(&hadc3);

	// Change wave shape based on shape POT
	if (shape_adc_val >= 0 && shape_adc_val < 1024) {
	    *lut = sin_lut;
	}
	else if (shape_adc_val >= 1024 && shape_adc_val < 2048) {
	 	*lut = tri_lut;
	}
	else if (shape_adc_val >= 2048 && shape_adc_val < 3072) {
	 	*lut = saw_lut;
	}
	else if (shape_adc_val >= 3072 && shape_adc_val < 4096) {
		*lut = sqr_lut;
	}
}

// Configure an ADC1 channel and store a new ADSR parameter
static uint16_t Update_ADSR_Param(uint32_t channel, uint8_t param)
 {
	 uint16_t result;
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
	 result = HAL_ADC_GetValue(&hadc1);
	 HAL_ADC_Stop(&hadc1);
	 return result;
 }

// Get new envelope multiplier
// ADSR values are incremented by 1 to avoid division by 0
static void Update_Env_Mult(struct voice * v, uint16_t * AD_ADSR)
{
	uint8_t i = 0;

	// Get ADC value from each channel
	AD_ADSR[0] = Update_ADSR_Param(ADC_CHANNEL_1, ATTACK);
	AD_ADSR[1] = Update_ADSR_Param(ADC_CHANNEL_2, DECAY);
	AD_ADSR[2] = Update_ADSR_Param(ADC_CHANNEL_3, SUSTAIN);
	AD_ADSR[3] = Update_ADSR_Param(ADC_CHANNEL_4, RELEASE);

	for (i = 0; i < MAX_NOTES; i++) {
		switch(v[i].state) {
		// Attack - Increase envelope value until 1.0 is reached
		case ATTACK:
			v[i].rate = 1.0 / (ATTACK_VAL + 1);
			// Check if index had reached end of attack phase
			if (v[i].env_val >= 1.0) {
				v[i].state = DECAY;
				v[i].env_val = 1.0;
			}
			else {
				v[i].env_val += v[i].rate;
			}
			break;
		// Decay - Decrease envelope value until sustain value is reached
		case DECAY:
			v[i].rate = (DECAY_NORM - 1.0) / (DECAY_VAL + 1);
			if (v[i].env_val <= SUSTAIN_NORM) {
				v[i].state = SUSTAIN;
				// Return sustain level as it is last value of decay phase
				v[i].env_val = SUSTAIN_NORM;
			}
			else {
				v[i].env_val += v[i].rate;
			}
			break;
		// Sustain - Hold sustain value until key is released
		case SUSTAIN:
			if (v[i].gate == OFF) {
				v[i].state = RELEASE;
			}
			// Remain here until gate has been turned off by key release
			else {
				v[i].env_val = SUSTAIN_NORM;
			}
			break;

		// Release - Decrease envelope value until value has reached 0 - turn off note
		case RELEASE:
			v[i].rate = SUSTAIN_NORM / (RELEASE_VAL + 1);
			if (v[i].env_val <= 0.0) {
				v[i].status = OFF;
				v[i].env_val = 0.0;
			}
			else {
				v[i].env_val -= v[i].rate;
			}
			break;
		}
	}
}

 // When timer overflows, set flag to tell main to
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
 {

	if (htim == &htim2) {
		flag_tim_adcs = 1;
	}
	else if (htim == &htim6) {
		flag_tim_voice1 = 1;
	}
	else if (htim == &htim7) {
		flag_tim_voice2 = 1;
	}
	else if (htim == &htim8) {
		flag_tim_voice3 = 1;
	}
 }

 // When UART message recieved, only valid if starts with 0x80 or 0x90
 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
 {
	 flag_midi_ready = 1;
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
