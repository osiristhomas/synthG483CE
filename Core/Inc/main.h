/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

#define F_CPU 170000000UL
#define NOTE pitches[midi_msg[1]]
#define MIDI_IN_LED_ON HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET)
#define MIDI_IN_LED_OFF HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET)
#define NUM_PTS 128
#define ON 1
#define OFF 0
#define GLOBAL_MIDI_NOTE_ON ((midi_tmp[0] & 0x90) == 0x90)
#define GLOBAL_MIDI_NOTE_OFF ((midi_tmp[0] & 0x80) == 0x80)
#define ARR_VAL(f) (((F_CPU)/(NUM_PTS*f))-1)
#define PUT_TO_DAC(v) DAC1->DHR12R1 = v
#define VOICE0 (voices[0].status*lut[voices[0].lut_index]*voices[0].env_val)
#define VOICE1 (voices[1].status*lut[voices[1].lut_index]*voices[1].env_val)
#define VOICE2 (voices[2].status*lut[voices[2].lut_index]*voices[2].env_val)
#define VOICE_SUM ((uint16_t)(multiplier * (VOICE0 + VOICE1 + VOICE2)))
#define INV_4096 0.00024414
#define ATTACK_VAL (AD_ADSR[0] >> 3)
#define DECAY_VAL (AD_ADSR[1] >> 3)
#define SUSTAIN_VAL (AD_ADSR[2])
#define RELEASE_VAL (AD_ADSR[3] >> 3)
#define ATTACK_NORM (ATTACK_VAL * INV_4096)
#define DECAY_NORM (DECAY_VAL * INV_4096)
#define SUSTAIN_NORM (SUSTAIN_VAL * INV_4096)
#define RELEASE_NORM (RELEASE_VAL * INV_4096)
#define RST_INDEX(i) if (voices[i].lut_index == NUM_PTS) voices[i].lut_index=0
#define STATUS_SUM (voices[0].status + voices[1].status + voices[2].status)
#define GATE_SUM (voices[0].gate + voices[1].gate + voices[2].gate)
#define MAX_NOTES 3
#define NUM_MIDI_BYTES 3

enum {
	ATTACK = 0,
	DECAY,
	SUSTAIN,
	RELEASE
};

struct voice {
	// 1 when a key is pressed, 0 otherwise
	uint8_t gate;

	// 1 when the note is being played, 0 otherwise
	// May or may not be equal to gate
	uint8_t status;

	// Current note being played, in Hz
	uint16_t note;

	// Index into the current lookup table
	uint8_t lut_index;

	// Slope of ADSR envelope
	float rate;

	// State of voice in the ADSR envelope
	uint8_t state;

	/* Multiplier for the waveform based on how far
	 * along the note is into the envelope
	 */
	float env_val;
};

void Error_Handler(void);



#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
