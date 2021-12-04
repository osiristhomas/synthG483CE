#ifndef _MACROS_H_
#define _MACROS_H_

#define F_CPU 170000000UL
#define NOTE pitches[midi_msg[1] + 1]
#define MIDI_IN_LED_ON HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET)
#define MIDI_IN_LED_OFF HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET)
#define DAC1_DATA DAC1->DHR12R1
#define NUM_PTS 128
#define ON 1
#define OFF 0
#define GLOBAL_MIDI_NOTE_ON midi_msg[0] == 0x90
#define GLOBAL_MIDI_NOTE_OFF midi_msg[0] == 0x80
#define ARR_VAL(f) (((F_CPU)/(NUM_PTS*f))-1)
#define PUT_TO_DAC(v) DAC1->DHR12R1 = v
//#define VOICE0 (uint16_t)(multiplier * (voices[0].status*lut[voices[0].lut_index++]*voices[0].env_val + voices[1].status*lut[voices[1].lut_index]*voices[1].env_val + voices[2].status*lut[voices[2].lut_index]*voices[2].env_val))
//#define VOICE1 (uint16_t)(multiplier * (voices[0].status*lut[voices[0].lut_index]*voices[0].env_val + voices[1].status*lut[voices[1].lut_index++]*voices[1].env_val + voices[2].status*lut[voices[2].lut_index]*voices[2].env_val))
//#define VOICE2 (uint16_t)(multiplier * (voices[0].status*lut[voices[0].lut_index]*voices[0].env_val + voices[1].status*lut[voices[1].lut_index]*voices[1].env_val + voices[2].status*lut[voices[2].lut_index++]*voices[2].env_val))
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
#define RST_INDEX(i) if (voices[i].lut_index == NUM_PTS) voices[i].lut_index = 0

#endif
