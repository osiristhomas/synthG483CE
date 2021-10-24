#define F_CPU 170000000UL
#define NOTE pitches[midi_msg[1] + 1]
#define MIDI_IN_LED_ON HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET)
#define MIDI_IN_LED_OFF HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET)
#define DAC1_DATA DAC1->DHR12R1
#define DAC2_DATA DAC2->DHR12R1
#define NUM_PTS 128
#define MAX_NOTES 4
#define ON 1
#define OFF 0
#define gate_BYTE (midi_msg[0] & 0x90)
#define MIDI_gate_ON 0x90
#define MIDI_gate_OFF 0x80
#define GLOBAL_MIDI_NOTE_ON midi_msg[0] == 0x90
#define GLOBAL_MIDI_NOTE_OFF midi_msg[0] == 0x80
#define ARR_VAL(f) (((F_CPU)/(NUM_PTS*f))-1)
#define PUT_TO_DAC(v) HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, v)
#define VOICE0 (uint16_t)(multiplier * (voices[0].gate*lut[voices[0].index++] + voices[1].gate*lut[voices[1].index]   + voices[2].gate*lut[voices[2].index]))
#define VOICE1 (uint16_t)(multiplier * (voices[0].gate*lut[voices[0].index]   + voices[1].gate*lut[voices[1].index++] + voices[2].gate*lut[voices[2].index]))
#define VOICE2 (uint16_t)(multiplier * (voices[0].gate*lut[voices[0].index]   + voices[1].gate*lut[voices[1].index]   + voices[2].gate*lut[voices[2].index++]))
#define RESET_INDEX(i) if (voices[i].index == NUM_PTS) voices[i].index = 0
#define BUFFER_SIZE 1024
