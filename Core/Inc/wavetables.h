/* Osiris Thomas
 *
 * wavetables.h
 * Says that there will be global arrays for four lookup tables
 */

#ifndef _WAVETABLES_H_
#define _WAVETABLES_H_

#include <stdint.h>
#include "main.h"

extern const uint16_t sin_lut[NUM_PTS];

extern const uint16_t tri_lut[NUM_PTS];

extern const uint16_t sqr_lut[NUM_PTS];

extern const uint16_t saw_lut[NUM_PTS];

#endif
