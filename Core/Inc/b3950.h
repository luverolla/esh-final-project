#ifndef B3950_H
#define B3950_H

#include <math.h>

#include "adc.h"

#define B3950_VCC  3.3
#define B3950_TREF 298.15
#define B3950_RSER 10000.0
#define B3950_RREF 100000.0
#define B3950_BVAL 3950.0
#define B3950_ADC_LVLS 4096

typedef struct B3950_Handle
{
	uint32_t* buffer;
	float volts;
	float resist;
} B3950_Handle;

B3950_Handle* B3950_Init(uint32_t* buffer);
void B3950_Deinit(B3950_Handle* hd);
float B3950_Read(B3950_Handle* hd);

#endif
