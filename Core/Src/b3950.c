#include <stdlib.h>

#include "b3950.h"

float __B3950_RAW2VLT(uint32_t raw)
{
	return (float)raw * B3950_VCC / B3950_ADC_LVLS;
}

float __B3950_VLT2RES(float v)
{
	return (v * B3950_RSER) / (B3950_VCC - v);
}

float __B3950_RES2TMP(float res)
{
	return 1 / ((1/B3950_TREF) + ((1/B3950_BVAL) * log(res/B3950_RREF))) - 273.15;
}

B3950_Handle* B3950_Init(uint32_t* buffer)
{
	B3950_Handle* hd = (B3950_Handle*)malloc(sizeof(B3950_Handle));
	hd->buffer = buffer;

	return hd;
}

void B3950_Deinit(B3950_Handle* hd)
{
	free(hd);
}

float B3950_Read(B3950_Handle* hd)
{
	uint32_t value = *hd->buffer;
	hd->volts = __B3950_RAW2VLT(value);
	hd->resist = __B3950_VLT2RES(hd->volts);
	return __B3950_RES2TMP(hd->resist);
}
