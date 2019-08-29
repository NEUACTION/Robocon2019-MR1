#ifndef __ADC_H
#define __ADC_H


#ifndef LASER_NUM
#define LASER_NUM (4)
#endif

#ifndef LASER_BUFF_SIZE
#define LASER_BUFF_SIZE (30)
#endif

#ifndef GAS_BUFF_SIZE
#define GAS_BUFF_SIZE (30)
#endif

#ifndef GAS_NUM
#define GAS_NUM (1)
#endif


void ADC1mixed_DMA_Config(void);

float GetLaserC0Value(void);

void GasPressureADCInit(void);

float GetGasPressureADCValue(void);
#endif
