#ifndef ADC_DRIVER_H
#define ADC_DRIVER_H

#define ADC_SAMPLES 		128

void ADCCreate(void);
uint16_t ADCGetSample(void);

#endif
