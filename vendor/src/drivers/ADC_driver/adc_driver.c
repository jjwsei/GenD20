#include <asf.h>
#include "adc_driver.h"

void configure_adc_callbacks(void);
void adc_complete_callback(struct adc_module *const module);

struct adc_module adc_instance;
volatile bool adc_read_done = false;
uint16_t adc_result_buffer[ADC_SAMPLES] = {0};

void ADCCreate(void)
{
	struct adc_config config_adc;

	adc_get_config_defaults(&config_adc);

	config_adc.gain_factor     = ADC_GAIN_FACTOR_DIV2;
	config_adc.clock_prescaler = ADC_CLOCK_PRESCALER_DIV8;
	config_adc.reference       = ADC_REFERENCE_INTVCC1;
	config_adc.positive_input  = ADC_POSITIVE_INPUT_PIN6;
        config_adc.resolution      = ADC_RESOLUTION_12BIT;

	adc_init(&adc_instance, ADC, &config_adc);

        configure_adc_callbacks();
        
        adc_enable(&adc_instance);
}

uint16_t ADCGetSample(void)
{
        uint32_t result = 0;
        uint8_t i;

        adc_read_buffer_job(&adc_instance, adc_result_buffer, ADC_SAMPLES);
	while (adc_read_done == false) {
        }
	for (i = 0; i < ADC_SAMPLES; i++)
        {
            result += adc_result_buffer[i];
        }
        return((uint16_t)(result / ADC_SAMPLES));
}

void configure_adc_callbacks(void)
{
	adc_register_callback(&adc_instance,
	            adc_complete_callback, ADC_CALLBACK_READ_BUFFER);
	adc_enable_callback(&adc_instance, ADC_CALLBACK_READ_BUFFER);
}

void adc_complete_callback(struct adc_module *const module)
{
	adc_read_done = true;
}

