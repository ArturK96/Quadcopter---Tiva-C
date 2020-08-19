#include <stdbool.h>
#include <stdint.h>


extern float adcVal, battery_voltage;


void ADC_Init(void);
uint32_t ADC_Value_Get(void);
