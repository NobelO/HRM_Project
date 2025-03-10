#ifndef _ADC_H
#define _ADC_H
#include <stm32f4xx.h>

#define ADC_Input_Port_PC3   GPIOC //Define GPIOC
#define ADC_Input_Pin_PC3    3 //Define pin C3 as the pin being configured in the GPIOC Port
#define ADC_Channel_13		  13 //Define Channel l3 (PULSE_OX)



extern volatile unsigned short adc_result_current; // Trigger an ADC conversion, read and then store result for PULSE_OX (13)
extern char adc_string_PULSE[8]; //Buffer for ADC result PULSE_OX stored as ASCII string
extern volatile unsigned short adc_average[5]; // Circular buffer for last 5 ADC results
extern volatile unsigned int adc_sum;  // Running sum for rolling average
extern volatile unsigned char buffer_index; // Tracks current position in buffer




void init_adc (void);
void adc_trigger (void);
void convert_adc_result(void);
void store_adc_sample(void); // Stores ADC sample & updates rolling average
unsigned short get_rolling_average(void); // Retrieves the rolling average


#endif
