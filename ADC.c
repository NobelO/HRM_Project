#include "ADC.h"
#include "Usart.h"

volatile unsigned short adc_result_current = 0;
char adc_string_PULSE[8] = "0000V";
volatile unsigned short adc_average[5] = {0}; // Circular buffer for last 5 ADC results
volatile unsigned int adc_sum = 0;  // Running sum for rolling average
volatile unsigned char buffer_index = 0; // Tracks current position in buffer

void init_adc (void)
{ 
	  //Port and Pin Config
		RCC->AHB1ENR|=(RCC_AHB1ENR_GPIOCEN);	//GPIOC clock enable
	  GPIOC->MODER|=(3u<< (2*ADC_Input_Pin_PC3)); //PinC3 set to analogue mode
	  RCC->APB2ENR|=RCC_APB2ENR_ADC1EN;		//ADC clock enable
	  //Port and Pin Config Complete
	
	  //ADC Conversion Sequence Config
	  ADC1->SQR1 &= ~ADC_SQR1_L;  // Clear sequence length
    ADC1->SQR1 |= (0 << ADC_SQR1_L_Pos); // **Set sequence length to 1 conversion (L = 0 means 1 conversion)**
	  ADC1->CR2 &= ~ADC_CR2_CONT; // **Ensure it's in single conversion mode**
	
	  ADC1->SQR3 &= ~ADC_SQR3_SQ1;  // Clear previous selection
    ADC1->SQR3 |= (ADC_Channel_13 << 0);  // Target Channel13 for conversion
	  //ADC Conversion Sequencce Config Complete 
	
	  //ADC Interrupt Config 
	  ADC1->CR1 |= ADC_CR1_EOCIE;  // **Enable End-of-Conversion interrupt**
    NVIC_EnableIRQ(ADC_IRQn);  // **Enable ADC IRQ in NVIC**
	  //ADC Interrupt Config Complete
	
	  ADC1->CR1 &= ~(0x3u << 24); // 12-bit resolution
		
    ADC1->CR2 |= ADC_CR2_ADON;  // Enable ADC
	}

void adc_trigger (void) 
{
	ADC1->CR2 |= ADC_CR2_SWSTART; //Start ADC Conversion
	
//	send_usart('R'); send_usart ('E'); send_usart ('A'); send_usart ('D'); send_usart('\r'); send_usart('\n');
	
}

void convert_adc_result(void)
{
    // Use the global adc_result_current directly
    unsigned long voltage_mv = ((unsigned long)adc_result_current * 3300) / 4095;
    
    unsigned short integer_part = (unsigned short)(voltage_mv / 1000);
    unsigned short decimal_part = voltage_mv % 1000;
    
    adc_string_PULSE[0] = (char)(integer_part + '0');            // Single digit integer part
    adc_string_PULSE[1] = '.';
    adc_string_PULSE[2] = (char)((decimal_part / 100) + '0');
    adc_string_PULSE[3] = (char)(((decimal_part / 10) % 10) + '0');
    adc_string_PULSE[4] = (char)((decimal_part % 10) + '0');
    adc_string_PULSE[5] = 'V';
    adc_string_PULSE[6] = '\0';
}

void store_adc_sample(void)
{
    // Subtract the oldest value from adc_sum
    adc_sum -= adc_average[buffer_index];

    // Store the latest ADC result from adc_result_current
    adc_average[buffer_index] = adc_result_current;

    // Add the new sample to adc_sum
    adc_sum += adc_result_current;

    // Move buffer_index forward (circular buffer)
    buffer_index = (buffer_index + 1) % 5;
}


unsigned short get_rolling_average(void) 
{
    return adc_sum / 5; // Compute the rolling average using the stored sum
}

