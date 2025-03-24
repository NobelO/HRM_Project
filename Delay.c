#include "delay.h"
#include "ADC.h"
#include "Usart.h"
#include <stdbool.h>

volatile unsigned long maxVal = 0;
volatile unsigned long upperThreshold;
volatile unsigned long lowerThreshold;
bool state1 = true;
bool state2 = true;
volatile uint32_t state = 1;			
volatile unsigned long count = 0;
unsigned long voltage_mv1;
volatile uint32_t start_time;
volatile uint32_t duration;
volatile unsigned short BPM;

void ADC_IRQHandler(void)
{
	if (ADC1->SR & ADC_SR_EOC) {  // Check if the End of Conversion flag is set
		adc_result_current = ADC1->DR & 0x0FFF;  // Read and store the ADC result
		
		voltage_mv1 = ((unsigned long)adc_result_current * 3300) / 4095;
		count += 1;
		//count = count % 15000;
		
		while(count <= 4999)
		{
			if(voltage_mv1 > maxVal)
			{
				maxVal = voltage_mv1;
				upperThreshold = 0.8*maxVal;
				lowerThreshold = 0.2*maxVal;
			}
		}
		
		while(count > 4999)
		{
			//Create a variable called state and set as Low
			
			
			if((state == 1) && (voltage_mv1 >= upperThreshold))
			{
				state = 0;
				//Start timer
				start_time = TIM6->CNT;  // Store start time
			}
			
			if((state == 0) && (voltage_mv1 <= lowerThreshold))
			{
				state = 1;
				//Collect timer value
				duration = TIM6->CNT - start_time;
				
				BPM = 60*(1/duration);
			}
			
			if((state == 1) && (voltage_mv1 >= upperThreshold))
			{
				//state = false;
				//Collect timer value
				duration = TIM6->CNT - start_time;
				
				BPM = ((char)60*(1/duration));
			}
		}

		// Convert ADC result to string
		convert_adc_result();

		// Send the converted voltage string over USART
		for (int i = 0; adc_string_PULSE[i] != '\0'; i++) {
			send_usart(adc_string_PULSE[i]);
		}
		send_usart('\r');
		send_usart('\n');

		ADC1->SR &= ~ADC_SR_EOC;  // Clear the EOC flag
		GPIOC->ODR ^= 1<<3; 									//TOGGLE YELLOW LED (PC3)
	}
}

void init_Timer6(void)
{
	// Enable Timer 6 Clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

	// Set Prescaler & Auto-Reload for 1000Hz (assuming 90MHz APB1 clock)
	TIM6->PSC = 90 - 1;  // Prescaler: 90MHz / 90 = 1MHz timer clock
	TIM6->ARR = 1000 - 1; // Auto-reload: 1000 counts of 1MHz = 1ms period (1000Hz)

	// Enable Timer 6 Update Interrupt
	TIM6->DIER |= TIM_DIER_UIE;

	// Enable Timer 6 in NVIC (Interrupt Controller)
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
	NVIC_SetPriority(TIM6_DAC_IRQn, 2); // Set priority if needed

	// Start Timer 6
	TIM6->CR1 |= TIM_CR1_CEN;
}

void TIM6_DAC_IRQHandler(void)
{
	if (TIM6->SR & TIM_SR_UIF)  // Check if update interrupt flag is set
	{
		adc_trigger();

		TIM6->SR &= ~TIM_SR_UIF;  // Clear the interrupt flag

		// ADC trigger will be added here later
	}
}



//TIMER 2 MUST BE USED FOR DAC1 (PA5)

//TIMER 5 MUST BE USED FOR DAC2 (PA4)


