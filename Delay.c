#include "delay.h"
#include "ADC.h"
#include "Usart.h"


void ADC_IRQHandler(void)
{
	if (ADC1->SR & ADC_SR_EOC) {  // Check if the End of Conversion flag is set
		adc_result_current = ADC1->DR & 0x0FFF;  // Read and store the ADC result

		// Convert ADC result to string
		convert_adc_result();

		// Send the converted voltage string over USART
		for (int i = 0; adc_string_PULSE[i] != '\0'; i++) {
			send_usart(adc_string_PULSE[i]);
		}
		send_usart('\r');
		send_usart('\n');

		ADC1->SR &= ~ADC_SR_EOC;  // Clear the EOC flag
	}
}

void init_Timer6(void)
{
	// Enable Timer 6 Clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

	// Set Prescaler & Auto-Reload for 200Hz (assuming 45MHz APB1 clock)
	TIM6->PSC = 45 - 1;  // Prescaler: 45MHz / 45 = 1MHz timer clock
	TIM6->ARR = 5000 - 1; // Auto-reload: 5000 counts of 1MHz = 5ms period (200Hz)

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


