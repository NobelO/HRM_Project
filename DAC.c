#include "DAC.h"
#include <math.h>


//GLOBAL VARIABLES
uint32_t triangleWave[WAVE_SAMPLES];		//TRIANGLE WAVE LOOKUP TABLE.
uint32_t triangle_index = 0;						//CURRENT INDEX IN THE TRIANGLE WAVE TABLE. A VALUE THAT IS CONTINUOUSLY INCREMENTED
uint32_t sine_wave[SINE_SAMPLES]; 			//SINE WAVE LOOKUP TABLE. A LIST THAT CAN HOLD 100 VALUES
uint32_t sine_index = 0;          			//CURRENT INDEX IN THE SINE WAVE TABLE. A VALUE THAT IS CONTINUOUSLY INCREMENTED
uint16_t squareWave[] = {0, 3732}; 			//ARRAY OF VALUE TO SWITCH HIGH OR LOW FOR SQUARE WAVE
uint32_t square_index = 0;							//CURRENT INDEX IN THE SQUARE WAVE


void INIT_DAC(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;						//ONLY ENABLE GPIO A
	
	GPIOA->MODER 	|= (3UL<<(2*DAC1_Pin)); 					//ONLY SET GPIOA (PA4) TO CONFIGURE AS ANALOG
	GPIOA->MODER 	|= (3UL<<(2*DAC2_Pin)); 					//ONLY SET GPIOA (PA5) TO CONFIGURE AS ANALOG

	RCC->APB1ENR |= RCC_APB1ENR_DACEN; 							//ONLY ENABLE DAC CLOCK
	
  DAC->CR |= DAC_CR_EN1;													//ONLY ENABLE DAC 1
	DAC->CR |= (DAC_CR_EN2);												//ONLY ENABLE DAC 2
	
	//DAC->SWTRIGR |= (1<<1);   //SOFTWARE TRIGGER ENABLED
	
	//BOFFx bit in the DAC_CR register.
	//. DAC noise generation is selected by setting WAVEx[1:0] to “01”
	//It is possible to reset LFSR wave generation by resetting the WAVEx[1:0] bits
	//The DAC trigger must be enabled for noise generation by setting the TENx bit in the DAC_CR register.
	//DAC triangle-wave generation is selected by setting WAVEx[1:0] to “10”
	//The amplitude is configured through the MAMPx[3:0] bits in the DAC_CR register
	//It is possible to reset triangle wave generation by resetting the WAVEx[1:0] bits PP443
	//The MAMPx[3:0] bits must be configured before enabling the DAC, otherwise they cannot be changed.
}


void DAC1_OUTPUT(unsigned short val)
{
	DAC->DHR12R1 = val;									//WRITE "val" VALUE IN THE DAC 1 REGISTER
}


void DAC2_OUTPUT(unsigned short val)
{
	DAC->DHR12R2 = val;									//WRITE "val" VALUE IN THE DAC 2 REGISTER
}

void generateSineWave(void)						//GENERATE SINE WAVE LOOKUP TABLE
{
	for (int i = 0; i < SINE_SAMPLES; i++)
	{
		sine_wave[i] = SINE_OFFSET + (uint16_t)(SINE_AMPLITUDE * sin((2 * PI * i) / SINE_SAMPLES));
	}
}

void generateTriangleWave(void)						//GENERATE TRIANGLE WAVE LOOKUP TABLE
{
	for (int i = 0; i < WAVE_SAMPLES / 2; i++)
	{
		triangleWave[i] = (4095 * i) / (WAVE_SAMPLES / 2);  //RISING EDGE OF TRIANGLE
		//GPIOC->ODR ^= 1<<3; 									//TOGGLE YELLOW LED (PC3)
	}
	
	for (int i = WAVE_SAMPLES / 2; i < WAVE_SAMPLES; i++)
	{
		triangleWave[i] = 4095 - ((4095 * (i - WAVE_SAMPLES / 2)) / (WAVE_SAMPLES / 2)); //FALLING EDGE OF TRIANGLE
	}
}

void generateComplexWave(void)						//GENERATE COMPLEX WAVE (SINE WAVE AND TRIANGLE WAVE) LOOKUP TABLE
{
  //A COMBINATION OF THE SINE AND TRIANGULAR WAVES
	for (int i = 0; i < SINE_SAMPLES; i++)
	{
		sine_wave[i] = SINE_OFFSET + (uint16_t)(SINE_AMPLITUDE * sin(2 * PI * i / SINE_SAMPLES));
  }
	
	for (int i = 0; i < WAVE_SAMPLES / 2; i++)
	{
		triangleWave[i] = (4095 * i) / (WAVE_SAMPLES / 2);  //RISING EDGE OF TRIANGLE
	}
	
	for (int i = WAVE_SAMPLES / 2; i < WAVE_SAMPLES; i++)
	{
		triangleWave[i] = 4095 - ((4095 * (i - WAVE_SAMPLES / 2)) / (WAVE_SAMPLES / 2)); //FALLING EDGE OF TRIANGLE
	}
}

void INIT_TIM4(void)									//SQUARE WAVE TIMER
{
	RCC->APB1ENR|=RCC_APB1ENR_TIM4EN;		//timer 4 clock enabled
	TIM4->DIER|=TIM_DIER_UIE;						//timer uptdate interrupt enabled
																			//APB clock is Fcy/2 = 180MHz/2 = 90MHz
	TIM4->PSC=9000-1;										//divide APB clock by 90000 = 90MHz/9000 = 10kHz
	TIM4->ARR=5000;											//Auto-reload: 10kHz / 5k = 2Hz = 0.5s DAC update rate
	TIM4->CNT=0;												//zero timer counter
	NVIC_EnableIRQ(TIM4_IRQn);					//timer 4 global interrupt enabled
	TIM4->CR1|=TIM_CR1_CEN;							//start timer counter
}

void TIM4_IRQHandler(void)
{
	if ((TIM4->SR & TIM_SR_UIF) != 0) 	  									//Check update interrupt flag
	{
		TIM4->SR &= ~TIM_SR_UIF;  														//Clear the flag
		DAC->DHR12R2 = squareWave[square_index]; 							//Write next value to DAC2 (PA5)
		square_index = (square_index + 1) % S_WAVE_SAMPLES;		//Increment index and wrap around
	}
}

void INIT_TIM5(void)									//COMPLEX WAVE TIMER
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; //Enable TIM5 clock
																			//APB clock is Fcy/2 = 180MHz/2 = 90MHz
	TIM5->PSC = 9000 - 1;   						//Prescaler: 90 MHz / 9000 = 10kHz
	TIM5->ARR = 100 - 1;  							//Auto-reload: 10kHz / 100 = 100Hz DAC update rate
	TIM5->DIER|=TIM_DIER_UIE;						//timer uptdate interrupt enabled
	TIM5->CNT=0;												//zero timer counter
	NVIC_EnableIRQ(TIM5_IRQn); 					//ENABLE TIM5 IRQ
	TIM5->CR1|=TIM_CR1_CEN;							//start timer counter
}

void TIM5_IRQHandler(void)
{
	if ((TIM5->SR & TIM_SR_UIF) != 0)   	//Check update interrupt flag
	{
		TIM5->SR &= ~TIM_SR_UIF;  					//Clear the flag
		DAC->DHR12R2 = (sine_wave[sine_index] + triangleWave[triangle_index]); //Write next value to DAC2 (PA5)
			
		//Increment index and wrap around
		sine_index = (sine_index + 1) % SINE_SAMPLES;
		triangle_index = (triangle_index + 1) % WAVE_SAMPLES;
	}
}

void INIT_TIM6(void)												//TRIANGLE WAVE TIMER
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;  			//Enable TIM6 clock
																						//APB clock is Fcy/2 = 180MHz/2 = 90MHz
	TIM6->PSC = 1000 - 1;   									//Prescaler: 90 MHz / 1000 = 90kHz
	TIM6->ARR = 1000 - 1;  										//Auto-reload: 90kHz / 1000 = 90Hz DAC update rate
	//TIM6->CR2 |= (3 << 4); // Set TRGO (Trigger Output) to Update Event   CHECK P440
	//TIM6->CR2 |= TIM_CR2_MMS_1; // Set TRGO (Trigger Output) to Update Event   CHECK P440
	//TIM_CR2_MMS_1
	
	TIM6->DIER|=TIM_DIER_UIE;									//timer uptdate interrupt enabled
	TIM6->CNT=0;															//zero timer counter
	NVIC_EnableIRQ(TIM6_DAC_IRQn); 						//ENABLE TIM6 IRQ
	TIM6->CR1|=TIM_CR1_CEN;										//start timer counter
}

/*
void TIM6_DAC_IRQHandler(void)
{
	 if ((TIM6->SR & TIM_SR_UIF) != 0) {  // Check update interrupt flag
        TIM6->SR &= ~TIM_SR_UIF;  // Clear the flag

        // Write next value to DAC2 (PA5)
        DAC->DHR12R2 = triangleWave[triangle_index]; 
        
        // Increment index and wrap around
        triangle_index = (triangle_index + 1) % WAVE_SAMPLES;
    }
}
*/

void INIT_TIM7(void)											//SINE WAVE TIMER
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;  		//Enable TIM7 clock
																					//APB clock is Fcy/2 = 180MHz/2 = 90MHz
	TIM7->PSC = 9000 - 1;   								//Prescaler: 90 MHz / 9000 = 10kHz
	TIM7->ARR = 100 - 1;  									//Auto-reload: 10kHz / 100 = 100Hz DAC update rate
	TIM7->DIER|=TIM_DIER_UIE;								//timer uptdate interrupt enabled
	TIM7->CNT=0;														//zero timer counter
	NVIC_EnableIRQ(TIM7_IRQn); 							//ENABLE TIM7 IRQ
	TIM7->CR1|=TIM_CR1_CEN;									//start timer counter
}

void TIM7_IRQHandler(void)
{
	if ((TIM7->SR & TIM_SR_UIF) != 0)   							//Check update interrupt flag
	{
		TIM7->SR &= ~TIM_SR_UIF;  											//Clear the flag
		DAC->DHR12R2 = sine_wave[sine_index]; 					//Write next value to DAC2 (PA5)
		sine_index = (sine_index + 1) % SINE_SAMPLES;		//Increment index and wrap around
	}
}




/*
Demonstrate DAC functionality by performing the following:
•	The DACs should generate a range of test signals, between 0-3v, to demonstrate functionality 
•	DC waveforms selectable on DAC1
1.	Variable DC voltage level inputted from UART
2.	2 Hz Square Wave, 
•	AC waveforms selectable on DAC2
1.	3 Hz Triangular Wave
2.	1 Hz Sine Wave 
3.	Complex Wave / Combination of the above to simulate multiple peaks
•	Samples should be outputted at regular intervals using a hardware TIMER.
*/