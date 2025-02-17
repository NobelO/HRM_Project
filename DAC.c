#include "DAC.h"
#include <math.h>


// Global variables
uint32_t triangleWave[WAVE_SAMPLES];
uint32_t triangle_index = 0;
uint32_t sine_wave[SINE_SAMPLES]; // Sine wave lookup table. A LIST THAT CAN HOLD 100 VALUES
uint32_t sine_index = 0;          // Current index in the sine wave table. A VALUE THAT IS CONTINUOUSLY INCREMENTED
uint16_t squareWave[] = {0, 3732};
uint32_t square_index = 0;


void INIT_DAC(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;						//ONLY ENABLE GPIO A CLOCKS
	
	GPIOA->MODER 	|= (3UL<<(2*DAC1_Pin)); 					//ONLY CLEAR GPIOC (PC13) TO CONFIGURE AS ANALOG
	GPIOA->MODER 	|= (3UL<<(2*DAC2_Pin)); 					//ONLY CLEAR GPIOC (PC13) TO CONFIGURE AS ANALOG

	RCC->APB1ENR |= RCC_APB1ENR_DACEN; 							//ONLY ENABLE DAC CLOCK
	
  DAC->CR |= DAC_CR_EN1;													//ONLY ENABLE DAC 1
	DAC->CR |= (DAC_CR_EN2);// | DAC_CR_DMAEN2);													//ONLY ENABLE DAC 2
	
	//DAC->CR |= DAC_CR_TEN2;													//ONLY ENABLE DAC 2 TRIGGER
	//DAC->CR |= (7 << 17);   // Select TIM6 as the trigger source (TSEL2 = 011)
	//DAC->CR |= (2UL << 22);
	//DAC->CR |= (7UL << 19);
	//DAC->CR |= (1UL << 18);
	//DAC->CR |= (1UL << 17);
	
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
	DAC->DHR12R1 = val;																//WRITE "val" VALUE IN THE REGISTER
}


void DAC2_OUTPUT(unsigned short val)
{
	DAC->DHR12R2 = val;																//WRITE "val" VALUE IN THE REGISTER
}

// Function to generate sine wave lookup table
void generate_sine_wave(void)
{
    for (int i = 0; i < SINE_SAMPLES; i++) {
        sine_wave[i] = SINE_OFFSET + (uint16_t)(SINE_AMPLITUDE * sin((2 * PI * i) / SINE_SAMPLES));
    }
}

void generate_complex_wave(void)
{
    //A COMBINATION OF THE SINE AND TRIANGULAR WAVES
	for (int i = 0; i < SINE_SAMPLES; i++) {
        sine_wave[i] = SINE_OFFSET + (uint16_t)(SINE_AMPLITUDE * sin(2 * PI * i / SINE_SAMPLES));
    }
}

void INIT_TIM4(void)
{
	RCC->APB1ENR|=RCC_APB1ENR_TIM4EN;		//timer 4 clock enabled
	TIM4->DIER|=TIM_DIER_UIE;						//timer uptdate interrupt enabled
																			//APB clock is Fcy/2 = 180MHz/2 = 90MHz
	TIM4->PSC=9000-1;										//divide APB clock by 90000 = 90MHz/9000 = 10kHz
	TIM4->ARR=5000;										//counter reload value, gives a timer period of 0.5s when F_APB = 90MHz and PSC = 9000
	TIM4->CNT=0;												//zero timer counter
	NVIC_EnableIRQ(TIM4_IRQn);						//timer 4 global interrupt enabled
	TIM4->CR1|=TIM_CR1_CEN;							//start timer counter
}

void TIM4_IRQHandler(void)
{
	 if ((TIM4->SR & TIM_SR_UIF) != 0) {  // Check update interrupt flag
        TIM4->SR &= ~TIM_SR_UIF;  // Clear the flag

        // Write next value to DAC1 (PA4)
        DAC->DHR12R1 = squareWave[square_index]; 
        
        // Increment index and wrap around
        square_index = (square_index + 1) % S_WAVE_SAMPLES;
    }
}

void INIT_TIM5(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;  // Enable TIM6 clock
																			//APB clock is Fcy/2 = 180MHz/2 = 90MHz
	TIM5->PSC = 9000 - 1;   // Prescaler: 90 MHz / 9000 = 10kHz
	TIM5->ARR = 100 - 1;  // Auto-reload: 10kHz / 1000 = 10Hz DAC update rate
	TIM5->DIER|=TIM_DIER_UIE;						//timer uptdate interrupt enabled
	TIM5->CNT=0;												//zero timer counter
	NVIC_EnableIRQ(TIM5_IRQn); 			//ENABLE TIM6 IRQ
	TIM5->CR1|=TIM_CR1_CEN;							//start timer counter
}

void TIM5_IRQHandler(void)
{
	 if ((TIM5->SR & TIM_SR_UIF) != 0) {  // Check update interrupt flag
        TIM5->SR &= ~TIM_SR_UIF;  // Clear the flag

        // Write next value to DAC2 (PA5)
        DAC->DHR12R2 = (sine_wave[sine_index] + triangleWave[triangle_index]); 
        
        // Increment index and wrap around
        sine_index = (sine_index + 1) % SINE_SAMPLES;
				triangle_index = (triangle_index + 1) % WAVE_SAMPLES;
    }
}

void INIT_TIM6(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;  // Enable TIM6 clock
	TIM6->PSC = 1000 - 1;   // Prescaler: 90 MHz / 1000 = 90kHz
	TIM6->ARR = 1000 - 1;  // Auto-reload: 90kHz / 1000 = 90Hz DAC update rate
	//TIM6->CR2 |= (3 << 4); // Set TRGO (Trigger Output) to Update Event   CHECK P440
	//TIM6->CR2 |= TIM_CR2_MMS_1; // Set TRGO (Trigger Output) to Update Event   CHECK P440
	//TIM_CR2_MMS_1
	
	
	TIM6->DIER|=TIM_DIER_UIE;						//timer uptdate interrupt enabled
																			//APB clock is Fcy/2 = 180MHz/2 = 90MHz
	TIM6->CNT=0;												//zero timer counter
	//NVIC_EnableIRQ(DMA1_Stream6_IRQn); 			//ENABLE TIM6 IRQ
	NVIC_EnableIRQ(TIM6_DAC_IRQn); 			//ENABLE TIM6 IRQ
	TIM6->CR1|=TIM_CR1_CEN;							//start timer counter
}

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

void INIT_TIM7(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;  // Enable TIM6 clock
																			//APB clock is Fcy/2 = 180MHz/2 = 90MHz
	TIM7->PSC = 9000 - 1;   // Prescaler: 90 MHz / 9000 = 10kHz
	TIM7->ARR = 100 - 1;  // Auto-reload: 10kHz / 1000 = 10Hz DAC update rate
	TIM7->DIER|=TIM_DIER_UIE;						//timer uptdate interrupt enabled
	TIM7->CNT=0;												//zero timer counter
	NVIC_EnableIRQ(TIM7_IRQn); 			//ENABLE TIM6 IRQ
	TIM7->CR1|=TIM_CR1_CEN;							//start timer counter
}

void TIM7_IRQHandler(void)
{
	 if ((TIM7->SR & TIM_SR_UIF) != 0) {  // Check update interrupt flag
        TIM7->SR &= ~TIM_SR_UIF;  // Clear the flag

        // Write next value to DAC2 (PA5)
        DAC->DHR12R2 = sine_wave[sine_index]; 
        
        // Increment index and wrap around
        sine_index = (sine_index + 1) % SINE_SAMPLES;
    }
}

void generateTriangleWave(void) {
    for (int i = 0; i < WAVE_SAMPLES / 2; i++) {
        triangleWave[i] = (4095 * i) / (WAVE_SAMPLES / 2);  //RISING EDGE OF TRIANGLE
				//GPIOC->ODR ^= 1<<3; 									//TOGGLE YELLOW LED (PC3)
    }
    for (int i = WAVE_SAMPLES / 2; i < WAVE_SAMPLES; i++) {
        triangleWave[i] = 4095 - ((4095 * (i - WAVE_SAMPLES / 2)) / (WAVE_SAMPLES / 2)); //FALLING EDGE OF TRIANGLE
    }
		
}

void DMA1_Stream6_Init(void)   //PP379
{
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;  // Enable DMA1 clock

    DMA1_Stream6->CR &= ~DMA_SxCR_EN;  // Disable stream before configuration

    DMA1_Stream6->PAR = (uint32_t) &DAC->DHR12R2; // DAC2 output register
    DMA1_Stream6->M0AR = (uint32_t) triangleWave; // Triangle wave buffer
    DMA1_Stream6->NDTR = WAVE_SAMPLES;  // Number of samples in one cycle
    DMA1_Stream6->CR |= (0x7 << 25);  // Channel 7 (DAC2 uses DMA1 Stream 6, Channel 7)
    DMA1_Stream6->CR |= (1 << 10);    // Memory-to-peripheral mode
    DMA1_Stream6->CR |= (2 << 13);    // 16-bit data size (Half-word)
    DMA1_Stream6->CR |= (1 << 8);     // Circular mode
    DMA1_Stream6->CR |= (1 << 6);     // Memory increment mode
    //DMA1_Stream6->CR |= (2 << 11);    // 16-bit peripheral size
		DMA1_Stream6->CR |= (1U << 4);   // Enable Transfer complete interrupt
		DMA1_Stream6->CR |= DMA_SxCR_EN; // Enable DMA stream
		//GPIOC->ODR ^= 1<<3; 									//TOGGLE YELLOW LED (PC3)
}

/*
void DMA1_Stream5_Init(void) {
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;  // Enable DMA1 clock

	DMA1_Stream5->CR &= ~DMA_SxCR_EN;  // Disable stream before configuration
	DMA1_Stream5->PAR = (uint32_t) &DAC->DHR12R2; // DAC2 output register
	DMA1_Stream5->M0AR = (uint32_t) triangleWave; // Triangle wave buffer
	DMA1_Stream5->NDTR = WAVE_SAMPLES;  // Number of samples in one cycle

	DMA1_Stream5->CR |= (0x7 << 25);  // Channel 7 (DAC2 uses DMA1 Stream 5, Channel 7)
	DMA1_Stream5->CR |= (1 << 10);    // Memory-to-peripheral mode
	DMA1_Stream5->CR |= (2 << 13);    // 16-bit data size (Half-word)
	DMA1_Stream5->CR |= (1 << 8);     // Circular mode
	DMA1_Stream5->CR |= (1 << 6);     // Memory increment mode
	DMA1_Stream5->CR |= (1 << 4);    // Enable Transfer complete interrupt
	
	//DMA1_Stream5->CR |= (2 << 11);    // 16-bit peripheral size

	
	DMA1_Stream5->CR |= DMA_SxCR_EN; // Enable DMA stream
}
*/


/*void Init_Timer2_1s(void)
{
	RCC->APB1ENR|=RCC_APB1ENR_TIM2EN;		//timer 2 clock enabled
	TIM2->DIER|=TIM_DIER_UIE;						//timer uptdate interrupt enabled
																			//APB clock is Fcy/2 = 180MHz/2 = 90MHz
	TIM2->PSC=90000-1;										//divide APB clock by 90000 = 90MHz/90000 = 1kHz
	TIM2->ARR=1000000;										//counter reload value, gives a timer period of 1s when F_APB = 90MHz and PSC = 1000
	TIM2->CNT=0;												//zero timer counter
	NVIC_EnableIRQ(TIM2_IRQn);						//timer 2 global interrupt enabled
	//WaiT3(1000000);
	TIM2->CR1|=TIM_CR1_CEN;							//start timer counter
}
*/

/*
void Init_Timer4_333ms(void)
{
	RCC->APB1ENR|=RCC_APB1ENR_TIM4EN;		//timer 4 clock enabled
	TIM4->DIER|=TIM_DIER_UIE;						//timer uptdate interrupt enabled
																			//APB clock is Fcy/2 = 180MHz/2 = 90MHz
	TIM4->PSC=90000-1;										//divide APB clock by 90000 = 90MHz/90000 = 1kHz
	TIM4->ARR=333;										//counter reload value, gives a timer period of 1s when F_APB = 90MHz and PSC = 1000
	TIM4->CNT=0;												//zero timer counter
	NVIC_EnableIRQ(TIM4_IRQn);						//timer 4 global interrupt enabled
	//WaiT3(1000000);
	TIM4->CR1|=TIM_CR1_CEN;							//start timer counter
}
*/

/*
void DAC_TIM2_IRQHandler(void)			//TIMER 2 INTERRUPT SERVICE ROUTINE
{
	if(TIM2->SR &  TIM_SR_UIF)// Check update interrupt flag
	{       
        TIM2->SR&=~TIM_SR_UIF;				//clear interrupt flag in status register
		
			GPIOC->ODR ^= 1<<3; 									//TOGGLE YELLOW LED (PC3)

        DAC->DHR12R2 = sine_wave[sine_index]; // Load the next value into DAC2
        sine_index = (sine_index + 1) % SINE_SAMPLES; // Increment and wrap the index
	}
}
*/

/*
void TIM4_IRQHandler(void)			//TIMER 4 INTERRUPT SERVICE ROUTINE
{
	if(TIM4->SR &  TIM_SR_UIF)// Check update interrupt flag
	{       
    TIM4->SR&=~TIM_SR_UIF;				//clear interrupt flag in status register
		generate_triangular_wave();
	}
}
*/

void generate_triangular_wave(void)
{
	DAC->CR |= DAC_CR_MAMP2_0;										//ONLY SET BIT TO SET AMPLITUDE AT 3V
	DAC->CR |= DAC_CR_MAMP2_1;										//ONLY SET BIT TO SET AMPLITUDE AT 3V
	DAC->CR &= ~DAC_CR_MAMP2_2;										//ONLY CLEAR BIT TO SET AMPLITUDE AT 3V
	DAC->CR |= DAC_CR_MAMP2_3;										//ONLY CLEAR BIT TO SET AMPLITUDE AT 3V
	
	DAC->CR &= ~DAC_CR_WAVE2_0;
	DAC->CR |= DAC_CR_WAVE2_1;
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