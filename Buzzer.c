#include "Buzzer.h"

void INIT_BUZZ(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;		//ONLY ENABLE GPIO B
	
	GPIOB->MODER 	|= (1UL<<(2*BUZZ_PIN)); 				//ONLY SET GPIOB (PB13) TO CONFIGURE AS INPUT
	GPIOB->MODER &= ~(2UL<<(2*BUZZ_PIN)); 				//ONLY CLEAR GPIOB (PB13) TO CONFIGURE AS INPUT
	
	GPIOB->OTYPER &= ~(1UL<<(2*BUZZ_PIN)); 				//ONLY CLEAR GPIOB (PB13) TO CONFIGURE AS PUSH-PULL
	
	GPIOB->OSPEEDR |= (3UL<<(2*BUZZ_PIN));				//ONLY CLEAR GPIOB (PB13) TO CONFIGURE AS VERY HIGH SPEED
	
	GPIOB->PUPDR |= (3UL<<(2*BUZZ_PIN));					//ONLY CLEAR GPIOB (PB13) TO CONFIGURE AS PULL UP OR PULL DOWN	
}

volatile uint8_t timer_flag = 0;  //FLAG FOR BUZZER DELAY COMPLETION

void INIT_TIM3(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;	//Enable TIM3 clock

	//Configure TIM3 for 1ms tick
																		//APB clock is Fcy/2 = 180MHz/2 = 90MHz
	TIM3->PSC = 90 - 1;  							//90 MHz / 90 = 1 MHz (1 us per tick)
	TIM3->ARR = 1000 - 1;   					//Auto-reload for 1000 us delay (1 milli second)
	TIM3->DIER |= TIM_DIER_UIE;  			//Enable update interrupt
	TIM3->CR1 |= TIM_CR1_CEN;    			//Start Timer
	NVIC_EnableIRQ(TIM3_IRQn); 				//Enable TIM3 interrupt in NVIC
}

void TIM3_DELAY(uint16_t delay_ms, uint32_t t)
{
	timer_flag = 0;
	TIM3->ARR = (delay_ms/t) - 1; 			//Set new delay time which can take octaves into consideration
	TIM3->CR1 |= TIM_CR1_CEN; 					//Start Timer
	while (timer_flag == 0);  					//Wait for flag to be set in ISR
}

void TIM3_IRQHandler(void) {
    if ((TIM3->SR & TIM_SR_UIF) !=0)	  	//Check update flag
		{
        TIM3->SR &= ~TIM_SR_UIF;  				//Clear flag
        timer_flag = 1;  									//Set flag for main loop
    }
}

//USING THE LENGTH OF A QUARTER NOTE AT 120BPM WHICH IS 500ms

void NOTE_Fs(uint32_t octaves, int length)
{
	for(int i = 0; i<length; i++)
	{
		TOGGLE_BUZZER();
		TIM3_DELAY(21622, octaves);
	}
}

void NOTE_F(uint32_t octaves, int length)
{
	for(int i = 0; i<length; i++)
	{
		TOGGLE_BUZZER();
		TIM3_DELAY(22907, octaves);
	}
}

void START_BEEP(void)								//SINGLE BEEP AT THE START OF MEASUREMENT
{
	for(int i = 0; i<100; i++)
	{
		TOGGLE_BUZZER();
		TIM3_DELAY(22907, 1);
	}
}

void END_BEEP(void)								//SINGLE BEEP AT THE END OF MEASUREMENT
{
	for(int i = 0; i<100; i++)
	{
		TOGGLE_BUZZER();
		TIM3_DELAY(22907, 2);
	}
}

void HEART_RATE_BEEP(void)
{
	for(int i = 0; i<3; i++) 					//REPEAT BLOCK OF CODE THREE TIMES
	{
		NOTE_Fs(20, 350);
		TIM3_DELAY(22907, 1);
		NOTE_Fs(20, 350);
		TIM3_DELAY(22907, 1);
	}
}

void OXG_LVL_BEEP(void)
{
		for(int i = 0; i<3; i++) 					//REPEAT BLOCK OF CODE THREE TIMES
	{
		NOTE_Fs(20, 350);
		TIM3_DELAY(22907, 1);
		NOTE_F(20, 350);
		TIM3_DELAY(22907, 1);
	}
}

void TEMP_BEEP(void)
{
	NOTE_Fs(20, 700);
	TIM3_DELAY(22907, 1);
}

void HUMIDITY_BEEP(void)
{
	for(int i = 0; i<4; i++) 					//REPEAT BLOCK OF CODE FOUR TIMES
	{
		NOTE_Fs(15, 300);
		TIM3_DELAY(22907, 1);
	}
}

void MOVEMENT_BEEP(void)
{
	NOTE_Fs(20, 350);
	TIM3_DELAY(2500, 1);
	NOTE_F(25, 350);
	TIM3_DELAY(2500, 1);
}

//(Heartrate/Oxygen Level/Temp/Humidity/Movement)

void TOGGLE_BUZZER(void)						//TOGGLE BUZZER
{
	GPIOB->ODR ^= 1<<BUZZ_PIN;
}



void AEOY(void)											//RECOGNISABLE TUNE INDICATING MODE OF OPERATION
{
	for(int i = 0; i<3; i++) 					//REPEAT BLOCK OF CODE THREE TIMES
	{
		NOTE_Fs(20, 250);
		TIM3_DELAY(2500, 1);
		NOTE_Fs(20, 250);
		TIM3_DELAY(2500, 1);
		NOTE_Fs(20, 250);
		TIM3_DELAY(2500, 1);
		NOTE_Fs(20, 900);
		TIM3_DELAY(50000, 1);				//REPEAT LINE 4 TIMES TO REPRESENT A DELAY VALUE OF 200000 (4*50000) AS THE MAXIMUM VALUE THE ARR CANOCOUNT UP TO IS 65656
		TIM3_DELAY(50000, 1);				//REPEAT LINE 4 TIMES TO REPRESENT A DELAY VALUE OF 200000 (4*50000) AS THE MAXIMUM VALUE THE ARR CANOCOUNT UP TO IS 65656
		TIM3_DELAY(50000, 1);				//REPEAT LINE 4 TIMES TO REPRESENT A DELAY VALUE OF 200000 (4*50000) AS THE MAXIMUM VALUE THE ARR CANOCOUNT UP TO IS 65656
		TIM3_DELAY(50000, 1);				//REPEAT LINE 4 TIMES TO REPRESENT A DELAY VALUE OF 200000 (4*50000) AS THE MAXIMUM VALUE THE ARR CANOCOUNT UP TO IS 65656
	}
	
	NOTE_Fs(20, 250);
	TIM3_DELAY(2500, 1);
	NOTE_Fs(20, 250);
	TIM3_DELAY(2500, 1);
	NOTE_Fs(20, 250);
	TIM3_DELAY(2500, 1);
	NOTE_F(20, 900);
	TIM3_DELAY(50000, 1);				//REPEAT LINE 4 TIMES TO REPRESENT A DELAY VALUE OF 200000 (4*50000) AS THE MAXIMUM VALUE THE ARR CANOCOUNT UP TO IS 65656
	TIM3_DELAY(50000, 1);				//REPEAT LINE 4 TIMES TO REPRESENT A DELAY VALUE OF 200000 (4*50000) AS THE MAXIMUM VALUE THE ARR CANOCOUNT UP TO IS 65656
	TIM3_DELAY(50000, 1);				//REPEAT LINE 4 TIMES TO REPRESENT A DELAY VALUE OF 200000 (4*50000) AS THE MAXIMUM VALUE THE ARR CANOCOUNT UP TO IS 65656
	TIM3_DELAY(50000, 1);				//REPEAT LINE 4 TIMES TO REPRESENT A DELAY VALUE OF 200000 (4*50000) AS THE MAXIMUM VALUE THE ARR CANOCOUNT UP TO IS 65656
}

//void syncedHeartRateBeep(uint16_t bpm)
//{
	
//}



/*
Demonstrate Buzzer functionality by performing the following:
•	Single Beep at start and end of measurement
•	Beep aligned with heartrate
•	Different beep pattern/tone indicates mode of operation (Heartrate/Oxygen Level/Temp/Humidity/Movement)
•	Plays a recognisable tune indicating mode of operation.
*/