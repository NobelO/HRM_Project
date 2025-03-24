#include "Switches.h"

void BUTTON_INIT(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;		//ONLY ENABLE GPIO A
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;		//ONLY ENABLE GPIO C
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;		//ONLY ENABLE GPIO G
		
	//CODE TO INITIALISE YELLOW LED FOR TESTING
	GPIOC->MODER &= ~(3UL<<(2*3));				//ONLY clear  GPIOC (PC3) TO CONFIGURE AS AN INPUT
	GPIOC->OTYPER &= ~(1UL<<3); 					//CONFIGURE PC3 AS PUSH-PULL
	
	//CODE TO INITIALISE PIN FOR RED AND INFRARED LED TOGGLING
		GPIOA->MODER &= ~(3UL<<(2*LED_Toggling));					//ONLY CLEAR  GPIOA (PA7) TO CONFIGURE AS AN OUTPUT

	GPIOA->MODER |= 1UL<<(2*LED_Toggling);						//ONLY SET  GPIOA (PA7) TO CONFIGURE AS AN OUTPUT
	GPIOA->OTYPER &= ~(1UL<<LED_Toggling); 						//CONFIGURE PA7 AS PUSH-PULL
	GPIOA->OSPEEDR &= ~(2UL<<(2*LED_Toggling));				//ONLY CLEAR GPIOA (PA7) TO SET TO HIGH SPEED
	GPIOA->PUPDR &= ~(1UL<<(2*LED_Toggling)); 				//ONLY CLEAR GPIOA (PA7) TO SET AS PULL DOWN
	
	
	GPIOC->MODER 	&= ~(3UL<<(2*blueButton)); 				//ONLY CLEAR GPIOC (PC13) TO SET AS INPUT
	
	GPIOG->MODER 	&= ~(3UL<<(2*buttonA)); 				//ONLY CLEAR GPIOG (PG0) TO SET AS INPUT
	GPIOG->MODER 	&= ~(3UL<<(2*buttonB)); 				//ONLY CLEAR GPIOG (PG1) TO SET AS INPUT
	GPIOG->MODER 	&= ~(3UL<<(2*buttonC)); 				//ONLY CLEAR GPIOG (PG2) TO SET AS INPUT
	GPIOG->MODER 	&= ~(3UL<<(2*buttonD)); 				//ONLY CLEAR GPIOG (PG3) TO SET AS INPUT
	
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	GPIOC->MODER &= ~(3<<(2*3));
	GPIOC->MODER |= (1<<(2*3));
	
	GPIOC->OTYPER &= ~(1UL<<(2*blueButton)); 				//ONLY CLEAR GPIOC (PC13) TO CONFIGURE AS PUSH-PULL
	
	GPIOG->OTYPER &= ~(1UL<<(2*buttonA)); 				//ONLY CLEAR GPIOG (PG0) TO CONFIGURE AS PUSH-PULL
	GPIOG->OTYPER &= ~(1UL<<(2*buttonB)); 				//ONLY CLEAR GPIOG (PG1) TO CONFIGURE AS PUSH-PULL
	GPIOG->OTYPER &= ~(1UL<<(2*buttonC)); 				//ONLY CLEAR GPIOG (PG2) TO CONFIGURE AS PUSH-PULL
	GPIOG->OTYPER &= ~(1UL<<(2*buttonD)); 				//ONLY CLEAR GPIOG (PG3) TO CONFIGURE AS PUSH-PULL
	
	
	GPIOC->OSPEEDR &= ~(3UL<<(2*blueButton));			//ONLY CLEAR GPIOC (PC13) TO SET TO LOW SPEED
	
	GPIOG->OSPEEDR &= ~(3UL<<(2*buttonA));				//ONLY CLEAR GPIOG (PG0) TO SET TO LOW SPEED
	GPIOG->OSPEEDR &= ~(3UL<<(2*buttonB));				//ONLY CLEAR GPIOG (PG1) TO SET TO LOW SPEED
	GPIOG->OSPEEDR &= ~(3UL<<(2*buttonC));				//ONLY CLEAR GPIOG (PG2) TO SET TO LOW SPEED
	GPIOG->OSPEEDR &= ~(3UL<<(2*buttonD));				//ONLY CLEAR GPIOG (PG3) TO SET TO LOW SPEED


	GPIOC->PUPDR |= (2UL<<(2*blueButton)); 				//ONLY CLEAR GPIOC (PC13) TO SET AS PULL DOWN
	GPIOC->PUPDR &= ~(1UL<<(2*blueButton)); 			//ONLY CLEAR GPIOC (PC13) TO SET AS PULL DOWN	
	
	GPIOG->PUPDR &= ~(2UL<<(2*buttonA));					//ONLY CLEAR GPIOG (PG0) TO SET AS PULL UP
	GPIOG->PUPDR &= ~(2UL<<(2*buttonB));					//ONLY CLEAR GPIOG (PG1) TO SET AS PULL UP
	GPIOG->PUPDR |= (2UL<<(2*buttonC));						//ONLY SET GPIOG (PG2) TO SET AS PULL DOWN
	GPIOG->PUPDR |= (2UL<<(2*buttonD));						//ONLY SET GPIOG (PG3) TO SET AS PULL DOWN
	
	GPIOG->PUPDR |= 1UL<<(2*buttonA);							//ONLY SET GPIOG (PG0) TO SET TO PULL UP
	GPIOG->PUPDR |= 1UL<<(2*buttonB);							//ONLY SET GPIOG (PG1) TO SET TO PULL UP
	GPIOG->PUPDR &= ~(1UL<<(2*buttonC));					//ONLY CLEAR GPIOG (PG2) TO SET TO PULL DOWN
	GPIOG->PUPDR &= ~(1UL<<(2*buttonD));					//ONLY CLEAR GPIOG (PG3) TO SET TO PULL DOWN
}


void SWITCH_BOUNCE_DELAY(uint32_t t)
{
	uint32_t start, cnt;
	start = DWT->CYCCNT;
	cnt = t * (SystemCoreClock/1000000); 
	while ((DWT->CYCCNT - start) < cnt);
}

void wait_us(uint32_t n)
{ 
	uint32_t start, cnt; 
	start = DWT->CYCCNT; 
	cnt = n * (SystemCoreClock/1000000); 
	while ((DWT->CYCCNT - start) < cnt);
}

void Init_Timer3_500ms(void)
{
	RCC->APB1ENR|=RCC_APB1ENR_TIM3EN;		//timer 3 clock enabled
	TIM3->DIER|=TIM_DIER_UIE;						//timer uptdate interrupt enabled
																			//APB clock is Fcy/2 = 180MHz/2 = 90MHz
	TIM3->PSC=9000-1;										//divide APB clock by 9000 = 90MHz/9000 = 10kHz
	TIM3->ARR=5000;										//counter reload value, gives a timer period of 500ms when F_APB = 90MHz and PSC = 9000
	TIM3->CNT=0;												//zero timer counter
	NVIC->ISER[0]|=(1u<<29);						//timer 3 global interrupt enabled
	TIM3->CR1|=TIM_CR1_CEN;							//start timer counter
}

//static 
volatile uint8_t hold_value = 0;
volatile uint8_t timer_flag2 = 0;  // Flag for delay completion


#define LONG_PRESS_TIME 20000  // 20000 * 0.1ms (2 seconds)
volatile uint32_t press_start_time = 0;
volatile uint8_t long_press_detected = 0;

void TIM2_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;  // Enable TIM2 clock
    TIM2->PSC = 16000 - 1;  // Prescaler: 16 MHz / 16000 = 1 kHz (1ms per tick)
    TIM2->ARR = 0xFFFFFFFF; // Max count
    TIM2->CR1 |= TIM_CR1_CEN; // Start TIM2
}

void Init_Timer3_2s(void)
{
	RCC->APB1ENR|=RCC_APB1ENR_TIM3EN;		//timer 3 clock enabled
	//TIM3->DIER|=TIM_DIER_UIE;						//timer uptdate interrupt enabled
																			//APB clock is Fcy/2 = 180MHz/2 = 90MHz
	TIM3->PSC=9000-1;										//divide APB clock by 9000 = 90MHz/9000 = 10kHz (0.1ms per tick)
	TIM3->ARR=0xFFFF;										//counter reload value, gives a timer period of 2s when F_APB = 90MHz and PSC = 9000
	//TIM3->CNT=0;												//zero timer counter
	//NVIC->ISER[0]|=(1UL<<29);						//timer 3 global interrupt enabled
	TIM3->CR1|=TIM_CR1_CEN;							//start timer counter
}


void BUTTON_PRESS(void)
{
	if ((GPIOC->IDR & 1UL<<blueButton) != 0)	//PIN STAYS HIGH WHEN THE BUTTON IS NOT PRESSED
		{
			GPIOC->ODR ^= 1<<3; 									//TOGGLE YELLOW LED (PC3)
			SWITCH_BOUNCE_DELAY(500000);					//DEBOUNCE BUTTON
		}
}

/*
void SWITCH_ON(char pin)
{
	GPIOB->BSRR = 1<<pin;
}

void SWITCH_OFF(char pin)
{
	GPIOB->BSRR = (1<<pin)<<16;
}
*/

/*
void TIM3_IRQHandler(void)			//TIMER 3 INTERRUPT SERVICE ROUTINE
{
	TIM3->SR&=~TIM_SR_UIF;				//clear interrupt flag in status register
	BUTTON_PRESS();											//CALL THE BLUE BUTTON FUNCTION
		
	//if(TIM3->DIER &= TIM_DIER_UIE != 0) 
	//GPIOC->BSRR = 1<<13; 
	//SWITCH_BOUNCE_DELAY();
	//wait_us(1000);
	//GPIOC->BSRR = 1<<3;				//ONLY LED ON
}
*/

void INIT_EXTI13(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
 	SYSCFG->EXTICR[3] &= ~(0xF << 4);  // Clear EXTI13 bits
  SYSCFG->EXTICR[3] |= (2UL << 4);   // Select GPIOC (0010). P302
	
	EXTI->IMR |= (1 << 13);     // Unmask EXTI line 13
  EXTI->FTSR |= (1 <<13);    // Enable falling edge trigger for EXTI line 13
  EXTI->RTSR |= (1 << 13);  // Rising edge (button release)
	
	NVIC_EnableIRQ(EXTI15_10_IRQn);			// Enable EXTI13 Interrupt in NVIC
}

void EXTI15_10_IRQHandler(void) {
    /*
	if(EXTI->PR & (1 << 13))	{  // Check if EXTI13 triggered
				SWITCH_BOUNCE_DELAY(200000);					//DEBOUNCE BUTTON
				if((GPIOC->IDR & 1UL<<blueButton) != 0) {  // Button Released (Rising Edge)
            uint32_t press_duration = TIM3->CNT - press_start_time;
            if(press_duration >= LONG_PRESS_TIME) {
                long_press_detected = 1;  // Long press detected
								//SWITCH_BOUNCE_DELAY(200000);					//DEBOUNCE BUTTON
								GPIOC->ODR ^= 1<<3; 									//TOGGLE YELLOW LED (PC3)
            }
        }
				else{  // Button Pressed (Falling Edge)
            press_start_time = TIM3->CNT;  // Store start time
        }
				
				EXTI->PR |= (1 << 13);   // Clear pending bit
		}
	*/
	/*
	if (EXTI->PR & (1 << 13)) {
		SWITCH_BOUNCE_DELAY(200000);					//DEBOUNCE BUTTON
		if ((GPIOC->IDR & 1UL<<blueButton) != 0) {  // Button Released (Rising Edge)
			uint32_t press_duration = TIM13->CNT - press_start_time;
			if (press_duration >= LONG_PRESS_TIME) {
				long_press_detected = 1;  // Long press detected
				GPIOC->ODR ^= 1<<3; 									//TOGGLE YELLOW LED (PC3)
				hold_value = !hold_value; // Toggle hold
			}
		}
		else										// Button Pressed (Falling Edge)
		{
				press_start_time = TIM13->CNT;  // Store start time
		}
		//GPIOC->ODR ^= 1<<3; 									//TOGGLE YELLOW LED (PC3)
		EXTI->PR |= (1 << 13);  // Clear interrupt flag
	}
	*/
	if (EXTI->PR & (1 << 13)) {
		SWITCH_BOUNCE_DELAY(100000);					//DEBOUNCE BUTTON
		if ((GPIOC->IDR & 1UL<<blueButton) != 0) {  // Button Released (Rising Edge)
			
			
			//TIM13->CR1 |= TIM_CR1_CEN;
			//timer_flag2 = 0;
			//uint32_t press_duration = TIM13->CNT;
			//uint32_t press_duration = 25000;
			SWITCH_BOUNCE_DELAY(10000);					//DEBOUNCE BUTTON
			//if (press_duration >= LONG_PRESS_TIME) {
			if ((TIM13->CNT) <= 20000) {
				
				long_press_detected = 1;  // Long press detected
				GPIOC->ODR ^= 1<<3; 									//TOGGLE YELLOW LED (PC3)
				hold_value = !hold_value; // Toggle hold
			}
		}
		else										// Button Pressed (Falling Edge)
		{
				press_start_time = TIM13->CNT;  // Store start time
		}
		//GPIOC->ODR ^= 1<<3; 									//TOGGLE YELLOW LED (PC3)
		EXTI->PR |= (1 << 13);  // Clear interrupt flag
	}
}
	
void INIT_EXTI0(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
 	SYSCFG->EXTICR[0] &= ~(0xF << 0);  // Clear EXTI13 bits
  SYSCFG->EXTICR[0] |= (6UL << 0);   // Select GPIOC (0110). P302
	
	EXTI->IMR |= (1 << 0);     // Unmask EXTI line 13
  EXTI->FTSR |= (1 <<0);    // Enable falling edge trigger for EXTI line 13
	//EXTI->PR |= (1<<13);  // Pending register
	
	NVIC_EnableIRQ(EXTI0_IRQn);			// Enable EXTI13 Interrupt in NVIC
}

void EXTI0_IRQHandler(void)
{
    if (EXTI->PR & (1 << 0)) {  // Check if EXTI13 triggered
			
				SWITCH_BOUNCE_DELAY(200000);					//DEBOUNCE BUTTON
        EXTI->PR |= (1 << 0);   // Clear pending bit
        GPIOC->ODR ^= 1<<3; 									//TOGGLE YELLOW LED (PC3)
    }
}

void INIT_EXTI1(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
 	SYSCFG->EXTICR[0] &= ~(0xF << 4);  // Clear EXTI1 bits
  SYSCFG->EXTICR[0] |= (6UL << 4);   // Select GPIOG (0110). P302
	
	EXTI->IMR |= (1 << 1);     // Unmask EXTI line 1
  EXTI->FTSR |= (1 << 1);    // Enable falling edge trigger for EXTI line 1
	//EXTI->PR |= (1<<13);  // Pending register
	
	NVIC_EnableIRQ(EXTI1_IRQn);			// Enable EXTI1 Interrupt in NVIC
}

void EXTI1_IRQHandler(void)
{
    if (EXTI->PR & (1 << 1)) {  // Check if EXTI1 triggered
			
				SWITCH_BOUNCE_DELAY(200000);					//DEBOUNCE BUTTON
        EXTI->PR |= (1 << 1);   // Clear pending bit
        GPIOC->ODR ^= 1<<3; 									//TOGGLE YELLOW LED (PC3)
    }
}

void INIT_EXTI2(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
 	SYSCFG->EXTICR[0] &= ~(0xF << 8);  // Clear EXTI2 bits
  SYSCFG->EXTICR[0] |= (6UL << 8);   // Select GPIOG (0110). P302
	
	EXTI->IMR |= (1 << 2);     // Unmask EXTI line 2
  EXTI->FTSR |= (1 << 2);    // Enable falling edge trigger for EXTI line 2
	//EXTI->PR |= (1<<13);  // Pending register
	
	NVIC_EnableIRQ(EXTI2_IRQn);			// Enable EXTI2 Interrupt in NVIC
}

void EXTI2_IRQHandler(void)
{
    if (EXTI->PR & (1 << 2)) {  // Check if EXTI2 triggered
			
				SWITCH_BOUNCE_DELAY(200000);					//DEBOUNCE BUTTON
        EXTI->PR |= (1 << 2);   // Clear pending bit
        GPIOC->ODR ^= 1<<3; 									//TOGGLE YELLOW LED (PC3)
    }
}

void INIT_EXTI3(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
 	SYSCFG->EXTICR[0] &= ~(0xF << 12);  // Clear EXTI3 bits
  SYSCFG->EXTICR[0] |= (6UL << 12);   // Select GPIOG (0110). P302
	
	EXTI->IMR |= (1 << 3);     // Unmask EXTI line 3
  EXTI->FTSR |= (1 << 3);    // Enable falling edge trigger for EXTI line 3
	//EXTI->PR |= (1<<13);  // Pending register
	
	NVIC_EnableIRQ(EXTI3_IRQn);			// Enable EXTI3 Interrupt in NVIC
}

void EXTI3_IRQHandler(void)
{
    if (EXTI->PR & (1 << 3)) {  // Check if EXTI3 triggered
			
				SWITCH_BOUNCE_DELAY(200000);					//DEBOUNCE BUTTON
        EXTI->PR |= (1 << 3);   // Clear pending bit
        GPIOC->ODR ^= 1<<3; 									//TOGGLE YELLOW LED (PC3)
    }
}



void TIM13_Init_Interrupt(void) {
    // Enable TIM3 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM13EN;

    // Configure TIM3 for 1ms tick
														//APB clock is Fcy/2 = 180MHz/2 = 90MHz
    TIM13->PSC = 9 - 1;  	// 90 MHz / 9000 = 10 KHz (0.1 ms per tick)
    TIM13->ARR = 20000 - 1;   // Auto-reload for 10000 0.1ms delay (1 second)

    // Enable Update Interrupt
    TIM13->DIER |= TIM_DIER_UIE;  // Enable update interrupt
    TIM13->CR1 |= TIM_CR1_CEN;    // Start Timer

    // Enable TIM3 interrupt in NVIC
    NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
}

/*
void TIM13_Delay(uint16_t delay_s) {
	timer_flag2 = 0;
	TIM13->ARR = delay_s - 1; // Set new delay time
	TIM13->CR1 |= TIM_CR1_CEN; // Start Timer
	while (timer_flag2 == 0);  // Wait for flag to be set in ISR
}
*/

void TIM8_UP_TIM13_IRQHandler(void)
{
    if ((TIM13->SR & TIM_SR_UIF) !=0) {  // Check update flag
        TIM13->SR &= ~TIM_SR_UIF;  // Clear flag
        timer_flag2 = 1;  // Set flag for main loop
    }
}

void INIT_TIM12(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM12EN;  // Enable TIM6 clock
																			//APB clock is Fcy/2 = 180MHz/2 = 90MHz
	TIM12->PSC = 9000 - 1;   // Prescaler: 90 MHz / 9000 = 10kHz
	TIM12->ARR = 1000 - 1;  // Auto-reload: 10kHz / 1000 = 10Hz DAC update rate
	TIM12->DIER|=TIM_DIER_UIE;						//timer uptdate interrupt enabled
	TIM12->CNT=0;												//zero timer counter
	NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn); 			//ENABLE TIM12 IRQ
	TIM12->CR1|=TIM_CR1_CEN;							//start timer counter
}

void TIM8_BRK_TIM12_IRQHandler(void)
{
    if ((TIM12->SR & TIM_SR_UIF) != 0) {  // Check if update event occurred
        TIM12->SR &= ~TIM_SR_UIF;  // Clear interrupt flag

         GPIOA->BSRR = (1<<LED_Toggling<<16); //TOGGLE PA7 TO TURN ON AND OFF RED AND INFRARED LED SIMULTANEOUSLY
		}
}




/*
Demonstrate Switch/Button functionality by performing the following:
•	The Blue push button should be used to select the mode of operation:
•	A short press (<1s) of the Blue button should START operation
o	Long press (2s) should select “HOLD” mode that maintains the current display output 
o	An indication of “HOLD” should be displayed on the bottom-line of the LCD
o	Another long press is required to return to normal sampling mode of operation.  
•	External Buttons {A,B,C,D} on the Module Support board should be used to select mode of operation indicated on LCD/UART.
•	The use of interrupts to capture switch activity is desirable.
•	Switch bounce should be removed using software
?	Advanced functionality - entering a coded sequence of pushes directly to select a given mode. (i.e double press) or calibration
*/
