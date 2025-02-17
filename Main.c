#include <stm32f429xx.h>		//INCLUDE THE HEADER FILE FOR THIS MCU FAMILY
													//this file contains the definitions for register addresses and values etc...
#include "PLL_Config.c"

#include "Switches.h"
#include "LCD.h"
#include "Buzzer.h"
#include "DAC.h"
#include "SPI.h"

float xg, yg, zg;
int16_t x,y,z;
extern uint8_t RxData[6];

void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void BUTTON_PRESS(void);
void SWITCH_ON(char pin);
void SWITCH_OFF(char pin);
//void generateTriangleWave(void);
//void DAC_TIM2_IRQHandler(void);
//


int main(void)
{
	PLL_Config();									// Set system clock to 180MHz
	SystemCoreClockUpdate();			// Update SystemCoreClock
	
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; 
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	
	BUTTON_INIT();
	
	
	INIT_BUZZ();
	
	//Init_Timer2_1s();
	
	
	CONFIG_GPIO_FOR_SPI();
	//TIM6Config ();
	INIT_SPI();
	SPI_ENABLE();
	ADXL_INIT();
	
	
	//Init_Timer3_500ms();
	
	set_LCD_bus_input();
	set_LCD_RW();
	clr_LCD_RS();
	set_LCD_E();
	set_LCD_bus_output();
	
	initLCD();									//INITIALISE LCD
		
	cmdLCD(LCD_LINE1);					//WRITE ON THE FIRST LCD ROW
	putLCD('H');
	putLCD('E');
	putLCD('A');
	putLCD('R');
	putLCD('T');
	putLCD(' ');
	putLCD('R');
	putLCD('A');
	putLCD('T');
	putLCD('E');
	
	cmdLCD(LCD_LINE2);					//WRITE ON THE SECOND LCD ROW
	putLCD('M');
	putLCD('O');
	putLCD('N');
	putLCD('I');
	putLCD('T');
	putLCD('O');
	putLCD('R');
	putLCD('I');
	putLCD('N');
	putLCD('G');
	putLCD('.');
	putLCD('.');
	putLCD('.');
	
	//while(1);
	
	
	
	
	INIT_DAC();
	
	
	//generate_sine_wave();	
	//generateTriangleWave();
	
	//DMA1_Stream6_Init();
	//INIT_TIM4();
	//INIT_TIM5();
	//INIT_TIM6();
	//INIT_TIM7();
	
	//INIT_TIM12();
	
	//TIM3_Init_Interrupt();
	//TIM13_Init_Interrupt();

	//generate_triangular_wave();
	//DAC->CR |= (1 << 16) | (1 << 28);  // Enable DAC2 and enable DMA for DAC2
	
	
	//START_BEEP();
	//END_BEEP();
	HEART_RATE_BEEP();
	//OXG_LVL_BEEP();
	//TEMP_BEEP();
	//HUMIDITY_BEEP();
	//MOVEMENT_BEEP();
	//AEOY();
	
	
	Init_Timer3_2s();
	INIT_EXTI0();
	INIT_EXTI1();
	INIT_EXTI2();
	INIT_EXTI3();
	INIT_EXTI13();
	
	while (1)
	{
				
		/*
		ADXL_READ(0x32);
		x = ((RxData[1]<<8)|RxData[0]);
		y = ((RxData[3]<<8)|RxData[2]);
		z = ((RxData[5]<<8)|RxData[4]);

	  xg = x*.0078;
    yg = y*.0078;
   	zg = z*.0078;
		
		wait_us(500000);
		*/
		
		//putStrLCD(5643);
		//putLCD('R');
		//putLCD(xg);
		//putLCD(yg);
		//putLCD(zg);
		
		//DAC1_OUTPUT(3723);
		//DAC1_OUTPUT(squareWave[1]);
		
		DAC2_OUTPUT(3723);
		//DAC2_OUTPUT(3723);
		//DAC2_OUTPUT(1000);
	
	}
		
	while (1)
	{
		//__WFI(); 										//TO REDUCE POWER CONSUMPTION WHILE WAITING FOR INTERRUPTS TO OCCUR
		
		
		
		
		
		//SWITCH_ON(13);								//TURNS ON THE BUZZER
		//START_BEEP();
		//NOTE_Fs(5);
		//SWITCH_BOUNCE_DELAY(2273);
		//wait_us(1000000);
		//SWITCH_OFF(13);								//TURNS OFF THE BUZZER
		//TOGGLE_BUZZER();
		
		//wait_us(1000000);
		//SWITCH_BOUNCE_DELAY(2273);
	}
	
}//END MAIN