#include <stm32f429xx.h>		//INCLUDE THE HEADER FILE FOR THIS MCU FAMILY
													//this file contains the definitions for register addresses and values etc...
#include "PLL_Config.c"

#include "Switches.h"
#include "LCD.h"
#include "Buzzer.h"
#include "DAC.h"
#include "SPI.h"

#include "ADC.h"
#include "Usart.h"
#include "Delay.h"

#include <stdio.h>

float xg, yg, zg;
int16_t x,y,z;
extern uint8_t RxData[6];

//volatile unsigned short BPM;

void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void BUTTON_PRESS(void);
void SWITCH_ON(char pin);
void SWITCH_OFF(char pin);
//void generateTriangleWave(void);



int main(void)
{
	PLL_Config();									// Set system clock to 180MHz
	SystemCoreClockUpdate();			// Update SystemCoreClock
	
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; 
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	

	BUTTON_INIT();
			GPIOC->ODR ^=(1<<3);
		for(int i=0;i<1999999;i++){
			__NOP();
		}
	}


	
	INIT_BUZZ();
	


	init_adc();
	init_usart();
	init_Timer6();
	
		while(1){

	
	//Init_Timer2_1s();
	
	
	CONFIG_GPIO_FOR_SPI();
	//TIM6Config ();
	//INIT_SPI();
	//SPI_ENABLE();
	//ADXL_INIT();
	
	
	//Init_Timer3_500ms();
	
	set_LCD_bus_input();
	set_LCD_RW();
	clr_LCD_RS();
	set_LCD_E();
	set_LCD_bus_output();
	
	INIT_LCD();									//INITIALISE LCD
		
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
	putStrLCD("Mon");
	
	//while(1);
	
	
	
	
	INIT_DAC();
	
	
	//generateSineWave();
	//generateTriangleWave();
	generateComplexWave();
	
	//INIT_TIM4();						//Square wave timer
	//INIT_TIM5();						//Complex wave timer
	//INIT_TIM6();						//Triangle wave timer
	INIT_TIM7();						//Sine wave timer
	
	INIT_TIM12();
	
	INIT_TIM3();						//DELAY FOR BUZZER
	//TIM13_Init_Interrupt();
	INIT_TIM14();

	
	
	START_BEEP();
	//END_BEEP();
	//HEART_RATE_BEEP();
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
		//adc_trigger();
		
		/*
		ADXL_READ(0x32);
		x = ((RxData[1]<<8)|RxData[0]);
		y = ((RxData[3]<<8)|RxData[2]);
		z = ((RxData[5]<<8)|RxData[4]);

	  xg = x*.0078;
    yg = y*.0078;
   	zg = z*.0078;
		*/
		cmdLCD(LCD_LINE1);					//WRITE ON THE FIRST LCD ROW
		
		//putLCD('.');
		//putStrLCD("Mon");
		/*char buffer[3];
		sprintf(buffer, "%d", BPM);
		putStrLCD(buffer);
		
		cmdLCD(LCD_LINE2);															//WRITE ON THE SECOND LCD ROW, AND IN THE SAME POSITION
		putStrLCD("Oxygen Lvl: 98zxx%");	
		*/
		wait_us(500000);
		
		
		//putStrLCD(5643);
		//putLCD('R');
		//putLCD(xg);
		//putLCD(yg);
		//putLCD(zg);
		
		//DAC1_OUTPUT(3723);
		//DAC1_OUTPUT(squareWave[1]);
		
		//DAC2_OUTPUT(2095);
		//DAC2_OUTPUT(3723);
		DAC2_OUTPUT(4095);
		//DAC2_OUTPUT(1000);
	
	}
	
	/*
	float curVal = maxVal = 0;
	//Sample at 1000Hz
	//The value from the ADC is assigned to the curVal (Current Value)
	if(curVal > maxVal)
	{
		maxVal = curVal;
	}
	//After reading for five seconds
	//Upper threshold = 0.8 of the maxVal
	//Lower threshold = 0.2 of the maxVal
	
	//Create a variable called state and set as Low
	string state = low;
	
	if((state == low) && (curVal >= upperThreshold))
	{
		state = high;
		//Start timer
	}
	
	if((state == high) && (curVal >= upperThreshold))
	{
		state = high;
		//Collect timer value
	}
	*/
		
	
	
}//END MAIN