#include "LCD.h"

//extern volatile uint8_t hold_value = 0;

void lcd_delayus(unsigned int us)		//blocking delay for LCD, argument is approximate number of micro-seconds to delay
{
	unsigned char i;
	while(us--)
	{
		for(i=0; i<SystemCoreClock/4000000; i++){__NOP();}
	}
}

void WAIT_LCD_BUSY(void)
{
	/*
	LCD_PORT->MODER &= ~(0xFFFF<<(2*LCD_D0_pin));				//CLEAR D0-D7 TO CONFIGURE AS OUTPUTS
	
	LCD_PORT->BSRR = (1U<<(LCD_RS_pin+16)); // RS = 0 (Command Mode)
	LCD_PORT->BSRR = (1U<<LCD_RW_pin);  // RW = 1 (Read Mode)
	
	do {
			// Pulse Enable to read BF
		set_LCD_E();
		lcd_delayus(5000);
		clr_LCD_E();
		lcd_delayus(5000);
	} while (LCD_PORT->IDR & (1U << 7));  // Check if D7 (BF) is high
	
	LCD_PORT->BSRR = (1U<<(LCD_RW_pin+16)); // RW = 0 (Write Mode)
	
	LCD_PORT->MODER &= (0x5555<<(2*LCD_D0_pin));				//CLEAR D0-D7 TO CONFIGURE AS OUTPUTS
	*/
	
	
	//LCD_PORT->BSRR = LCD_RW_pin;  // RW = 1 (Read Mode)
  //LCD_PORT->BSRR = ~LCD_RS_pin; // RS = 0 (Command Mode)
	
	set_LCD_RW();
	clr_LCD_RS();

	do {set_LCD_E(); //LCD_PORT->BSRR = LCD_E_pin;  // E = 1 (Enable LCD)
			__NOP();  // Small delay
			__NOP();
	}
	while ((LCD_PORT->IDR & LCD_D0_pin) != 0);  // Loop while DB7 (Busy Flag) is HIGH

	//LCD_PORT->BSRR = ~LCD_E_pin;  // E = 0 (Disable LCD)
	//LCD_PORT->BSRR = ~LCD_RW_pin; // RW = 0 (Write Mode)
	
	clr_LCD_E();
	clr_LCD_RW();
	
}

void set_LCD_data(unsigned char d)
{
	LCD_PORT->BSRR=(0xff<<(LCD_D0_pin+16));	//clear data lines
	LCD_PORT->BSRR=(d<<LCD_D0_pin);					//put data on lines
}

void LCD_Write_String(const char *str) {
		LCD_PORT->BSRR=(0xff<<(LCD_D0_pin+16));	//clear data lines
    while (*str) {
			LCD_PORT->BSRR=(*str);  
			//LCD_Write_Char(*str++); // Write each character
    }
}

void LCD_strobe(void)		//10us high pulse on LCD enable line
{
	lcd_delayus(10);
	set_LCD_E();
	lcd_delayus(10);
	clr_LCD_E();
}


void cmdLCD(unsigned char cmd)		//sends a byte to the LCD control register
{
	WAIT_LCD_BUSY();				//wait for LCD to be not busy
	clr_LCD_RS();					//control command
	clr_LCD_RW();					//write command
	set_LCD_data(cmd);		//set data on bus
	LCD_strobe();					//apply command
}

void putLCD(unsigned char put)	//sends a char to the LCD display
{
	WAIT_LCD_BUSY();				//wait for LCD to be not busy
	set_LCD_RS();					//text command
	clr_LCD_RW();					//write command
	//if (!hold_value)
	{  
		set_LCD_data(put);		//set data on bus
  }
	LCD_strobe();					//apply command
}

void putStrLCD(char *str)	//sends a char to the LCD display
{
	WAIT_LCD_BUSY();				//wait for LCD to be not busy
	set_LCD_RS();					//text command
	clr_LCD_RW();					//write command
	
	//if (!hold_value)
	while(*str){  
    set_LCD_data(*str++);		//set data on bus
		LCD_strobe();					//apply command
  }
	
}

void INIT_LCD(void)
{
	SystemCoreClockUpdate();
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIODEN;	//enable LCD port clock


	//CONFIGURE LCD GPIO PINS
	LCD_PORT->MODER&=~(					//clear pin direction settings
		(3u<<(2*LCD_RS_pin))
		|(3u<<(2*LCD_RW_pin))
		|(3u<<(2*LCD_E_pin))
		|(0xffff<<(2*LCD_D0_pin))				//CLEAR D0-D7 TO CONFIGURE AS OUTPUTS
	);


	LCD_PORT->MODER|=(				//reset pin direction settings to digital outputs
		(1u<<(2*LCD_RS_pin))
		|(1u<<(2*LCD_RW_pin))
		|(1u<<(2*LCD_E_pin))
		|(0x5555<<(2*LCD_D0_pin))				//SET D0-D7 TO CONFIGURE AS OUTPUTS
	);
	
	LCD_PORT->OTYPER&=~(					//CONFIGURE CONTROL LINES AS PUSH-PULL FOR DIRECT CONTROL
			(1UL<<(LCD_RS_pin))
			|(1UL<<(LCD_RW_pin))
			|(1UL<(LCD_E_pin))
		);
		
	LCD_PORT->OTYPER |= (0xff<<(LCD_D0_pin));				//CONFIGURE D0-D7 AS OPEN-DRAIN FOR DIRECT CONTROL
	
	LCD_PORT->PUPDR &= ~(0xffff<<(2*LCD_D0_pin));				//CLEAR D0-D7 TO CONFIGURE AS PULL-UP
	
	LCD_PORT->PUPDR |= (0x5555<<(2*LCD_D0_pin));				//SET D0-D7 TO CONFIGURE AS PULL-UP

	
	//LCD INIT COMMANDS
	clr_LCD_RS();					//all lines default low
	clr_LCD_RW();
	clr_LCD_E();
	
	lcd_delayus(25000);		//25ms startup delay
	cmdLCD(0x38);	//Function set: 2 Line, 8-bit, 5x7 dots
	//cmdLCD(0x32);	//SWITCH TO 4-BIT MODE
	//cmdLCD(0x28);	//Function set: 2 Line, 4-bit, 5x7 dots
	cmdLCD(0x0c);	//Display on, Cursor blinking command
	cmdLCD(0x06);	//Entry mode, auto increment with no shift
	cmdLCD(0x01);	//Clear LCD
}

void INIT_TIM14(void)											//LCD TIMER
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;  		//Enable TIM7 clock
																					//APB clock is Fcy/2 = 180MHz/2 = 90MHz
	TIM14->PSC = 9000 - 1;   								//Prescaler: 90 MHz / 9000 = 10kHz
	TIM14->ARR = 10000 - 1;  									//Auto-reload: 10kHz / 10000 = 1Hz = 1s LCD update rate
	TIM14->DIER|=TIM_DIER_UIE;								//timer uptdate interrupt enabled
	TIM14->CNT=0;														//zero timer counter
	NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn); 							//ENABLE TIM7 IRQ
	TIM14->CR1|=TIM_CR1_CEN;									//start timer counter
}

void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
	if ((TIM14->SR & TIM_SR_UIF) != 0)   							//Check update interrupt flag
	{
		TIM14->SR &= ~TIM_SR_UIF;  											//Clear the flag
		cmdLCD(LCD_LINE1);															//WRITE ON THE FIRST LCD ROW, AND IN THE SAME POSITION
		putStrLCD("Heart Rt: 88BPM");
		//cmdLCD(LCD_LINE2);															//WRITE ON THE SECOND LCD ROW, AND IN THE SAME POSITION
		//putStrLCD("Oxygen Lvl: 96%");										
	}
}



/*
Demonstrate LCD functionality by performing the following:
•	The LCD should be interfaced to the Nucleo board using a 4-bit open-drain data bus and push-pull control lines and should monitor the busy flag to allow for reliable communications.
•	The Heartrate and Oxygen levels should be displayed on the top line of the LCD. Scrolling should be used if the message exceeds the screen length.
•	The bottom line of the LCD should be used to display the menu system to display the mode and outputs.
?	Advanced functionality - use of LCD features/graphics such as custom characters or animation are appropriately rewarded.
*/