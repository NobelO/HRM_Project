#ifndef _USART_H
#define _USART_H
#include <stm32f4xx.h>

#define USART_MODULE	USART3    //USART3 to be used for lower power consumption
#define USART_PORT		GPIOD     //USART 3 is GPIOD port
#define USART_TX_pin	8         //Pin is correct 
#define USART_RX_pin	9         //Pin is correct
#define BAUDRATE			9600    //Baud Rate set to 128000


void send_usart(unsigned char d);    //Send a byte of data over USART
void init_usart(void);               //Set the baud rate, GPIO pins and clocks for USART

#endif
