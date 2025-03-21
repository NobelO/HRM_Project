#include "Usart.h"

void init_usart(void)
{
	
	//delcare and define UTX/RTX pin labels
 unsigned char UTx,URx;                   //define USART TX/RX pin labels
  UTx = USART_TX_pin / 8;  // Determine AFR register for TX
  URx = USART_RX_pin << 3;
	
	//Clock Enables
		RCC->AHB1ENR|=RCC_AHB1ENR_GPIODEN;		//GPIOD clock enable
	  RCC->APB1ENR|=RCC_APB1ENR_USART3EN;		//Usart clock enable
	//  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;   //DMA Clock Enable 
	
   USART_PORT->MODER &= ~(
    (3u << (2 * USART_TX_pin)) |  // Clear mode bits for TX pin
    (3u << (2 * USART_RX_pin))    // Clear mode bits for RX pin
   );
    USART_PORT->MODER |= (
        (2u << (2 * USART_TX_pin)) | // AF mode for TX
        (2u << (2 * USART_RX_pin))  // AF mode for RX
    );

		// ALTERNATE FUNCTION SELECT BITS
    USART_PORT->AFR[UTx] &= ~(0x0fu << (4 * (USART_TX_pin - (UTx * 8))));  // clear pin function
    USART_PORT->AFR[UTx] |=  (0x07u << (4 * (USART_TX_pin - (UTx * 8))));  // set USART as alternate function for TX_pin
    USART_PORT->AFR[URx] &= ~(0x0fu << (4 * (USART_RX_pin - (URx * 8))));  // clear pin function
    USART_PORT->AFR[URx] |=  (0x07u << (4 * (USART_RX_pin - (URx * 8))));  // set USART as alternate function for RX_pin
	
  //Enable DMA feature in Usart 3
 //	USART3->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR;  // Enable DMA mode for TX and RX


	USART_MODULE->CR1|=(										//USART CONFIG
			USART_CR1_TE												//transmit enable
			|USART_CR1_RE												//receive enable
			|USART_CR1_UE												//usart main enable bit
				);
		//		NVIC_EnableIRQ(USART3_IRQn);     //enable the TXE handler function
	//	USART3->BRR = (45000000 /230400);		//BAUDRATE rate set but not EXACT as decimal portion is discarded, it is within the margin of error though
	// USART3->BRR = 45000000 / BAUDRATE;
	int temp = (((SystemCoreClock/4)<<5) / (16 * 230400)); //twice BRR needed for LSB accuracy 0.5%
  USART_MODULE->BRR = (temp>>1) + (temp&0x1);
	
	}	 



void send_usart(unsigned char d)
{
	while(!(USART_MODULE->SR & USART_SR_TXE));
	USART_MODULE->DR=d;												//write byte to usart data register
}
	
	
