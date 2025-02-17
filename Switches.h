#ifndef SWITCHES_H
#define SWITCHES_H
#include <stm32f429xx.h>		//INCLUDE THE HEADER FILE FOR THIS MCU FAMILY




#define blueButton 13
#define buttonA 0
#define buttonB 1
#define buttonC 2
#define buttonD 3

#define LED_Toggling 7

//extern uint32_t value;
 //volatile uint8_t hold_value = 0;


void BUTTON_INIT(void);
void SWITCH_BOUNCE_DELAY(uint32_t t);
void wait_us(uint32_t n);
void Init_Timer3_500ms(void);
void Init_Timer3_2s(void);
void INIT_EXTI13(void);
void EXTI15_10_IRQHandler(void);
void TIM3_IRQHandler(void);
void INIT_EXTI0(void);
void EXTI0_IRQHandler(void);
void INIT_EXTI1(void);
void EXTI1_IRQHandler(void);
void INIT_EXTI2(void);
void EXTI2_IRQHandler(void);
void INIT_EXTI3(void);
void EXTI3_IRQHandler(void);
void TIM13_Init_Interrupt(void);
void TIM13_Delay(uint16_t delay_ms);
void TIM8_UP_TIM13_IRQHandler(void);

void INIT_TIM12(void);
void TIM8_BRK_TIM12_IRQHandler(void);

void BUTTON_PRESS(void);
void SWITCH_ON(char pin);
void SWITCH_OFF(char pin);
#endif