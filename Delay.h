#ifndef __delay_h
#define __delay_h
#include <stm32f429xx.h>

void init_Timer6(void);
void TIM6_DAC_IRQHandler(void);
void ADC_IRQHandler(void);

extern volatile unsigned short BPM;

#endif
