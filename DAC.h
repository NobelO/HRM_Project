#include <stm32f429xx.h>		//INCLUDE THE HEADER FILE FOR THIS MCU FAMILY
#ifndef _DAC_H
#define _DAC_H


#define DAC1_Pin 4
#define DAC2_Pin 5
#define PI 3.14159265359

// Sine wave parameters
#define SINE_SAMPLES    100       // Number of points in the sine wave
#define SINE_AMPLITUDE  2047      // Amplitude (12-bit max is 4095; half scale is 2047)
#define SINE_OFFSET     2048      // Offset to center the wave (12-bit DAC is 0–4095)

#define WAVE_SAMPLES 100  					// Number of points in one wave cycle
#define S_WAVE_SAMPLES 2					// Number of points in one wave cycle



void INIT_DAC(void);
void DAC1_OUTPUT(unsigned short val);
void DAC2_OUTPUT(unsigned short val);

void generateSineWave(void);
void generateTriangleWave(void);
void generateComplexWave(void);

void Init_Timer2_1s(void);
void Init_Timer4_333ms(void);

void INIT_TIM4(void);
void INIT_TIM5(void);
void INIT_TIM6(void);
void INIT_TIM7(void);
	
#endif