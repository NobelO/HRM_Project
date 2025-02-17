#include <stm32f429xx.h>


#define BUZZ_PIN 13

void INIT_BUZZ(void);
void MUS_NOTES_DELAY(uint32_t s, uint32_t t);
void TOGGLE_BUZZER(void);
void NOTE_Fs(uint32_t octaves, int length);
void NOTE_F(uint32_t octaves, int length);
void START_BEEP(void);
void END_BEEP(void);
void HEART_RATE_BEEP(void);
void OXG_LVL_BEEP(void);
void TEMP_BEEP(void);
void HUMIDITY_BEEP(void);
void MOVEMENT_BEEP(void);
void AEOY(void);

void TIM3_Init_Interrupt(void);
void TIM3_Delay(uint16_t delay_ms, uint32_t t);
void TIM3_IRQHandler(void);
