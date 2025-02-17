#include <stm32f429xx.h>		//INCLUDE THE HEADER FILE FOR THIS MCU FAMILY
													//this file contains the definitions for register addresses and values etc
													
#define BMP280_CS 2
#define SPI1_SCK 3
#define SPI1_MISO 4
#define SPI1_MOSI 5



void CONFIG_GPIO_FOR_SPI(void);
void INIT_SPI(void);
void TEMP_PRES_READER_INIT(void);
void SPI_ENABLE(void);
void SPI_DISABLE(void);
void CS_ENABLE(void);
void CS_DISABLE(void);
void SPI_TRANSMIT(uint8_t *data, int size);
void SPI_RECEIVE(uint8_t *data, int size);
void ADXL_WRITE(uint8_t address, uint8_t value);
void ADXL_READ(uint8_t address);
void ADXL_INIT(void);






