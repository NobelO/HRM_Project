#include "SPI.h"


uint8_t RxData[6];
uint8_t data[3] = {0x02, 0x32, 0xf6};

// Start sequence in master and slave mode
//In full-duplex (BIDIMODE=0 and RXONLY=0)
//In bidirectional mode, when transmitting (BIDIMODE=1 and BIDIOE=1)
//In bidirectional mode, when receiving (BIDIMODE=1 and BIDIOE=0)
//SPI status register (SPI_SR)
//Check from page 888, 922 & 920


void CONFIG_GPIO_FOR_SPI(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;		//ONLY ENABLE GPIO B
	
	GPIOB->MODER &= ~(1UL<<(2*BMP280_CS));	//ONLY CLEAR (PB2) TO CONFIGURE AS ALTERNATE FUNCTION
	GPIOB->MODER &= ~(1UL<<(2*SPI1_SCK));		//ONLY CLEAR (PB3) TO CONFIGURE AS ALTERNATE FUNCTION
	GPIOB->MODER &= ~(1UL<<(2*SPI1_MISO));	//ONLY CLEAR (PB4) TO CONFIGURE AS ALTERNATE FUNCTION
	GPIOB->MODER &= ~(1UL<<(2*SPI1_MOSI));	//ONLY CLEAR (PB5) TO CONFIGURE AS ALTERNATE FUNCTION
	
	GPIOB->MODER |= (2UL<<(2*BMP280_CS));		//ONLY SET (PB2) TO CONFIGURE AS ALTERNATE FUNCTION
	GPIOB->MODER |= (2UL<<(2*SPI1_SCK));		//ONLY SET (PB3) TO CONFIGURE AS ALTERNATE FUNCTION
	GPIOB->MODER |= (2UL<<(2*SPI1_MISO));		//ONLY SET (PB4) TO CONFIGURE AS ALTERNATE FUNCTION
	GPIOB->MODER |= (2UL<<(2*SPI1_MOSI));		//ONLY SET (PB5) TO CONFIGURE AS ALTERNATE FUNCTION
	
	GPIOB->OSPEEDR |= (3UL<<(2*BMP280_CS));		//ONLY SET (PB2) TO CONFIGURE AS HIGH SPEED
	GPIOB->OSPEEDR |= (3UL<<(2*SPI1_SCK));		//ONLY SET (PB3) TO CONFIGURE AS HIGH SPEED
	GPIOB->OSPEEDR |= (3UL<<(2*SPI1_MISO));		//ONLY SET (PB4) TO CONFIGURE AS HIGH SPEED
	GPIOB->OSPEEDR |= (3UL<<(2*SPI1_MOSI));		//ONLY SET (PB5) TO CONFIGURE AS HIGH SPEED
	
	GPIOB->AFR[0] |= (5UL<<(4*SPI1_SCK));			//ONLY SET (PB3) TO CONFIGURE FOR SPI
	GPIOB->AFR[0] |= (5UL<<(4*SPI1_MISO));		//ONLY SET (PB4) TO CONFIGURE FOR SPI
	GPIOB->AFR[0] |= (5UL<<(4*SPI1_MOSI));		//ONLY SET (PB5) TO CONFIGURE FOR SPI
}

void TEMP_PRES_READER_INIT(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;		//ONLY ENABLE GPIO B
	
	GPIOB->MODER |= 1UL<<(2*6);							//ONLY set GPIOB (PB6)***
	GPIOB->MODER |= 1UL<<(2*6);							//ONLY set GPIOB (PB6)***
}



	//SPI4_IRQn                   = 84,     /*!< SPI4 global Interrupt                                             */
  //SPI5_IRQn                   = 85,     /*!< SPI5 global Interrupt                                             */
  //SPI6_IRQn                   = 86,     /*!< SPI6 global Interrupt                                             */


void INIT_SPI(void)
{
	/************** STEPS TO FOLLOW *****************
	1. Enable SPI clock
	2. Configure the Control Register 1
	3. Configure the CR2
	************************************************/	
	
	RCC->APB2ENR |= (RCC_APB2ENR_SPI1EN);  // Enable SPI1 CLock
	
	//SPI1->CR1 |= (1<<0)|(1<<1);   // CPOL=1, CPHA=1
	
	SPI1->CR1 |= (1<<2);  // Master Mode
	
	SPI1->CR1 |= (3<<3);  // BR[2:0] = 011: fPCLK/16, PCLK2 = 80MHz, SPI clk = 5MHz
	
	SPI1->CR1 &= ~(1<<7);  // LSBFIRST = 0, MSB first
	
	SPI1->CR1 |= (1<<8) | (1<<9);  // SSM=1, SSi=1 -> Software Slave Management
	
	SPI1->CR1 &= ~(1<<10);  // RXONLY = 0, full-duplex
	
	SPI1->CR1 &= ~(1<<11);  // DFF=0, 8 bit data
	
	SPI1->CR2 = 0;
	// 3. Configure SPI1 (Master Mode, 8-bit, Baud Rate = fPCLK/16)
    //SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_BR_1 | SPI_CR1_SSI | SPI_CR1_SSM; 
    //SPI1->CR1 |= SPI_CR1_SPE;  // Enable SPI1
}


void SPI_ENABLE(void)
{
	SPI1->CR1 |= (1<<6);   // SPE=1, Peripheral enabled
}

void SPI_DISABLE(void)
{
	SPI1->CR1 &= ~(1<<6);   // SPE=0, Peripheral Disabled
}

void CS_ENABLE(void)
{
	GPIOB->BSRR = (1<<2)<<16; //***
}

void CS_DISABLE(void)
{
	GPIOB->BSRR = (1<<2);		//***
}


void SPI_TRANSMIT(uint8_t *data, int size)
{
	
	/************** STEPS TO FOLLOW *****************
	1. Wait for the TXE bit to set in the Status Register
	2. Write the data to the Data Register
	3. After the data has been transmitted, wait for the BSY bit to reset in Status Register
	4. Clear the Overrun flag by reading DR and SR
	************************************************/		
	
	int i=0;
	while (i<size)
	{
	   while (!((SPI1->SR)&(1<<1))) {};  // wait for TXE bit to set -> This will indicate that the buffer is empty
	   SPI1->DR = data[i];  // load the data into the Data Register
	   i++;
	}	
	
	
/*During discontinuous communications, there is a 2 APB clock period delay between the
write operation to the SPI_DR register and BSY bit setting. As a consequence it is
mandatory to wait first until TXE is set and then until BSY is cleared after writing the last
data.
*/
	while (!((SPI1->SR)&(1<<1))) {};  // wait for TXE bit to set -> This will indicate that the buffer is empty
	while (((SPI1->SR)&(1<<7))) {};  // wait for BSY bit to Reset -> This will indicate that SPI is not busy in communication	
	
	//  Clear the Overrun flag by reading DR and SR
	//uint8_t temp = SPI1->DR;
	//				temp = SPI1->SR;
	
}

void SPI_RECEIVE(uint8_t *data, int size)
{
	/************** STEPS TO FOLLOW *****************
	1. Wait for the BSY bit to reset in Status Register
	2. Send some Dummy data before reading the DATA
	3. Wait for the RXNE bit to Set in the status Register
	4. Read data from Data Register
	************************************************/		

	while(size)
	{
		while (((SPI1->SR)&(1<<7))) {};  // wait for BSY bit to Reset -> This will indicate that SPI is not busy in communication
		SPI1->DR = 0;  // send dummy data
		while (!((SPI1->SR) &(1<<0))){};  // Wait for RXNE to set -> This will indicate that the Rx buffer is not empty
	  *data++ = (SPI1->DR);
		size--;
	}	
}
	
void ADXL_WRITE(uint8_t address, uint8_t value)
{
	uint8_t data[2];
	data[0] = address|0x40;  // multibyte write
	data[1] = value;
	CS_ENABLE();  // pull the cs pin low
	SPI_TRANSMIT(data, 2);  // write data to register
	CS_DISABLE();  // pull the cs pin high
}
	

void ADXL_READ(uint8_t address)
{
	address |= 0x80;  // read operation
	address |= 0x40;  // multibyte read
	//uint8_t rec;
	CS_ENABLE();  // pull the pin low
	SPI_TRANSMIT(&address, 1);  // send address
	SPI_RECEIVE(RxData, 6);  // receive 6 bytes data
	CS_DISABLE();;  // pull the pin high
}

void ADXL_INIT(void)
{
	ADXL_WRITE(0x31, 0x01);  // data_format range= +- 4g
	ADXL_WRITE(0x2d, 0x00);  // reset all bits
	ADXL_WRITE(0x2d, 0x08);  // power_cntl measure and wake up 8hz
}
	
	

	//SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                             */
  //SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                             */
  



/*
Demonstrate SPI functionality by performing the following:
•	Initialisation setup GPIO / Registers for I2C
•	Read IMU 3DOF  / identify movement
•	Use Interrupts to manage priority
•	Capture waveform showing SPI transfer for known data and label
*/




/*

#include "mbed.h"
#include "BMP280_SPI.h"

BMP280_SPI::BMP280_SPI(PinName mosi, PinName miso, PinName sclk, PinName cs)
    :
    _spi(mosi, miso, sclk),
    _cs(cs),
    t_fine(0)
{
    //initialize();
}


BMP280_SPI::~BMP280_SPI()
{
}

void BMP280_SPI::initialize()
{
    char cmd[18];

    _cs = 1;
    _spi.format(8, 0); // 8-bit, mode=0
    _spi.frequency(1000000); // 1MHZ

    _cs = 0;
    _spi.write(0xd0); // read chip_id
    cmd[0] = _spi.write(0); // read chip_id
    _cs = 1;
    
    DEBUG_PRINT("chip_id = 0x%x\n", cmd[0]);

    _cs = 0;
    _spi.write(0xf4 & BMP280_SPI_WRITE); // ctrl_meas
    _spi.write((3<<5) | (3<<2) | 3); // Temparature oversampling x4, Pressure oversampling x4, Normal mode
    _cs = 1;

    _cs = 0;
    _spi.write(0xf5 & BMP280_SPI_WRITE); // config
    _spi.write((5<<5) | (0<<2) | 0); // Standby 1000ms, Filter off, 4-wire SPI interface
    _cs = 1;

    wait_us(1000000);
    
    _cs = 0;
    _spi.write(0x88); // read dig_T regs
    for(int i = 0; i < 6; i++)
        cmd[i] = _spi.write(0);
    _cs = 1;

    dig_T1 = (cmd[1] << 8) | cmd[0];
    dig_T2 = (cmd[3] << 8) | cmd[2];
    dig_T3 = (cmd[5] << 8) | cmd[4];

    DEBUG_PRINT("dig_T = 0x%x, 0x%x, 0x%x\n", dig_T1, dig_T2, dig_T3);
    DEBUG_PRINT("dig_T = %d, %d, %d\n", dig_T1, dig_T2, dig_T3);

    _cs = 0;
    _spi.write(0x8e); // read dig_P regs
    for(int i = 0; i < 18; i++)
        cmd[i] = _spi.write(0);
    _cs = 1;

    dig_P1 = (cmd[ 1] << 8) | cmd[ 0];
    dig_P2 = (cmd[ 3] << 8) | cmd[ 2];
    dig_P3 = (cmd[ 5] << 8) | cmd[ 4];
    dig_P4 = (cmd[ 7] << 8) | cmd[ 6];
    dig_P5 = (cmd[ 9] << 8) | cmd[ 8];
    dig_P6 = (cmd[11] << 8) | cmd[10];
    dig_P7 = (cmd[13] << 8) | cmd[12];
    dig_P8 = (cmd[15] << 8) | cmd[14];
    dig_P9 = (cmd[17] << 8) | cmd[16];

    DEBUG_PRINT("dig_P = 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n", dig_P1, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9);
    DEBUG_PRINT("dig_P = %d, %d, %d, %d, %d, %d, %d, %d, %d\n", dig_P1, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9);

}

float BMP280_SPI::getTemperature()
{
    int32_t temp_raw, var1, var2, temp;
    float tempf;
    char cmd[3];

    _cs = 0;
    _spi.write(0xfa);
    for(int i = 0; i < 3; i++)
        cmd[i] = _spi.write(0);
    _cs = 1;

    temp_raw = (cmd[0] << 12) | (cmd[1] << 4) | (cmd[2] >> 4);

    var1 = ((((temp_raw >> 3) - (dig_T1 << 1))) * dig_T2) >> 11;
    var2 = (((((temp_raw >> 4) - dig_T1) * ((temp_raw >> 4) - dig_T1)) >> 12) * dig_T3) >> 14;

    t_fine = var1 + var2;
    temp = (t_fine * 5 + 128) >> 8;
    tempf = (float)temp;

    return (tempf/100.0f);
}

float BMP280_SPI::getPressure()
{
    int32_t press_raw;
    float pressf;
    char cmd[3];

    _cs = 0;
    _spi.write(0xf7); // press_msb
    for(int i = 0; i < 3; i++)
        cmd[i] = _spi.write(0);
    _cs = 1;

    press_raw = (cmd[0] << 12) | (cmd[1] << 4) | (cmd[2] >> 4);

    int64_t var1, var2, p;
    uint32_t press;

    var1 = (int64_t)t_fine - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + ((int64_t)dig_P4 << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) + ((var1 * (int64_t)dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;
    
    if (var1 == 0) {
        return 0;
    }
    p = 1048576 - press_raw;
    p = (((p<<31)-var2)*3125)/var1;
    var1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);
    press = (uint32_t)p/256;

    pressf = (float)press;
    return (pressf/100.0f);
}
*/

/*


 
#ifndef MBED_BMP280_SPI_H
#define MBED_BMP280_SPI_H

#include "mbed.h"

#ifdef _DEBUG
//extern Serial pc;
#define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif
*/

/**  Interface for controlling BMP280 pressure sensor
 *
 * @code
 * #include "mbed.h"
 * #include "BMP280_SPI.h"
 *
 * Serial pc(USBTX, USBRX);
 * BMP280_SPI sensor(D11, D12, D13, D9); // mosi, miso, sclk, cs
 *
 * int main() {
 *
 *     while(1) {
 *         pc.printf("%2.2f degC, %04.2f hPa\n", sensor.getTemperature(), sensor.getPressure());
 *         wait(1);
 *     }
 * }
 *
 * @endcode
 */

/** BMP280_SPI class
 *
 *  BMP280_SPI: A library to correct data using Boshe BMP280 pressure sensor device
 *
 */
 
 /*
class BMP280_SPI
{
public:

    enum spi_mask {
        BMP280_SPI_WRITE = 0x7F
    };

*/

    //BMP280_SPI(PinName mosi, PinName miso, PinName sclk, PinName cs);

    /** Destructor of BMP280_SPI
     */
    //virtual ~BMP280_SPI();

    /** Initializa BMP280 sensor
     *
     *  Configure sensor setting and read parameters for calibration
     *
     */
    //void initialize(void);

    /** Read the current temperature value (degree Celsius) from BMP280 sensor
     *
     * @return Temperature value (degree Celsius)
     */
    //float getTemperature(void);

    /** Read the current pressure value (hectopascal) from BMP280 sensor
     *
     * @return Pressure value (hectopascal)
     */
    //float getPressure(void);
/*
private:

    SPI         _spi;
    DigitalOut  _cs;
    uint16_t    dig_T1;
    int16_t     dig_T2, dig_T3;
    uint16_t    dig_P1;
    int16_t     dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    int32_t     t_fine;

};

#endif // MBED_BMP280_SPI_H
*/