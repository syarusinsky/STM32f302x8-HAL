#include "LLPD.hpp"

#if defined( STM32F302X8 )
#include "stm32f302x8.h"
#elif defined( STM32F302XC )
#include "stm32f302xc.h"
#endif

#include <cstdarg>

static void setI2CRegisters (I2C_TypeDef* i2cPtr, uint32_t timingRegVal)
{
	// set the i2c timing register
	i2cPtr->TIMINGR = (uint32_t)timingRegVal;

	// enable i2c peripheral
	i2cPtr->CR1 |= I2C_CR1_PE;
}

void LLPD::i2c_master_setup (const I2C_NUM& i2cNum, uint32_t timingRegVal)
{
	I2C_TypeDef* i2cPtr = nullptr;
	uint32_t rccI2CEnableBits = 0;

	if ( i2cNum == I2C_NUM::I2C_1 )
	{
		rccI2CEnableBits = RCC_APB1ENR_I2C1EN;

		i2cPtr = I2C1;

		// set alternate function registers to af4 for b7 and b6
		GPIOB->AFR[0] |= (4 << (7 * 4)) | (4 << (6 * 4));

		// i2c1 sda
		gpio_output_setup( GPIO_PORT::B, GPIO_PIN::PIN_7, GPIO_PUPD::NONE, GPIO_OUTPUT_TYPE::OPEN_DRAIN,
					GPIO_OUTPUT_SPEED::HIGH, true );

		// i2c1 scl
		gpio_output_setup( GPIO_PORT::B, GPIO_PIN::PIN_6, GPIO_PUPD::NONE, GPIO_OUTPUT_TYPE::OPEN_DRAIN,
					GPIO_OUTPUT_SPEED::HIGH, true );
	}
	else if ( i2cNum == I2C_NUM::I2C_2 )
	{
		rccI2CEnableBits = RCC_APB1ENR_I2C2EN;

		i2cPtr = I2C2;

		// set alternate function registers to af4 for a10 and a9
		GPIOA->AFR[1] |= (4 << (1 * 4)) | (4 << (2 * 4));

		// i2c1 sda
		gpio_output_setup( GPIO_PORT::A, GPIO_PIN::PIN_10, GPIO_PUPD::NONE, GPIO_OUTPUT_TYPE::OPEN_DRAIN,
					GPIO_OUTPUT_SPEED::HIGH, true );

		// i2c1 scl
		gpio_output_setup( GPIO_PORT::A, GPIO_PIN::PIN_9, GPIO_PUPD::NONE, GPIO_OUTPUT_TYPE::OPEN_DRAIN,
					GPIO_OUTPUT_SPEED::HIGH, true );
	}
#if defined( STM32F302X8 )
	else if ( i2cNum == I2C_NUM::I2C_3 )
	{
		rccI2CEnableBits = RCC_APB1ENR_I2C3EN;

		i2cPtr = I2C3;

		// set alternate function registers to af8 for b5 and af3 for a8
		GPIOB->AFR[0] |= (8 << (5 * 4));
		GPIOA->AFR[1] |= (3 << (0 * 4));

		// i2c1 sda
		gpio_output_setup( GPIO_PORT::B, GPIO_PIN::PIN_5, GPIO_PUPD::NONE, GPIO_OUTPUT_TYPE::OPEN_DRAIN,
					GPIO_OUTPUT_SPEED::HIGH, true );

		// i2c1 scl
		gpio_output_setup( GPIO_PORT::A, GPIO_PIN::PIN_8, GPIO_PUPD::NONE, GPIO_OUTPUT_TYPE::OPEN_DRAIN,
					GPIO_OUTPUT_SPEED::HIGH, true );
	}
#endif

	// enable i2c peripheral clock
	RCC->APB1ENR |= rccI2CEnableBits;

	setI2CRegisters( i2cPtr, timingRegVal );
}

void LLPD::i2c_master_set_slave_address (const I2C_NUM& i2cNum, const I2C_ADDR_MODE& addrMode, uint16_t address)
{
	if ( addrMode == I2C_ADDR_MODE::BITS_7 )
	{
		// mask address to ensure it's 7 bits max
		address = (address & 0b0000000001111111);

		// clear slave address register and set to new address
		switch ( i2cNum )
		{
			case I2C_NUM::I2C_1:
				I2C1->CR2 &= (0b11111111111111111111110000000000);
				I2C1->CR2 |= (address << 1);
				break;
			case I2C_NUM::I2C_2:
				I2C2->CR2 &= (0b11111111111111111111110000000000);
				I2C2->CR2 |= (address << 1);
				break;
#if defined( STM32F302X8 )
			case I2C_NUM::I2C_3:
				I2C3->CR2 &= (0b11111111111111111111110000000000);
				I2C3->CR2 |= (address << 1);
#endif
		}
	}
	else if ( addrMode == I2C_ADDR_MODE::BITS_10 )
	{
		// mask address to ensure it's 10 bits max
		address = (address & 0b0000001111111111);

		// clear slave address register and set to new address
		switch ( i2cNum )
		{
			case I2C_NUM::I2C_1:
				I2C1->CR2 &= (0b11111111111111111111110000000000);
				I2C1->CR2 |= address;
				break;
			case I2C_NUM::I2C_2:
				I2C2->CR2 &= (0b11111111111111111111110000000000);
				I2C2->CR2 |= address;
				break;
#if defined( STM32F302X8 )
			case I2C_NUM::I2C_3:
				I2C3->CR2 &= (0b11111111111111111111110000000000);
				I2C3->CR2 |= address;
#endif
		}
	}
}

void LLPD::i2c_master_write (const I2C_NUM& i2cNum, bool setStopCondition, uint8_t numBytes, uint8_t bytes...)
{
	I2C_TypeDef* i2cPtr = nullptr;

	// get correct i2c pointer
	if ( i2cNum == I2C_NUM::I2C_1 )
	{
		i2cPtr = I2C1;
	}
	else if ( i2cNum == I2C_NUM::I2C_2 )
	{
		i2cPtr = I2C2;
	}
#if defined( STM32F302X8 )
	else if ( i2cNum == I2C_NUM::I2C_3 )
	{
		i2cPtr = I2C3;
	}
#endif

	// set write flag
	i2cPtr->CR2 &= ~(I2C_CR2_RD_WRN);

	// set number of bytes to send
	i2cPtr->CR2 &= 0b11111111000000001111111111111111;
	i2cPtr->CR2 |= (numBytes << 16);

	// set start condition
	i2cPtr->CR2 |= I2C_CR2_START;

	// spinlock to ensure start condition and address bits have been sent
	while ( i2cPtr->CR2 & I2C_CR2_START ) {}

	// in case slave is busy
	while ( i2cPtr->ISR & I2C_ISR_NACKF )
	{
		// clear the stopf and nackf flags
		i2cPtr->ICR |= I2C_ICR_STOPCF;
		i2cPtr->ICR |= I2C_ICR_NACKCF;

		// restart the transfer
		i2cPtr->CR2 |= I2C_CR2_START;

		// spinlock to ensure start condition and address bits have been sent
		while ( i2cPtr->CR2 & I2C_CR2_START ) {}
	}

	// transfer bytes
	va_list args;
	va_start( args, bytes );

	// first transfer
	uint8_t data = bytes;
	i2cPtr->TXDR = data;

	// spinlock to ensure first transmission complete
	while ( !(i2cPtr->ISR & I2C_ISR_TXE) ) {}

	// subsequent transfers
	for ( uint8_t byte = 1; byte < numBytes; byte++ )
	{
		uint8_t data = static_cast<uint8_t>( va_arg(args, unsigned int) );
		i2cPtr->TXDR = data;

		// spinlock to ensure last transmission complete
		while ( !(i2cPtr->ISR & I2C_ISR_TXE) ) {}
	}

	va_end( args );

	// spinlock to ensure transfer is complete
	while ( !(i2cPtr->ISR & I2C_ISR_TC) ) {}

	if ( setStopCondition )
	{
		// set stop condition
		i2cPtr->CR2 |= I2C_CR2_STOP;

		// spinlock to ensure stop condition is detected and bus is not busy
		while ( (i2cPtr->CR2 & I2C_CR2_STOP) && (i2cPtr->ISR & I2C_ISR_BUSY) && (i2cPtr->ISR & I2C_ISR_STOPF) ) {}
	}
}

void LLPD::i2c_master_read (const I2C_NUM& i2cNum, bool setStopCondition, uint8_t numBytes, uint8_t* bytes...)
{
	I2C_TypeDef* i2cPtr = nullptr;

	// get correct i2c pointer
	if ( i2cNum == I2C_NUM::I2C_1 )
	{
		i2cPtr = I2C1;
	}
	else if ( i2cNum == I2C_NUM::I2C_2 )
	{
		i2cPtr = I2C2;
	}
#if defined( STM32F302X8 )
	else if ( i2cNum == I2C_NUM::I2C_3 )
	{
		i2cPtr = I2C3;
	}
#endif

	// set read flag
	i2cPtr->CR2 |= I2C_CR2_RD_WRN;

	// set number of bytes to read
	i2cPtr->CR2 &= 0b11111111000000001111111111111111;
	i2cPtr->CR2 |= (numBytes << 16);

	// set start condition
	i2cPtr->CR2 |= I2C_CR2_START;

	// spinlock to ensure start condition address bits have been sent
	while ( i2cPtr->CR2 & I2C_CR2_START ) {}

	// in case slave is busy
	while ( i2cPtr->ISR & I2C_ISR_NACKF )
	{
		// clear the stopf and nackf flags
		i2cPtr->ICR |= I2C_ICR_STOPCF;
		i2cPtr->ICR |= I2C_ICR_NACKCF;

		// restart the transfer
		i2cPtr->CR2 |= I2C_CR2_START;

		// spinlock to ensure start condition and address bits have been sent
		while ( i2cPtr->CR2 & I2C_CR2_START ) {}
	}

	// read bytes
	va_list args;
	va_start( args, bytes );

	// spinlock to ensure recieve data register has something to read
	while ( !(i2cPtr->ISR & I2C_ISR_RXNE) ) {}

	// first read
	uint8_t* data = bytes;
	*data = i2cPtr->RXDR;

	// subsequent reads
	for ( uint8_t byte = 1; byte < numBytes; byte++ )
	{
		// spinlock to ensure recieve data register has something to read
		while ( !(i2cPtr->ISR & I2C_ISR_RXNE) ) {}

		uint8_t* data = va_arg(args, uint8_t*);
		*data = i2cPtr->RXDR;
	}

	va_end( args );

	// spinlock to ensure transfer is complete
	while ( !(i2cPtr->ISR & I2C_ISR_TC) ) {}

	if ( setStopCondition )
	{
		// set stop condition
		i2cPtr->CR2 |= I2C_CR2_STOP;

		// spinlock to ensure stop condition is detected and bus is not busy
		while ( (i2cPtr->CR2 & I2C_CR2_STOP) && (i2cPtr->ISR & I2C_ISR_BUSY) && (i2cPtr->ISR & I2C_ISR_STOPF) ) {}
	}
}

void LLPD::i2c_master_read_into_array (const I2C_NUM& i2cNum, bool setStopCondition, uint8_t numBytes, uint8_t* arr)
{
	I2C_TypeDef* i2cPtr = nullptr;

	// get correct i2c pointer
	if ( i2cNum == I2C_NUM::I2C_1 )
	{
		i2cPtr = I2C1;
	}
	else if ( i2cNum == I2C_NUM::I2C_2 )
	{
		i2cPtr = I2C2;
	}
#if defined( STM32F302X8 )
	else if ( i2cNum == I2C_NUM::I2C_3 )
	{
		i2cPtr = I2C3;
	}
#endif

	// set read flag
	i2cPtr->CR2 |= I2C_CR2_RD_WRN;

	// set number of bytes to read
	i2cPtr->CR2 &= 0b11111111000000001111111111111111;
	i2cPtr->CR2 |= (numBytes << 16);

	// set start condition
	i2cPtr->CR2 |= I2C_CR2_START;

	// spinlock to ensure start condition address bits have been sent
	while ( i2cPtr->CR2 & I2C_CR2_START ) {}

	// in case slave is busy
	while ( i2cPtr->ISR & I2C_ISR_NACKF )
	{
		// clear the stopf and nackf flags
		i2cPtr->ICR |= I2C_ICR_STOPCF;
		i2cPtr->ICR |= I2C_ICR_NACKCF;

		// restart the transfer
		i2cPtr->CR2 |= I2C_CR2_START;

		// spinlock to ensure start condition and address bits have been sent
		while ( i2cPtr->CR2 & I2C_CR2_START ) {}
	}

	// spinlock to ensure recieve data register has something to read
	while ( !(i2cPtr->ISR & I2C_ISR_RXNE) ) {}

	// read data
	for ( uint8_t byte = 0; byte < numBytes; byte++ )
	{
		// spinlock to ensure recieve data register has something to read
		while ( !(i2cPtr->ISR & I2C_ISR_RXNE) ) {}

		arr[byte] = i2cPtr->RXDR;
	}

	// spinlock to ensure transfer is complete
	while ( !(i2cPtr->ISR & I2C_ISR_TC) ) {}

	if ( setStopCondition )
	{
		// set stop condition
		i2cPtr->CR2 |= I2C_CR2_STOP;

		// spinlock to ensure stop condition is detected and bus is not busy
		while ( (i2cPtr->CR2 & I2C_CR2_STOP) && (i2cPtr->ISR & I2C_ISR_BUSY) && (i2cPtr->ISR & I2C_ISR_STOPF) ) {}
	}
}
