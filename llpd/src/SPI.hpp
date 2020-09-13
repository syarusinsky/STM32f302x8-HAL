#include "LLPD.hpp"

#if defined( STM32F302X8 )
#include "stm32f302x8.h"
#elif defined( STM32F302XC )
#include "stm32f302xc.h"
#endif

static void setup_spi_registers (SPI_TypeDef* spiPtr, const SPI_BAUD_RATE& baudRate, const SPI_CLK_POL& pol,
				const SPI_CLK_PHASE& phase, const SPI_DUPLEX& duplex, const SPI_FRAME_FORMAT& frameFormat,
				const SPI_DATA_SIZE& dataSize, bool useNSS)
{
	if ( spiPtr )
	{
		// enable spi peripheral clock
		if ( spiPtr == SPI2 )
		{
			RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
		}
		else if ( spiPtr == SPI3 )
		{
			RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
		}

		// ensure spi is off
		spiPtr->CR1 &= ~(SPI_CR1_SPE);

		// reset registers
		if ( spiPtr == SPI2 )
		{
			RCC->APB2RSTR |= RCC_APB1RSTR_SPI2RST;
			RCC->APB2RSTR &= ~(RCC_APB1RSTR_SPI2RST);
		}
		else if ( spiPtr == SPI3 )
		{
			RCC->APB2RSTR |= RCC_APB1RSTR_SPI3RST;
			RCC->APB2RSTR &= ~(RCC_APB1RSTR_SPI3RST);
		}

		// set baud rate
		if ( baudRate == SPI_BAUD_RATE::SYSCLK_DIV_BY_2 )
		{
			spiPtr->CR1 &= ~(SPI_CR1_BR_0);
			spiPtr->CR1 &= ~(SPI_CR1_BR_1);
			spiPtr->CR1 &= ~(SPI_CR1_BR_2);
		}
		else if ( baudRate == SPI_BAUD_RATE::SYSCLK_DIV_BY_4 )
		{
			spiPtr->CR1 |= SPI_CR1_BR_0;
			spiPtr->CR1 &= ~(SPI_CR1_BR_1);
			spiPtr->CR1 &= ~(SPI_CR1_BR_2);
		}
		else if ( baudRate == SPI_BAUD_RATE::SYSCLK_DIV_BY_8 )
		{
			spiPtr->CR1 &= ~(SPI_CR1_BR_0);
			spiPtr->CR1 |= SPI_CR1_BR_1;
			spiPtr->CR1 &= ~(SPI_CR1_BR_2);
		}
		else if ( baudRate == SPI_BAUD_RATE::SYSCLK_DIV_BY_16 )
		{
			spiPtr->CR1 |= SPI_CR1_BR_0;
			spiPtr->CR1 |= SPI_CR1_BR_1;
			spiPtr->CR1 &= ~(SPI_CR1_BR_2);
		}
		else if ( baudRate == SPI_BAUD_RATE::SYSCLK_DIV_BY_32 )
		{
			spiPtr->CR1 &= ~(SPI_CR1_BR_0);
			spiPtr->CR1 &= ~(SPI_CR1_BR_1);
			spiPtr->CR1 |= SPI_CR1_BR_2;
		}
		else if ( baudRate == SPI_BAUD_RATE::SYSCLK_DIV_BY_64 )
		{
			spiPtr->CR1 |= SPI_CR1_BR_0;
			spiPtr->CR1 &= ~(SPI_CR1_BR_1);
			spiPtr->CR1 |= SPI_CR1_BR_2;
		}
		else if ( baudRate == SPI_BAUD_RATE::SYSCLK_DIV_BY_128 )
		{
			spiPtr->CR1 &= ~(SPI_CR1_BR_0);
			spiPtr->CR1 |= SPI_CR1_BR_1;
			spiPtr->CR1 |= SPI_CR1_BR_2;
		}
		else if ( baudRate == SPI_BAUD_RATE::SYSCLK_DIV_BY_256 )
		{
			spiPtr->CR1 |= SPI_CR1_BR_0;
			spiPtr->CR1 |= SPI_CR1_BR_1;
			spiPtr->CR1 |= SPI_CR1_BR_2;
		}

		// set clock polarity and phase
		if ( pol == SPI_CLK_POL::LOW_IDLE )
		{
			spiPtr->CR1 &= ~(SPI_CR1_CPOL);
		}
		else if ( pol == SPI_CLK_POL::HIGH_IDLE )
		{
			spiPtr->CR1 |= SPI_CR1_CPOL;
		}

		if ( phase == SPI_CLK_PHASE::FIRST )
		{
			spiPtr->CR1 &= ~(SPI_CR1_CPHA);
		}
		else if ( phase == SPI_CLK_PHASE::SECOND )
		{
			spiPtr->CR1 |= SPI_CR1_CPHA;
		}

		// set as master
		spiPtr->CR1 |= SPI_CR1_MSTR;

		// set duplex
		if ( duplex == SPI_DUPLEX::FULL )
		{
			spiPtr->CR1 &= ~(SPI_CR1_BIDIMODE);
			spiPtr->CR1 &= ~(SPI_CR1_RXONLY);
		}
		else if ( duplex == SPI_DUPLEX::HALF )
		{
			spiPtr->CR1 |= SPI_CR1_BIDIMODE;
			spiPtr->CR1 &= ~(SPI_CR1_RXONLY);
		}

		// set the frame format
		if ( frameFormat == SPI_FRAME_FORMAT::MSB_FIRST )
		{
			spiPtr->CR1 &= ~(SPI_CR1_LSBFIRST);
		}
		else if ( frameFormat == SPI_FRAME_FORMAT::LSB_FIRST )
		{
			spiPtr->CR1 |= SPI_CR1_LSBFIRST;
		}

		// ensure CRC is disabled
		spiPtr->CR1 &= ~(SPI_CR1_CRCEN);

		// enable NSS pin if desired
		if ( !useNSS ) // for when using gpio outputs as chip select pins
		{
			spiPtr->CR1 |= SPI_CR1_SSM;
			spiPtr->CR1 |= SPI_CR1_SSI;
			spiPtr->CR2 &= ~(SPI_CR2_SSOE);
		}
		else // for when using nss pin as chip select pin
		{
			spiPtr->CR1 &= ~(SPI_CR1_SSM);
			spiPtr->CR2 |= SPI_CR2_SSOE;
		}

		// set data size
		if ( dataSize == SPI_DATA_SIZE::BITS_4 )
		{
			spiPtr->CR2 |= SPI_CR2_DS_0;
			spiPtr->CR2 |= SPI_CR2_DS_1;
			spiPtr->CR2 &= ~(SPI_CR2_DS_2);
			spiPtr->CR2 &= ~(SPI_CR2_DS_3);
		}
		else if ( dataSize == SPI_DATA_SIZE::BITS_5 )
		{
			spiPtr->CR2 &= ~(SPI_CR2_DS_0);
			spiPtr->CR2 &= ~(SPI_CR2_DS_1);
			spiPtr->CR2 |= SPI_CR2_DS_2;
			spiPtr->CR2 &= ~(SPI_CR2_DS_3);
		}
		else if ( dataSize == SPI_DATA_SIZE::BITS_6 )
		{
			spiPtr->CR2 |= SPI_CR2_DS_0;
			spiPtr->CR2 &= ~(SPI_CR2_DS_1);
			spiPtr->CR2 |= SPI_CR2_DS_2;
			spiPtr->CR2 &= ~(SPI_CR2_DS_3);
		}
		else if ( dataSize == SPI_DATA_SIZE::BITS_6 )
		{
			spiPtr->CR2 &= ~(SPI_CR2_DS_0);
			spiPtr->CR2 |= SPI_CR2_DS_1;
			spiPtr->CR2 |= SPI_CR2_DS_2;
			spiPtr->CR2 &= ~(SPI_CR2_DS_3);
		}
		else if ( dataSize == SPI_DATA_SIZE::BITS_8 )
		{
			spiPtr->CR2 |= SPI_CR2_DS_0;
			spiPtr->CR2 |= SPI_CR2_DS_1;
			spiPtr->CR2 |= SPI_CR2_DS_2;
			spiPtr->CR2 &= ~(SPI_CR2_DS_3);
		}
		else if ( dataSize == SPI_DATA_SIZE::BITS_9 )
		{
			spiPtr->CR2 &= ~(SPI_CR2_DS_0);
			spiPtr->CR2 &= ~(SPI_CR2_DS_1);
			spiPtr->CR2 &= ~(SPI_CR2_DS_2);
			spiPtr->CR2 |= SPI_CR2_DS_3;
		}
		else if ( dataSize == SPI_DATA_SIZE::BITS_10 )
		{
			spiPtr->CR2 |= SPI_CR2_DS_0;
			spiPtr->CR2 &= ~(SPI_CR2_DS_1);
			spiPtr->CR2 &= ~(SPI_CR2_DS_2);
			spiPtr->CR2 |= SPI_CR2_DS_3;
		}
		else if ( dataSize == SPI_DATA_SIZE::BITS_11 )
		{
			spiPtr->CR2 &= ~(SPI_CR2_DS_0);
			spiPtr->CR2 |= SPI_CR2_DS_1;
			spiPtr->CR2 &= ~(SPI_CR2_DS_2);
			spiPtr->CR2 |= SPI_CR2_DS_3;
		}
		else if ( dataSize == SPI_DATA_SIZE::BITS_12 )
		{
			spiPtr->CR2 |= SPI_CR2_DS_0;
			spiPtr->CR2 |= SPI_CR2_DS_1;
			spiPtr->CR2 &= ~(SPI_CR2_DS_2);
			spiPtr->CR2 |= SPI_CR2_DS_3;
		}
		else if ( dataSize == SPI_DATA_SIZE::BITS_13 )
		{
			spiPtr->CR2 &= ~(SPI_CR2_DS_0);
			spiPtr->CR2 &= ~(SPI_CR2_DS_1);
			spiPtr->CR2 |= SPI_CR2_DS_2;
			spiPtr->CR2 |= SPI_CR2_DS_3;
		}
		else if ( dataSize == SPI_DATA_SIZE::BITS_14 )
		{
			spiPtr->CR2 |= SPI_CR2_DS_0;
			spiPtr->CR2 &= ~(SPI_CR2_DS_1);
			spiPtr->CR2 |= SPI_CR2_DS_2;
			spiPtr->CR2 |= SPI_CR2_DS_3;
		}
		else if ( dataSize == SPI_DATA_SIZE::BITS_15 )
		{
			spiPtr->CR2 &= ~(SPI_CR2_DS_0);
			spiPtr->CR2 |= SPI_CR2_DS_1;
			spiPtr->CR2 |= SPI_CR2_DS_2;
			spiPtr->CR2 |= SPI_CR2_DS_3;
		}
		else if ( dataSize == SPI_DATA_SIZE::BITS_16 )
		{
			spiPtr->CR2 |= SPI_CR2_DS_0;
			spiPtr->CR2 |= SPI_CR2_DS_1;
			spiPtr->CR2 |= SPI_CR2_DS_2;
			spiPtr->CR2 |= SPI_CR2_DS_3;
		}

		// set fifo reception threshold
		if ( dataSize == SPI_DATA_SIZE::BITS_8 ||
				dataSize == SPI_DATA_SIZE::BITS_7 ||
				dataSize == SPI_DATA_SIZE::BITS_6 ||
				dataSize == SPI_DATA_SIZE::BITS_5 ||
				dataSize == SPI_DATA_SIZE::BITS_4 )
		{
			spiPtr->CR2 |= SPI_CR2_FRXTH;
		}

		// lastly, enable SPI peripheral
		spiPtr->CR1 |= SPI_CR1_SPE;
	}
}

void LLPD::spi_master_init (const SPI_NUM& spiNum, const SPI_BAUD_RATE& baudRate, const SPI_CLK_POL& pol,
				const SPI_CLK_PHASE& phase, const SPI_DUPLEX& duplex, const SPI_FRAME_FORMAT& frameFormat,
				const SPI_DATA_SIZE& dataSize)
{
	SPI_TypeDef* spiPtr = nullptr;

	// enable clock to gpio port B (which is what port spi pins use)
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	// set up appropriate gpio
	if ( spiNum == SPI_NUM::SPI_2 )
	{
		// set alternate function registers to af5 for b13, b14, b15
		const int afValue = 5;
		const int afWidth = 4;
		const int afPin13 = 13 - 8; // minus 8 since we will modify the high register instead of low
		const int afPin14 = 14 - 8;
		const int afPin15 = 15 - 8;
		GPIOB->AFR[1] |= (afValue << (afPin13 * afWidth))
				| (afValue << (afPin14 * afWidth))
				| (afValue << (afPin15 * afWidth));

		// spi2 sck
		gpio_output_setup( GPIO_PORT::B, GPIO_PIN::PIN_13, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::HIGH, true );

		// spi2 miso
		gpio_digital_input_setup( GPIO_PORT::B, GPIO_PIN::PIN_14, GPIO_PUPD::PULL_UP, true );

		// spi2 mosi
		gpio_output_setup( GPIO_PORT::B, GPIO_PIN::PIN_15, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::HIGH, true );

		// set sck low
		gpio_output_set( GPIO_PORT::B, GPIO_PIN::PIN_13, false );

		spiPtr = SPI2;
	}
	else if ( spiNum == SPI_NUM::SPI_3 )
	{
		// set alternate function registers to af6 for b3, b4, b5
		const int afValue = 6;
		const int afWidth = 4;
		const int afPin3 = 3;
		const int afPin4 = 4;
		const int afPin5 = 5;
		GPIOB->AFR[0] |= (afValue << (afPin3 * afWidth))
				| (afValue << (afPin4 * afWidth))
				| (afValue << (afPin5 * afWidth));

		// spi3 sck
		gpio_output_setup( GPIO_PORT::B, GPIO_PIN::PIN_3, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::HIGH, true );

		// spi3 miso
		gpio_digital_input_setup( GPIO_PORT::B, GPIO_PIN::PIN_4, GPIO_PUPD::PULL_DOWN, true );

		// spi3 mosi
		gpio_output_setup( GPIO_PORT::B, GPIO_PIN::PIN_5, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::HIGH, true );

		// set sck low
		gpio_output_set( GPIO_PORT::B, GPIO_PIN::PIN_3, false );

		spiPtr = SPI3;
	}

	setup_spi_registers( spiPtr, baudRate, pol, phase, duplex, frameFormat, dataSize, false );
}

uint16_t LLPD::spi_master_send_and_recieve (const SPI_NUM& spiNum, uint8_t data)
{
	if ( spiNum == SPI_NUM::SPI_2 )
	{
		// spinlock until ready to send
		while ( !(SPI2->SR & SPI_SR_TXE) ) {}

		*(uint8_t*)&(SPI2->DR) = data;

		// spinlock until transmission complete
		while ( SPI2->SR & SPI_SR_BSY ) {}

		// spinlock until something to read
		while ( !(SPI2->SR & SPI_SR_RXNE) ) {}

		return SPI2->DR;
	}
	else if ( spiNum == SPI_NUM::SPI_3 )
	{
		// spinlock until ready to send
		while ( !(SPI3->SR & SPI_SR_TXE) ) {}

		*(uint8_t*)&(SPI3->DR) = data;

		// spinlock until transmission complete
		while ( SPI3->SR & SPI_SR_BSY ) {}

		// spinlock until something to read
		while ( !(SPI3->SR & SPI_SR_RXNE) ) {}

		return SPI3->DR;
	}

	return 0;
}
