#include "LLPD.hpp"

#if defined( STM32F302X8 )
#include "stm32f302x8.h"
#elif defined( STM32F302XC )
#include "stm32f302xc.h"
#endif

static void setup_spi_registers (SPI_TypeDef* spiPtr, const SPI_BAUD_RATE& baudRate, const SPI_CLK_POL& pol,
				const SPI_CLK_PHASE& phase, const SPI_DUPLEX& duplex, const SPI_FRAME_FORMAT& frameFormat,
				const SPI_DATA_SIZE& dataSize, bool useNSS, const bool enableDmaChannels,
				const bool slaveSetup = false)
{
	if ( spiPtr )
	{
		// enable spi peripheral clock
		if ( spiPtr == SPI1 )
		{
			RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
		}
		else if ( spiPtr == SPI2 )
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
		if ( spiPtr == SPI1 )
		{
			RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
			RCC->APB2RSTR &= ~(RCC_APB2RSTR_SPI1RST);
		}
		else if ( spiPtr == SPI2 )
		{
			RCC->APB1RSTR |= RCC_APB1RSTR_SPI2RST;
			RCC->APB1RSTR &= ~(RCC_APB1RSTR_SPI2RST);
		}
		else if ( spiPtr == SPI3 )
		{
			RCC->APB1RSTR |= RCC_APB1RSTR_SPI3RST;
			RCC->APB1RSTR &= ~(RCC_APB1RSTR_SPI3RST);
		}

		// set baud rate
		if ( ! slaveSetup )
		{
			if ( baudRate == SPI_BAUD_RATE::APB1CLK_DIV_BY_2 )
			{
				spiPtr->CR1 &= ~(SPI_CR1_BR_0);
				spiPtr->CR1 &= ~(SPI_CR1_BR_1);
				spiPtr->CR1 &= ~(SPI_CR1_BR_2);
			}
			else if ( baudRate == SPI_BAUD_RATE::APB1CLK_DIV_BY_4 )
			{
				spiPtr->CR1 |= SPI_CR1_BR_0;
				spiPtr->CR1 &= ~(SPI_CR1_BR_1);
				spiPtr->CR1 &= ~(SPI_CR1_BR_2);
			}
			else if ( baudRate == SPI_BAUD_RATE::APB1CLK_DIV_BY_8 )
			{
				spiPtr->CR1 &= ~(SPI_CR1_BR_0);
				spiPtr->CR1 |= SPI_CR1_BR_1;
				spiPtr->CR1 &= ~(SPI_CR1_BR_2);
			}
			else if ( baudRate == SPI_BAUD_RATE::APB1CLK_DIV_BY_16 )
			{
				spiPtr->CR1 |= SPI_CR1_BR_0;
				spiPtr->CR1 |= SPI_CR1_BR_1;
				spiPtr->CR1 &= ~(SPI_CR1_BR_2);
			}
			else if ( baudRate == SPI_BAUD_RATE::APB1CLK_DIV_BY_32 )
			{
				spiPtr->CR1 &= ~(SPI_CR1_BR_0);
				spiPtr->CR1 &= ~(SPI_CR1_BR_1);
				spiPtr->CR1 |= SPI_CR1_BR_2;
			}
			else if ( baudRate == SPI_BAUD_RATE::APB1CLK_DIV_BY_64 )
			{
				spiPtr->CR1 |= SPI_CR1_BR_0;
				spiPtr->CR1 &= ~(SPI_CR1_BR_1);
				spiPtr->CR1 |= SPI_CR1_BR_2;
			}
			else if ( baudRate == SPI_BAUD_RATE::APB1CLK_DIV_BY_128 )
			{
				spiPtr->CR1 &= ~(SPI_CR1_BR_0);
				spiPtr->CR1 |= SPI_CR1_BR_1;
				spiPtr->CR1 |= SPI_CR1_BR_2;
			}
			else if ( baudRate == SPI_BAUD_RATE::APB1CLK_DIV_BY_256 )
			{
				spiPtr->CR1 |= SPI_CR1_BR_0;
				spiPtr->CR1 |= SPI_CR1_BR_1;
				spiPtr->CR1 |= SPI_CR1_BR_2;
			}
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

		// set as master or slave
		if ( ! slaveSetup )
		{
			spiPtr->CR1 |= SPI_CR1_MSTR;
		}
		else
		{
			spiPtr->CR1 &= ~(SPI_CR1_MSTR);
		}

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
		if ( useNSS ) // for when using gpio outputs as chip select pins
		{
			spiPtr->CR1 |= SPI_CR1_SSM;
			spiPtr->CR1 |= SPI_CR1_SSI; // to avoid MODF error
			spiPtr->CR2 &= ~(SPI_CR2_SSOE);
		}
		else // for when using nss pin as chip select pin
		{
			spiPtr->CR1 &= ~(SPI_CR1_SSM);
			if ( slaveSetup )
			{
				spiPtr->CR2 &= ~(SPI_CR2_SSOE);
			}
			else
			{
				spiPtr->CR2 |= SPI_CR2_SSOE;
			}
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
		else
		{
			spiPtr->CR2 &= ~(SPI_CR2_FRXTH);
		}

		if ( spiPtr == SPI2 && enableDmaChannels )
		{
			// enable dma1 clock
			RCC->AHBENR |= RCC_AHBENR_DMA1EN;

			// setup rx dma stream
			// disable channel
			DMA1_Channel4->CCR &= ~(DMA_CCR_EN);

			// configure channel priority to low
			DMA1_Channel4->CCR &= ~(DMA_CCR_PL);

			// set data transfer direction from peripheral to memory
			DMA1_Channel4->CCR &= DMA_CCR_DIR;

			// ensure peripheral incrementing is off
			DMA1_Channel4->CCR &= ~(DMA_CCR_PINC);

			// enable memory incrementing
			DMA1_Channel4->CCR |= DMA_CCR_MINC;

			// ensure memory-to-memory mapping is disabled
			DMA1_Channel4->CCR &= ~(DMA_CCR_MEM2MEM);

			// ensure circular mode is off
			DMA1_Channel4->CCR &= ~(DMA_CCR_CIRC);

			// disable interrupts
			DMA1_Channel4->CCR &= ~(DMA_CCR_TCIE | DMA_CCR_TEIE);
			NVIC_SetPriority( DMA1_Channel4_IRQn, 0x00 );
			NVIC_EnableIRQ( DMA1_Channel4_IRQn );

			if ( dataSize == SPI_DATA_SIZE::BITS_16 )
			{
				// set the peripheral and memory data sizes to 16 bits
				DMA1_Channel4->CCR &= ~(DMA_CCR_MSIZE);
				DMA1_Channel4->CCR |= DMA_CCR_MSIZE_0;
				DMA1_Channel4->CCR &= ~(DMA_CCR_PSIZE);
				DMA1_Channel4->CCR |= DMA_CCR_PSIZE_0;
			}
			else
			{
				// set the peripheral and memory data sizes to 8 bits
				DMA1_Channel4->CCR &= ~(DMA_CCR_MSIZE);
				DMA1_Channel4->CCR &= ~(DMA_CCR_PSIZE);
			}

			// enable stream
			DMA1_Channel4->CCR |= DMA_CCR_EN;
			while ( (DMA1_Channel4->CCR & DMA_CCR_EN) != DMA_CCR_EN ) {}

			// setup tx dma stream
			// disable channel
			DMA1_Channel5->CCR &= ~(DMA_CCR_EN);

			// configure channel priority to low
			DMA1_Channel5->CCR &= ~(DMA_CCR_PL);

			// set data transfer direction from memory to peripheral
			DMA1_Channel5->CCR |= DMA_CCR_DIR;

			// ensure peripheral incrementing is off
			DMA1_Channel5->CCR &= ~(DMA_CCR_PINC);

			// enable memory incrementing
			DMA1_Channel5->CCR |= DMA_CCR_MINC;

			// ensure memory-to-memory mapping is disabled
			DMA1_Channel5->CCR &= ~(DMA_CCR_MEM2MEM);

			// ensure circular mode is off
			DMA1_Channel5->CCR &= ~(DMA_CCR_CIRC);

			// disable interrupts
			DMA1_Channel5->CCR &= ~(DMA_CCR_TCIE | DMA_CCR_TEIE);
			NVIC_SetPriority( DMA1_Channel5_IRQn, 0x00 );
			NVIC_EnableIRQ( DMA1_Channel5_IRQn );

			if ( dataSize == SPI_DATA_SIZE::BITS_16 )
			{
				// set the peripheral and memory data sizes to 16 bits
				DMA1_Channel5->CCR &= ~(DMA_CCR_MSIZE);
				DMA1_Channel5->CCR |= DMA_CCR_MSIZE_0;
				DMA1_Channel5->CCR &= ~(DMA_CCR_PSIZE);
				DMA1_Channel5->CCR |= DMA_CCR_PSIZE_0;
			}
			else
			{
				// set the peripheral and memory data sizes to 8 bits
				DMA1_Channel5->CCR &= ~(DMA_CCR_MSIZE);
				DMA1_Channel5->CCR &= ~(DMA_CCR_PSIZE);
			}

			// enable stream
			DMA1_Channel5->CCR |= DMA_CCR_EN;
		}
		else if ( enableDmaChannels && slaveSetup )
		{
			// enable dma1 clock
			RCC->AHBENR |= RCC_AHBENR_DMA1EN;

			// setup rx dma stream
			// disable channel
			DMA1_Channel2->CCR &= ~(DMA_CCR_EN);

			// configure channel priority to low
			DMA1_Channel2->CCR &= ~(DMA_CCR_PL);

			// set data transfer direction from peripheral to memory
			DMA1_Channel2->CCR &= DMA_CCR_DIR;

			// ensure peripheral incrementing is off
			DMA1_Channel2->CCR &= ~(DMA_CCR_PINC);

			// enable memory incrementing
			DMA1_Channel2->CCR |= DMA_CCR_MINC;

			// ensure memory-to-memory mapping is disabled
			DMA1_Channel2->CCR &= ~(DMA_CCR_MEM2MEM);

			// ensure circular mode is on
			DMA1_Channel2->CCR |= DMA_CCR_CIRC;

			// disable interrupts
			DMA1_Channel2->CCR &= ~(DMA_CCR_TCIE | DMA_CCR_TEIE);
			NVIC_SetPriority( DMA1_Channel2_IRQn, 0x00 );
			NVIC_EnableIRQ( DMA1_Channel2_IRQn );

			if ( dataSize == SPI_DATA_SIZE::BITS_16 )
			{
				// set the peripheral and memory data sizes to 16 bits
				DMA1_Channel2->CCR &= ~(DMA_CCR_MSIZE);
				DMA1_Channel2->CCR |= DMA_CCR_MSIZE_0;
				DMA1_Channel2->CCR &= ~(DMA_CCR_PSIZE);
				DMA1_Channel2->CCR |= DMA_CCR_PSIZE_0;
			}
			else
			{
				// set the peripheral and memory data sizes to 8 bits
				DMA1_Channel2->CCR &= ~(DMA_CCR_MSIZE);
				DMA1_Channel2->CCR &= ~(DMA_CCR_PSIZE);
			}

			// enable stream
			DMA1_Channel2->CCR |= DMA_CCR_EN;
			while ( (DMA1_Channel2->CCR & DMA_CCR_EN) != DMA_CCR_EN ) {}

			// setup tx dma stream
			// disable channel
			DMA1_Channel3->CCR &= ~(DMA_CCR_EN);

			// configure channel priority to low
			DMA1_Channel3->CCR &= ~(DMA_CCR_PL);

			// set data transfer direction from memory to peripheral
			DMA1_Channel3->CCR |= DMA_CCR_DIR;

			// ensure peripheral incrementing is off
			DMA1_Channel3->CCR &= ~(DMA_CCR_PINC);

			// enable memory incrementing
			DMA1_Channel3->CCR |= DMA_CCR_MINC;

			// ensure memory-to-memory mapping is disabled
			DMA1_Channel3->CCR &= ~(DMA_CCR_MEM2MEM);

			// ensure circular mode is on
			DMA1_Channel3->CCR |= DMA_CCR_CIRC;

			// disable interrupts
			DMA1_Channel3->CCR &= ~(DMA_CCR_TCIE | DMA_CCR_TEIE);
			NVIC_SetPriority( DMA1_Channel3_IRQn, 0x00 );
			NVIC_EnableIRQ( DMA1_Channel3_IRQn );

			if ( dataSize == SPI_DATA_SIZE::BITS_16 )
			{
				// set the peripheral and memory data sizes to 16 bits
				DMA1_Channel3->CCR &= ~(DMA_CCR_MSIZE);
				DMA1_Channel3->CCR |= DMA_CCR_MSIZE_0;
				DMA1_Channel3->CCR &= ~(DMA_CCR_PSIZE);
				DMA1_Channel3->CCR |= DMA_CCR_PSIZE_0;
			}
			else
			{
				// set the peripheral and memory data sizes to 8 bits
				DMA1_Channel3->CCR &= ~(DMA_CCR_MSIZE);
				DMA1_Channel3->CCR &= ~(DMA_CCR_PSIZE);
			}

			// enable stream
			DMA1_Channel3->CCR |= DMA_CCR_EN;
		}

		// lastly, enable SPI peripheral
		spiPtr->CR1 |= SPI_CR1_SPE;
	}
}

void LLPD::spi_master_init (const SPI_NUM& spiNum, const SPI_BAUD_RATE& baudRate, const SPI_CLK_POL& pol,
				const SPI_CLK_PHASE& phase, const SPI_DUPLEX& duplex, const SPI_FRAME_FORMAT& frameFormat,
				const SPI_DATA_SIZE& dataSize, const bool enableDmaChannels)
{
	SPI_TypeDef* spiPtr = nullptr;


	// set up appropriate gpio
	if ( spiNum == SPI_NUM::SPI_1 )
	{
		// enable clock to gpio port B (which is what port spi pins use)
		RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

		// set alternate function registers to af5 for b13, b14, b15
		constexpr int afValue = 5;
		constexpr int afWidth = 4;
		constexpr int afPin5 = 5;
		constexpr int afPin6 = 6;
		constexpr int afPin7 = 7;
		GPIOA->AFR[0] |= (afValue << (afPin5 * afWidth))
				| (afValue << (afPin6 * afWidth))
				| (afValue << (afPin7 * afWidth));

		// spi1 sck
		gpio_output_setup( GPIO_PORT::A, GPIO_PIN::PIN_5, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::HIGH, true );

		// spi1 miso
		gpio_output_setup( GPIO_PORT::A, GPIO_PIN::PIN_6, GPIO_PUPD::NONE, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::HIGH, true );

		// spi1 mosi
		gpio_output_setup( GPIO_PORT::A, GPIO_PIN::PIN_7, GPIO_PUPD::NONE, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::HIGH, true );

		// set sck idle
		gpio_output_set( GPIO_PORT::A, GPIO_PIN::PIN_5, (pol == SPI_CLK_POL::LOW_IDLE) ? false : true );

		spiPtr = SPI1;
	}
	else if ( spiNum == SPI_NUM::SPI_2 )
	{
		// enable clock to gpio port B (which is what port spi pins use)
		RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

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
		gpio_output_setup( GPIO_PORT::B, GPIO_PIN::PIN_14, GPIO_PUPD::NONE, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::HIGH, true );

		// spi2 mosi
		gpio_output_setup( GPIO_PORT::B, GPIO_PIN::PIN_15, GPIO_PUPD::NONE, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::HIGH, true );

		// set sck idle
		gpio_output_set( GPIO_PORT::B, GPIO_PIN::PIN_13, (pol == SPI_CLK_POL::LOW_IDLE) ? false : true );

		spiPtr = SPI2;
	}
	else if ( spiNum == SPI_NUM::SPI_3 )
	{
		// enable clock to gpio port B (which is what port spi pins use)
		RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

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
		gpio_output_setup( GPIO_PORT::B, GPIO_PIN::PIN_4, GPIO_PUPD::NONE, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::HIGH, true );

		// spi3 mosi
		gpio_output_setup( GPIO_PORT::B, GPIO_PIN::PIN_5, GPIO_PUPD::NONE, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::HIGH, true );

		// set sck idle
		gpio_output_set( GPIO_PORT::B, GPIO_PIN::PIN_3, (pol == SPI_CLK_POL::LOW_IDLE) ? false : true );

		spiPtr = SPI3;
	}

	setup_spi_registers( spiPtr, baudRate, pol, phase, duplex, frameFormat, dataSize, false, enableDmaChannels );
}

bool LLPD::spi1_dma_slave_init (const SPI_NUM& spiNum, const SPI_CLK_POL& pol, const SPI_CLK_PHASE& phase,
				const SPI_DUPLEX& duplex, const SPI_FRAME_FORMAT& frameFormat,
				const SPI_DATA_SIZE& dataSize, const bool enableDmaChannels)
{
	SPI_TypeDef* spiPtr = nullptr;

	// enable clock to gpio port A (which is what port spi pins use)
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// set up appropriate gpio
	if ( spiNum == SPI_NUM::SPI_1 )
	{
		// set alternate function registers to af5 for b13, b14, b15
		constexpr int afValue = 5;
		constexpr int afWidth = 4;
		constexpr int afPin4 = 4;
		constexpr int afPin5 = 5;
		constexpr int afPin6 = 6;
		constexpr int afPin7 = 7;
		GPIOA->AFR[0] |= (afValue << (afPin4 * afWidth))
				| (afValue << (afPin5 * afWidth))
				| (afValue << (afPin6 * afWidth))
				| (afValue << (afPin7 * afWidth));

		// spi1 nss
		gpio_output_setup( GPIO_PORT::A, GPIO_PIN::PIN_4, GPIO_PUPD::PULL_UP, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::HIGH, true );

		// spi1 sck
		gpio_output_setup( GPIO_PORT::A, GPIO_PIN::PIN_5, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::HIGH, true );

		// spi1 miso
		gpio_output_setup( GPIO_PORT::A, GPIO_PIN::PIN_6, GPIO_PUPD::NONE, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::HIGH, true );

		// spi1 mosi
		gpio_output_setup( GPIO_PORT::A, GPIO_PIN::PIN_7, GPIO_PUPD::NONE, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::HIGH, true );

		// set nss high
		gpio_output_set( GPIO_PORT::A, GPIO_PIN::PIN_4, true );

		// set sck idle
		gpio_output_set( GPIO_PORT::A, GPIO_PIN::PIN_5, (pol == SPI_CLK_POL::LOW_IDLE) ? false : true );

		spiPtr = SPI1;
	}
	else
	{
		return false;
	}

	setup_spi_registers(spiPtr, SPI_BAUD_RATE::SLAVE_INVALID, pol, phase, duplex, frameFormat, dataSize, false,
				enableDmaChannels, true);

	return true;
}

bool LLPD::spi1_dma_slave_start (void* txBuffer, void* rxBuffer, unsigned int bufferSize)
{
	SPI_TypeDef* spiPtr = SPI1;

	// spinlock until tx fifo is clear
	while ( spiPtr->SR & SPI_SR_FTLVL ) {}

	// spinlock until transmission complete
	while ( spiPtr->SR & SPI_SR_BSY ) {}

	// spinlock until rx fifo is clear
	while ( spiPtr->SR & SPI_SR_FRLVL ) {}

	// spinlock until ready to send
	while ( !(spiPtr->SR & SPI_SR_TXE) ) {}

	// configure dma
	if ( (spiPtr != SPI2 && spiPtr != SPI3) && (txBuffer != nullptr) && (rxBuffer != nullptr) )
	{
		// setup the rx dma channel
		// disable rx channel
		DMA1_Channel2->CCR &= ~(DMA_CCR_EN);

		// clear rx channel flags
		DMA1->IFCR |= DMA_IFCR_CGIF2;

		// configure number of data to be transferred
		DMA1_Channel2->CNDTR = bufferSize;

		// set peripheral address
		DMA1_Channel2->CPAR = (uint32_t) &( spiPtr->DR );

		// set the memory address for where the spi data will be transfered to
		DMA1_Channel2->CMAR = (uint32_t) rxBuffer;

		// disable interrupts
		DMA1_Channel2->CCR &= ~(DMA_CCR_TCIE | DMA_CCR_TEIE | DMA_CCR_HTIE);

		// enable stream
		DMA1_Channel2->CCR |= DMA_CCR_EN;
		while ( !(DMA1_Channel2->CCR & DMA_CCR_EN) )
		{
			DMA1_Channel2->CCR |= DMA_CCR_EN;
		}

		// enable the rx dma
		spiPtr->CR2 |= SPI_CR2_RXDMAEN;

		// setup tx dma stream
		// disable tx channel
		DMA1_Channel3->CCR &= ~(DMA_CCR_EN);

		// clear tx channel flags
		DMA1->IFCR |= DMA_IFCR_CGIF3;

		// configure number of data to be transferred
		DMA1_Channel3->CNDTR = bufferSize;

		// set peripheral address
		DMA1_Channel3->CPAR = (uint32_t) &( spiPtr->DR );

		// set the memory address for where the spi data will be transfered from
		DMA1_Channel3->CMAR = (uint32_t) txBuffer;

		// disable interrupts
		DMA1_Channel3->CCR &= ~(DMA_CCR_TCIE | DMA_CCR_TEIE);

		// enable the tx dma channel to begin sending data
		while ( !(DMA1_Channel3->CCR & DMA_CCR_EN) )
		{
			DMA1_Channel3->CCR |= DMA_CCR_EN;
		}

		// enable spi tx dma
		spiPtr->CR2 |= SPI_CR2_TXDMAEN;

		return true;
	}

	return false;
}

void LLPD::spi1_dma_stop()
{
	// clear interrupt flags
	DMA1->IFCR |= DMA_IFCR_CGIF2 | DMA_IFCR_CGIF3;

	// disable stream
	DMA1_Channel2->CCR &= ~(DMA_CCR_EN);
	DMA1_Channel3->CCR &= ~(DMA_CCR_EN);

	// clear interrupt flags
	DMA1->IFCR |= DMA_IFCR_CGIF2 | DMA_IFCR_CGIF3;
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

bool LLPD::spi1_dma_master_start (uint16_t* buffer, unsigned int bufferSize)
{
	SPI_TypeDef* spiPtr = SPI1;

	// spinlock until tx fifo is clear
	while ( spiPtr->SR & SPI_SR_FTLVL ) {}

	// spinlock until transmission complete
	while ( spiPtr->SR & SPI_SR_BSY ) {}

	// spinlock until rx fifo is clear
	while ( spiPtr->SR & SPI_SR_FRLVL ) {}

	// spinlock until ready to send
	while ( !(spiPtr->SR & SPI_SR_TXE) ) {}

	// configure dma
	if ( (spiPtr != SPI2 && spiPtr != SPI3) && (buffer != nullptr) )
	{
		// enable dma1 clock
		RCC->AHBENR |= RCC_AHBENR_DMA1EN;

		// disable dma1 channel 3 (tim3 up)
		DMA1_Channel3->CCR &= ~(DMA_CCR_EN);
		while ( DMA1_Channel3->CCR & DMA_CCR_EN ) {}

		// clear tx channel flags
		DMA1->IFCR |= DMA_IFCR_CGIF3;

		// set peripheral address to data register
		DMA1_Channel3->CPAR = (uint32_t) &( spiPtr->DR );

		// set the memory address for where the data will be sent from
		DMA1_Channel3->CMAR = (uint32_t) buffer;

		// configure number of data to be transferred
		DMA1_Channel3->CNDTR = bufferSize;

		// configure channel priority to very high
		DMA1_Channel3->CCR |= DMA_CCR_PL;

		// set data transfer direction from memory to peripheral
		DMA1_Channel3->CCR |= DMA_CCR_DIR;

		// ensure peripheral incrementing is off
		DMA1_Channel3->CCR &= ~(DMA_CCR_PINC);

		// enable memory incrementing
		DMA1_Channel3->CCR |= DMA_CCR_MINC;

		// ensure memory-to-memory mapping is disabled
		DMA1_Channel3->CCR &= ~(DMA_CCR_MEM2MEM);

		// ensure circular mode is on
		DMA1_Channel3->CCR |= DMA_CCR_CIRC;

		// enable interrupts
		DMA1_Channel3->CCR |= DMA_CCR_TCIE | DMA_CCR_TEIE;
		NVIC_SetPriority( DMA1_Channel3_IRQn, 0x00 );
		NVIC_EnableIRQ( DMA1_Channel3_IRQn );

		// set the peripheral and memory data sizes to 16 bits
		DMA1_Channel3->CCR &= ~(DMA_CCR_MSIZE);
		DMA1_Channel3->CCR |= DMA_CCR_MSIZE_0;
		DMA1_Channel3->CCR &= ~(DMA_CCR_PSIZE);
		DMA1_Channel3->CCR |= DMA_CCR_PSIZE_0;

		// setup the rx dma channel
		// disable rx channel
		DMA1_Channel2->CCR &= ~(DMA_CCR_EN);
		while ( DMA1_Channel2->CCR & DMA_CCR_EN ) {}

		// clear rx channel flags
		DMA1->IFCR |= DMA_IFCR_CGIF2;

		// configure number of data to be transferred
		DMA1_Channel2->CNDTR = bufferSize;

		// set peripheral address
		DMA1_Channel2->CPAR = (uint32_t) &( spiPtr->DR );

		// set the memory address for where the spi data will be transfered to
		DMA1_Channel2->CMAR = (uint32_t) buffer;

		// configure channel priority to very high
		DMA1_Channel2->CCR |= DMA_CCR_PL;

		// set data transfer direction from peripheral to memory
		DMA1_Channel2->CCR &= ~(DMA_CCR_DIR);

		// ensure peripheral incrementing is off
		DMA1_Channel2->CCR &= ~(DMA_CCR_PINC);

		// enable memory incrementing
		DMA1_Channel2->CCR |= DMA_CCR_MINC;

		// ensure memory-to-memory mapping is disabled
		DMA1_Channel2->CCR &= ~(DMA_CCR_MEM2MEM);

		// ensure circular mode is on
		DMA1_Channel2->CCR |= DMA_CCR_CIRC;

		// set the peripheral and memory data sizes to 16 bits
		DMA1_Channel2->CCR &= ~(DMA_CCR_MSIZE);
		DMA1_Channel2->CCR |= DMA_CCR_MSIZE_0;
		DMA1_Channel2->CCR &= ~(DMA_CCR_PSIZE);
		DMA1_Channel2->CCR |= DMA_CCR_PSIZE_0;

		// enable interrupts
		DMA1_Channel2->CCR |= DMA_CCR_TCIE | DMA_CCR_TEIE | DMA_CCR_HTIE;
		NVIC_SetPriority( DMA1_Channel2_IRQn, 0x00 );
		NVIC_EnableIRQ( DMA1_Channel2_IRQn );

		// enable stream
		DMA1_Channel2->CCR |= DMA_CCR_EN;
		while ( !(DMA1_Channel2->CCR & DMA_CCR_EN) )
		{
			DMA1_Channel2->CCR |= DMA_CCR_EN;
		}

		// enable the rx dma
		spiPtr->CR2 |= SPI_CR2_RXDMAEN;

		// enable dma1 channel 3 (tim3 up)
		DMA1_Channel3->CCR |= DMA_CCR_EN;
		while ( !(DMA1_Channel3->CCR & DMA_CCR_EN) )
		{
			DMA1_Channel3->CCR |= DMA_CCR_EN;
		}

		return true;
	}

	return false;
}

uint16_t LLPD::spi1_tx_dma_get_num_transfers_left()
{
	return DMA1_Channel3->CNDTR;
}

bool LLPD::spi2_dma_start (uint8_t* txBuffer, uint8_t* rxBuffer, unsigned int bufferSize)
{
	SPI_TypeDef* spiPtr = SPI2;

	// spinlock until tx fifo is clear
	while ( spiPtr->SR & SPI_SR_FTLVL ) {}

	// spinlock until transmission complete
	while ( spiPtr->SR & SPI_SR_BSY ) {}

	// spinlock until rx fifo is clear
	while ( spiPtr->SR & SPI_SR_FRLVL ) {}

	// spinlock until ready to send
	while ( !(spiPtr->SR & SPI_SR_TXE) ) {}

	// configure dma
	if ( (spiPtr != SPI1 && spiPtr != SPI3) && (txBuffer != nullptr) && (rxBuffer != nullptr) )
	{
		// setup the rx dma channel
		// disable rx channel
		DMA1_Channel4->CCR &= ~(DMA_CCR_EN);

		// clear rx channel flags
		DMA1->IFCR |= DMA_IFCR_CGIF4;

		// configure number of data to be transferred
		DMA1_Channel4->CNDTR = bufferSize;

		// set peripheral address
		DMA1_Channel4->CPAR = (uint32_t) &( spiPtr->DR );

		// set the memory address for where the spi data will be transfered to
		DMA1_Channel4->CMAR = (uint32_t) rxBuffer;

		// enable interrupts
		DMA1_Channel4->CCR |= DMA_CCR_TCIE | DMA_CCR_TEIE | DMA_CCR_HTIE;

		// enable stream
		DMA1_Channel4->CCR |= DMA_CCR_EN;
		while ( (DMA1_Channel4->CCR & DMA_CCR_EN) != DMA_CCR_EN ) {}

		// enable the rx dma
		spiPtr->CR2 |= SPI_CR2_RXDMAEN;

		// setup tx dma stream
		// disable tx channel
		DMA1_Channel5->CCR &= ~(DMA_CCR_EN);

		// clear tx channel flags
		DMA1->IFCR |= DMA_IFCR_CGIF5;

		// configure number of data to be transferred
		DMA1_Channel5->CNDTR = bufferSize;

		// set peripheral address
		DMA1_Channel5->CPAR = (uint32_t) &( spiPtr->DR );

		// set the memory address for where the spi data will be transfered from
		DMA1_Channel5->CMAR = (uint32_t) txBuffer;

		// enable interrupts
		DMA1_Channel5->CCR |= DMA_CCR_TCIE | DMA_CCR_TEIE;

		// enable the tx dma channel to begin sending data
		while ( !(DMA1_Channel5->CCR & DMA_CCR_EN) )
		{
			DMA1_Channel5->CCR |= DMA_CCR_EN;
		}

		// enable spi tx dma
		spiPtr->CR2 |= SPI_CR2_TXDMAEN;

		return true;
	}

	return false;
}

void LLPD::spi2_dma_wait_for_transfer_complete()
{
	// spinlock until tx fifo is clear
	while ( SPI2->SR & SPI_SR_FTLVL ) {}

	// spinlock until rx fifo is clear
	while ( SPI2->SR & SPI_SR_FRLVL ) {}

	// spinlock until transmission complete
	while ( SPI2->SR & SPI_SR_BSY ) {}

	// spinlock until ready to send
	while ( !(SPI2->SR & SPI_SR_TXE) ) {}
}

void LLPD::spi2_dma_stop()
{
	// clear interrupt flags
	DMA1->IFCR |= DMA_IFCR_CGIF4 | DMA_IFCR_CGIF5;

	// disable stream
	DMA1_Channel4->CCR &= ~(DMA_CCR_EN);
	DMA1_Channel5->CCR &= ~(DMA_CCR_EN);

	// clear interrupt flags
	DMA1->IFCR |= DMA_IFCR_CGIF4 | DMA_IFCR_CGIF5;
}
