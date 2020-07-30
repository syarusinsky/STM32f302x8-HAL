#include "LLPD.hpp"

#if defined( STM32F302X8 )
#include "stm32f302x8.h"
#elif defined( STM32F302XC )
#include "stm32f302xc.h"
#endif

// these variables store masks for transmitting data based on word length determined in usart_init (default 8 bits)
static uint16_t usart1WordLenMask = 0b0000000011111111;
static uint16_t usart2WordLenMask = 0b0000000011111111;
static uint16_t usart3WordLenMask = 0b0000000011111111;

void LLPD::usart_init (const USART_NUM& usartNum, const USART_WORD_LENGTH& wordLen, const USART_PARITY& parity,
			const USART_CONF& conf, const USART_STOP_BITS& stopBits, const unsigned int sysClockFreq,
			const unsigned int baudRate)
{
	USART_TypeDef* usart = nullptr;
	uint16_t* usartWordLenMask = nullptr;

	// enable usart peripheral clock
	if ( usartNum == USART_NUM::USART_1 )
	{
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
		usart = USART1;
		usartWordLenMask = &usart1WordLenMask;

		// set alternate function registers to af7 for b6 and b7
		const int afValue = 7;
		const int afWidth = 4;
		const int afPin6 = 6;
		const int afPin7 = 7;
		GPIOB->AFR[0] |= (afValue << (afPin6 * afWidth)) | (afValue << (afPin7 * afWidth));

		// tx
		gpio_output_setup( GPIO_PORT::B, GPIO_PIN::PIN_6, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::HIGH, true );
		gpio_output_set( GPIO_PORT::B, GPIO_PIN::PIN_6, false );

		// rx
		gpio_digital_input_setup( GPIO_PORT::B, GPIO_PIN::PIN_7, GPIO_PUPD::NONE, true );
		gpio_output_set( GPIO_PORT::B, GPIO_PIN::PIN_7, false );
	}
	else if ( usartNum == USART_NUM::USART_2 )
	{
		RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
		usart = USART2;
		usartWordLenMask = &usart2WordLenMask;

		// set alternate function registers to af7 for b3 and b4
		const int afValue = 7;
		const int afWidth = 4;
		const int afPin3 = 3;
		const int afPin4 = 4;
		GPIOB->AFR[0] |= (afValue << (afPin3 * afWidth)) | (afValue << (afPin4 * afWidth));

		// tx
		gpio_output_setup( GPIO_PORT::B, GPIO_PIN::PIN_3, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::HIGH, true );
		gpio_output_set( GPIO_PORT::B, GPIO_PIN::PIN_3, false );

		// rx
		gpio_digital_input_setup( GPIO_PORT::B, GPIO_PIN::PIN_4, GPIO_PUPD::NONE, true );
		gpio_output_set( GPIO_PORT::B, GPIO_PIN::PIN_4, false );
	}
	else if ( usartNum == USART_NUM::USART_3 )
	{
		RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
		usart = USART3;
		usartWordLenMask = &usart3WordLenMask;

#if defined( STM32F302X8 )
		// set alternate function registers to af7 for b8 and b9
		const int afValue = 7;
		const int afWidth = 4;
		const int afPin8 = 8 - 8; // minus 8 since we will modify the high register instead of low
		const int afPin9 = 9 - 8;
		GPIOB->AFR[1] |= (afValue << (afPin8 * afWidth)) | (afValue << (afPin9 * afWidth));

		// tx
		gpio_output_setup( GPIO_PORT::B, GPIO_PIN::PIN_9, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::HIGH, true );
		gpio_output_set( GPIO_PORT::B, GPIO_PIN::PIN_9, false );

		// rx
		gpio_digital_input_setup( GPIO_PORT::B, GPIO_PIN::PIN_8, GPIO_PUPD::NONE, true );
		gpio_output_set( GPIO_PORT::B, GPIO_PIN::PIN_8, false );

#elif defined( STM32F302XC )
		// set alternate function registers to af7 for b10 and b11
		const int afValue = 7;
		const int afWidth = 4;
		const int afPin10 = 10 - 8; // minus 8 since we will modify the high register instead of low
		const int afPin11 = 11 - 8;
		GPIOB->AFR[1] |= (afValue << (afPin10 * afWidth)) | (afValue << (afPin11 * afWidth));

		// tx
		gpio_output_setup( GPIO_PORT::B, GPIO_PIN::PIN_10, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::HIGH, true );
		gpio_output_set( GPIO_PORT::B, GPIO_PIN::PIN_10, false );

		// rx
		gpio_digital_input_setup( GPIO_PORT::B, GPIO_PIN::PIN_11, GPIO_PUPD::NONE, true );
		gpio_output_set( GPIO_PORT::B, GPIO_PIN::PIN_11, false );
#endif
	}

	if ( usart && usartWordLenMask )
	{
		// ensure usart is off
		usart->CR1 &= ~(USART_CR1_UE);

		// reset registers
		if ( usartNum == USART_NUM::USART_1 )
		{
			RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST;
			RCC->APB2RSTR &= ~(RCC_APB2RSTR_USART1RST);
		}
		else if ( usartNum == USART_NUM::USART_2 )
		{
			RCC->APB1RSTR |= RCC_APB1RSTR_USART2RST;
			RCC->APB1RSTR &= ~(RCC_APB1RSTR_USART2RST);
		}
		else if ( usartNum == USART_NUM::USART_3 )
		{
			RCC->APB1RSTR |= RCC_APB1RSTR_USART3RST;
			RCC->APB1RSTR &= ~(RCC_APB1RSTR_USART3RST);
		}

		// set word length
#if defined( STM32F302X8 )
		if ( wordLen == USART_WORD_LENGTH::BITS_7 )
		{
			usart->CR1 &= ~(USART_CR1_M0);
			usart->CR1 |= USART_CR1_M1;
			*usartWordLenMask = 0b0000000001111111;
		}
		else if ( wordLen == USART_WORD_LENGTH::BITS_8 )
		{
			usart->CR1 &= ~(USART_CR1_M0);
			usart->CR1 &= ~(USART_CR1_M1);
			*usartWordLenMask = 0b0000000011111111;
		}
		else if ( wordLen == USART_WORD_LENGTH::BITS_9 )
		{
			usart->CR1 |= USART_CR1_M0;
			usart->CR1 &= ~(USART_CR1_M1);
			*usartWordLenMask = 0b0000000111111111;
		}
#elif defined( STM32F302XC )
		if ( wordLen == USART_WORD_LENGTH::BITS_8 )
		{
			usart->CR1 &= ~(USART_CR1_M0);
			*usartWordLenMask = 0b0000000011111111;
		}
		else if ( wordLen == USART_WORD_LENGTH::BITS_9 )
		{
			usart->CR1 |= USART_CR1_M0;
			*usartWordLenMask = 0b0000000111111111;
		}
#endif

		// set parity
		if ( parity == USART_PARITY::ODD )
		{
			usart->CR1 |= USART_CR1_PS;
		}
		else if ( parity == USART_PARITY::EVEN )
		{
			usart->CR1 &= ~(USART_CR1_PS);
		}

		// enable transmitter and/or reciever
		bool usingReceiver = false;
		if ( conf == USART_CONF::TX_ONLY )
		{
			usart->CR1 |= USART_CR1_TE;
			usart->CR1 &= ~(USART_CR1_RE);
			usart->CR1 &= ~(USART_CR1_RXNEIE);
			usingReceiver = true;
		}
		else if ( conf == USART_CONF::RX_ONLY )
		{
			usart->CR1 &= ~(USART_CR1_TE);
			usart->CR1 |= USART_CR1_RE;
			usart->CR1 |= USART_CR1_RXNEIE;
			usingReceiver = true;
		}
		else if ( conf == USART_CONF::TX_AND_RX )
		{
			usart->CR1 |= USART_CR1_TE;
			usart->CR1 |= USART_CR1_RE;
			usart->CR1 |= USART_CR1_RXNEIE;
			usingReceiver = true;
		}

		if ( usingReceiver )
		{
			// enable interrupt service routine with priority behind audio timer
			if ( usartNum == USART_NUM::USART_1 )
			{
				NVIC_SetPriority( USART1_IRQn, 0x01 );
				NVIC_EnableIRQ( USART1_IRQn );
			}
			else if ( usartNum == USART_NUM::USART_2 )
			{
				NVIC_SetPriority( USART2_IRQn, 0x01 );
				NVIC_EnableIRQ( USART2_IRQn );
			}
			else if ( usartNum == USART_NUM::USART_3 )
			{
				NVIC_SetPriority( USART3_IRQn, 0x01 );
				NVIC_EnableIRQ( USART3_IRQn );
			}
		}

		// set oversampling to by 16
		usart->CR1 &= ~(USART_CR1_OVER8);

		// set number of stop bits
		if ( stopBits == USART_STOP_BITS::BITS_1 )
		{
			usart->CR2 &= ~(USART_CR2_STOP_0);
			usart->CR2 &= ~(USART_CR2_STOP_1);
		}
		else if ( stopBits == USART_STOP_BITS::BITS_2 )
		{
			usart->CR2 &= ~(USART_CR2_STOP_0);
			usart->CR2 |= USART_CR2_STOP_1;
		}

		// set baud rate
		uint16_t usartDiv = sysClockFreq / baudRate;
		usart->BRR = usartDiv;

		// enable usart
		usart->CR1 |= USART_CR1_UE;
	}
}

void LLPD::usart_transmit (const USART_NUM& usartNum, uint16_t data)
{
	USART_TypeDef* usart = nullptr;
	uint16_t* usartWordLenMask = nullptr;

	if ( usartNum == USART_NUM::USART_1 )
	{
		usart = USART1;
		usartWordLenMask = &usart1WordLenMask;
	}
	else if ( usartNum == USART_NUM::USART_2 )
	{
		usart = USART2;
		usartWordLenMask = &usart2WordLenMask;
	}
	else if ( usartNum == USART_NUM::USART_3 )
	{
		usart = USART3;
		usartWordLenMask = &usart3WordLenMask;
	}

	if ( usart && usartWordLenMask )
	{
		// mask the data so it correctly fits in buffer
		uint16_t dataMasked = data & *usartWordLenMask;

		// wait for transmit data register to become empty
		while ( ! (usart->ISR & USART_ISR_TXE) ) {}

		// put data in the transmit data register
		usart->TDR = dataMasked;
	}
}

uint16_t LLPD::usart_receive (const USART_NUM& usartNum)
{
	USART_TypeDef* usart = nullptr;

	if ( usartNum == USART_NUM::USART_1 )
	{
		usart = USART1;
	}
	else if ( usartNum == USART_NUM::USART_2 )
	{
		usart = USART2;
	}
	else if ( usartNum == USART_NUM::USART_3 )
	{
		usart = USART3;
	}

	if ( usart )
	{
		// wait until there is data to read in the read data register
		while ( ! (usart->ISR & USART_ISR_RXNE) ) {}

		// read data from register
		uint16_t data = usart->RDR;

		return data;
	}

	return 0;
}
