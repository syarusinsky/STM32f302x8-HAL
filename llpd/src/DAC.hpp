#include "LLPD.hpp"

#if defined( STM32F302X8 )
#include "stm32f302x8.h"
#elif defined( STM32F302XC )
#include "stm32f302xc.h"
#endif

void LLPD::dac_init (bool useVoltageBuffer)
{
	// enable clock to dac
	RCC->APB1ENR |= RCC_APB1ENR_DAC1EN;

	// set pin a4 to analog mode
	GPIOA->MODER &= ~(GPIO_MODER_MODER4_0 | GPIO_MODER_MODER4_1);
	GPIOA->MODER |= GPIO_MODER_MODER4_0 | GPIO_MODER_MODER4_1;

	// enable dac
	DAC->CR |= DAC_CR_EN1;

	// disable voltage buffer
	if ( !useVoltageBuffer )
	{
		DAC->CR |= DAC_CR_BOFF1;
	}
}

void LLPD::dac_send (uint16_t data)
{
	// put data in data register and ensure only 12 bits are used
	DAC->DHR12R1 = (data & 0b0000111111111111);
}
