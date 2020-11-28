#include "LLPD.hpp"

#if defined( STM32F302X8 )
#include "stm32f302x8.h"
#elif defined( STM32F302XC )
#include "stm32f302xc.h"
#endif

void LLPD::rcc_clock_setup (const RCC_CLOCK_SOURCE& source, bool usePllAsSystemClock)
{
	// set clock source
	if ( source == RCC_CLOCK_SOURCE::EXTERNAL )
	{
		// enable hse
		RCC->CR |= RCC_CR_HSEON;

		// spinlock to wait for HSE ready
		while ( !(RCC->CR & RCC_CR_HSERDY) ){}
	}

	if ( usePllAsSystemClock )
	{
		RCC->CFGR &= ~(RCC_CFGR_SW);
		RCC->CFGR |= RCC_CFGR_SW_PLL;

		// spinlock to wait for PLL as system clock
		while ( !(RCC->CFGR & RCC_CFGR_SWS_PLL) ) {};
	}
	else if ( source == RCC_CLOCK_SOURCE::INTERNAL )
	{
		// spinlock to wait for HSI ready
		while ( !(RCC->CR & RCC_CR_HSIRDY) ){}

		// enable hsi
		RCC->CR |= RCC_CR_HSION;

		// set system clock to hsi
		RCC->CFGR &= ~(RCC_CFGR_SW);
	}
	else if ( source == RCC_CLOCK_SOURCE::EXTERNAL )
	{
		// set system clock to hse
		RCC->CFGR &= ~(RCC_CFGR_SW);
		RCC->CFGR |= RCC_CFGR_SW_HSE;

		// turn on clock security system for monitoring
		RCC->CR |= RCC_CR_CSSON;
	}

	// set APB1, APB2, and AHB clocks to no prescaling
	RCC->CFGR &= ~(RCC_CFGR_PPRE1);
	RCC->CFGR &= ~(RCC_CFGR_PPRE2);
	RCC->CFGR &= ~(RCC_CFGR_HPRE);
}

void LLPD::rcc_pll_enable (const RCC_CLOCK_SOURCE& pllSource, const RCC_PLL_MULTIPLY& pllMultiply)
{
	// disable pll
	RCC->CR &= ~(RCC_CR_PLLON);

	// spinlock until pll is off
	while ( (RCC->CR & RCC_CR_PLLRDY) ) {}

	if ( pllSource == RCC_CLOCK_SOURCE::INTERNAL )
	{
		RCC->CFGR &= ~(RCC_CFGR_PLLSRC_HSE_PREDIV);
	}
	else if ( pllSource == RCC_CLOCK_SOURCE::EXTERNAL )
	{
		RCC->CFGR |= RCC_CFGR_PLLSRC_HSE_PREDIV;
	}

	// clear pll multiply
	RCC->CFGR &= ~(RCC_CFGR_PLLMUL);

	if ( pllMultiply == RCC_PLL_MULTIPLY::BY_2 )
	{
		RCC->CFGR |= RCC_CFGR_PLLMUL2;
	}
	else if ( pllMultiply == RCC_PLL_MULTIPLY::BY_3 )
	{
		RCC->CFGR |= RCC_CFGR_PLLMUL3;
	}
	else if ( pllMultiply == RCC_PLL_MULTIPLY::BY_4 )
	{
		RCC->CFGR |= RCC_CFGR_PLLMUL4;
	}
	else if ( pllMultiply == RCC_PLL_MULTIPLY::BY_5 )
	{
		RCC->CFGR |= RCC_CFGR_PLLMUL5;
	}
	else if ( pllMultiply == RCC_PLL_MULTIPLY::BY_6 )
	{
		RCC->CFGR |= RCC_CFGR_PLLMUL6;
	}
	else if ( pllMultiply == RCC_PLL_MULTIPLY::BY_7 )
	{
		RCC->CFGR |= RCC_CFGR_PLLMUL7;
	}
	else if ( pllMultiply == RCC_PLL_MULTIPLY::BY_8 )
	{
		RCC->CFGR |= RCC_CFGR_PLLMUL8;
	}
	else if ( pllMultiply == RCC_PLL_MULTIPLY::BY_9 )
	{
		RCC->CFGR |= RCC_CFGR_PLLMUL9;
	}
	else if ( pllMultiply == RCC_PLL_MULTIPLY::BY_10 )
	{
		RCC->CFGR |= RCC_CFGR_PLLMUL10;
	}
	else if ( pllMultiply == RCC_PLL_MULTIPLY::BY_11 )
	{
		RCC->CFGR |= RCC_CFGR_PLLMUL11;
	}
	else if ( pllMultiply == RCC_PLL_MULTIPLY::BY_12 )
	{
		RCC->CFGR |= RCC_CFGR_PLLMUL12;
	}
	else if ( pllMultiply == RCC_PLL_MULTIPLY::BY_13 )
	{
		RCC->CFGR |= RCC_CFGR_PLLMUL13;
	}
	else if ( pllMultiply == RCC_PLL_MULTIPLY::BY_14 )
	{
		RCC->CFGR |= RCC_CFGR_PLLMUL14;
	}
	else if ( pllMultiply == RCC_PLL_MULTIPLY::BY_15 )
	{
		RCC->CFGR |= RCC_CFGR_PLLMUL15;
	}
	else if ( pllMultiply == RCC_PLL_MULTIPLY::BY_16 )
	{
		RCC->CFGR |= RCC_CFGR_PLLMUL16;
	}

	// turn pll back on
	RCC->CR |= RCC_CR_PLLON;

	// spinlock to wait for PLL ready
	while ( !(RCC->CR & RCC_CR_PLLRDY) ){}
}

void LLPD::rcc_pll_disable()
{
	// disable pll
	RCC->CR &= ~(RCC_CR_PLLON);

	// spinlock until pll is off
	while ( (RCC->CR & RCC_CR_PLLRDY) ) {}
}
