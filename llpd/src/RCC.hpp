#include "LLPD.hpp"

#if defined( STM32F302X8 )
#include "stm32f302x8.h"
#elif defined( STM32F302XC )
#include "stm32f302xc.h"
#endif

static void ensureCorrectFlashLatency (unsigned int freq)
{
	uint32_t intendedLatency = 0;
	if ( freq <= 24000000 )
	{
		intendedLatency = 0;
	}
	else if ( freq <= 48000000 )
	{
		intendedLatency = FLASH_ACR_LATENCY_0;
	}
	else
	{
		intendedLatency = FLASH_ACR_LATENCY_1;
	}

	if ( (FLASH->ACR & FLASH_ACR_LATENCY_Msk) != intendedLatency )
	{

		// enable flash prefetch buffer
		FLASH->ACR |= FLASH_ACR_PRFTBE;

		FLASH->ACR &= ~(FLASH_ACR_LATENCY_Msk);
		FLASH->ACR |= intendedLatency;
	}
}

void LLPD::rcc_clock_setup (const RCC_CLOCK_SOURCE& source, unsigned int freq)
{
	// since the flash latency might not be set correctly, we check for this first
	ensureCorrectFlashLatency( freq );

	// set clock source
	if ( source == RCC_CLOCK_SOURCE::INTERNAL )
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
		// enable hse
		RCC->CR |= RCC_CR_HSEON;

		// spinlock to wait for HSE ready
		while ( !(RCC->CR & RCC_CR_HSERDY) ) {}

		// set system clock to hse
		RCC->CFGR &= ~(RCC_CFGR_SW);
		RCC->CFGR |= RCC_CFGR_SW_HSE;

		// turn on clock security system for monitoring
		RCC->CR |= RCC_CR_CSSON;
	}
}

void LLPD::rcc_clock_setup (const RCC_CLOCK_SOURCE& pllSource, bool hseDivBy2, const RCC_PLL_MULTIPLY& pllMultiply, unsigned int freq)
{
	// since the flash latency might not be set correctly, we check for this first
	ensureCorrectFlashLatency( freq );

	if ( pllSource == RCC_CLOCK_SOURCE::INTERNAL )
	{
		// spinlock to wait for HSI ready
		while ( !(RCC->CR & RCC_CR_HSIRDY) ){}

		// enable hsi
		RCC->CR |= RCC_CR_HSION;
	}
	else if ( pllSource == RCC_CLOCK_SOURCE::EXTERNAL )
	{
		// enable hse
		RCC->CR |= RCC_CR_HSEON;

		// spinlock to wait for HSE ready
		while ( !(RCC->CR & RCC_CR_HSERDY) ) {}
	}

	LLPD::rcc_pll_enable( pllSource, hseDivBy2, pllMultiply );

	// set pll as system clock
	RCC->CFGR &= ~(RCC_CFGR_SW);
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	// spinlock to wait for PLL as system clock
	while ( !(RCC->CFGR & RCC_CFGR_SWS_PLL) ) {};
}

void LLPD::rcc_pll_enable (const RCC_CLOCK_SOURCE& pllSource, bool hseDivBy2, const RCC_PLL_MULTIPLY& pllMultiply)
{
	// disable pll
	RCC->CR &= ~(RCC_CR_PLLON);

	if ( hseDivBy2 )
	{
		RCC->CFGR &= ~(RCC_CFGR_PLLXTPRE_Msk);
		RCC->CFGR |= RCC_CFGR_PLLXTPRE_HSE_PREDIV_DIV2;
	}

	RCC->CFGR &= ~(RCC_CFGR_PLLSRC_Msk);
	if ( pllSource == RCC_CLOCK_SOURCE::INTERNAL )
	{
		RCC->CFGR |= RCC_CFGR_PLLSRC_HSI_DIV2;
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

void LLPD::rcc_set_periph_clock_prescalers (const RCC_AHB_PRES &ahbPres, const RCC_APB1_PRES &apb1Pres, const RCC_APB2_PRES &apb2Pres)
{
	// clear ahb clock prescaler and reset
	RCC->CFGR &= ~(RCC_CFGR_HPRE_Msk);
	switch ( ahbPres )
	{
		case RCC_AHB_PRES::BY_1:
			RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

			break;
		case RCC_AHB_PRES::BY_2:
			RCC->CFGR |= RCC_CFGR_HPRE_DIV2;

			break;
		case RCC_AHB_PRES::BY_4:
			RCC->CFGR |= RCC_CFGR_HPRE_DIV4;

			break;
		case RCC_AHB_PRES::BY_8:
			RCC->CFGR |= RCC_CFGR_HPRE_DIV8;

			break;
		case RCC_AHB_PRES::BY_16:
			RCC->CFGR |= RCC_CFGR_HPRE_DIV16;

			break;
		case RCC_AHB_PRES::BY_64:
			RCC->CFGR |= RCC_CFGR_HPRE_DIV64;

			break;
		case RCC_AHB_PRES::BY_128:
			RCC->CFGR |= RCC_CFGR_HPRE_DIV128;

			break;
		case RCC_AHB_PRES::BY_256:
			RCC->CFGR |= RCC_CFGR_HPRE_DIV256;

			break;
		case RCC_AHB_PRES::BY_512:
			RCC->CFGR |= RCC_CFGR_HPRE_DIV512;

			break;
		default:
			break;
	}

	// clear apb1 clock prescaler and reset
	RCC->CFGR &= ~(RCC_CFGR_PPRE1_Msk);
	switch ( apb1Pres )
	{
		case RCC_APB1_PRES::AHB_BY_1:
			RCC->CFGR |= RCC_CFGR_PPRE1_DIV1;

			break;
		case RCC_APB1_PRES::AHB_BY_2:
			RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

			break;
		case RCC_APB1_PRES::AHB_BY_4:
			RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;

			break;
		case RCC_APB1_PRES::AHB_BY_8:
			RCC->CFGR |= RCC_CFGR_PPRE1_DIV8;

			break;
		case RCC_APB1_PRES::AHB_BY_16:
			RCC->CFGR |= RCC_CFGR_PPRE1_DIV16;

			break;
		default:
			break;
	}

	// clear apb2 clock prescaler and reset
	RCC->CFGR &= ~(RCC_CFGR_PPRE2_Msk);
	switch ( apb2Pres )
	{
		case RCC_APB2_PRES::AHB_BY_1:
			RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;

			break;
		case RCC_APB2_PRES::AHB_BY_2:
			RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;

			break;
		case RCC_APB2_PRES::AHB_BY_4:
			RCC->CFGR |= RCC_CFGR_PPRE2_DIV4;

			break;
		case RCC_APB2_PRES::AHB_BY_8:
			RCC->CFGR |= RCC_CFGR_PPRE2_DIV8;

			break;
		case RCC_APB2_PRES::AHB_BY_16:
			RCC->CFGR |= RCC_CFGR_PPRE2_DIV16;

			break;
		default:
			break;
	}
}
