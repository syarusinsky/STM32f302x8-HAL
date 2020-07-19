#include "LLPD.hpp"

#include "stm32f302x8.h"
#include <limits>

static volatile float    tim6DelayVal = std::numeric_limits<float>::max(); // value for delay functions
static volatile uint32_t tim6InterruptRate = 0; 	// interrupt rate used for delay functions
static volatile float    tim6USecondMax = 0.0f; 	// when tim6USecondIncr reaches this value, the delay is over
static volatile float    tim6USecondIncr = 0.0f; 	// how much to increment per interrupt for microsecond delay

void LLPD::tim6_counter_setup (uint32_t prescalerDivisor, uint32_t cyclesPerInterrupt, uint32_t interruptRate)
{
	// store sample rate for delay functions
	tim6InterruptRate = interruptRate;
	tim6USecondIncr = 1000000.0f / tim6InterruptRate;

	// make sure timer is disabled during setup
	TIM6->CR1 &= ~(TIM_CR1_CEN);

	// enable peripheral clock to TIM6
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

	// reset registers
	RCC->APB1RSTR |= (RCC_APB1RSTR_TIM6RST);
	RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM6RST);

	// set timer prescaler and auto-reload values
	TIM6->PSC = prescalerDivisor;
	TIM6->ARR = cyclesPerInterrupt;

	// send an update event to apply the settings
	TIM6->EGR |= TIM_EGR_UG;
}

void LLPD::tim6_counter_enable_interrupts()
{
	NVIC_SetPriority( TIM6_DAC_IRQn, 0x00 );
	NVIC_EnableIRQ( TIM6_DAC_IRQn );

	TIM6->DIER |= TIM_DIER_UIE;
}

void LLPD::tim6_counter_disable_interrupts()
{
	NVIC_DisableIRQ( TIM6_DAC_IRQn );

	TIM6->DIER &= ~(TIM_DIER_UIE);
}

void LLPD::tim6_counter_start()
{
	TIM6->CR1 |= TIM_CR1_CEN;
}

void LLPD::tim6_counter_stop()
{
	TIM6->CR1 &= ~(TIM_CR1_CEN);
	TIM6->SR  &= ~(TIM_SR_UIF);
}

void LLPD::tim6_counter_clear_interrupt_flag()
{
	TIM6->SR &= ~(TIM_SR_UIF);
}

void LLPD::tim6_delay (uint32_t microseconds)
{
	tim6DelayVal = 0;
	tim6USecondMax = static_cast<float>( microseconds );

	// wait for delay to complete
	while ( tim6DelayVal < tim6USecondMax ) {}
}

bool LLPD::tim6_isr_handle_delay()
{
	if ( tim6DelayVal < tim6USecondMax )
	{
		tim6DelayVal += tim6USecondIncr;

		return true;
	}

	return false;
}
