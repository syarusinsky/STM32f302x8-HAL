#include "LLPD.hpp"

#if defined( STM32F302X8 )
#include "stm32f302x8.h"
#elif defined( STM32F302XC )
#include "stm32f302xc.h"
#endif

void LLPD::opamp_init()
{
	// enable clock to op amp
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	// disable op amp
	OPAMP2->CSR &= ~(OPAMP2_CSR_OPAMP2EN);

	// select inputs
	OPAMP2->CSR &= ~(OPAMP2_CSR_VMSEL); // clear inverting input selection
	OPAMP2->CSR |= OPAMP2_CSR_VMSEL_0;
	OPAMP2->CSR &= ~(OPAMP2_CSR_VPSEL); // clear non-inverting input selection
	OPAMP2->CSR |= OPAMP2_CSR_VPSEL;

	// TODO probably should calibrate, but do we really need high precision for audio?

	// enable op amp
	OPAMP2->CSR |= OPAMP2_CSR_OPAMP2EN;
}
