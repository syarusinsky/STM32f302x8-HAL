#include "LLPD.hpp"

#include "stm32f302x8.h"

static inline GPIO_TypeDef* PortToPortPtr (const GPIO_PORT& port)
{
	if ( port == GPIO_PORT::A )
	{
		return GPIOA;
	}
	else if ( port == GPIO_PORT::B )
	{
		return GPIOB;
	}
	else if ( port == GPIO_PORT::C )
	{
		return GPIOC;
	}
	else if ( port == GPIO_PORT::F )
	{
		return GPIOF;
	}

	return nullptr;
}

void LLPD::gpio_enable_clock (const GPIO_PORT& port)
{
	uint32_t portClockVal;

	if ( port == GPIO_PORT::A )
	{
		portClockVal = RCC_AHBENR_GPIOAEN;
	}
	else if ( port == GPIO_PORT::B )
	{
		portClockVal = RCC_AHBENR_GPIOBEN;
	}
	else if ( port == GPIO_PORT::C )
	{
		portClockVal = RCC_AHBENR_GPIOCEN;
	}
	else if ( port == GPIO_PORT::F )
	{
		portClockVal = RCC_AHBENR_GPIOFEN;
	}
	else
	{
		return;
	}

	RCC->AHBENR |= portClockVal;
}

void LLPD::gpio_digital_input_setup (const GPIO_PORT& port, const GPIO_PIN& pin, const GPIO_PUPD& pupd, bool alternateFunc)
{
	GPIO_TypeDef* portPtr = PortToPortPtr( port );

	uint32_t modeBit0;
	uint32_t modeBit1;
	uint32_t pupdBit0;
	uint32_t pupdBit1;

	if ( pin == GPIO_PIN::PIN_0 )
	{
		modeBit0 = GPIO_MODER_MODER0_0;
		modeBit1 = GPIO_MODER_MODER0_1;
		pupdBit0 = GPIO_PUPDR_PUPDR0_0;
		pupdBit1 = GPIO_PUPDR_PUPDR0_1;
	}
	else if ( pin == GPIO_PIN::PIN_1 )
	{
		modeBit0 = GPIO_MODER_MODER1_0;
		modeBit1 = GPIO_MODER_MODER1_1;
		pupdBit0 = GPIO_PUPDR_PUPDR1_0;
		pupdBit1 = GPIO_PUPDR_PUPDR1_1;
	}
	else if ( pin == GPIO_PIN::PIN_2 )
	{
		modeBit0 = GPIO_MODER_MODER2_0;
		modeBit1 = GPIO_MODER_MODER2_1;
		pupdBit0 = GPIO_PUPDR_PUPDR2_0;
		pupdBit1 = GPIO_PUPDR_PUPDR2_1;
	}
	else if ( pin == GPIO_PIN::PIN_3 )
	{
		modeBit0 = GPIO_MODER_MODER3_0;
		modeBit1 = GPIO_MODER_MODER3_1;
		pupdBit0 = GPIO_PUPDR_PUPDR3_0;
		pupdBit1 = GPIO_PUPDR_PUPDR3_1;
	}
	else if ( pin == GPIO_PIN::PIN_4 )
	{
		modeBit0 = GPIO_MODER_MODER4_0;
		modeBit1 = GPIO_MODER_MODER4_1;
		pupdBit0 = GPIO_PUPDR_PUPDR4_0;
		pupdBit1 = GPIO_PUPDR_PUPDR4_1;
	}
	else if ( pin == GPIO_PIN::PIN_5 )
	{
		modeBit0 = GPIO_MODER_MODER5_0;
		modeBit1 = GPIO_MODER_MODER5_1;
		pupdBit0 = GPIO_PUPDR_PUPDR5_0;
		pupdBit1 = GPIO_PUPDR_PUPDR5_1;
	}
	else if ( pin == GPIO_PIN::PIN_6 )
	{
		modeBit0 = GPIO_MODER_MODER6_0;
		modeBit1 = GPIO_MODER_MODER6_1;
		pupdBit0 = GPIO_PUPDR_PUPDR6_0;
		pupdBit1 = GPIO_PUPDR_PUPDR6_1;
	}
	else if ( pin == GPIO_PIN::PIN_7 )
	{
		modeBit0 = GPIO_MODER_MODER7_0;
		modeBit1 = GPIO_MODER_MODER7_1;
		pupdBit0 = GPIO_PUPDR_PUPDR7_0;
		pupdBit1 = GPIO_PUPDR_PUPDR7_1;
	}
	else if ( pin == GPIO_PIN::PIN_8 )
	{
		modeBit0 = GPIO_MODER_MODER8_0;
		modeBit1 = GPIO_MODER_MODER8_1;
		pupdBit0 = GPIO_PUPDR_PUPDR8_0;
		pupdBit1 = GPIO_PUPDR_PUPDR8_1;
	}
	else if ( pin == GPIO_PIN::PIN_9 )
	{
		modeBit0 = GPIO_MODER_MODER9_0;
		modeBit1 = GPIO_MODER_MODER9_1;
		pupdBit0 = GPIO_PUPDR_PUPDR9_0;
		pupdBit1 = GPIO_PUPDR_PUPDR9_1;
	}
	else if ( pin == GPIO_PIN::PIN_10 )
	{
		modeBit0 = GPIO_MODER_MODER10_0;
		modeBit1 = GPIO_MODER_MODER10_1;
		pupdBit0 = GPIO_PUPDR_PUPDR10_0;
		pupdBit1 = GPIO_PUPDR_PUPDR10_1;
	}
	else if ( pin == GPIO_PIN::PIN_11 )
	{
		modeBit0 = GPIO_MODER_MODER11_0;
		modeBit1 = GPIO_MODER_MODER11_1;
		pupdBit0 = GPIO_PUPDR_PUPDR11_0;
		pupdBit1 = GPIO_PUPDR_PUPDR11_1;
	}
	else if ( pin == GPIO_PIN::PIN_12 )
	{
		modeBit0 = GPIO_MODER_MODER12_0;
		modeBit1 = GPIO_MODER_MODER12_1;
		pupdBit0 = GPIO_PUPDR_PUPDR12_0;
		pupdBit1 = GPIO_PUPDR_PUPDR12_1;
	}
	else if ( pin == GPIO_PIN::PIN_13 )
	{
		modeBit0 = GPIO_MODER_MODER13_0;
		modeBit1 = GPIO_MODER_MODER13_1;
		pupdBit0 = GPIO_PUPDR_PUPDR13_0;
		pupdBit1 = GPIO_PUPDR_PUPDR13_1;
	}
	else if ( pin == GPIO_PIN::PIN_14 )
	{
		modeBit0 = GPIO_MODER_MODER14_0;
		modeBit1 = GPIO_MODER_MODER14_1;
		pupdBit0 = GPIO_PUPDR_PUPDR14_0;
		pupdBit1 = GPIO_PUPDR_PUPDR14_1;
	}
	else if ( pin == GPIO_PIN::PIN_15 )
	{
		modeBit0 = GPIO_MODER_MODER15_0;
		modeBit1 = GPIO_MODER_MODER15_1;
		pupdBit0 = GPIO_PUPDR_PUPDR15_0;
		pupdBit1 = GPIO_PUPDR_PUPDR15_1;
	}
	else
	{
		return;
	}

	// set mode to input or alternate function if desired
	if ( alternateFunc )
	{
		portPtr->MODER &= ~(modeBit0);
		portPtr->MODER |= modeBit1;
	}
	else
	{
		portPtr->MODER &= ~(modeBit0);
		portPtr->MODER &= ~(modeBit1);
	}

	// set pull-up/pull-down resistor settings
	if ( pupd == GPIO_PUPD::NONE )
	{
		portPtr->PUPDR &= ~(pupdBit0);
		portPtr->PUPDR &= ~(pupdBit1);
	}
	else if ( pupd == GPIO_PUPD::PULL_UP )
	{
		portPtr->PUPDR |= pupdBit0;
		portPtr->PUPDR &= ~(pupdBit1);
	}
	else if ( pupd == GPIO_PUPD::PULL_DOWN )
	{
		portPtr->PUPDR &= ~(pupdBit0);
		portPtr->PUPDR |= pupdBit1;
	}
}

void LLPD::gpio_analog_setup (const GPIO_PORT& port, const GPIO_PIN& pin)
{
	GPIO_TypeDef* portPtr = PortToPortPtr( port );

	uint32_t modeBit0;
	uint32_t modeBit1;

	if ( pin == GPIO_PIN::PIN_0 )
	{
		modeBit0 = GPIO_MODER_MODER0_0;
		modeBit1 = GPIO_MODER_MODER0_1;
	}
	else if ( pin == GPIO_PIN::PIN_1 )
	{
		modeBit0 = GPIO_MODER_MODER1_0;
		modeBit1 = GPIO_MODER_MODER1_1;
	}
	else if ( pin == GPIO_PIN::PIN_2 )
	{
		modeBit0 = GPIO_MODER_MODER2_0;
		modeBit1 = GPIO_MODER_MODER2_1;
	}
	else if ( pin == GPIO_PIN::PIN_3 )
	{
		modeBit0 = GPIO_MODER_MODER3_0;
		modeBit1 = GPIO_MODER_MODER3_1;
	}
	else if ( pin == GPIO_PIN::PIN_4 )
	{
		modeBit0 = GPIO_MODER_MODER4_0;
		modeBit1 = GPIO_MODER_MODER4_1;
	}
	else if ( pin == GPIO_PIN::PIN_5 )
	{
		modeBit0 = GPIO_MODER_MODER5_0;
		modeBit1 = GPIO_MODER_MODER5_1;
	}
	else if ( pin == GPIO_PIN::PIN_6 )
	{
		modeBit0 = GPIO_MODER_MODER6_0;
		modeBit1 = GPIO_MODER_MODER6_1;
	}
	else if ( pin == GPIO_PIN::PIN_7 )
	{
		modeBit0 = GPIO_MODER_MODER7_0;
		modeBit1 = GPIO_MODER_MODER7_1;
	}
	else if ( pin == GPIO_PIN::PIN_8 )
	{
		modeBit0 = GPIO_MODER_MODER8_0;
		modeBit1 = GPIO_MODER_MODER8_1;
	}
	else if ( pin == GPIO_PIN::PIN_9 )
	{
		modeBit0 = GPIO_MODER_MODER9_0;
		modeBit1 = GPIO_MODER_MODER9_1;
	}
	else if ( pin == GPIO_PIN::PIN_10 )
	{
		modeBit0 = GPIO_MODER_MODER10_0;
		modeBit1 = GPIO_MODER_MODER10_1;
	}
	else if ( pin == GPIO_PIN::PIN_11 )
	{
		modeBit0 = GPIO_MODER_MODER11_0;
		modeBit1 = GPIO_MODER_MODER11_1;
	}
	else if ( pin == GPIO_PIN::PIN_12 )
	{
		modeBit0 = GPIO_MODER_MODER12_0;
		modeBit1 = GPIO_MODER_MODER12_1;
	}
	else if ( pin == GPIO_PIN::PIN_13 )
	{
		modeBit0 = GPIO_MODER_MODER13_0;
		modeBit1 = GPIO_MODER_MODER13_1;
	}
	else if ( pin == GPIO_PIN::PIN_14 )
	{
		modeBit0 = GPIO_MODER_MODER14_0;
		modeBit1 = GPIO_MODER_MODER14_1;
	}
	else if ( pin == GPIO_PIN::PIN_15 )
	{
		modeBit0 = GPIO_MODER_MODER15_0;
		modeBit1 = GPIO_MODER_MODER15_1;
	}
	else
	{
		return;
	}

	// set mode to analog
	portPtr->MODER |= modeBit0;
	portPtr->MODER |= modeBit1;
}

void LLPD::gpio_output_setup (const GPIO_PORT& port, const GPIO_PIN& pin, const GPIO_PUPD& pupd,
				const GPIO_OUTPUT_TYPE& type, const GPIO_OUTPUT_SPEED& speed, bool alternateFunc)
{
	GPIO_TypeDef* portPtr = PortToPortPtr( port );

	uint32_t modeBit0;
	uint32_t modeBit1;
	uint32_t pupdBit0;
	uint32_t pupdBit1;
	uint32_t typeBit;
	uint32_t speedBit0;
	uint32_t speedBit1;

	if ( pin == GPIO_PIN::PIN_0 )
	{
		modeBit0  = GPIO_MODER_MODER0_0;
		modeBit1  = GPIO_MODER_MODER0_1;
		pupdBit0  = GPIO_PUPDR_PUPDR0_0;
		pupdBit1  = GPIO_PUPDR_PUPDR0_1;
		typeBit   = GPIO_OTYPER_OT_0;
		speedBit0 = GPIO_OSPEEDER_OSPEEDR0_0;
		speedBit1 = GPIO_OSPEEDER_OSPEEDR0_1;
	}
	else if ( pin == GPIO_PIN::PIN_1 )
	{
		modeBit0  = GPIO_MODER_MODER1_0;
		modeBit1  = GPIO_MODER_MODER1_1;
		pupdBit0  = GPIO_PUPDR_PUPDR1_0;
		pupdBit1  = GPIO_PUPDR_PUPDR1_1;
		typeBit   = GPIO_OTYPER_OT_1;
		speedBit0 = GPIO_OSPEEDER_OSPEEDR1_0;
		speedBit1 = GPIO_OSPEEDER_OSPEEDR1_1;
	}
	else if ( pin == GPIO_PIN::PIN_2 )
	{
		modeBit0  = GPIO_MODER_MODER2_0;
		modeBit1  = GPIO_MODER_MODER2_1;
		pupdBit0  = GPIO_PUPDR_PUPDR2_0;
		pupdBit1  = GPIO_PUPDR_PUPDR2_1;
		typeBit   = GPIO_OTYPER_OT_2;
		speedBit0 = GPIO_OSPEEDER_OSPEEDR2_0;
		speedBit1 = GPIO_OSPEEDER_OSPEEDR2_1;
	}
	else if ( pin == GPIO_PIN::PIN_3 )
	{
		modeBit0  = GPIO_MODER_MODER3_0;
		modeBit1  = GPIO_MODER_MODER3_1;
		pupdBit0  = GPIO_PUPDR_PUPDR3_0;
		pupdBit1  = GPIO_PUPDR_PUPDR3_1;
		typeBit   = GPIO_OTYPER_OT_3;
		speedBit0 = GPIO_OSPEEDER_OSPEEDR3_0;
		speedBit1 = GPIO_OSPEEDER_OSPEEDR3_1;
	}
	else if ( pin == GPIO_PIN::PIN_4 )
	{
		modeBit0  = GPIO_MODER_MODER4_0;
		modeBit1  = GPIO_MODER_MODER4_1;
		pupdBit0  = GPIO_PUPDR_PUPDR4_0;
		pupdBit1  = GPIO_PUPDR_PUPDR4_1;
		typeBit   = GPIO_OTYPER_OT_4;
		speedBit0 = GPIO_OSPEEDER_OSPEEDR4_0;
		speedBit1 = GPIO_OSPEEDER_OSPEEDR4_1;
	}
	else if ( pin == GPIO_PIN::PIN_5 )
	{
		modeBit0  = GPIO_MODER_MODER5_0;
		modeBit1  = GPIO_MODER_MODER5_1;
		pupdBit0  = GPIO_PUPDR_PUPDR5_0;
		pupdBit1  = GPIO_PUPDR_PUPDR5_1;
		typeBit   = GPIO_OTYPER_OT_5;
		speedBit0 = GPIO_OSPEEDER_OSPEEDR5_0;
		speedBit1 = GPIO_OSPEEDER_OSPEEDR5_1;
	}
	else if ( pin == GPIO_PIN::PIN_6 )
	{
		modeBit0  = GPIO_MODER_MODER6_0;
		modeBit1  = GPIO_MODER_MODER6_1;
		pupdBit0  = GPIO_PUPDR_PUPDR6_0;
		pupdBit1  = GPIO_PUPDR_PUPDR6_1;
		typeBit   = GPIO_OTYPER_OT_6;
		speedBit0 = GPIO_OSPEEDER_OSPEEDR6_0;
		speedBit1 = GPIO_OSPEEDER_OSPEEDR6_1;
	}
	else if ( pin == GPIO_PIN::PIN_7 )
	{
		modeBit0  = GPIO_MODER_MODER7_0;
		modeBit1  = GPIO_MODER_MODER7_1;
		pupdBit0  = GPIO_PUPDR_PUPDR7_0;
		pupdBit1  = GPIO_PUPDR_PUPDR7_1;
		typeBit   = GPIO_OTYPER_OT_7;
		speedBit0 = GPIO_OSPEEDER_OSPEEDR7_0;
		speedBit1 = GPIO_OSPEEDER_OSPEEDR7_1;
	}
	else if ( pin == GPIO_PIN::PIN_8 )
	{
		modeBit0  = GPIO_MODER_MODER8_0;
		modeBit1  = GPIO_MODER_MODER8_1;
		pupdBit0  = GPIO_PUPDR_PUPDR8_0;
		pupdBit1  = GPIO_PUPDR_PUPDR8_1;
		typeBit   = GPIO_OTYPER_OT_8;
		speedBit0 = GPIO_OSPEEDER_OSPEEDR8_0;
		speedBit1 = GPIO_OSPEEDER_OSPEEDR8_1;
	}
	else if ( pin == GPIO_PIN::PIN_9 )
	{
		modeBit0  = GPIO_MODER_MODER9_0;
		modeBit1  = GPIO_MODER_MODER9_1;
		pupdBit0  = GPIO_PUPDR_PUPDR9_0;
		pupdBit1  = GPIO_PUPDR_PUPDR9_1;
		typeBit   = GPIO_OTYPER_OT_9;
		speedBit0 = GPIO_OSPEEDER_OSPEEDR9_0;
		speedBit1 = GPIO_OSPEEDER_OSPEEDR9_1;
	}
	else if ( pin == GPIO_PIN::PIN_10 )
	{
		modeBit0  = GPIO_MODER_MODER10_0;
		modeBit1  = GPIO_MODER_MODER10_1;
		pupdBit0  = GPIO_PUPDR_PUPDR10_0;
		pupdBit1  = GPIO_PUPDR_PUPDR10_1;
		typeBit   = GPIO_OTYPER_OT_10;
		speedBit0 = GPIO_OSPEEDER_OSPEEDR10_0;
		speedBit1 = GPIO_OSPEEDER_OSPEEDR10_1;
	}
	else if ( pin == GPIO_PIN::PIN_11 )
	{
		modeBit0  = GPIO_MODER_MODER11_0;
		modeBit1  = GPIO_MODER_MODER11_1;
		pupdBit0  = GPIO_PUPDR_PUPDR11_0;
		pupdBit1  = GPIO_PUPDR_PUPDR11_1;
		typeBit   = GPIO_OTYPER_OT_11;
		speedBit0 = GPIO_OSPEEDER_OSPEEDR11_0;
		speedBit1 = GPIO_OSPEEDER_OSPEEDR11_1;
	}
	else if ( pin == GPIO_PIN::PIN_12 )
	{
		modeBit0  = GPIO_MODER_MODER12_0;
		modeBit1  = GPIO_MODER_MODER12_1;
		pupdBit0  = GPIO_PUPDR_PUPDR12_0;
		pupdBit1  = GPIO_PUPDR_PUPDR12_1;
		typeBit   = GPIO_OTYPER_OT_12;
		speedBit0 = GPIO_OSPEEDER_OSPEEDR12_0;
		speedBit1 = GPIO_OSPEEDER_OSPEEDR12_1;
	}
	else if ( pin == GPIO_PIN::PIN_13 )
	{
		modeBit0  = GPIO_MODER_MODER13_0;
		modeBit1  = GPIO_MODER_MODER13_1;
		pupdBit0  = GPIO_PUPDR_PUPDR13_0;
		pupdBit1  = GPIO_PUPDR_PUPDR13_1;
		typeBit   = GPIO_OTYPER_OT_13;
		speedBit0 = GPIO_OSPEEDER_OSPEEDR13_0;
		speedBit1 = GPIO_OSPEEDER_OSPEEDR13_1;
	}
	else if ( pin == GPIO_PIN::PIN_14 )
	{
		modeBit0  = GPIO_MODER_MODER14_0;
		modeBit1  = GPIO_MODER_MODER14_1;
		pupdBit0  = GPIO_PUPDR_PUPDR14_0;
		pupdBit1  = GPIO_PUPDR_PUPDR14_1;
		typeBit   = GPIO_OTYPER_OT_14;
		speedBit0 = GPIO_OSPEEDER_OSPEEDR14_0;
		speedBit1 = GPIO_OSPEEDER_OSPEEDR14_1;
	}
	else if ( pin == GPIO_PIN::PIN_15 )
	{
		modeBit0  = GPIO_MODER_MODER15_0;
		modeBit1  = GPIO_MODER_MODER15_1;
		pupdBit0  = GPIO_PUPDR_PUPDR15_0;
		pupdBit1  = GPIO_PUPDR_PUPDR15_1;
		typeBit   = GPIO_OTYPER_OT_15;
		speedBit0 = GPIO_OSPEEDER_OSPEEDR15_0;
		speedBit1 = GPIO_OSPEEDER_OSPEEDR15_1;
	}
	else
	{
		return;
	}

	// set mode to output or alternate function if desired
	if ( alternateFunc )
	{
		portPtr->MODER &= ~(modeBit0);
		portPtr->MODER |= modeBit1;
	}
	else
	{
		portPtr->MODER |= modeBit0;
		portPtr->MODER &= ~(modeBit1);
	}

	// set pull-up/pull-down resistor settings
	if ( pupd == GPIO_PUPD::NONE )
	{
		portPtr->PUPDR &= ~(pupdBit0);
		portPtr->PUPDR &= ~(pupdBit1);
	}
	else if ( pupd == GPIO_PUPD::PULL_UP )
	{
		portPtr->PUPDR |= pupdBit0;
		portPtr->PUPDR &= ~(pupdBit1);
	}
	else if ( pupd == GPIO_PUPD::PULL_DOWN )
	{
		portPtr->PUPDR &= ~(pupdBit0);
		portPtr->PUPDR |= pupdBit1;
	}

	// set output type settings
	if ( type == GPIO_OUTPUT_TYPE::PUSH_PULL )
	{
		portPtr->OTYPER &= ~(typeBit);
	}
	else if ( type == GPIO_OUTPUT_TYPE::OPEN_DRAIN )
	{
		portPtr->OTYPER |= typeBit;
	}

	// set output speed settings
	if ( speed == GPIO_OUTPUT_SPEED::LOW )
	{
		portPtr->OSPEEDR &= ~(speedBit0);
		portPtr->OSPEEDR &= ~(speedBit1);
	}
	else if ( speed == GPIO_OUTPUT_SPEED::MEDIUM )
	{
		portPtr->OSPEEDR |= speedBit0;
		portPtr->OSPEEDR &= ~(speedBit1);
	}
	else if ( speed == GPIO_OUTPUT_SPEED::HIGH )
	{
		portPtr->OSPEEDR |= speedBit0;
		portPtr->OSPEEDR |= speedBit1;
	}
}

bool LLPD::gpio_input_get (const GPIO_PORT& port, const GPIO_PIN& pin)
{
	GPIO_TypeDef* portPtr = PortToPortPtr( port );

	if ( pin == GPIO_PIN::PIN_0 )
	{
		return portPtr->IDR & GPIO_IDR_0;
	}
	else if ( pin == GPIO_PIN::PIN_1 )
	{
		return portPtr->IDR & GPIO_IDR_1;
	}
	else if ( pin == GPIO_PIN::PIN_2 )
	{
		return portPtr->IDR & GPIO_IDR_2;
	}
	else if ( pin == GPIO_PIN::PIN_3 )
	{
		return portPtr->IDR & GPIO_IDR_3;
	}
	else if ( pin == GPIO_PIN::PIN_4 )
	{
		return portPtr->IDR & GPIO_IDR_4;
	}
	else if ( pin == GPIO_PIN::PIN_5 )
	{
		return portPtr->IDR & GPIO_IDR_5;
	}
	else if ( pin == GPIO_PIN::PIN_6 )
	{
		return portPtr->IDR & GPIO_IDR_6;
	}
	else if ( pin == GPIO_PIN::PIN_7 )
	{
		return portPtr->IDR & GPIO_IDR_7;
	}
	else if ( pin == GPIO_PIN::PIN_8 )
	{
		return portPtr->IDR & GPIO_IDR_8;
	}
	else if ( pin == GPIO_PIN::PIN_9 )
	{
		return portPtr->IDR & GPIO_IDR_9;
	}
	else if ( pin == GPIO_PIN::PIN_10 )
	{
		return portPtr->IDR & GPIO_IDR_10;
	}
	else if ( pin == GPIO_PIN::PIN_11 )
	{
		return portPtr->IDR & GPIO_IDR_11;
	}
	else if ( pin == GPIO_PIN::PIN_12 )
	{
		return portPtr->IDR & GPIO_IDR_12;
	}
	else if ( pin == GPIO_PIN::PIN_13 )
	{
		return portPtr->IDR & GPIO_IDR_13;
	}
	else if ( pin == GPIO_PIN::PIN_14 )
	{
		return portPtr->IDR & GPIO_IDR_14;
	}
	else if ( pin == GPIO_PIN::PIN_15 )
	{
		return portPtr->IDR & GPIO_IDR_15;
	}
	else
	{
		return false;
	}
}

void LLPD::gpio_output_set (const GPIO_PORT& port, const GPIO_PIN& pin, bool set)
{
	GPIO_TypeDef* portPtr = PortToPortPtr( port );

	uint32_t bsrrBit;
	uint32_t brrBit;

	if ( pin == GPIO_PIN::PIN_0 )
	{
		bsrrBit = GPIO_BSRR_BS_0;
		brrBit  = GPIO_BRR_BR_0;
	}
	else if ( pin == GPIO_PIN::PIN_1 )
	{
		bsrrBit = GPIO_BSRR_BS_1;
		brrBit  = GPIO_BRR_BR_1;
	}
	else if ( pin == GPIO_PIN::PIN_2 )
	{
		bsrrBit = GPIO_BSRR_BS_2;
		brrBit  = GPIO_BRR_BR_2;
	}
	else if ( pin == GPIO_PIN::PIN_3 )
	{
		bsrrBit = GPIO_BSRR_BS_3;
		brrBit  = GPIO_BRR_BR_3;
	}
	else if ( pin == GPIO_PIN::PIN_4 )
	{
		bsrrBit = GPIO_BSRR_BS_4;
		brrBit  = GPIO_BRR_BR_4;
	}
	else if ( pin == GPIO_PIN::PIN_5 )
	{
		bsrrBit = GPIO_BSRR_BS_5;
		brrBit  = GPIO_BRR_BR_5;
	}
	else if ( pin == GPIO_PIN::PIN_6 )
	{
		bsrrBit = GPIO_BSRR_BS_6;
		brrBit  = GPIO_BRR_BR_6;
	}
	else if ( pin == GPIO_PIN::PIN_7 )
	{
		bsrrBit = GPIO_BSRR_BS_7;
		brrBit  = GPIO_BRR_BR_7;
	}
	else if ( pin == GPIO_PIN::PIN_8 )
	{
		bsrrBit = GPIO_BSRR_BS_8;
		brrBit  = GPIO_BRR_BR_8;
	}
	else if ( pin == GPIO_PIN::PIN_9 )
	{
		bsrrBit = GPIO_BSRR_BS_9;
		brrBit  = GPIO_BRR_BR_9;
	}
	else if ( pin == GPIO_PIN::PIN_10 )
	{
		bsrrBit = GPIO_BSRR_BS_10;
		brrBit  = GPIO_BRR_BR_10;
	}
	else if ( pin == GPIO_PIN::PIN_11 )
	{
		bsrrBit = GPIO_BSRR_BS_11;
		brrBit  = GPIO_BRR_BR_11;
	}
	else if ( pin == GPIO_PIN::PIN_12 )
	{
		bsrrBit = GPIO_BSRR_BS_12;
		brrBit  = GPIO_BRR_BR_12;
	}
	else if ( pin == GPIO_PIN::PIN_13 )
	{
		bsrrBit = GPIO_BSRR_BS_13;
		brrBit  = GPIO_BRR_BR_13;
	}
	else if ( pin == GPIO_PIN::PIN_14 )
	{
		bsrrBit = GPIO_BSRR_BS_14;
		brrBit  = GPIO_BRR_BR_14;
	}
	else if ( pin == GPIO_PIN::PIN_15 )
	{
		bsrrBit = GPIO_BSRR_BS_15;
		brrBit  = GPIO_BRR_BR_15;
	}
	else
	{
		return;
	}

	if ( set )
	{
		portPtr->BSRR |= bsrrBit;
	}
	else
	{
		portPtr->BRR |= brrBit;
	}
}
