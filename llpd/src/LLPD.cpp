#include "GPIO.hpp"
#include "RCC.hpp"
#include "Timers.hpp"
#include "SPI.hpp"
#include "I2C.hpp"
#include "DAC.hpp"
#include "ADC.hpp"
#include "USART.hpp"
#include "OpAmp.hpp"

// this function is called in system_stm32f3xx.c and can be used to ensure certain things are done on reset
extern "C" void Custom_Reset_Handler(void)
{
	// dma may still be running from the last reset
	LLPD::adc_dma_stop();
	LLPD::dac_dma_stop();
}
