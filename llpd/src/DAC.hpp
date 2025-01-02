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

void LLPD::dac_init_use_dma (bool useVoltageBuffer, uint16_t numSamples, uint16_t* buffer)
{
	// enable clock to dac
	RCC->APB1ENR |= RCC_APB1ENR_DAC1EN;

	// set pin a4 to analog mode
	GPIOA->MODER &= ~(GPIO_MODER_MODER4_0 | GPIO_MODER_MODER4_1);
	GPIOA->MODER |= GPIO_MODER_MODER4_0 | GPIO_MODER_MODER4_1;

	// set trigger to tim6
	DAC->CR &= ~(DAC_CR_TSEL1);

	// enable dac
	DAC->CR |= DAC_CR_EN1;

	// disable voltage buffer
	if ( !useVoltageBuffer )
	{
		DAC->CR |= DAC_CR_BOFF1;
	}

	// set up dma2 channel 3 (dac)
	// enable dma2 clock
	RCC->AHBENR |= RCC_AHBENR_DMA2EN;

	// disable channel 3
	DMA2_Channel3->CCR &= ~(DMA_CCR_EN);

	// set peripheral address
	DMA2_Channel3->CPAR = (uint32_t) &(DAC->DHR12R1);

	// set the memory address for where the adc data will be stored
	DMA2_Channel3->CMAR = (uint32_t) buffer;

	// configure number of data to be transferred
	DMA2_Channel3->CNDTR = numSamples;

	// configure channel priority to very high
	DMA2_Channel3->CCR |= DMA_CCR_PL;

	// set data transfer direction from memory to peripheral
	DMA2_Channel3->CCR |= DMA_CCR_DIR;

	// ensure peripheral incrementing is off
	DMA2_Channel3->CCR &= ~(DMA_CCR_PINC);

	// enable memory incrementing
	DMA2_Channel3->CCR |= DMA_CCR_MINC;

	// ensure memory-to-memory mapping is disabled
	DMA2_Channel3->CCR &= ~(DMA_CCR_MEM2MEM);

	// ensure circular mode is on
	DMA2_Channel3->CCR |= DMA_CCR_CIRC;

	// set the peripheral and memory data sizes to 16 bits
	DMA2_Channel3->CCR &= ~(DMA_CCR_MSIZE);
	DMA2_Channel3->CCR |= DMA_CCR_MSIZE_0;
	DMA2_Channel3->CCR &= ~(DMA_CCR_PSIZE);
	DMA2_Channel3->CCR |= DMA_CCR_PSIZE_0;

	// enable stream
	DMA2_Channel3->CCR |= DMA_CCR_EN;

	// enable dac dma
	DAC->CR |= DAC_CR_DMAEN1;

	// enable trigger
	DAC->CR |= DAC_CR_TEN1;
}

void LLPD::dac_send (uint16_t data)
{
	// put data in data register and ensure only 12 bits are used
	DAC->DHR12R1 = (data & 0b0000111111111111);
}

uint16_t LLPD::dac_dma_get_num_transfers_left()
{
	return DMA2_Channel3->CNDTR;
}
