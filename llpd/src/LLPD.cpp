#include "GPIO.hpp"
#include "RCC.hpp"
#include "Timers.hpp"
#include "SPI.hpp"
#include "I2C.hpp"
#include "DAC.hpp"
#include "ADC.hpp"
#include "USART.hpp"
#include "OpAmp.hpp"

std::function<void()> LLPD::spi2_dma_tx_tc_callback = [](){};
std::function<void()> LLPD::spi2_dma_rx_tc_callback = [](){};

// this function is called in system_stm32f3xx.c and can be used to ensure certain things are done on reset
extern "C" void Custom_Reset_Handler(void)
{
	// dma may still be running from the last reset
	LLPD::adc_dma_stop();
	LLPD::dac_dma_stop();
	LLPD::spi2_dma_stop();
}

// spi2 rx dma
extern "C" void DMA1_Channel4_IRQHandler (void)
{
	if ( DMA1->ISR & DMA_ISR_TEIF4 )
	{
		// clear the flag
		DMA1->IFCR |= DMA_IFCR_CTEIF4;
	}

	if ( DMA1->ISR & DMA_ISR_HTIF4 )
	{
		// clear the flag
		DMA1->IFCR |= DMA_IFCR_CHTIF4;
	}

	if ( DMA1->ISR & DMA_ISR_TCIF4 )
	{
		while ( DMA1_Channel5->CCR & DMA_CCR_TCIE )
		{
			DMA1_Channel5->CCR &= ~(DMA_CCR_TCIE);
		}

		while ( DMA1_Channel5->CCR & DMA_CCR_TEIE )
		{
			DMA1_Channel5->CCR &= ~(DMA_CCR_TEIE);
		}

		while ( SPI2->CR2 & SPI_CR2_RXDMAEN )
		{
			SPI2->CR2 &= ~(SPI_CR2_RXDMAEN);
		}

		// clear the flag
		DMA1->IFCR |= DMA_IFCR_CTCIF4;

		// call the user defined callback
		LLPD::spi2_dma_rx_tc_callback();
	}
}

// spi2 tx dma
extern "C" void DMA1_Channel5_IRQHandler (void)
{
	if ( DMA1->ISR & DMA_ISR_TEIF5 )
	{
		// clear the flag
		DMA1->IFCR |= DMA_IFCR_CTEIF5;
	}

	if ( DMA1->ISR & DMA_ISR_TCIF5 )
	{
		while ( DMA1_Channel5->CCR & DMA_CCR_TCIE )
		{
			DMA1_Channel5->CCR &= ~(DMA_CCR_TCIE);
		}

		while ( DMA1_Channel5->CCR & DMA_CCR_TEIE )
		{
			DMA1_Channel5->CCR &= ~(DMA_CCR_TEIE);
		}

		while ( SPI2->CR2 & SPI_CR2_TXDMAEN )
		{
			SPI2->CR2 &= ~(SPI_CR2_TXDMAEN);
		}


		// clear the flag
		DMA1->IFCR |= DMA_IFCR_CTCIF5;

		// call the user defined callback
		LLPD::spi2_dma_tx_tc_callback();
	}
}
