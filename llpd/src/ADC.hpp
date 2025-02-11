#include "LLPD.hpp"

#if defined( STM32F302X8 )
#include "stm32f302x8.h"
#elif defined( STM32F302XC )
#include "stm32f302xc.h"
#endif

#include <cstdarg>

// this array is used to hold the values of each channel after a conversion sequence
static uint16_t channelValues[16] = { 0 };
static uint8_t  numberOfChannels = 0;

// this array is used to hold the mapping of the channel order to channel number
static ADC_CHANNEL channelOrder[16] = { ADC_CHANNEL::CHAN_INVALID };

static uint8_t adcChannelToNum (const ADC_CHANNEL& channel)
{
	uint8_t channelNum = 1;

	switch ( channel )
	{
		case ADC_CHANNEL::CHAN_1:
			channelNum = 1;
			break;
		case ADC_CHANNEL::CHAN_2:
			channelNum = 2;
			break;
		case ADC_CHANNEL::CHAN_3:
			channelNum = 3;
			break;
		case ADC_CHANNEL::CHAN_4:
			channelNum = 4;
			break;
		case ADC_CHANNEL::CHAN_5:
			channelNum = 5;
			break;
		case ADC_CHANNEL::CHAN_6:
			channelNum = 6;
			break;
		case ADC_CHANNEL::CHAN_7:
			channelNum = 7;
			break;
		case ADC_CHANNEL::CHAN_8:
			channelNum = 8;
			break;
		case ADC_CHANNEL::CHAN_9:
			channelNum = 9;
			break;
		case ADC_CHANNEL::CHAN_10:
			channelNum = 10;
			break;
		case ADC_CHANNEL::CHAN_11:
			channelNum = 11;
			break;
		case ADC_CHANNEL::CHAN_12:
			channelNum = 12;
			break;
		case ADC_CHANNEL::CHAN_13:
			channelNum = 13;
			break;
		case ADC_CHANNEL::CHAN_14:
			channelNum = 14;
			break;
		case ADC_CHANNEL::CHAN_15:
			channelNum = 15;
			break;
		case ADC_CHANNEL::CHAN_16:
			channelNum = 16;
			break;
		case ADC_CHANNEL::CHAN_17:
			channelNum = 17;
			break;
		case ADC_CHANNEL::CHAN_18:
			channelNum = 18;
			break;
		default:
			break;
	}

	return channelNum;
}


static uint8_t adcChannelOrderToValueIndex (const ADC_CHANNEL& channel)
{
	uint8_t index = 0;

	for ( unsigned int idx = 0; idx < 16; idx++ )
	{
		index = idx;

		if ( channelOrder[idx] == channel )
		{
			break;
		}
	}

	return index;
}

static uint8_t orderNumToPos (uint8_t orderNum)
{
	uint8_t position = ADC_SQR1_SQ1_Pos;

	if ( orderNum > 1 && orderNum <= 16 )
	{
		switch ( orderNum )
		{
			case 1:
				position = ADC_SQR1_SQ1_Pos;
				break;
			case 2:
				position = ADC_SQR1_SQ2_Pos;
				break;
			case 3:
				position = ADC_SQR1_SQ3_Pos;
				break;
			case 4:
				position = ADC_SQR1_SQ4_Pos;
				break;
			case 5:
				position = ADC_SQR2_SQ5_Pos;
				break;
			case 6:
				position = ADC_SQR2_SQ6_Pos;
				break;
			case 7:
				position = ADC_SQR2_SQ7_Pos;
				break;
			case 8:
				position = ADC_SQR2_SQ8_Pos;
				break;
			case 9:
				position = ADC_SQR2_SQ9_Pos;
				break;
			case 10:
				position = ADC_SQR3_SQ10_Pos;
				break;
			case 11:
				position = ADC_SQR3_SQ11_Pos;
				break;
			case 12:
				position = ADC_SQR3_SQ12_Pos;
				break;
			case 13:
				position = ADC_SQR3_SQ13_Pos;
				break;
			case 14:
				position = ADC_SQR3_SQ14_Pos;
				break;
			case 15:
				position = ADC_SQR4_SQ15_Pos;
				break;
			case 16:
				position = ADC_SQR4_SQ16_Pos;
				break;
			default:
				break;
		}
	}

	return position;
}

static uint32_t orderNumToMask (uint8_t orderNum)
{
	uint32_t mask = ADC_SQR1_SQ1;

	if ( orderNum > 1 && orderNum <= 16 )
	{
		switch ( orderNum )
		{
			case 1:
				mask = ADC_SQR1_SQ1;
				break;
			case 2:
				mask = ADC_SQR1_SQ2;
				break;
			case 3:
				mask = ADC_SQR1_SQ3;
				break;
			case 4:
				mask = ADC_SQR1_SQ4;
				break;
			case 5:
				mask = ADC_SQR2_SQ5;
				break;
			case 6:
				mask = ADC_SQR2_SQ6;
				break;
			case 7:
				mask = ADC_SQR2_SQ7;
				break;
			case 8:
				mask = ADC_SQR2_SQ8;
				break;
			case 9:
				mask = ADC_SQR2_SQ9;
				break;
			case 10:
				mask = ADC_SQR3_SQ10;
				break;
			case 11:
				mask = ADC_SQR3_SQ11;
				break;
			case 12:
				mask = ADC_SQR3_SQ12;
				break;
			case 13:
				mask = ADC_SQR3_SQ13;
				break;
			case 14:
				mask = ADC_SQR3_SQ14;
				break;
			case 15:
				mask = ADC_SQR4_SQ15;
				break;
			case 16:
				mask = ADC_SQR4_SQ16;
				break;
			default:
				break;
		}
	}

	return mask;
}

void LLPD::adc_init (const ADC_CYCLES_PER_SAMPLE& cyclesPerSample)
{
	// reset adc registers
#if defined ( STM32F302X8 )
	RCC->AHBRSTR |= RCC_AHBRSTR_ADC1RST;
	RCC->AHBRSTR &= ~(RCC_AHBRSTR_ADC1RST);
#elif defined ( STM32F302XC )
	RCC->AHBRSTR |= RCC_AHBRSTR_ADC12RST;
	RCC->AHBRSTR &= ~(RCC_AHBRSTR_ADC12RST);
#endif

	// set adc prescaler to use AHB clock
#if defined( STM32F302X8 )
	RCC->CFGR2 &= ~(RCC_CFGR2_ADC1PRES);
#elif defined( STM32F302XC )
	RCC->CFGR2 &= ~(RCC_CFGR2_ADCPRE12);
#endif

	// enable clock to adc
#if defined( STM32F302X8 )
	RCC->AHBENR |= RCC_AHBENR_ADC1EN;
#elif defined( STM32F302XC )
	RCC->AHBENR |= RCC_AHBENR_ADC12EN;
#endif

	// set clock mode to use synchronous
#if defined( STM32F302X8 )
	ADC1_COMMON->CCR &= ~(ADC_CCR_CKMODE);
	ADC1_COMMON->CCR |= ADC_CCR_CKMODE_0;
#elif defined( STM32F302XC )
	ADC12_COMMON->CCR &= ~(ADC_CCR_CKMODE);
	ADC12_COMMON->CCR |= ADC_CCR_CKMODE_0;
#endif

	// set adcs to independent mode
	ADC12_COMMON->CCR &= ~(ADC12_CCR_MULTI);

	// setup voltage regulator
	ADC1->CR &= ~(ADC_CR_ADVREGEN);
	ADC1->CR |= ADC_CR_ADVREGEN_0;

	// wait 100us for voltage regulator to initialize
	LLPD::tim6_delay( 100 );

	// ensure adc is disabled
	while ( ADC1->CR & ADC_CR_ADEN ) {}

	// start adc calibration
	ADC1->CR |= ADC_CR_ADCAL;

	// wait for adc calibration to complete
	while ( ADC1->CR & ADC_CR_ADCAL ) {}

	// wait until CR is clear except adc voltage regulator bits (otherwise ADEN cannot be set)
	while ( ADC1->CR & ~(ADC_CR_ADVREGEN) ) {}

	// enable adc
	ADC1->CR |= ADC_CR_ADEN;

	// wait until adc is ready
	while ( ADC1->ISR & ADC_ISR_ADRDY ) {}

	// set cycles per adc sample
	uint8_t clkRegVal = 0;

	switch ( cyclesPerSample )
	{
		case ADC_CYCLES_PER_SAMPLE::CPS_1p5:
			clkRegVal = 0b000;
			break;
		case ADC_CYCLES_PER_SAMPLE::CPS_2p5:
			clkRegVal = 0b001;
			break;
		case ADC_CYCLES_PER_SAMPLE::CPS_4p5:
			clkRegVal = 0b010;
			break;
		case ADC_CYCLES_PER_SAMPLE::CPS_7p5:
			clkRegVal = 0b011;
			break;
		case ADC_CYCLES_PER_SAMPLE::CPS_19p5:
			clkRegVal = 0b100;
			break;
		case ADC_CYCLES_PER_SAMPLE::CPS_61p5:
			clkRegVal = 0b101;
			break;
		case ADC_CYCLES_PER_SAMPLE::CPS_181p5:
			clkRegVal = 0b110;
			break;
		case ADC_CYCLES_PER_SAMPLE::CPS_601p5:
			clkRegVal = 0b111;
			break;
	}

	// set each timing clock in sample time registers
	uint8_t spacing = 3; // 3 bits per entry
	uint8_t offset = 10; // SMPR2 begins at channel 10
	ADC1->SMPR1 = 	( clkRegVal << (spacing * 1) ) |
			( clkRegVal << (spacing * 2) ) |
			( clkRegVal << (spacing * 3) ) |
			( clkRegVal << (spacing * 4) ) |
			( clkRegVal << (spacing * 5) ) |
			( clkRegVal << (spacing * 6) ) |
			( clkRegVal << (spacing * 7) ) |
			( clkRegVal << (spacing * 8) ) |
			( clkRegVal << (spacing * 9) );
	ADC1->SMPR2 = 	( clkRegVal << (spacing * (10 - offset)) ) |
			( clkRegVal << (spacing * (11 - offset)) ) |
			( clkRegVal << (spacing * (12 - offset)) ) |
			( clkRegVal << (spacing * (13 - offset)) ) |
			( clkRegVal << (spacing * (14 - offset)) ) |
			( clkRegVal << (spacing * (15 - offset)) ) |
			( clkRegVal << (spacing * (16 - offset)) ) |
			( clkRegVal << (spacing * (17 - offset)) ) |
			( clkRegVal << (spacing * (18 - offset)) );
}

void LLPD::adc_set_channel_order (bool tim6Trig, uint8_t numChannels, ADC_CHANNEL chanToDma, uint32_t* dmaLoc, uint32_t dmaLocSize, const ADC_CHANNEL& channel...)
{
	// ensure valid amount of channels
	if ( numChannels > 0 && numChannels <= 16 )
	{
		// cache the number of channels so we can use this later
		numberOfChannels = numChannels;

		// reset 'order' array
		for ( uint8_t chanNum = 0; chanNum < 16; chanNum++ )
		{
			channelOrder[chanNum] = ADC_CHANNEL::CHAN_INVALID;
		}

		// set number of channels in sequence
		uint8_t numChannelsOffset = numChannels - 1;
		ADC1->SQR1 &= ~(ADC_SQR1_L);
		ADC1->SQR1 |= ( 0x0000000F & numChannelsOffset );

		va_list channels;
		va_start( channels, channel );

		// order first channel
		ADC_CHANNEL chanFirst = channel;

		ADC1->SQR1 &= ~(ADC_SQR1_SQ1);
		ADC1->SQR1 |= ( adcChannelToNum(chanFirst) << ADC_SQR1_SQ1_Pos );

		// add first channel to the 'order' array
		channelOrder[0] = chanFirst;

		// order rest of channels
		for ( uint8_t orderNum = 2; orderNum <= numChannels; orderNum++ )
		{
			ADC_CHANNEL chanEnum = va_arg(channels, ADC_CHANNEL);
			uint32_t mask = orderNumToMask( orderNum );
			uint8_t position =  orderNumToPos( orderNum );
			uint8_t chan = adcChannelToNum( chanEnum );

			// add channel to the 'order' array
			channelOrder[orderNum - 1] = chanEnum;

			// we need to use different registers depending on the order number
			if ( orderNum < 5 )
			{
				ADC1->SQR1 &= ~(mask);
				ADC1->SQR1 |= ( chan << position );
			}
			else if ( orderNum < 10 )
			{
				ADC1->SQR2 &= ~(mask);
				ADC1->SQR2 |= ( chan << position );
			}
			else if ( orderNum < 15 )
			{
				ADC1->SQR3 &= ~(mask);
				ADC1->SQR3 |= ( chan << position );
			}
			else
			{
				ADC1->SQR4 &= ~(mask);
				ADC1->SQR4 |= ( chan << position );
			}
		}

		va_end( channels );

		// enable dma clock and syscfg clock
		RCC->AHBENR |= RCC_AHBENR_DMA1EN;
		RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

		// disable channel 1 (adc)
		DMA1_Channel1->CCR &= ~(DMA_CCR_EN);

		// set peripheral address for channel 1 (adc)
		DMA1_Channel1->CPAR = (uint32_t) &ADC1->DR;

		// set the memory address for where the adc data will be stored
		DMA1_Channel1->CMAR = (uint32_t) channelValues;

		// configure number of data to be transferred
		DMA1_Channel1->CNDTR = numChannels;

		// configure channel priority to high
		DMA1_Channel1->CCR &= ~(DMA_CCR_PL);
		DMA1_Channel1->CCR |= DMA_CCR_PL_1;

		// set data transfer direction from peripheral to memory
		DMA1_Channel1->CCR &= ~(DMA_CCR_DIR);

		// ensure peripheral incrementing is off
		DMA1_Channel1->CCR &= ~(DMA_CCR_PINC);

		// enable memory incrementing
		DMA1_Channel1->CCR |= DMA_CCR_MINC;

		// ensure memory-to-memory mapping is disabled
		DMA1_Channel1->CCR &= ~(DMA_CCR_MEM2MEM);

		// ensure circular mode is on
		DMA1_Channel1->CCR |= DMA_CCR_CIRC;

		// set the peripheral and memory data sizes to 16 bits
		DMA1_Channel1->CCR &= ~(DMA_CCR_MSIZE);
		DMA1_Channel1->CCR |= DMA_CCR_MSIZE_0;
		DMA1_Channel1->CCR &= ~(DMA_CCR_PSIZE);
		DMA1_Channel1->CCR |= DMA_CCR_PSIZE_0;

		// set up adc to use dma one-shot mode
		ADC1->CFGR &= ~(ADC_CFGR_DMACFG);

		if ( tim6Trig )
		{
			// we actually want dma to use circular mode
			ADC1->CFGR |= ADC_CFGR_DMACFG;

			// set external trigger rising edge with source to tim6
			ADC1->CFGR |= ADC_CFGR_EXTEN_0;
			ADC1->CFGR &= ~(ADC_CFGR_EXTEN_1);
			ADC1->CFGR |= ADC_CFGR_EXTSEL_0;
			ADC1->CFGR &= ~(ADC_CFGR_EXTSEL_1);
			ADC1->CFGR |= ADC_CFGR_EXTSEL_2;
			ADC1->CFGR |= ADC_CFGR_EXTSEL_3;

			if ( chanToDma != ADC_CHANNEL::CHAN_INVALID && dmaLoc != nullptr )
			{
				// disable dma1 channel 3 (tim3 up)
				DMA1_Channel3->CCR &= ~(DMA_CCR_EN);

				// set peripheral address for the location of the stored channel value
				DMA1_Channel3->CPAR = (uint32_t) &channelValues[ adcChannelOrderToValueIndex(chanToDma) ];

				// set the memory address for where the adc data will be stored
				DMA1_Channel3->CMAR = (uint32_t) dmaLoc;

				// configure number of data to be transferred
				DMA1_Channel3->CNDTR = dmaLocSize;

				// configure channel priority to medium
				DMA1_Channel3->CCR &= ~(DMA_CCR_PL);
				DMA1_Channel3->CCR |= DMA_CCR_PL_0;

				// set data transfer direction from peripheral to memory
				DMA1_Channel3->CCR &= ~(DMA_CCR_DIR);

				// ensure peripheral incrementing is off
				DMA1_Channel3->CCR &= ~(DMA_CCR_PINC);

				// enable memory incrementing
				DMA1_Channel3->CCR |= DMA_CCR_MINC;

				// ensure memory-to-memory mapping is disabled
				DMA1_Channel3->CCR &= ~(DMA_CCR_MEM2MEM);

				// ensure circular mode is on
				DMA1_Channel3->CCR |= DMA_CCR_CIRC;

				// set the peripheral and memory data sizes to 16 bits
				DMA1_Channel3->CCR &= ~(DMA_CCR_MSIZE);
				DMA1_Channel3->CCR |= DMA_CCR_MSIZE_0;
				DMA1_Channel3->CCR &= ~(DMA_CCR_PSIZE);
				DMA1_Channel3->CCR |= DMA_CCR_PSIZE_0;
			}
		}

		// enable dma
		ADC1->CFGR |= ADC_CFGR_DMAEN;

		// enable dma channel 1 (adc)
		DMA1_Channel1->CCR |= DMA_CCR_EN;

		// start conversion
		ADC1->CR |= ADC_CR_ADSTART;
	}
}

void LLPD::adc_perform_conversion_sequence()
{
	// wait for end of sequence to ensure the last transfer was completed
	while ( (ADC1->ISR & ADC_ISR_EOS) == 0 ) {}

	// clear end of sequence flag
	ADC1->ISR |= ADC_ISR_EOS;

	// start conversion
	ADC1->CR |= ADC_CR_ADSTART;
}

uint16_t LLPD::adc_get_channel_value (const ADC_CHANNEL& channel)
{
	for ( uint8_t index = 0; index < 16; index++ )
	{
		if ( channel == channelOrder[index] )
		{
			return channelValues[index];
		}
	}

	return 65535;
}

void LLPD::adc_dma_start()
{
	// enable dma1 channel 3 (tim3 up)
	DMA1_Channel3->CCR |= DMA_CCR_EN;
}

void LLPD::adc_dma_stop()
{
	// clear interrupt flags
	DMA1->IFCR |= 0x0FFFFFFF;

	// disable channel 1 (adc)
	DMA1_Channel1->CCR &= ~(DMA_CCR_EN);

	// disable dma1 channel 3 (tim3 up)
	DMA1_Channel3->CCR &= ~(DMA_CCR_EN);

	// clear interrupt flags
	DMA1->IFCR |= 0x0FFFFFFF;
}

uint16_t LLPD::adc_dma_get_num_transfers_left()
{
	return DMA1_Channel3->CNDTR;
}
