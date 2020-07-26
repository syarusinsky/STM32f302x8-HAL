#include "LLPD.hpp"

#if defined( STM32F302X8 )
#include "stm32f302x8.h"
#elif defined( STM32F302XC )
#include "stm32f302xc.h"
#endif

#include <cstdarg>

// this array is used to hold the values of each channel after a conversion sequence
static uint16_t channelValues[16] = { 0 };

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
	// ensure adc is off
	ADC1->CR |= ADC_CR_ADDIS;

	// setup voltage regulator
	ADC1->CR &= ~(ADC_CR_ADVREGEN);
	ADC1->CR |= ADC_CR_ADVREGEN_0;

	// wait 10us for voltage regulator to initialize
	LLPD::tim6_delay( 10 );

	// start adc calibration
	ADC1->CR |= ADC_CR_ADCAL;

	// wait for adc calibration to complete
	while ( (ADC1->CR & ADC_CR_ADCAL) != 0 ) {}

	// set clock mode to use PLL
#if defined( STM32F302X8 )
	ADC1_COMMON->CCR &= ~(ADC_CCR_CKMODE);
#elif defined( STM32F302XC )
	ADC12_COMMON->CCR &= ~(ADC_CCR_CKMODE);
#endif

	// set PLL prescaler to 1
#if defined( STM32F302X8 )
	RCC->CFGR2 &= ~(RCC_CFGR2_ADC1PRES);
	RCC->CFGR2 |= RCC_CFGR2_ADC1PRES_DIV1;
#elif defined( STM32F302XC )
	RCC->CFGR2 &= ~(RCC_CFGR2_ADCPRE12);
	RCC->CFGR2 |= RCC_CFGR2_ADCPRE12_DIV1;
#endif

	// enable clock to adc
#if defined( STM32F302X8 )
	RCC->AHBENR |= RCC_AHBENR_ADC1EN;
#elif defined( STM32F302XC )
	RCC->AHBENR |= RCC_AHBENR_ADC12EN;
#endif

	// enable adc
	ADC1->CR |= ADC_CR_ADEN;

	// wait until adc is ready
	while ( (ADC1->ISR & ADC_ISR_ADRDY) == 0 ) {}

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

	// TODO possibly set vrefen????
#if defined( STM32F302X8 )
	ADC1_COMMON->CCR |= ADC_CCR_VREFEN;
#elif defined( STM32F302XC )
	ADC12_COMMON->CCR |= ADC_CCR_VREFEN;
#endif

	// TODO possibly set data alignment?

	// TODO possibly set adc prescaler
#if defined( STM32F302X8 )
	RCC->CFGR2 &= ~(RCC_CFGR2_ADC1PRES);
	RCC->CFGR2 |= (0b11011 << 4);
#elif defined( STM32F302XC )
	RCC->CFGR2 &= ~(RCC_CFGR2_ADCPRE12);
	RCC->CFGR2 |= (0b11011 << 4);
#endif
}

uint16_t LLPD::adc_test()
{
	uint8_t numConversions = 1;
	uint8_t numConversionsOffset = numConversions - 1;

	// set how many conversions will take place
	ADC1->SQR1 &= ~(ADC_SQR1_L);
	ADC1->SQR1 |= ( 0x0000000F & numConversionsOffset );

	// setting channel 2 (PINA1) as first conversion
	ADC1->SQR1 &= ~(ADC_SQR1_SQ1);
	ADC1->SQR1 |= ( 2 << ADC_SQR1_SQ1_Pos );

	// start conversion
	ADC1->CR |= ADC_CR_ADSTART;

	// wait for end of conversion flag
	while ( (ADC1->ISR & ADC_ISR_EOC) == 0 ) {}

	// read data from data register
	uint16_t data = ADC1->DR;

	// wait for end of sequence
	while ( (ADC1->ISR & ADC_ISR_EOS) == 0 ) {}

	// clear end of sequence flag
	ADC1->ISR |= ADC_ISR_EOS;

	return data;
}

void LLPD::adc_set_channel_order (uint8_t numChannels, const ADC_CHANNEL& channel...)
{
	// ensure valid amount of channels
	if ( numChannels > 0 && numChannels <= 16 )
	{
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
	}
}

void LLPD::adc_perform_conversion_sequence()
{
	// start conversion
	ADC1->CR |= ADC_CR_ADSTART;

	uint8_t channelIndex = 0;
	ADC_CHANNEL channel = channelOrder[channelIndex];

	while ( channel != ADC_CHANNEL::CHAN_INVALID && channelIndex < 16 )
	{
		// wait for end of conversion flag
		while ( (ADC1->ISR & ADC_ISR_EOC) == 0 ) {}

		// read data from data register into channel value array
		channelValues[channelIndex] = ADC1->DR;

		channelIndex++;
		channel = channelOrder[channelIndex];
	}

	// wait for end of sequence
	while ( (ADC1->ISR & ADC_ISR_EOS) == 0 ) {}

	// clear end of sequence flag
	ADC1->ISR |= ADC_ISR_EOS;
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
