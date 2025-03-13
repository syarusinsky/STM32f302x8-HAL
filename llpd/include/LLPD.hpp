#ifndef LLPD_H
#define LLPD_H

#include <stdint.h>

enum class RCC_CLOCK_SOURCE
{
	INTERNAL,
	EXTERNAL
};

enum class RCC_PLL_MULTIPLY
{
	NONE,
	BY_2,
	BY_3,
	BY_4,
	BY_5,
	BY_6,
	BY_7,
	BY_8,
	BY_9,
	BY_10,
	BY_11,
	BY_12,
	BY_13,
	BY_14,
	BY_15,
	BY_16
};

enum class RCC_AHB_PRES
{
	BY_1,
	BY_2,
	BY_4,
	BY_8,
	BY_16,
	BY_64,
	BY_128,
	BY_256,
	BY_512
};

enum class RCC_APB1_PRES
{
	AHB_BY_1,
	AHB_BY_2,
	AHB_BY_4,
	AHB_BY_8,
	AHB_BY_16
};

enum class RCC_APB2_PRES
{
	AHB_BY_1,
	AHB_BY_2,
	AHB_BY_4,
	AHB_BY_8,
	AHB_BY_16
};

enum class GPIO_PORT
{
	A,
	B,
	C,
	F,
};

enum class GPIO_PIN
{
	PIN_0  = 0,
	PIN_1  = 1,
	PIN_2  = 2,
	PIN_3  = 3,
	PIN_4  = 4,
	PIN_5  = 5,
	PIN_6  = 6,
	PIN_7  = 7,
	PIN_8  = 8,
	PIN_9  = 9,
	PIN_10 = 10,
	PIN_11 = 11,
	PIN_12 = 12,
	PIN_13 = 13,
	PIN_14 = 14,
	PIN_15 = 15,
};

enum class GPIO_OUTPUT_TYPE
{
	PUSH_PULL,
	OPEN_DRAIN
};

enum class GPIO_OUTPUT_SPEED
{
	LOW,
	MEDIUM,
	HIGH
};

enum class GPIO_PUPD
{
	NONE,
	PULL_UP,
	PULL_DOWN
};

enum class SPI_NUM
{
	SPI_2,
	SPI_3
};

enum class SPI_BAUD_RATE
{
	APB1CLK_DIV_BY_2,
	APB1CLK_DIV_BY_4,
	APB1CLK_DIV_BY_8,
	APB1CLK_DIV_BY_16,
	APB1CLK_DIV_BY_32,
	APB1CLK_DIV_BY_64,
	APB1CLK_DIV_BY_128,
	APB1CLK_DIV_BY_256,
};

enum class SPI_CLK_POL
{
	LOW_IDLE,
	HIGH_IDLE
};

enum class SPI_CLK_PHASE
{
	FIRST,
	SECOND
};

enum class SPI_DUPLEX
{
	FULL,
	HALF
};

enum class SPI_FRAME_FORMAT
{
	MSB_FIRST,
	LSB_FIRST
};

enum class SPI_DATA_SIZE
{
	BITS_4,
	BITS_5,
	BITS_6,
	BITS_7,
	BITS_8,
	BITS_9,
	BITS_10,
	BITS_11,
	BITS_12,
	BITS_13,
	BITS_14,
	BITS_15,
	BITS_16
};

enum class I2C_NUM
{
	I2C_1,
	I2C_2
#if defined( STM32F302X8 )
	, I2C_3
#endif
};

enum class I2C_ADDR_MODE
{
	BITS_7,
	BITS_10
};

enum class ADC_CYCLES_PER_SAMPLE
{
	CPS_1p5, 	// 1.5
	CPS_2p5, 	// 2.5
	CPS_4p5, 	// 4.5
	CPS_7p5, 	// 7.5
	CPS_19p5, 	// 19.5
	CPS_61p5, 	// 61.5
	CPS_181p5, 	// 181.5
	CPS_601p5 	// 601.5
};

enum class ADC_CHANNEL
{
	CHAN_1,
	CHAN_2,
	CHAN_3,
	CHAN_4,
	CHAN_5,
	CHAN_6,
	CHAN_7,
	CHAN_8,
	CHAN_9,
	CHAN_10,
	CHAN_11,
	CHAN_12,
	CHAN_13,
	CHAN_14,
	CHAN_15,
	CHAN_16,
	CHAN_17,
	CHAN_18,
	CHAN_INVALID
};

enum class USART_NUM
{
	USART_1,
	USART_2,
	USART_3
};

enum class USART_WORD_LENGTH
{
#if defined( STM32F302X8 )
	BITS_7,
#endif
	BITS_8,
	BITS_9
};

enum class USART_PARITY
{
	ODD,
	EVEN
};

enum class USART_CONF
{
	TX_ONLY,
	RX_ONLY,   // Recieve interrupt will be enabled
	TX_AND_RX  // Recieve interrupt will be enabled
};

enum class USART_STOP_BITS
{
	BITS_1,
	BITS_2
};

class LLPD
{
	public:
		// RCC
		static void rcc_clock_setup (const RCC_CLOCK_SOURCE& source, unsigned int freq);
		static void rcc_clock_setup (const RCC_CLOCK_SOURCE& pllSource, bool hseDivBy2, const RCC_PLL_MULTIPLY& pllMultiply,
						unsigned int freq); 	// use this function for pll as system clock, note that hsi as pll source
									// will be divided by 2
		static void rcc_pll_enable (const RCC_CLOCK_SOURCE& pllSource, bool hseDivBy2, const RCC_PLL_MULTIPLY& pllMultiply);
		static void rcc_pll_disable();
		static void rcc_set_periph_clock_prescalers (const RCC_AHB_PRES &ahbPres, const RCC_APB1_PRES &apb1Pres,
								const RCC_APB2_PRES &apb2Pres ); // note that APB1 clock must be <= 36MHz

		// GPIO
		static void gpio_enable_clock (const GPIO_PORT& port);
		static void gpio_digital_input_setup (const GPIO_PORT& port, const GPIO_PIN& pin, const GPIO_PUPD& pupd,
						bool alternateFunc = false);
		static void gpio_analog_setup (const GPIO_PORT& port, const GPIO_PIN& pin);
		static void gpio_output_setup (const GPIO_PORT& port, const GPIO_PIN& pin, const GPIO_PUPD& pupd,
						const GPIO_OUTPUT_TYPE& type, const GPIO_OUTPUT_SPEED& speed,
						bool alternateFunc = false);
		static bool gpio_input_get (const GPIO_PORT& port, const GPIO_PIN& pin);
		static void gpio_output_set (const GPIO_PORT& port, const GPIO_PIN& pin, bool set);

		// TIM6
		static void tim6_counter_setup (uint32_t prescalerDivisor, uint32_t cyclesPerInterrupt, uint32_t interruptRate);
		static void tim6_counter_enable_interrupts();
		static void tim6_counter_disable_interrupts();
		static void tim6_counter_start();
		static void tim6_counter_stop();
		static void tim6_counter_clear_interrupt_flag();
		static void tim6_delay (uint32_t microseconds); // delay function may not be 100% accurate
		static bool tim6_isr_handle_delay(); 	// To use delay functions, this function needs to be in the tim6 isr.
							// It will return true if a delay is not finished, or false if it is.

		// TIM3
		static void tim3_counter_setup (uint32_t prescalerDivisor, uint32_t cyclesPerInterrupt, uint32_t interruptRate);
		static void tim3_counter_enable_interrupts();
		static void tim3_counter_disable_interrupts();
		static void tim3_counter_start();
		static void tim3_counter_stop();
		static void tim3_counter_clear_interrupt_flag();
		static void tim3_delay (uint32_t microseconds); // delay function may not be 100% accurate
		static bool tim3_isr_handle_delay(); 	// To use delay functions, this function needs to be in the tim6 isr.
							// It will return true if a delay is not finished, or false if it is.
		static void tim3_sync_to_tim6(); // syncs tim3's counter to tim6's counter (tim3 will actually lag behind a tiny bit)

		// SPI spi2( nss = b12, sck = b13, miso = b14, mosi = b15 )
		//     spi3( nss = a15, sck = b3,  miso = b4,  mosi = b5  ) // spi3 can't use dma
		static void spi_master_init (const SPI_NUM& spiNum, const SPI_BAUD_RATE& baudRate, const SPI_CLK_POL& pol,
						const SPI_CLK_PHASE& phase, const SPI_DUPLEX& duplex,
						const SPI_FRAME_FORMAT& frameFormat, const SPI_DATA_SIZE& dataSize,
						const bool enableDmaChannels = false);
		static uint16_t spi_master_send_and_recieve (const SPI_NUM& spiNum, uint8_t data);
		static bool spi2_dma_start (uint8_t* txBuffer, uint8_t* rxBuffer,
						unsigned int bufferSize); // returns false if failed
		static void spi2_dma_wait_for_transfer_complete();
		static void spi2_dma_stop();
		static void spi2_look_at_registers();

		// I2C i2c1( sda = b7,  scl = b6 )
		//     i2c2( sda = a10, scl = a9 )
		//     i2c3( sda = b5,  scl = a8 ) i2c3 is not available on stm32f302xc
		//     timingRegVal can be calculated using this tool https://www.st.com/en/embedded-software/stsw-stm32126.html
		static void i2c_master_setup (const I2C_NUM& i2cNum, uint32_t timingRegVal);
		static void i2c_master_set_slave_address (const I2C_NUM& i2cNum, const I2C_ADDR_MODE& addrMode, uint16_t address);
		static void i2c_master_write (const I2C_NUM& i2cNum, bool setStopCondition, uint8_t numBytes, uint8_t bytes...);
		static void i2c_master_read (const I2C_NUM& i2cNum, bool setStopCondition, uint8_t numBytes, uint8_t* bytes...);
		static void i2c_master_read_into_array (const I2C_NUM& i2cNum, bool setStopCondition, uint8_t numBytes, uint8_t* arr);

		// DAC dac1( vout = a4 )
		static void dac_init (bool useVoltageBuffer);
		static void dac_init_use_dma (bool useVoltageBuffer, uint16_t numSamples, uint16_t* buffer);
		static void dac_send (uint16_t data); // don't use if using dma
		static void dac_dma_start();
		static void dac_dma_stop();
		static uint16_t dac_dma_get_num_transfers_left(); // returns the number of dma transfers left in the buffer

		// ADC
		// initialization requires PLL
		// initialization needs to take place after counter is started for tim6, since it uses delay function
		// dma'ing a single channel to a memory location will also require tim3 as a trigger
		static void adc_init (const ADC_CYCLES_PER_SAMPLE& cyclesPerSample);
		static void adc_set_channel_order (bool tim6Trig, // whether or not to trigger conversions by tim6
							uint8_t numChannels,
							ADC_CHANNEL chanToDma, // the channel you want to dma to dmaLoc, needs tim3Trig
							uint32_t* dmaLoc, // the memory location you want to dma to, or nullptr if not desired
							uint32_t dmaLocSize, // circular buffer size
							const ADC_CHANNEL& channel...); // channels to convert given in order of conversion
		static void adc_perform_conversion_sequence(); // do not use if triggering conversions by tim6
		static uint16_t adc_get_channel_value (const ADC_CHANNEL& channel); // 65535 is an invalid value for debugging
		static void adc_dma_start(); // starts the dma transfers for chanToDma
		static void adc_dma_stop(); // stops the dma transfers for chanToDma
		static uint16_t adc_dma_get_num_transfers_left(); // returns the number of dma transfers left in the buffer for chanToDma

		// USART usart1( tx = b6, rx = b7 )
		//       usart2( tx = b3, rx = b4 )
		//       usart3( tx = b9, rx = b8 ) for stm32f302x8 OR ( tx = b10, rx = b11 ) for stm32f302xc
		static void usart_init (const USART_NUM& usartNum, const USART_WORD_LENGTH& wordLen, const USART_PARITY& parity,
					const USART_CONF& conf, const USART_STOP_BITS& stopBits, const unsigned int periphClockFreq,
					const unsigned int baudRate);
		static void usart_transmit (const USART_NUM& usartNum, uint16_t data);
		static uint16_t usart_receive (const USART_NUM& usartNum);
		static void usart_log (const USART_NUM& usartNum, const char* cStr); // needs to be proper c string with terminator
		static void usart_log_int (const USART_NUM& usartNum, const char* cStr, int val);
		static void usart_log_float (const USART_NUM& usartNum, const char* cStr, float val);

		// Op Amp opamp2( v+ = a7, v- = a5, vout = a6 )
		static void opamp_init();
};

#endif // LLPD_H
