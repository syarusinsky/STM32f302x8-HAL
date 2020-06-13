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
	SYSCLK_DIV_BY_2,
	SYSCLK_DIV_BY_4,
	SYSCLK_DIV_BY_8,
	SYSCLK_DIV_BY_16,
	SYSCLK_DIV_BY_32,
	SYSCLK_DIV_BY_64,
	SYSCLK_DIV_BY_128,
	SYSCLK_DIV_BY_256,
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
	I2C_2,
	I2C_3
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

class LLPD
{
	public:
		// SYSTEM CLOCK (pll setup is not currently working, need to debug why in the future, may just need VDDA and VSSA)
		// when using PLL as system clock, you should call rcc_pll_setup first
		static void rcc_clock_setup (const RCC_CLOCK_SOURCE& source, bool usePllAsSystemClock);
		static void rcc_pll_setup (const RCC_CLOCK_SOURCE& pllSource, bool pllDivideBy2, const RCC_PLL_MULTIPLY& pllMultiply);

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

		// SPI spi2( nss = b12, sck = b13, miso = b14, mosi = b15 )
		//     spi3( nss = a15, sck = b3,  miso = b4,  mosi = b5  )
		static void spi_master_init (const SPI_NUM& spiNum, const SPI_BAUD_RATE& baudRate, const SPI_CLK_POL& pol,
						const SPI_CLK_PHASE& phase, const SPI_DUPLEX& duplex,
						const SPI_FRAME_FORMAT& frameFormat, const SPI_DATA_SIZE& dataSize);
		static uint16_t spi_master_send_and_recieve (const SPI_NUM& spiNum, uint8_t data);

		// I2C i2c1( sda = b7,  scl = b6 )
		//     i2c2( sda = a10, scl = a9 )
		//     i2c3( sda = b5,  scl = a8 )
		//     timingRegVal can be calculated using this tool https://www.st.com/en/embedded-software/stsw-stm32126.html
		static void i2c_master_setup (const I2C_NUM& i2cNum, uint32_t timingRegVal);
		static void i2c_master_set_slave_address (const I2C_NUM& i2cNum, const I2C_ADDR_MODE& addrMode, uint16_t address);
		static void i2c_master_write (const I2C_NUM& i2cNum, bool setStopCondition, uint8_t numBytes, uint8_t bytes...);
		static void i2c_master_read (const I2C_NUM& i2cNum, bool setStopCondition, uint8_t numBytes, uint8_t* bytes...);

		// DAC
		static void dac_init (bool useVoltageBuffer);
		static void dac_send (uint16_t data);

		// ADC
		// initialization needs to take place after counter is started for tim6, since it uses delay function
		static void adc_init (const ADC_CYCLES_PER_SAMPLE& cyclesPerSample);
		static void adc_set_channel_order (uint8_t numChannels, const ADC_CHANNEL& channel...);
		static void adc_perform_conversion_sequence();
		static uint16_t adc_get_channel_value (const ADC_CHANNEL& channel); // 65535 is an invalid value for debugging
		static uint16_t adc_test();
};

#endif // LLPD_H