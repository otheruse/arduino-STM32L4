/*
 * Copyright (c) 2016 Thomas Roell.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of Thomas Roell, nor the names of its contributors
 *     may be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

#include "Arduino.h"
#include "stm32l4_wiring_private.h"

#define PWM_INSTANCE_TIM1      0
#define PWM_INSTANCE_TIM2      1
#define PWM_INSTANCE_TIM15     2
#define PWM_INSTANCE_TIM16     3

/*
 * Pins descriptions
 */
extern const PinDescription g_APinDescription[NUM_TOTAL_PINS] =
{
	// 0..13 - Digital pins
	{ GPIOA, GPIO_PIN_MASK(GPIO_PIN_PA3),  GPIO_PIN_PA3,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_INPUT_NONE },
	{ GPIOA, GPIO_PIN_MASK(GPIO_PIN_PA2),  GPIO_PIN_PA2,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_INPUT_NONE },
	{ GPIOA, GPIO_PIN_MASK(GPIO_PIN_PA10), GPIO_PIN_PA10_TIM1_CH3,  (PIN_ATTR_PWM | PIN_ATTR_EXTI),                PWM_INSTANCE_TIM1,  PWM_CHANNEL_3,    ADC_INPUT_NONE },
	{ GPIOB, GPIO_PIN_MASK(GPIO_PIN_PB3),  GPIO_PIN_PB3_TIM2_CH2,   (PIN_ATTR_PWM | PIN_ATTR_EXTI),                PWM_INSTANCE_TIM2,  PWM_CHANNEL_2,    ADC_INPUT_NONE },
	{ GPIOB, GPIO_PIN_MASK(GPIO_PIN_PB5),  GPIO_PIN_PB5,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_INPUT_NONE },
	{ GPIOB, GPIO_PIN_MASK(GPIO_PIN_PB4),  GPIO_PIN_PB14_TIM15_CH1, (PIN_ATTR_PWM | PIN_ATTR_EXTI),                PWM_INSTANCE_TIM15, PWM_CHANNEL_1,    ADC_INPUT_NONE },
	{ GPIOB, GPIO_PIN_MASK(GPIO_PIN_PB10), GPIO_PIN_PB10_TIM2_CH3,  (PIN_ATTR_PWM | PIN_ATTR_EXTI),                PWM_INSTANCE_TIM2,  PWM_CHANNEL_3,    ADC_INPUT_NONE },
	{ GPIOA, GPIO_PIN_MASK(GPIO_PIN_PA8),  GPIO_PIN_PA8_TIM1_CH1,   (PIN_ATTR_PWM | PIN_ATTR_EXTI),                PWM_INSTANCE_TIM1,  PWM_CHANNEL_1,    ADC_INPUT_NONE },
	{ GPIOA, GPIO_PIN_MASK(GPIO_PIN_PA9),  GPIO_PIN_PA9_TIM1_CH2,   (PIN_ATTR_PWM | PIN_ATTR_EXTI),                PWM_INSTANCE_TIM1,  PWM_CHANNEL_2,    ADC_INPUT_NONE },
	{ GPIOC, GPIO_PIN_MASK(GPIO_PIN_PC7),  GPIO_PIN_PC7,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_INPUT_NONE },
	{ GPIOB, GPIO_PIN_MASK(GPIO_PIN_PB6),  GPIO_PIN_PB8_TIM16_CH1,  (PIN_ATTR_PWM | PIN_ATTR_EXTI),                PWM_INSTANCE_TIM16, PWM_CHANNEL_1,    ADC_INPUT_NONE },
	{ GPIOA, GPIO_PIN_MASK(GPIO_PIN_PA7),  GPIO_PIN_PA7,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_INPUT_NONE },
	{ GPIOA, GPIO_PIN_MASK(GPIO_PIN_PA6),  GPIO_PIN_PA6,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_INPUT_NONE },
	{ GPIOA, GPIO_PIN_MASK(GPIO_PIN_PA5),  GPIO_PIN_PA5_TIM2_CH1,   (PIN_ATTR_PWM | PIN_ATTR_DAC | PIN_ATTR_EXTI), PWM_INSTANCE_TIM2,  PWM_CHANNEL_1,    ADC_INPUT_NONE },

	// 14..15 - I2C pins (SDA,SCL)
	{ GPIOB, GPIO_PIN_MASK(GPIO_PIN_PB9),  GPIO_PIN_PB9,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_INPUT_NONE },
	{ GPIOB, GPIO_PIN_MASK(GPIO_PIN_PB8),  GPIO_PIN_PB8,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_INPUT_NONE },

	// 16..21 - Analog pins
	{ GPIOA, GPIO_PIN_MASK(GPIO_PIN_PA0),  GPIO_PIN_PA0,            (PIN_ATTR_ADC),                                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_INPUT_5    },
	{ GPIOA, GPIO_PIN_MASK(GPIO_PIN_PA1),  GPIO_PIN_PA1,            (PIN_ATTR_ADC),                                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_INPUT_6    },
    { GPIOA, GPIO_PIN_MASK(GPIO_PIN_PA4),  GPIO_PIN_PA4,            (PIN_ATTR_ADC | PIN_ATTR_DAC | PIN_ATTR_EXTI), PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_INPUT_9    },
	{ GPIOB, GPIO_PIN_MASK(GPIO_PIN_PB0),  GPIO_PIN_PB0,            (PIN_ATTR_ADC),                                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_INPUT_15   },
	{ GPIOC, GPIO_PIN_MASK(GPIO_PIN_PC1),  GPIO_PIN_PC1,            (PIN_ATTR_ADC),                                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_INPUT_2    },
	{ GPIOC, GPIO_PIN_MASK(GPIO_PIN_PC0),  GPIO_PIN_PC0,            (PIN_ATTR_ADC),                                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_INPUT_1    },

	// 22..46 - Internally connected pins
	// 22 - A11 - USB_DM
	{ GPIOA, GPIO_PIN_MASK(GPIO_PIN_PA11), GPIO_PIN_PA11,           (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_INPUT_NONE },
	// 23 - A12 - USB_DP
	{ GPIOA, GPIO_PIN_MASK(GPIO_PIN_PA12), GPIO_PIN_PA12,           (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_INPUT_NONE },
	// 24 - A15 - CHARGING
	{ GPIOA, GPIO_PIN_MASK(GPIO_PIN_PA15), GPIO_PIN_PA15,           (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_INPUT_NONE },
	// 25 - B1 - VUSB
	{ GPIOB, GPIO_PIN_MASK(GPIO_PIN_PB1),  GPIO_PIN_PB1,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_INPUT_NONE },
	// 26 - B2 - BMP280_CS
	{ GPIOB, GPIO_PIN_MASK(GPIO_PIN_PB2),  GPIO_PIN_PB2,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_INPUT_NONE },
	// 27 - B7 - RADIO_RI
	{ GPIOB, GPIO_PIN_MASK(GPIO_PIN_PB7),  GPIO_PIN_PB7,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_INPUT_NONE },
	// 28 - B12 - LED
	{ GPIOB, GPIO_PIN_MASK(GPIO_PIN_PB12), GPIO_PIN_PB12,           (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_INPUT_NONE },
	// 29 - B13 - FLASH_SCK
	{ GPIOB, GPIO_PIN_MASK(GPIO_PIN_PB13), GPIO_PIN_PB13,           (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_INPUT_NONE },
	// 30 - B14 - FLASH_MISO
	{ GPIOB, GPIO_PIN_MASK(GPIO_PIN_PB14), GPIO_PIN_PB14,           (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_INPUT_NONE },
	// 31 - B15 - FLASH_MOSI
	{ GPIOB, GPIO_PIN_MASK(GPIO_PIN_PB15), GPIO_PIN_PB15,           (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_INPUT_NONE },
	// 32 - C2 - BT_IRQ
	{ GPIOC, GPIO_PIN_MASK(GPIO_PIN_PC2),  GPIO_PIN_PC2,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_INPUT_NONE },
	// 33 - C3 - VBAT (ANALOG)
	{ GPIOC, GPIO_PIN_MASK(GPIO_PIN_PC3),  GPIO_PIN_PC3,            (PIN_ATTR_ADC),                                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_INPUT_4    },
	// 34 - C4 - VSUPPLY (ANALOG)
	{ GPIOC, GPIO_PIN_MASK(GPIO_PIN_PC4),  GPIO_PIN_PC4,            (PIN_ATTR_ADC),                                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_INPUT_13   },
	// 35 - C5 -VBAT/VSUPPY ENABLE
	{ GPIOC, GPIO_PIN_MASK(GPIO_PIN_PC5),  GPIO_PIN_PC5,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_INPUT_NONE },
	// 36 - C6 - BUTTON
	{ GPIOC, GPIO_PIN_MASK(GPIO_PIN_PC6),  GPIO_PIN_PC6,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_INPUT_NONE },
	// 37 - C8 - FLASH_CS
	{ GPIOC, GPIO_PIN_MASK(GPIO_PIN_PC8),  GPIO_PIN_PC8,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_INPUT_NONE },
	// 38 - C9 - PERIPH_ENABLE
	{ GPIOC, GPIO_PIN_MASK(GPIO_PIN_PC9),  GPIO_PIN_PC9,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_INPUT_NONE },
	// 39 - C10 - ACCEL_INT1
	{ GPIOC, GPIO_PIN_MASK(GPIO_PIN_PC10), GPIO_PIN_PC10,           (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_INPUT_NONE },
	// 40 - C11 - ACCEL_INT2
	{ GPIOC, GPIO_PIN_MASK(GPIO_PIN_PC11), GPIO_PIN_PC11,           (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_INPUT_NONE },
	// 41 - C12 - RADIO_RESET
	{ GPIOC, GPIO_PIN_MASK(GPIO_PIN_PC12), GPIO_PIN_PC12,           (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_INPUT_NONE },
	// 42 - C13 - SHIELD_ENABLE
	{ GPIOC, GPIO_PIN_MASK(GPIO_PIN_PC13), GPIO_PIN_PC13,           (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_INPUT_NONE },
	// 43 - D2 - BT_RESET
	{ GPIOD, GPIO_PIN_MASK(GPIO_PIN_PD2),  GPIO_PIN_PD2,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_INPUT_NONE },
	// 44 - H0 - BT_CS
	{ GPIOH, GPIO_PIN_MASK(GPIO_PIN_PH0),  GPIO_PIN_PH0,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_INPUT_NONE },
	// 45 - H1 - BT_ENABLE
	{ GPIOH, GPIO_PIN_MASK(GPIO_PIN_PH1),  GPIO_PIN_PH1,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_INPUT_NONE },
	// 46 - B11 - RADIO_TX
	{ GPIOB, GPIO_PIN_MASK(GPIO_PIN_PB11), GPIO_PIN_PB11,           (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_INPUT_NONE },

};

extern const unsigned int g_PWMInstances[PWM_INSTANCE_COUNT] = {
    TIMER_INSTANCE_TIM1,
    TIMER_INSTANCE_TIM2,
    TIMER_INSTANCE_TIM15,
    TIMER_INSTANCE_TIM16,
};

// UART
extern const stm32l4_uart_pins_t g_SerialPins = { GPIO_PIN_PA3_USART2_RX, GPIO_PIN_PA2_USART2_TX, GPIO_PIN_NONE, GPIO_PIN_NONE };
extern const unsigned int g_SerialInstance = UART_INSTANCE_USART2;
extern const unsigned int g_SerialMode = 0;

extern const stm32l4_uart_pins_t g_Serial2Pins = { GPIO_PIN_PB10_LPUART1_RX, GPIO_PIN_PB11_LPUART1_TX, GPIO_PIN_NONE, GPIO_PIN_NONE };
extern const unsigned int g_Serial2Instance = UART_INSTANCE_LPUART1;
extern const unsigned int g_Serial2Mode = 0;

extern const stm32l4_uart_pins_t g_Serial3Pins = { GPIO_PIN_PA10_USART1_RX, GPIO_PIN_PA9_USART1_TX, GPIO_PIN_NONE, GPIO_PIN_NONE };
extern const unsigned int g_Serial3Instance = UART_INSTANCE_USART1;
extern const unsigned int g_Serial3Mode = UART_MODE_RX_DMA | UART_MODE_TX_DMA;

// SPI
extern const stm32l4_spi_pins_t g_SPI2ins = { GPIO_PIN_PB15_SPI2_MOSI, GPIO_PIN_PB14_SPI2_MISO, GPIO_PIN_PB13_SPI2_SCK, GPIO_PIN_NONE };
extern const unsigned int g_SPIInstance = SPI_INSTANCE_SPI2;
extern const unsigned int g_SPIMode = 0;


extern const stm32l4_spi_pins_t g_SPI2Pins = { GPIO_PIN_PA7_SPI1_MOSI, GPIO_PIN_PA6_SPI1_MISO, GPIO_PIN_PA1_SPI1_SCK, GPIO_PIN_NONE };
extern const unsigned int g_SPI2Instance = SPI_INSTANCE_SPI1;
extern const unsigned int g_SPI2Mode = SPI_MODE_RX_DMA | SPI_MODE_TX_DMA | SPI_MODE_RX_DMA_SECONDARY | SPI_MODE_TX_DMA_SECONDARY;

extern const stm32l4_spi_pins_t g_SPI1Pins = { GPIO_PIN_PB5_SPI3_MOSI, GPIO_PIN_PB4_SPI3_MISO, GPIO_PIN_PB3_SPI3_SCK, GPIO_PIN_NONE };
extern const unsigned int g_SPI1Instance = SPI_INSTANCE_SPI3;
extern const unsigned int g_SPI1Mode = SPI_MODE_RX_DMA | SPI_MODE_TX_DMA | SPI_MODE_RX_DMA_SECONDARY | SPI_MODE_TX_DMA_SECONDARY;


// I2C
extern const stm32l4_i2c_pins_t g_WirePins = { GPIO_PIN_PB8_I2C1_SCL, GPIO_PIN_PB9_I2C1_SDA };
extern const unsigned int g_WireInstance = I2C_INSTANCE_I2C1;
extern const unsigned int g_WireMode = I2C_MODE_RX_DMA;

