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

#ifndef _STM32L4_WIRING_PRIVATE_
#define _STM32L4_WIRING_PRIVATE_

#include "Arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx.h"

#include "stm32l4_adc.h"
#include "stm32l4_dac.h"
#include "stm32l4_exti.h"
#include "stm32l4_timer.h"
#include "stm32l4_gpio.h"
#include "stm32l4_uart.h"
#include "stm32l4_i2c.h"
#include "stm32l4_servo.h"
#include "stm32l4_spi.h"
#include "stm32l4_usbd_cdc.h"
#include "stm32l4_system.h"
#include "stm32l4_rtc.h"

#define STM32L4_SVCALL_IRQ_PRIORITY  15
#define STM32L4_PENDSV_IRQ_PRIORITY  15

#define STM32L4_ADC_IRQ_PRIORITY     15
#define STM32L4_DAC_IRQ_PRIORITY     15
#define STM32L4_PWM_IRQ_PRIORITY     15

#define STM32L4_USB_IRQ_PRIORITY     14
#define STM32L4_RTC_IRQ_PRIORITY     13
#define STM32L4_I2C_IRQ_PRIORITY     12
#define STM32L4_UART_IRQ_PRIORITY    11
#define STM32L4_SPI_IRQ_PRIORITY     10

#define STM32L4_EXTI_IRQ_PRIORITY    4
#define STM32L4_SYSTICK_IRQ_PRIORITY 3
#define STM32L4_TONE_IRQ_PRIORITY    2
#define STM32L4_SERVO_IRQ_PRIORITY   1

extern void USBD_Attach(unsigned int pin_vusb, unsigned int priority);

/************************************************************************
 * DMA map:
 * 
 * DMA_CHANNEL_DMA1_CH1_ADC1
 * DMA_CHANNEL_DMA1_CH2_USART3_TX
 * DMA_CHANNEL_DMA1_CH3_USART3_RX
 * DMA_CHANNEL_DMA1_CH4_SPI2_RX / DMA_CHANNEL_DMA1_CH4_DFSDM0
 * DMA_CHANNEL_DMA1_CH5_SPI2_TX / DMA_CHANNEL_DMA1_CH5_DFSDM1 
 * DMA_CHANNEL_DMA1_CH6_SAI2_A
 * DMA_CHANNEL_DMA1_CH7_I2C1_RX
 * DMA_CHANNEL_DMA2_CH1_SPI3_RX
 * DMA_CHANNEL_DMA2_CH2_SPI3_TX
 * DMA_CHANNEL_DMA2_CH3_SPI1_RX
 * DMA_CHANNEL_DMA2_CH4_SPI1_TX
 * DMA_CHANNEL_DMA2_CH5_UART4_RX
 * DMA_CHANNEL_DMA2_CH6_SAI1_A
 * DMA_CHANNEL_DMA2_CH7_QUADSPI
 *
 ************************************************************************/

/************************************************************************
 * TIM map:
 * 
 * TIM1    PWM
 * TIM2    CAPTURE
 * TIM3    PWM
 * TIM4    PWM
 * TIM5    PWM
 * TIM6    (ADC)
 * TIM7    TONE
 * TIM8
 * TIM15   SERVO
 * TIM16   IR
 * TIM17   IR
 * 
 ************************************************************************/

extern stm32l4_adc_t stm32l4_adc;
extern stm32l4_exti_t stm32l4_exti;

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* _STM32L4_WIRING_PRIVATE_ */