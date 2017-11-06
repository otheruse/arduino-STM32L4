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

#ifndef _VARIANT_OTHERUSE1_STM32L433RC_
#define _VARIANT_OTHERUSE1_STM32L433RC_

// The definitions here needs a STM32L4 core >=1.6.6
#define ARDUINO_STM32L4_VARIANT_COMPLIANCE 10606

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

#define STM32L4_CONFIG_LSECLK             32768
#define STM32L4_CONFIG_HSECLK             0
#define STM32L4_CONFIG_SYSOPT             0


#define STM32L4_CONFIG_USB_VBUS           GPIO_PIN_NONE

#define STM32L4_CONFIG_DAP_SWCLK          GPIO_PIN_PB15
#define STM32L4_CONFIG_DAP_SWDIO          GPIO_PIN_PB8

#define USBCON

/** Master clock frequency */
#define VARIANT_MCK			  F_CPU

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
#include "USBAPI.h"
#include "Uart.h"
#endif // __cplusplus

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

#define PA3		(0)
#define PA2		(1)
#define PA10	(2)
#define PB3		(3)
#define PB5		(4)
#define PB4		(5)
#define PB10	(6)
#define PA8		(7)
#define PA9		(8)
#define PC7		(9)
#define PB6		(10)
#define PA7		(11)
#define PA6		(12)
#define PA5		(13)
#define PB9		(14)
#define PB8		(15)
#define PA0		(16)
#define PA1		(17)
#define PA4		(18)
#define PB0		(19)
#define PC1		(20)
#define PC0		(21)
#define PA11	(22)
#define PA12	(23)
#define PA15	(24)
#define PB1		(25)
#define PB2		(26)
#define PB7		(27)
#define PB12	(28)
#define PB13	(29)
#define PB14	(30)
#define PB15	(31)
#define PC2		(32)
#define PC3		(33)
#define PC4		(34)
#define PC5		(35)
#define PC6		(36)
#define PC8		(37)
#define PC9		(38)
#define PC10	(39)
#define PC11	(40)
#define PC12	(41)
#define PC13	(42)
#define PD2		(43)
#define PH0		(44)
#define PH1		(45)
#define PB11	(46)

// Number of pins defined in PinDescription array
#define PINS_COUNT           (47u)
#define NUM_DIGITAL_PINS     (38u)
#define NUM_TOTAL_PINS       (47u)
#define NUM_ANALOG_INPUTS    (8u)
#define NUM_ANALOG_OUTPUTS   (2u)
#define analogInputToDigitalPin(p)  ((p < 6u) ? (p) + 16u : ((p < 8) ? (p) + 26 : -1))

// LEDs
#define PIN_LED              (PB12)
#define LED_BUILTIN          PIN_LED

/*
 * Analog pins
 */
#define PIN_A0               (16u)
#define PIN_A1               (17u)
#define PIN_A2               (18u)
#define PIN_A3               (19u)
#define PIN_A4               (20u)
#define PIN_A5               (21u)
#define PIN_A6               (33u)
#define PIN_A7               (34u)
#define PIN_DAC0             (18u)
#define PIN_DAC1             (13u)

static const uint8_t A0  = PIN_A0;
static const uint8_t A1  = PIN_A1;
static const uint8_t A2  = PIN_A2;
static const uint8_t A3  = PIN_A3;
static const uint8_t A4  = PIN_A4;
static const uint8_t A5  = PIN_A5;
static const uint8_t A6  = PIN_A6;
static const uint8_t A7  = PIN_A7;
static const uint8_t DAC0 = PIN_DAC0;
static const uint8_t DAC1 = PIN_DAC1;
#define ADC_RESOLUTION		12
#define DAC_RESOLUTION		12

// Other pins

#define PIN_BUTTON           (36u)
static const uint8_t BUTTON = PIN_BUTTON;
/*
 * Serial interfaces
 */

#define SERIAL_INTERFACES_COUNT 4

#define PIN_SERIAL1_RX       (0ul)
#define PIN_SERIAL1_TX       (1ul)

#define PIN_SERIAL2_RX       (6ul)
#define PIN_SERIAL2_TX       (46ul)

#define PIN_SERIAL3_RX       (2ul)
#define PIN_SERIAL3_TX       (8ul)

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 3

#define PIN_SPI_MOSI         (PC12)
#define PIN_SPI_MISO         (PC11)
#define PIN_SPI_SCK          (PC10)

#define PIN_SPI1_MISO        (PB4)
#define PIN_SPI1_MOSI        (PB5)
#define PIN_SPI1_SCK         (PB3)

#define PIN_SPI2_MISO        (PA6)
#define PIN_SPI2_MOSI        (PA7)
#define PIN_SPI2_SCK         (PA1)

//static const uint8_t SS	  = 10;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (PB9)
#define PIN_WIRE_SCL         (PB8)
static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

/*
 * USB
 */
//#define PIN_USB_VBUS         (25u) // Pin 25 in V1.2 board!
#define PIN_USB_VBUS         GPIO_PIN_NONE
#define PIN_USB_DM           (22u)
#define PIN_USB_DP           (23u)

#define PWM_INSTANCE_COUNT   4

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
extern CDC  Serial;
extern Uart Serial1;
extern Uart Serial2;
extern Uart Serial3;
#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_USBVIRTUAL      Serial
#define SERIAL_PORT_MONITOR         Serial
#define SERIAL_PORT_HARDWARE1       Serial1
#define SERIAL_PORT_HARDWARE2       Serial2
#define SERIAL_PORT_HARDWARE3       Serial3
#define SERIAL_PORT_HARDWARE_OPEN1  Serial1
#define SERIAL_PORT_HARDWARE_OPEN2  Serial2
#define SERIAL_PORT_HARDWARE_OPEN3  Serial3

// Alias SerialUSB to Serial
#define SerialUSB SERIAL_PORT_USBVIRTUAL

#endif /* _VARIANT_OTHERUSE1_STM32L433RC_ */

