/**
 * Copyright (c) 2016 - 2018, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
#ifndef CUSTOM_BOARD_H
#define CUSTOM_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_gpio.h"
#include "ovo_thermostat_config.h"

#ifdef OVO_THERMOSTAT_BOARD
#define LEDS_NUMBER    1
#else
// LEDs definitions for nRF52840-MDK
#define LEDS_NUMBER    3
#endif //OVO_THERMOSTAT_BOARD

#ifdef OVO_THERMOSTAT_BOARD
#define LED_1          NRF_GPIO_PIN_MAP(0,13)
#else
#define LED_1          NRF_GPIO_PIN_MAP(0,22)
#define LED_2          NRF_GPIO_PIN_MAP(0,23)
#define LED_3          NRF_GPIO_PIN_MAP(0,24)
#endif //OVO_THERMOSTAT_BOARD

#define LED_START      LED_1
#ifdef OVO_THERMOSTAT_BOARD
#define LED_STOP       LED_1
#else
#define LED_STOP       LED_3
#endif //OVO_THERMOSTAT_BOARD

#define LEDS_ACTIVE_STATE 0

#ifdef OVO_THERMOSTAT_BOARD
#define LEDS_LIST { LED_1 }
#else
#define LEDS_LIST { LED_1, LED_2, LED_3 }
#endif //OVO_THERMOSTAT_BOARD

#define LEDS_INV_MASK  LEDS_MASK

#ifdef OVO_THERMOSTAT_BOARD
#define BSP_LED_0      13
#else
#define BSP_LED_0      22
#define BSP_LED_1      23
#define BSP_LED_2      24
#endif //OVO_THERMOSTAT_BOARD

#define BUTTONS_NUMBER 1

#ifdef OVO_THERMOSTAT_BOARD
#define BUTTON_1       NRF_GPIO_PIN_MAP(1,8)
#else
#define BUTTON_1       NRF_GPIO_PIN_MAP(1,0)
#endif //OVO_THERMOSTAT_BOARD
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_ACTIVE_STATE 0

#define BUTTONS_LIST { BUTTON_1 }

#define BSP_BUTTON_0   BUTTON_1

#ifdef OVO_THERMOSTAT_BOARD
#define RX_PIN_NUMBER  NRF_GPIO_PIN_MAP(1,1)
#define TX_PIN_NUMBER  NRF_GPIO_PIN_MAP(1,2)
#else
#define RX_PIN_NUMBER  19
#define TX_PIN_NUMBER  20
#define CTS_PIN_NUMBER 7
#define RTS_PIN_NUMBER 5
#define HWFC           false
#endif //OVO_THERMOSTAT_BOARD

#define BSP_QSPI_SCK_PIN   NRF_GPIO_PIN_MAP(1,3)
#define BSP_QSPI_CSN_PIN   NRF_GPIO_PIN_MAP(1,6)
#define BSP_QSPI_IO0_PIN   NRF_GPIO_PIN_MAP(1,5)
#define BSP_QSPI_IO1_PIN   NRF_GPIO_PIN_MAP(1,4)
#define BSP_QSPI_IO2_PIN   NRF_GPIO_PIN_MAP(1,2)
#define BSP_QSPI_IO3_PIN   NRF_GPIO_PIN_MAP(1,1)

// Arduino board mappings
#define ARDUINO_SCL_PIN             27    // SCL signal pin
#define ARDUINO_SDA_PIN             26    // SDA signal pin
#define ARDUINO_AREF_PIN            2     // Aref pin

// SPI Pins
#define SPI_SS_PIN         NRF_GPIO_PIN_MAP(1,104)
#define SPI_MOSI_PIN       NRF_GPIO_PIN_MAP(0,15)
#define SPI_SCK_PIN        NRF_GPIO_PIN_MAP(0,17)

// Power Gating pins

#define POWER_GATE_PINS_NUMBER    1

#if POWER_GATE_PINS_NUMBER > 0
#define POWER_GATE_SHT30_PIN_IDX      0
#define POWER_GATE_SHT30_PIN          NRF_GPIO_PIN_MAP(0,20)

#define POWER_GATE_LP55231_PIN_IDX    1
#define POWER_GATE_LP55231_PIN        NRF_GPIO_PIN_MAP(1,3)

#define POWER_GATE_SENSOR_TOP_PIN_IDX 2
#define POWER_GATE_SENSOR_TOP_PIN     NRF_GPIO_PIN_MAP(0,12)

#define POWER_GATE_BOOST_PIN_IDX      3
#define POWER_GATE_BOOST_PIN          NRF_GPIO_PIN_MAP(1,7)

#define POWER_GATE_PIN_LIST      {POWER_GATE_SHT30_PIN, POWER_GATE_LP55231_PIN}

#define POWER_GATE_ACTIVE_STATE  1
#endif // POWER_GATE_PINS_NUMBER > 0
#ifdef __cplusplus
}
#endif

#endif // CUSTOM_BOARD_H
