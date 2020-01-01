/*
  grbl.h - main Grbl include file
  Part of Grbl

  Copyright (c) 2015-2016 Sungeun K. Jeon for Gnea Research LLC

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef grbl_h
#define grbl_h

// Grbl versioning system
#define GRBL_VERSION "1.1f"
#define GRBL_VERSION_BUILD "20170801"

#if !defined(STM32F103C8) && !defined(STM32F407xx)
#define AVRTARGET
#endif

// Define standard libraries used by Grbl.
#ifdef AVRTARGET
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <inttypes.h>
#include <stdbool.h>
#define PORTPINDEF uint8_t
#endif
#include <math.h>
#ifdef STM32F103C8
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_tim.h"
#include "misc.h"
#define PSTR(x) x
#define pgm_read_byte_near(x) *(x)
void _delay_ms(uint32_t x);
void _delay_us(uint32_t x);
#define false 0
#define true 1
#define PORTPINDEF uint16_t
typedef int bool;
//#define NOEEPROMSUPPORT
#define printPgmString printString
#endif
#ifdef STM32F407xx
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_exti.h"
#include "stm32f4xx_hal_tim.h"
extern UART_HandleTypeDef huart2;
#define PSTR(x) x
#define pgm_read_byte_near(x) *(x)
void _delay_ms(uint32_t x);
void _delay_us(uint32_t x);
#define false 0
#define true 1
#define PORTPINDEF uint16_t
typedef int bool;
//#define NOEEPROMSUPPORT
#define printPgmString printString
// TODO: (basneu) Read and Write GPIOS from STEPDIR value
//#define GPIO_ReadInputData(PORT) (uint16_t)PORT->IDR & 0xffff
//#define GPIO_ReadOutputData(PORT) (uint16_t)PORT->ODR & 0xffff

#endif

#include <string.h>
#include <stdlib.h>
#include <stdint.h>

// Define the Grbl system include files. NOTE: Do not alter organization.
#include "config.h"
#include "nuts_bolts.h"
#include "settings.h"
#include "system.h"
#include "defaults.h"
#include "cpu_map.h"
#include "planner.h"
#include "coolant_control.h"
#include "eeprom.h"
#include "gcode.h"
#include "limits.h"
#include "motion_control.h"
#include "planner.h"
#include "print.h"
#include "probe.h"
#include "protocol.h"
#include "report.h"
#include "serial.h"
#include "spindle_control.h"
#include "stepper.h"
#include "jog.h"

// ---------------------------------------------------------------------------------------
// COMPILE-TIME ERROR CHECKING OF DEFINE VALUES:

#ifndef HOMING_CYCLE_0
  #error "Required HOMING_CYCLE_0 not defined."
#endif

#if defined(USE_SPINDLE_DIR_AS_ENABLE_PIN) && !defined(VARIABLE_SPINDLE)
  #error "USE_SPINDLE_DIR_AS_ENABLE_PIN may only be used with VARIABLE_SPINDLE enabled"
#endif

#if defined(USE_SPINDLE_DIR_AS_ENABLE_PIN) && !defined(CPU_MAP_ATMEGA328P)
  #error "USE_SPINDLE_DIR_AS_ENABLE_PIN may only be used with a 328p processor"
#endif

#if !defined(USE_SPINDLE_DIR_AS_ENABLE_PIN) && defined(SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED)
	#error "SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED may only be used with USE_SPINDLE_DIR_AS_ENABLE_PIN enabled"
#endif

#if defined(PARKING_ENABLE)
  #if defined(HOMING_FORCE_SET_ORIGIN)
    #error "HOMING_FORCE_SET_ORIGIN is not supported with PARKING_ENABLE at this time."
  #endif
#endif

#if defined(ENABLE_PARKING_OVERRIDE_CONTROL)
	#if !defined(PARKING_ENABLE)
		#error "ENABLE_PARKING_OVERRIDE_CONTROL must be enabled with PARKING_ENABLE."
	#endif
#endif

#if defined(SPINDLE_PWM_MIN_VALUE)
  #if !(SPINDLE_PWM_MIN_VALUE > 0)
    #error "SPINDLE_PWM_MIN_VALUE must be greater than zero."
  #endif
#endif

#if (REPORT_WCO_REFRESH_BUSY_COUNT < REPORT_WCO_REFRESH_IDLE_COUNT)
  #error "WCO busy refresh is less than idle refresh."
#endif
#if (REPORT_OVR_REFRESH_BUSY_COUNT < REPORT_OVR_REFRESH_IDLE_COUNT)
  #error "Override busy refresh is less than idle refresh."
#endif
#if (REPORT_WCO_REFRESH_IDLE_COUNT < 2)
  #error "WCO refresh must be greater than one."
#endif
#if (REPORT_OVR_REFRESH_IDLE_COUNT < 1)
  #error "Override refresh must be greater than zero."
#endif
// ---------------------------------------------------------------------------------------

#endif
