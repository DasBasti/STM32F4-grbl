#ifndef __cpu_map_stm32_h__
#define __cpu_map_stm32_h__

#ifdef CPU_MAP_STM32F103

// Define step pulse output pins. NOTE: All step bit pins must be on the same port.
#define STEP_PORT GPIOA
#define RCC_STEP_PORT RCC_APB2Periph_GPIOA
#define X_STEP_BIT 0
#define Y_STEP_BIT 1
#define Z_STEP_BIT 2
#define STEP_MASK ((1 << X_STEP_BIT) | (1 << Y_STEP_BIT) | (1 << Z_STEP_BIT)) // All step bits

// Define step direction output pins. NOTE: All direction pins must be on the same port.
#define DIRECTION_PORT GPIOA
#define RCC_DIRECTION_PORT RCC_APB2Periph_GPIOA
#define X_DIRECTION_BIT 3
#define Y_DIRECTION_BIT 4
#define Z_DIRECTION_BIT 5
#define DIRECTION_MASK ((1 << X_DIRECTION_BIT) | (1 << Y_DIRECTION_BIT) | (1 << Z_DIRECTION_BIT)) // All direction bits

// Define stepper driver enable/disable output pin.
#define STEPPERS_DISABLE_PORT GPIOA
#define RCC_STEPPERS_DISABLE_PORT RCC_APB2Periph_GPIOA
#define STEPPERS_DISABLE_BIT 6
#define STEPPERS_DISABLE_MASK (1 << STEPPERS_DISABLE_BIT)
#define SetStepperDisableBit() GPIO_SetBits(STEPPERS_DISABLE_PORT, STEPPERS_DISABLE_MASK)
#define ResetStepperDisableBit() GPIO_ResetBits(STEPPERS_DISABLE_PORT, STEPPERS_DISABLE_MASK)

// Define homing/hard limit switch input pins and limit interrupt vectors.
// NOTE: All limit bit pins must be on the same port
#define LIMIT_PIN GPIOB
#define LIMIT_PORT GPIOB
#define RCC_LIMIT_PORT RCC_APB2Periph_GPIOB
#define GPIO_LIMIT_PORT GPIO_PortSourceGPIOB
#define X_LIMIT_BIT 10
#define Y_LIMIT_BIT 11
#define Z_LIMIT_BIT 12

#define LIMIT_MASK ((1 << X_LIMIT_BIT) | (1 << Y_LIMIT_BIT) | (1 << Z_LIMIT_BIT)) // All limit bits

// Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PORT GPIOB
#define RCC_SPINDLE_ENABLE_PORT RCC_APB2Periph_GPIOB
#define SPINDLE_ENABLE_BIT 13 //
#ifndef USE_SPINDLE_DIR_AS_ENABLE_PIN
#define SPINDLE_DIRECTION_DDR GPIOB
#define SPINDLE_DIRECTION_PORT GPIOB
#define SPINDLE_DIRECTION_BIT 14 //
#endif
#define SetSpindleEnablebit() GPIO_WriteBit(SPINDLE_ENABLE_PORT, 1 << SPINDLE_ENABLE_BIT, Bit_SET)
#define ResetSpindleEnablebit() GPIO_WriteBit(SPINDLE_ENABLE_PORT, 1 << SPINDLE_ENABLE_BIT, Bit_RESET)
#define SetSpindleDirectionBit() GPIO_WriteBit(SPINDLE_DIRECTION_PORT, 1 << SPINDLE_DIRECTION_BIT, Bit_SET)
#define ResetSpindleDirectionBit() GPIO_WriteBit(SPINDLE_DIRECTION_PORT, 1 << SPINDLE_DIRECTION_BIT, Bit_RESET)

// Define flood and mist coolant enable output pins.
// a later date if flash and memory space allows.
#define COOLANT_FLOOD_PORT GPIOB
#define RCC_COOLANT_FLOOD_PORT RCC_APB2Periph_GPIOB
#define COOLANT_FLOOD_BIT 3
#define COOLANT_MIST_PORT GPIOB
#define RCC_COOLANT_MIST_PORT RCC_APB2Periph_GPIOB
#define COOLANT_MIST_BIT 4

// Define user-control controls (cycle start, reset, feed hold) input pins.
// NOTE: All CONTROLs pins must be on the same port and not on a port with other input pins (limits).
#define CONTROL_PIN_PORT GPIOB
#define CONTROL_PORT GPIOB
#define RCC_CONTROL_PORT RCC_APB2Periph_GPIOB
#define GPIO_CONTROL_PORT GPIO_PortSourceGPIOB
#define CONTROL_RESET_BIT 5
#define CONTROL_FEED_HOLD_BIT 6
#define CONTROL_CYCLE_START_BIT 7
#define CONTROL_SAFETY_DOOR_BIT 8
#define CONTROL_MASK ((1 << CONTROL_RESET_BIT) | (1 << CONTROL_FEED_HOLD_BIT) | (1 << CONTROL_CYCLE_START_BIT) | (1 << CONTROL_SAFETY_DOOR_BIT))

// Define probe switch input pin.
#define PROBE_PORT GPIOA
#define RCC_PROBE_PORT RCC_APB2Periph_GPIOA
#define PROBE_BIT 15
#define PROBE_MASK (1 << PROBE_BIT)

// Start of PWM & Stepper Enabled Spindle
#ifdef VARIABLE_SPINDLE

// NOTE: On the 328p, these must be the same as the SPINDLE_ENABLE settings.
#define SPINDLE_PWM_FREQUENCY 10000 // KHz
#define SPINDLE_PWM_DDR GPIOA
#define SPINDLE_PWM_PORT GPIOA
#define RCC_SPINDLE_PWM_PORT RCC_APB2Periph_GPIOA
#define SPINDLE_PWM_BIT 8
#endif // End of VARIABLE_SPINDLE
#define SPINDLE_PWM_MAX_VALUE (1000000 / SPINDLE_PWM_FREQUENCY)
#ifndef SPINDLE_PWM_MIN_VALUE
#define SPINDLE_PWM_MIN_VALUE 1 // Must be greater than zero.
#endif
#define SPINDLE_PWM_OFF_VALUE 0
#define SPINDLE_PWM_RANGE (SPINDLE_PWM_MAX_VALUE - SPINDLE_PWM_MIN_VALUE)

//  Port A                                         Port B
//   0      X_STEP_BIT
//   1      Y_STEP_BIT
//   2      Z_STEP_BIT
//   3      X_DIRECTION_BIT                       COOLANT_FLOOD_BIT
//   4      Y_DIRECTION_BIT                       COOLANT_MIST_BIT
//   5      Z_DIRECTION_BIT                       CONTROL_RESET_BIT
//   6      STEPPERS_DISABLE_BIT                  CONTROL_FEED_HOLD_BIT
//   7                                            CONTROL_CYCLE_START_BIT
//   8      SPINDLE_PWM_BIT                       CONTROL_SAFETY_DOOR_BIT
//   9
//   10                                            X_LIMIT_BIT
//   11                                            Y_LIMIT_BIT
//   12                                            Z_LIMIT_BIT
//   13 14 SWD																		SPINDLE_ENABLE_BIT
//     14																						SPINDLE_DIRECTION_BIT
//   15     PROBE_BIT

#endif // CPU_MAP_STM32F103

#ifdef CPU_MAP_STM32F407
#include "main.h" // include GPIO pin names from Cube project
// Since we do not have the stepper connected directly to the corresponding ports on the chip we need to work around this issue
// with a temporary buffer 'PORT'
// This 'PORT' is read and the correct pins are set high or low depending on the values in Timer3 ISR
extern volatile uint32_t ioPort;

// Define step pulse output pins. NOTE: All step bit pins must be on the same port.
#define X_STEP_BIT 0
#define Y_STEP_BIT 1
#define Z_STEP_BIT 2
#define A_STEP_BIT 3
#define B_STEP_BIT 4
#define STEP_MASK ((1 << X_STEP_BIT) | (1 << Y_STEP_BIT) | (1 << Z_STEP_BIT) | (1 << A_STEP_BIT)) // All step bits

// Define step direction output pins. NOTE: All direction pins must be on the same port.
#define X_DIRECTION_BIT 5
#define Y_DIRECTION_BIT 6
#define Z_DIRECTION_BIT 7
#define A_DIRECTION_BIT 8
#define B_DIRECTION_BIT 9
#define DIRECTION_MASK ((1 << X_DIRECTION_BIT) | (1 << Y_DIRECTION_BIT) | (1 << Z_DIRECTION_BIT) | (1 << A_DIRECTION_BIT)) // All direction bits

// Define stepper driver enable/disable output pin.
// We use HAL functions directly to disable the Stepper Pins from here. X/Y axis do not have enable pins.
#define SetStepperDisableBit() {\
	HAL_GPIO_WritePin(STP1_nEN_GPIO_Port, STP1_nEN_Pin, GPIO_PIN_SET);\
	HAL_GPIO_WritePin(STP2_nEN_GPIO_Port, STP1_nEN_Pin, GPIO_PIN_SET);\
}
#define ResetStepperDisableBit() {\
	HAL_GPIO_WritePin(STP1_nEN_GPIO_Port, STP1_nEN_Pin, GPIO_PIN_RESET);\
	HAL_GPIO_WritePin(STP2_nEN_GPIO_Port, STP1_nEN_Pin, GPIO_PIN_RESET);\
}

// Define homing/hard limit switch input pins and limit interrupt vectors.
// NOTE: All limit bit pins must be on the same port
// TODO: optimize this bit? or leave it if it works ok
#define X_LIMIT_BIT 10
#define Y_LIMIT_BIT 11
#define Z_LIMIT_BIT 12
#define A_LIMIT_BIT 13
#define B_LIMIT_BIT 14
// Axis A/B do not have limit switches
#define LIMIT_MASK ((1 << X_LIMIT_BIT) | (1 << Y_LIMIT_BIT) | (1 << Z_LIMIT_BIT) | (1 << A_LIMIT_BIT) | (1 << B_LIMIT_BIT)) // All limit bits
#define IGNORE_LIMITS_BIT 15

// Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PORT GPIOB
#define RCC_SPINDLE_ENABLE_PORT RCC_APB2Periph_GPIOB
#define SPINDLE_ENABLE_BIT 13 //
#ifndef USE_SPINDLE_DIR_AS_ENABLE_PIN
#define SPINDLE_DIRECTION_DDR GPIOB
#define SPINDLE_DIRECTION_PORT GPIOB
#define SPINDLE_DIRECTION_BIT 14 //
#endif
#define SetSpindleEnablebit() HAL_GPIO_WritePin(SPINDLE_ENABLE_PORT, 1 << SPINDLE_ENABLE_BIT, GPIO_PIN_SET)
#define ResetSpindleEnablebit() HAL_GPIO_WritePin(SPINDLE_ENABLE_PORT, 1 << SPINDLE_ENABLE_BIT, GPIO_PIN_RESET)
#define SetSpindleDirectionBit() HAL_GPIO_WritePin(SPINDLE_DIRECTION_PORT, 1 << SPINDLE_DIRECTION_BIT, GPIO_PIN_SET)
#define ResetSpindleDirectionBit() HAL_GPIO_WritePin(SPINDLE_DIRECTION_PORT, 1 << SPINDLE_DIRECTION_BIT, GPIO_PIN_RESET)

// Define flood and mist coolant enable output pins.
// a later date if flash and memory space allows.
#define COOLANT_FLOOD_PORT GPIOB
#define RCC_COOLANT_FLOOD_PORT RCC_APB2Periph_GPIOB
#define COOLANT_FLOOD_BIT 3
#define COOLANT_MIST_PORT GPIOB
#define RCC_COOLANT_MIST_PORT RCC_APB2Periph_GPIOB
#define COOLANT_MIST_BIT 4

// Define user-control controls (cycle start, reset, feed hold) input pins.
// NOTE: All CONTROLs pins must be on the same port and not on a port with other input pins (limits).
#define CONTROL_PIN_PORT ioPort;
#define CONTROL_FAST_BIT 28
#define CONTROL_TEACH_BIT 27
#define CONTROL_X_PLUS_BIT 26
#define CONTROL_X_MINUS_BIT 25
#define CONTROL_Y_PLUS_BIT 24
#define CONTROL_Y_MINUS_BIT 23
#define CONTROL_MASK ((1 << CONTROL_FAST_BIT) | (1 << CONTROL_TEACH_BIT) | (1 << CONTROL_X_PLUS_BIT) | (1 << CONTROL_X_MINUS_BIT) | (1 << CONTROL_Y_PLUS_BIT) | (1 << CONTROL_Y_MINUS_BIT))
// unused information
#define CONTROL_RESET_BIT 31
#define CONTROL_FEED_HOLD_BIT 30
#define CONTROL_CYCLE_START_BIT 29
//#define CONTROL_SAFETY_DOOR_BIT 28


// Define probe switch input pin.
#define PROBE_PORT GPIOA
#define RCC_PROBE_PORT RCC_APB2Periph_GPIOA
#define PROBE_BIT 15
#define PROBE_MASK (1 << PROBE_BIT)

// Start of PWM & Stepper Enabled Spindle
#ifdef VARIABLE_SPINDLE

// NOTE: On the 328p, these must be the same as the SPINDLE_ENABLE settings.
#define SPINDLE_PWM_FREQUENCY 10000 // KHz
#define SPINDLE_PWM_DDR GPIOA
#define SPINDLE_PWM_PORT GPIOA
#define RCC_SPINDLE_PWM_PORT RCC_APB2Periph_GPIOA
#define SPINDLE_PWM_BIT 8
#endif // End of VARIABLE_SPINDLE
#define SPINDLE_PWM_MAX_VALUE (1000000 / SPINDLE_PWM_FREQUENCY)
#ifndef SPINDLE_PWM_MIN_VALUE
#define SPINDLE_PWM_MIN_VALUE 1 // Must be greater than zero.
#endif
#define SPINDLE_PWM_OFF_VALUE 0
#define SPINDLE_PWM_RANGE (SPINDLE_PWM_MAX_VALUE - SPINDLE_PWM_MIN_VALUE)

//   IO Port                                          Port B
//   0      X_STEP_BIT
//   1      Y_STEP_BIT
//	 2		Z_STEP_BIT
//   3      A_STEP_BIT
//   4		B_STEP_BIT
//   5		X_DIRECTION_BIT
//   6      Y_DIRECTION_BIT
//   7      Z_DIRECTION_BIT
//   8		A_DIRECTION_BIT
//   9		B_DIRECTION_BIT
//   10		X_LIMIT
//   11		Y_LIMIT
//   12		Z_LIMIT
//   13		A_LIMIT
//   14		B_LIMIT
//   15		IGNORE_LIMITS
//   16
//   17
//   18
//   19
//   20
//   21
//   22
//   23		CONTROL_Y_MINUS
//   24		CONTROL_Y_PLUS
//   25		CONTROL_X_MINUS
//   26		CONTROL_X_PLUS
//   27		CONTROL_TEACH
//   28		CONTROL_FAST
//   29		CONTROL_CYCLE_START
//   30		CONTROL_FEED_HOLD
//   31     CONTROL_RESET
// NOTE: We include inline functions to convert from the hardware interface that grbl expects to one that is available in the machine

typedef struct{
	GPIO_TypeDef *port;
	uint32_t 	  pin;
	unsigned char key;
} gpio;

extern const gpio actuators[];
extern const uint8_t actuators_limit;
extern const gpio sensors[];
extern const uint8_t sensors_limit;

#define ACTUATE_PIN(PIN) if( ((uint32_t)PIN) < (actuators_limit) ){HAL_GPIO_WritePin(actuators[((uint32_t)PIN)].port, actuators[((uint32_t)PIN)].pin, GPIO_PIN_SET);}
#define DEACTUATE_PIN(PIN) if( ((uint32_t)PIN) < (actuators_limit) ){HAL_GPIO_WritePin(actuators[((uint32_t)PIN)].port, actuators[((uint32_t)PIN)].pin, GPIO_PIN_RESET);}
#define READ_PIN(PIN) HAL_GPIO_ReadPin(sensors[((uint32_t)PIN)].port, sensors[((uint32_t)PIN)].pin)
#define PIN_TO_CHAR(PIN) sensors[((uint32_t)PIN)].key



#endif // CPU_MAP_STM32F4xx
#endif
