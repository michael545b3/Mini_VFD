/*
 * values.h
 *
 *  Created on: Jan 27, 2025
 *      Author: Michael Adam
 *      E-Mail: michael.adam4@proton.me
 */

#ifndef INC_VALUES_H_
#define INC_VALUES_H_

// Includes
#include "main.h"
#include "eeprom.h"

// Defines
#define DEFAULT_MOTOR_FREQ_NOMINAL		500
#define DEFAULT_MAX_FREQUENCY 			1200	// 120Hz
#define DEFAULT_MIN_FREQUENCY 			50		// 5Hz
#define DEFAULT_MOTOR_VOLT_NOMINAL		230
#define DEFAULT_MOTOR_RATE_ACC			10
#define DEFAULT_MOTOR_RATE_DEC			10

#define PARAMETER_COUNT					18

#define EEPROM_DEV_ADDR					M24C02_ADDR_0

// Enumerators
enum
{
	MOTOR_STATE_STOP = 0, MOTOR_STATE_FWD, MOTOR_STATE_REV, MOTOR_STATE_ERROR
};

// Typedefs
typedef struct
{
	char parameter_string[15];
	uint16_t parameter_val;
	uint16_t default_val;
	uint16_t max_val;
	uint8_t eeprom_addr;
} parameter;

// Variables
extern uint8_t motor_enable;
extern uint8_t motor_status;
extern uint8_t parameter_changed_flag;
//extern uint16_t set_voltage;
extern float bus_voltage_meas;
extern uint16_t p00_freq_nominal;
extern uint16_t p01_voltage_nominal;
extern uint16_t p02_freq_minimal;
extern uint16_t p03_freq_maximal;
extern uint16_t p04_accel_rate;
extern uint16_t p05_decel_rate;
extern uint8_t p06_freq_source;
extern uint8_t p07_stop_mode;
extern uint8_t p08_in_a_function;
extern uint8_t p09_in_b_function;
extern uint8_t p10_in_c_function;
extern uint8_t p11_in_d_function;
extern uint8_t p12_in_e_function;
extern uint8_t p13_reverse_forbid;
extern uint8_t p14_analog_in_range;
extern uint16_t p15_jump_freq;
extern uint8_t p16_jump_rng;

extern uint16_t motor_voltage_temp;
extern uint16_t adc_freq_raw;

extern uint16_t actual_frequency_display;
extern float amplitude_divider_display;

extern uint8_t emergency_off;

// Pushbutton Flags
extern uint8_t pb_pressed_fwd;
extern uint8_t pb_pressed_rev;
extern uint8_t pb_pressed_stop;
extern uint8_t pb_pressed_menu;


// DEBUG
extern uint16_t set_frequency;
extern uint16_t ampl_div;

// Typedefs
extern parameter param[PARAMETER_COUNT];  // Array of 50 parameter structures

// Function prototypes
ErrorStatus Values_Init(void);

#endif /* INC_VALUES_H_ */
