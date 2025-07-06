/*
 * values.c
 *
 *  Created on: Jan 27, 2025
 *      Author: micha
 */

// Includes
#include "main.h"
#include "values.h"
#include <string.h>

// Defines
#define EEPROM_WRITE_DELAY				20 //ms




// Maximal Parameters
#define MAX_PARAM_P00					200 // 200Hz
#define MAX_PARAM_P01					250 // 250V
#define MAX_PARAM_P02					30 // 30Hz
#define MAX_PARAM_P03					200 // 200Hz
#define MAX_PARAM_P04					1000
#define MAX_PARAM_P05					1000
#define MAX_PARAM_P06					2
#define MAX_PARAM_P07					1
#define MAX_PARAM_P08					5
#define MAX_PARAM_P09					5
#define MAX_PARAM_P10					5
#define MAX_PARAM_P11					5
#define MAX_PARAM_P12					5
#define MAX_PARAM_P13					1
#define MAX_PARAM_P14					1
#define MAX_PARAM_P15					100
#define MAX_PARAM_P16					30

// Default Parameters
#define DEFAULT_PARAM_P00				50 // 50Hz
#define DEFAULT_PARAM_P01				230 // 230V
#define DEFAULT_PARAM_P02				5 // 5Hz
#define DEFAULT_PARAM_P03				100 // 100Hz
#define DEFAULT_PARAM_P04				500
#define DEFAULT_PARAM_P05				500
#define DEFAULT_PARAM_P06				0 // Local
#define DEFAULT_PARAM_P07				0 // Stop mode OFF
#define DEFAULT_PARAM_P08				0 // No function
#define DEFAULT_PARAM_P09				0 // No function
#define DEFAULT_PARAM_P10				0 // No function
#define DEFAULT_PARAM_P11				0 // No function
#define DEFAULT_PARAM_P12				0 // No function
#define DEFAULT_PARAM_P13				0 // Not forbidden
#define DEFAULT_PARAM_P14				0 // Low range
#define DEFAULT_PARAM_P15				30 // 30Hz
#define DEFAULT_PARAM_P16				0 // 0Hz

// EEPROM Addresses
#define EEPROM_INIT_ADDR				0x00
#define EEPROM_P00_FREQ_NOM				0x02
#define EEPROM_P01_VOLT_NOM				0x04
#define EEPROM_P02_FREQ_MIN				0x06
#define EEPROM_P03_FREQ_MAX				0x08
#define EEPROM_P04_RATE_ACC				0x0C
#define EEPROM_P05_RATE_DEC				0x0E
#define EEPROM_P06_FREQ_SOURCE			0x10
#define EEPROM_P07_STOP_MODE			0x11
#define EEPROM_P08_IN_A_FUNCTION		0x12
#define EEPROM_P09_IN_B_FUNCTION		0x13
#define EEPROM_P10_IN_C_FUNCTION		0x14
#define EEPROM_P11_IN_D_FUNCTION		0x15
#define EEPROM_P12_IN_E_FUNCTION		0x16
#define EEPROM_P13_REV_FORBID			0x17
#define EEPROM_P14_ANALOG_IN_RNG		0x18
#define EEPROM_P15_JUMP_FREQ			0x19
#define EEPROM_P16_JUMP_RNG				0x1B

// Values
uint8_t motor_enable = 0;
uint8_t motor_status = MOTOR_STATE_STOP;
uint8_t parameter_changed_flag = 0;
uint16_t set_voltage;
float bus_voltage_meas;
uint16_t motor_voltage_temp;
uint16_t adc_freq_raw;

// Parameter Values p..
uint16_t p00_freq_nominal;
uint16_t p01_voltage_nominal;
uint16_t p04_accel_rate;
uint16_t p05_decel_rate;
uint8_t p06_freq_source;
uint8_t p07_stop_mode;
uint8_t p08_in_a_function;
uint8_t p09_in_b_function;
uint8_t p10_in_c_function;
uint8_t p11_in_d_function;
uint8_t p12_in_e_function;
uint8_t p13_reverse_forbid;
uint8_t p14_analog_in_range;
uint16_t p15_jump_freq;
uint8_t p16_jump_rng;

// Array for maximal parameter values
const uint16_t max_values[] =
{
	MAX_PARAM_P00,
	MAX_PARAM_P01,
	MAX_PARAM_P02,
	MAX_PARAM_P03,
	MAX_PARAM_P04,
	MAX_PARAM_P05,
	MAX_PARAM_P06,
	MAX_PARAM_P07,
	MAX_PARAM_P08,
	MAX_PARAM_P09,
	MAX_PARAM_P10,
	MAX_PARAM_P11,
	MAX_PARAM_P12,
	MAX_PARAM_P13,
	MAX_PARAM_P14,
	MAX_PARAM_P15,
	MAX_PARAM_P16
};

// Array for default parameter values
const uint16_t default_values[] =
{
	DEFAULT_PARAM_P00,
	DEFAULT_PARAM_P01,
	DEFAULT_PARAM_P02,
	DEFAULT_PARAM_P03,
	DEFAULT_PARAM_P04,
	DEFAULT_PARAM_P05,
	DEFAULT_PARAM_P06,
	DEFAULT_PARAM_P07,
	DEFAULT_PARAM_P08,
	DEFAULT_PARAM_P09,
	DEFAULT_PARAM_P10,
	DEFAULT_PARAM_P11,
	DEFAULT_PARAM_P12,
	DEFAULT_PARAM_P13,
	DEFAULT_PARAM_P14,
	DEFAULT_PARAM_P15,
	DEFAULT_PARAM_P16
};

const uint8_t eeprom_address_values[] =
{
	EEPROM_P00_FREQ_NOM,
	EEPROM_P01_VOLT_NOM,
	EEPROM_P02_FREQ_MIN,
	EEPROM_P03_FREQ_MAX,
	EEPROM_P04_RATE_ACC,
	EEPROM_P05_RATE_DEC,
	EEPROM_P06_FREQ_SOURCE,
	EEPROM_P07_STOP_MODE,
	EEPROM_P08_IN_A_FUNCTION,
	EEPROM_P09_IN_B_FUNCTION,
	EEPROM_P10_IN_C_FUNCTION,
	EEPROM_P11_IN_D_FUNCTION,
	EEPROM_P12_IN_E_FUNCTION,
	EEPROM_P13_REV_FORBID,
	EEPROM_P14_ANALOG_IN_RNG,
	EEPROM_P15_JUMP_FREQ,
	EEPROM_P16_JUMP_RNG
};

const char *setup_strings[] =
{
	"Nom.Frequency",
	"Nom.Voltage  ",
	"Min.Frequency",
	"Max.Frequency",
	"Accel.Time   ",
	"Decel.Time   ",
	"Freq.Source  ",
	"Stop.Mode    ",
	"IN-A.Func    ",
	"IN-B.Func    ",
	"IN-C.Func    ",
	"IN-D.Func    ",
	"IN-E.Func    ",
	"Min.Voltage  ",
	"Max.Voltage  ",
	"Button.Lock  ",
	"Knob.Lock    ",
	"Knob.Dec     "
};

// Typedefs
parameter param[PARAMETER_COUNT];  // Array of 50 parameter structures

// Values
uint16_t actual_frequency_display;
float amplitude_divider_display;

uint8_t emergency_off = 0;

// Pushbutton Flags
uint8_t pb_pressed_fwd = 0;
uint8_t pb_pressed_rev = 0;
uint8_t pb_pressed_stop = 0;
uint8_t pb_pressed_menu = 0;

// DEBUG
uint16_t set_frequency;
uint16_t ampl_div;


// Functions
ErrorStatus Values_Init(void)
{
	if (M24C0X_Init(EEPROM_DEV_ADDR) != SUCCESS)
	{
		return ERROR;
	}

	for(uint8_t z = 0; z<PARAMETER_COUNT; z++)
	{
		strncpy(param[z].parameter_string, setup_strings[z], sizeof(param[z].parameter_string) - 1);
		param[z].max_val = max_values[z];
		param[z].default_val = default_values[z];
		param[z].eeprom_addr = eeprom_address_values[z];
	}

	// If there is a new EEPROM, write default values to the EEPROM
	if (M24C0X_Read_Byte(EEPROM_DEV_ADDR, EEPROM_INIT_ADDR) != 0x00)
	{
		HAL_Delay(1);
		for(uint8_t i = 0; i<PARAMETER_COUNT; i++)
		{
			if(param[i].max_val > 0XFF)
			{
				M24C0X_Write_Word(EEPROM_DEV_ADDR, param[i].eeprom_addr, param[i].default_val);
			}
			else
			{
				uint8_t byte_to_write = param[i].default_val;
				M24C0X_Write_Byte(EEPROM_DEV_ADDR, param[i].eeprom_addr, byte_to_write);
			}
			HAL_Delay(EEPROM_WRITE_DELAY);
		}

		M24C0X_Write_Byte(EEPROM_DEV_ADDR, EEPROM_INIT_ADDR, 0x00);
	}

	uint8_t error_counter_v = 0;

	for(uint8_t i = 0; i<PARAMETER_COUNT; i++)
	{
		HAL_Delay(1);

		if(param[i].max_val > 0xFF)
		{
			uint16_t temp_param;

			temp_param = M24C0X_Read_Word(EEPROM_DEV_ADDR, param[i].eeprom_addr);

			param[i].parameter_val = temp_param;

			if(param[i].parameter_val != param[i].default_val)
			{
				error_counter_v++;
			}
		}
		else
		{
			param[i].parameter_val = M24C0X_Read_Byte(EEPROM_DEV_ADDR, param[i].eeprom_addr);

			if(param[i].parameter_val != param[i].default_val)
			{
				error_counter_v++;
			}
		}
	}

	// @formatter:off
	p00_freq_nominal = 		param[0].parameter_val;
	p01_voltage_nominal = 	param[1].parameter_val;
	p04_accel_rate = 		param[4].parameter_val;
	p05_decel_rate = 		param[5].parameter_val;
	p06_freq_source = 		param[6].parameter_val;
	p07_stop_mode = 		param[7].parameter_val;
	p08_in_a_function = 	param[8].parameter_val;
	p09_in_b_function = 	param[9].parameter_val;
	p10_in_c_function = 	param[10].parameter_val;
	p11_in_d_function = 	param[11].parameter_val;
	p12_in_e_function =	 	param[12].parameter_val;
	p13_reverse_forbid = 	param[13].parameter_val;
	p14_analog_in_range = 	param[14].parameter_val;
	p15_jump_freq = 		param[15].parameter_val;
	p16_jump_rng = 			param[16].parameter_val;
	// @formatter:on

	return SUCCESS;
}
