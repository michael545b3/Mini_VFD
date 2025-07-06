/*
 * menu.c
 *
 *  Created on: Mar 16, 2025
 *      Author: Michael Adam
 *      E-Mail: michael.adam4@proton.me
 */

// Includes
#include "main.h"
#include "values.h"
#include "fonts.h"
#include "mgui.h"
#include <stdio.h>

// Defines
#define PB_OK_PRESSED		1
#define ENCODER_INIT_COUNTER_VAL	10000

// Enumerators
enum
{
	MENU_MODE_RUN = 0, MENU_MODE_MENU, MENU_MODE_PARAM, MENU_MODE_DEBUG
};

enum
{
	MENU_P00 = 0, MENU_P01, MENU_P02, MENU_P03,
	MENU_P04, MENU_P05, MENU_P06, MENU_P07,
	MENU_P08, MENU_P09, MENU_P10, MENU_P11
};

// Values
extern TIM_HandleTypeDef htim4;

uint16_t maximal_encoder_value = 0;
int32_t encoder_setup;

// Local function prototypes

// Functions
void Encoder_Init(void)
{
	// IMPORTANT: Use Encoder Mode TI1 in CubeMX!
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COUNTER(&htim4, ENCODER_INIT_COUNTER_VAL);
}

void Menu_Init(void)
{
	Encoder_Init();
	Display_GotoXY(2, 2);
	Display_Puts("Mini VFD", &Font_7x10, Display_COLOR_WHITE);
	Display_GotoXY(2, 14);
	Display_Puts("FW Ver: 1.0.0", &Font_7x10, Display_COLOR_WHITE);
	Display_GotoXY(2, 32);
	Display_Puts("ADAM+", &Font_16x26, Display_COLOR_WHITE);
}

void Menu_Encoder_Handler(uint8_t encoder_mode)
{
	static int32_t encoder_accumulator = 0;
	static int32_t encoder_set_freq = 0;
	static uint32_t encoder_counter = ENCODER_INIT_COUNTER_VAL;
	static uint32_t encoder_counter_old = ENCODER_INIT_COUNTER_VAL;

	// Poll encoder value
	encoder_counter = __HAL_TIM_GET_COUNTER(&htim4);

	// Calculate difference from the old value
	int32_t difference = encoder_counter - encoder_counter_old;

	// Accumulate difference
	encoder_accumulator += difference;

	// Incremental accumulator
    while (encoder_accumulator >= 2) {
    	switch (encoder_mode)
    	{
    	case 0: // encoder_set_freq
    		encoder_set_freq++;
    		break;
    	case 1: // encoder_setup
    		encoder_setup++;
    		break;
    	}
        encoder_accumulator -= 2;
    }

    // Decremental accumulator
    while (encoder_accumulator <= -2) {
    	switch (encoder_mode)
    	{
    	case 0: // encoder_set_freq
    		encoder_set_freq--;
    		break;
    	case 1: // encoder_setup
    		encoder_setup--;
    		break;
    	}
        encoder_accumulator += 2;
    }
    encoder_counter_old = encoder_counter;

    // Check min/ max values on encoder_set_freq
	if(encoder_set_freq > param[MENU_P03].parameter_val)
	{
		encoder_set_freq = param[MENU_P03].parameter_val;
	}
	if(encoder_set_freq < (param[MENU_P02].parameter_val)) // min freq
	{
		encoder_set_freq = param[MENU_P02].parameter_val;
	}
	set_frequency = encoder_set_freq;

	// Check min/ max values on encoder_setup
	if (encoder_setup > maximal_encoder_value)
	{
		encoder_setup = maximal_encoder_value;
	}
	if (encoder_setup < 1)
	{
		encoder_setup = 0;
	}
}


// This function is the main menu handler function of the display, Input parameter is the menu pushbutton
void Menu_Handler(uint8_t menu_mode_enabled)
{
	static uint8_t previous_mode = 0xFF;
	static uint16_t previous_encoder = 0xFFFF;
	static uint8_t pb_ok_value;
	static uint8_t pb_ok_value_old;
	static uint8_t mode = MENU_MODE_RUN;
	static uint8_t selected_parameter = 0;

	pb_ok_value = HAL_GPIO_ReadPin(ENCODER_PB_GPIO_Port, ENCODER_PB_Pin);

	if(menu_mode_enabled == MENU_MODE_RUN)
	{
		mode = MENU_MODE_RUN;
	}
	else
	{
		if(previous_mode == MENU_MODE_RUN)
		{
			mode = MENU_MODE_MENU;
		}
	}

	switch (mode)
	{
	case MENU_MODE_RUN:
		if (previous_mode != MENU_MODE_RUN)
		{
			encoder_setup = 0;
			previous_mode = MENU_MODE_RUN;
			Display_Clear();
			Display_GotoXY(1, 1);
			Display_Puts("Freq:", &Font_11x18, Display_COLOR_WHITE);
			Display_GotoXY(1, 25);
			Display_Puts("Set.Fq:", &Font_7x10, Display_COLOR_WHITE);
			Display_GotoXY(1, 37);
			Display_Puts("Status:", &Font_7x10, Display_COLOR_WHITE);
			Display_GotoXY(1, 49);
			Display_Puts("V Bus:", &Font_7x10, Display_COLOR_WHITE);
			Display_DrawLine(0, 21, 127, 21, Display_COLOR_WHITE);
		}

		Print_Float(actual_frequency_display, &Font_11x18, 60, 1);

		Print_Float(set_frequency, &Font_7x10, 52, 25);

		//Print_Float(bus_voltage_meas, &Font_7x10, 95, 50);

		Display_GotoXY(52, 37);
		switch (motor_status)
		{
		case MOTOR_STATE_STOP:
			Display_Puts("Stop   ", &Font_7x10, Display_COLOR_WHITE);
			break;
		case MOTOR_STATE_FWD:
			Display_Puts("Forward", &Font_7x10, Display_COLOR_WHITE);
			break;
		case MOTOR_STATE_REV:
			Display_Puts("Reverse", &Font_7x10, Display_COLOR_WHITE);
			break;
		case MOTOR_STATE_ERROR:
			Display_Puts("Error  ", &Font_7x10, Display_COLOR_WHITE);
			break;
		}

		Print_Float(bus_voltage_meas, &Font_7x10, 52, 49);
		Display_Putc('V', &Font_7x10, Display_COLOR_WHITE);
		break;

	case MENU_MODE_MENU:
		char buf[30];

		if (previous_mode != MENU_MODE_MENU)
		{
			previous_mode = MENU_MODE_MENU;
			previous_encoder = 0xFFFF;
			maximal_encoder_value = 17;

			// Graphics + text
			Display_Clear();
			Display_DrawLine(0, 0, 127, 0, Display_COLOR_WHITE);
			Display_DrawLine(0, 14, 128, 14, Display_COLOR_WHITE);
			Display_DrawLine(32, 0, 32, 14, Display_COLOR_WHITE);
			Display_DrawLine(0, 0, 0, 14, Display_COLOR_WHITE);
			Display_DrawLine(127, 0, 127, 14, Display_COLOR_WHITE);
			Display_GotoXY(9, 20);
			Display_Puts("actual:", &Font_7x10, Display_COLOR_WHITE);

			Display_GotoXY(30, 32);
			Display_Puts("new:", &Font_7x10, Display_COLOR_WHITE);
			Display_GotoXY(2, 50);
			Display_Puts("default:", &Font_7x10, Display_COLOR_WHITE);
		}

		if (previous_encoder != encoder_setup)
		{
			previous_encoder = encoder_setup;

			Display_GotoXY(2, 3);
			sprintf(buf, "P%d ", encoder_setup);
			Display_Puts(buf, &Font_7x10, Display_COLOR_WHITE);
			sprintf(buf, "%s", param[encoder_setup].parameter_string);
			Display_GotoXY(34, 3);
			Display_Puts(buf, &Font_7x10, Display_COLOR_WHITE);

			// Print default and actual parameters
			Print_Float(param[encoder_setup].parameter_val, &Font_7x10, 60, 20);
			Print_Float(param[encoder_setup].default_val, &Font_7x10, 60, 50);
		}

		if(pb_ok_value_old != pb_ok_value)
		{
			pb_ok_value_old = pb_ok_value;

			if(pb_ok_value == PB_OK_PRESSED)
			{
				mode = MENU_MODE_PARAM;
				selected_parameter = encoder_setup;
			}
		}

		break;

	case MENU_MODE_PARAM:
		if(previous_mode != MENU_MODE_PARAM)
		{
			previous_mode = MENU_MODE_PARAM;
			encoder_setup = param[selected_parameter].parameter_val;
			previous_encoder = 0xFFFF;
			maximal_encoder_value = param[selected_parameter].max_val;
		}

		if(previous_encoder != encoder_setup)
		{
			previous_encoder = encoder_setup;

			Print_Float(encoder_setup, &Font_7x10, 60, 32);
		}

		if(pb_ok_value_old != pb_ok_value)
		{
			pb_ok_value_old = pb_ok_value;

			if(pb_ok_value == PB_OK_PRESSED)
			{
				mode = MENU_MODE_MENU;
				param[selected_parameter].parameter_val = encoder_setup;
				parameter_changed_flag = 1;
				encoder_setup = selected_parameter; // try to optimize away

				if(param[selected_parameter].max_val > 0XFF)
				{
					M24C0X_Write_Word(EEPROM_DEV_ADDR, param[selected_parameter].eeprom_addr, param[selected_parameter].parameter_val);
				}
				else
				{
					uint8_t byte_to_write = param[selected_parameter].parameter_val;
					M24C0X_Write_Byte(EEPROM_DEV_ADDR, param[selected_parameter].eeprom_addr, byte_to_write);
				}
			}
		}

		break;

	case MENU_MODE_DEBUG:
		if(previous_mode != MENU_MODE_DEBUG)
		{
			previous_mode = MENU_MODE_DEBUG;
			Display_Clear();
			Display_GotoXY(1, 1);
			sprintf(buf, "Debug_Mode");
			Display_Puts(buf, &Font_7x10, Display_COLOR_WHITE);
		}
		break;

	default:
		break;
	}
}
