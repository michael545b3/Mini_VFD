/*
 * main2.c
 *
 *  Created on: Jan 27, 2025
 *      Author: Michael Adam
 *      E-Mail: michael.adam4@proton.me
 */

// Includes
#include "main.h"
#include "vfd.h"
#include "dogl128.h"
#include "fonts.h"
#include "mgui.h"
#include "menu.h"
#include "eeprom.h"
#include "values.h"
#include "adc.h"
#include "tca9534.h"
#include "gpio.h"
#include "diagnosis.h"

// Defines
#define STARTUP_DELAY				1500

// Variables

extern IWDG_HandleTypeDef hiwdg;

uint8_t tca9534_ready = 1;

uint32_t main_clk_freq;

// Functions
// Init
void Init(void)
{
	uint32_t init_delay_counter = HAL_GetTick() + STARTUP_DELAY;
	uint32_t display_delay_counter = HAL_GetTick() + 500;
	main_clk_freq = HAL_RCC_GetSysClockFreq();

	GPIO_Init();

	VFD_Init();

	if (Values_Init() != SUCCESS)
	{
		Display_Fill(Display_COLOR_BLACK);
		Display_GotoXY(2, 2);
		Display_Puts("EEPROM ERROR", &Font_7x10, Display_COLOR_WHITE);
	}

	if (TCA9534_Init(TCA9534_ADDR_0, IO_EXPANDER_DIRECTION) != SUCCESS)
	{
		Display_Fill(Display_COLOR_BLACK);
		Display_GotoXY(2, 14);
		Display_Puts("IO_EXPANDER ERROR", &Font_7x10, Display_COLOR_WHITE);
		tca9534_ready = 0;
	}

	if (ADC_Init() != SUCCESS)
	{
		Display_Fill(Display_COLOR_BLACK);
		Display_GotoXY(2, 26);
		Display_Puts("ADC_REF ERROR", &Font_7x10, Display_COLOR_WHITE);
	}

	do
	{
		HAL_IWDG_Refresh(&hiwdg);
		if (display_delay_counter < HAL_GetTick())
		{
			display_delay_counter = 0;
		}
	} while (display_delay_counter);

	Display_Init();
	Menu_Init();
	Display_UpdateScreen();

	do
	{
		HAL_IWDG_Refresh(&hiwdg);
		if (init_delay_counter < HAL_GetTick())
		{
			init_delay_counter = 0;
		}
	} while (init_delay_counter);

	// Clear display
	Display_Clear();

	if (tca9534_ready)
	{
		Write_4LED(0,0,1,0);
	}
}

// Main Loop
void Loop(void)
{
	// Values
	uint8_t menu_mode_enabled = 0;
	uint8_t pb_pressed_menu_old = 0;
	uint8_t set_direction = SET_STOP;
	uint16_t pb_menu_counter = 0;
	uint32_t tick_counter = HAL_GetTick() + 100;


	// Main While Loop
	while (1)
	{
		// Refresh IWDG Timer
		HAL_IWDG_Refresh(&hiwdg);

		// VFD Handler
		VFD_Handler(set_frequency, set_direction);

		// IO Expander Routine
		if (tca9534_ready)
		{
			Read_PB_All();

			if(pb_pressed_stop || emergency_off)
			{
				Write_4LED(0, 0, 1, menu_mode_enabled);
				motor_status = MOTOR_STATE_STOP;
				set_direction = SET_STOP;
			}
			else
			{
				if (pb_pressed_fwd)
				{
					Write_4LED(1, 0, 0, menu_mode_enabled);
					motor_status = MOTOR_STATE_FWD;
					set_direction = SET_DIR_FWD;
				}
				if (pb_pressed_rev)
				{
					Write_4LED(0, 1, 0, menu_mode_enabled);
					motor_status = MOTOR_STATE_REV;
					set_direction = SET_DIR_REV;
				}
			}

			if(pb_pressed_menu)
			{
				if(pb_pressed_menu != pb_pressed_menu_old)
				{
					menu_mode_enabled = !menu_mode_enabled;
					Write_LED(menu_mode_enabled, LED_MENU);
					pb_menu_counter++;
					if(pb_menu_counter)
					{
						//debug mode here
					}
				}
			}
			else
			{
				pb_menu_counter = 0;
			}

			pb_pressed_menu_old = pb_pressed_menu;

			pb_pressed_fwd = 0;
			pb_pressed_rev = 0;
			pb_pressed_stop = 0;
			pb_pressed_menu = 0;

		}
		else
		{
			if (emergency_off)
			{
				motor_status = MOTOR_STATE_STOP;
				set_direction = SET_STOP;
			}
		}

		GPIO_Relay_Handler(RELAY_MODE_OFF);

		Menu_Encoder_Handler(menu_mode_enabled);

		Menu_Handler(menu_mode_enabled);

		Display_UpdateScreen();

		Diag_Check_Reference();

		do
		{
			if (tick_counter < HAL_GetTick())
			{
				tick_counter = 0;
			}
		} while (tick_counter);

		tick_counter = HAL_GetTick() + 100;
	}
}
