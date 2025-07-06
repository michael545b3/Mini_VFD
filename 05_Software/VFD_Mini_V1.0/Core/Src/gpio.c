/*
 * gpio.c
 *
 *  Created on: Mar 29, 2025
 *      Author: Michael Adam
 *      E-Mail: michael.adam4@proton.me
 */

#include "main.h"
#include "values.h"
#include "gpio.h"

void GPIO_Init(void)
{
	HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, RESET);
}

void GPIO_Relay_Handler(uint8_t mode)
{
	switch (mode)
	{
	case RELAY_MODE_ERROR:

		break;
	case RELAY_MODE_RUN:
		if (actual_frequency_display > 0)
		{
			HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, SET);
		}
		else
		{
			HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, RESET);
		}
		break;
	case RELAY_MODE_OFF:
		HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, RESET);
		break;
	default:
		HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, RESET);
		break;
	}
}
