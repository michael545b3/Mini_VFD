/*
 * diagnosis.c
 *
 *  Created on: Jan 27, 2025
 *      Author: Michael Adam
 *      E-Mail: michael.adam4@proton.me
 */

// Includes
#include "main.h"
#include "adc.h"

// Defines
#define VREF_INT_MAX		2200 // Max VREFINT value following datasheet: 1.24V
#define VREF_INT_MIN		1700 // Min VREFINT value following datasheet: 1.18V

// Functions
// Check if the external reference is working, by checking the internal reference
uint8_t Diag_Check_Reference(void)
{
	uint16_t vref_int = ADC_Get_VREF();

	if ((vref_int > VREF_INT_MAX) || (vref_int < VREF_INT_MIN))
	{
		return ERROR;
	}
	return SUCCESS;
}

