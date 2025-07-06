/*
 * adc.c
 *
 *  Created on: Jan 27, 2025
 *      Author: Michael Adam
 *      E-Mail: michael.adam4@proton.me
 */

// Includes
#include "main.h"
#include "vfd.h"
#include "adc.h"
#include "values.h"
#include "diagnosis.h"

// Defines
#define ADC_BUF_LEN			4
#define VBUS_CALC_FACTOR	323.1842628f

// Typedefs
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;

// Values
volatile uint16_t adc_buf[ADC_BUF_LEN];

uint16_t adc_val_v_in;
uint16_t adc_val_i_in;
uint16_t adc_val_vref;
uint16_t adc_val_v_bus;

// Enumerators
enum
{
	V_IN_ADC = 0, I_IN_ADC, VREF_ADC, V_BUS_ADC
};

// Functions
// Start ADC
void ADC_Start_Conversion(void)
{
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_buf, ADC_BUF_LEN);
}

uint8_t ADC_Init(void)
{
	// Start ADC DMA
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_buf, ADC_BUF_LEN);

	// Start ADC conversion timer
	HAL_TIM_Base_Start(&htim2);

	// Wait until the ADC finishes
	HAL_Delay(5);

	// Check if the external reference is working
	if (Diag_Check_Reference() != SUCCESS)
	{
		HAL_TIM_Base_Stop(&htim2);
		HAL_ADC_Stop_DMA(&hadc1);
		return ERROR;
	}

	return SUCCESS;
}

uint16_t ADC_Get_V_IN(void)
{
	return adc_val_v_in;
}

uint16_t ADC_Get_I_IN(void)
{
	return adc_val_i_in;
}

uint16_t ADC_Get_VREF(void)
{
	return adc_val_vref;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	// Save ADC values
	adc_val_v_in = adc_buf[V_IN_ADC]; 	// Analog_IN
	adc_val_i_in = adc_buf[I_IN_ADC]; 	// 4-20mA IN
	adc_val_vref = adc_buf[VREF_ADC]; 	// VREF INT
	adc_val_v_bus = adc_buf[V_BUS_ADC];	// V_Bus Measure

	// Calculate vbus voltage
	bus_voltage_meas = (float) adc_val_v_bus*(VREF_EXT/ADC_MAX_VAL) * VBUS_CALC_FACTOR;

	Motor_LUT_Stepper();
}
