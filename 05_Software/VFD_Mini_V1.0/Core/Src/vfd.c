/*
 * vfd.c
 *
 *  Created on: Jan 27, 2025
 *      Author: micha
 */

// Includes
#include <math.h>

#include "vfd.h"
#include "main.h"
#include "values.h"

// Defines
#define MAIN_CLK_FREQ			168000000
#define LUT_SIZE				16384/2

#define VBUS_UVLO_LEVEL			250.0f

// Values
uint8_t timers_started = 0;
uint16_t point_count;
uint16_t lut[LUT_SIZE];
uint16_t num_points = LUT_SIZE; // Setze die Gesamtzahl der Punkte in der LUT (z.B. 360 oder eine andere Größe)
uint8_t vfd_direction = SET_DIR_FWD;
uint16_t actual_frequency;
float amplitudeDivider = 1;

float amplitudeReduction = 1;

// Typedefs
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim14;

// Functions
void Motor_Timers_Stop(void)
{
	HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, RESET);

	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);

	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

	timers_started = 0;
}

uint8_t Motor_Timers_Start(void)
{
	if (timers_started == 1)
	{
		return SUCCESS;
	}

	uint8_t error_flag = 0;

	// Start pwm timers
	if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
		error_flag = ERROR;
	if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
		error_flag = ERROR;
	if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)
		error_flag = ERROR;
	if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
		error_flag = ERROR;
	if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
		error_flag = ERROR;
	if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)
		error_flag = ERROR;

	__HAL_TIM_SET_COUNTER(&htim14, 1);
	__HAL_TIM_CLEAR_FLAG(&htim14, TIM_FLAG_UPDATE);
	HAL_TIM_Base_Start_IT(&htim14);

	if (error_flag == ERROR)
	{
		Motor_Timers_Stop();
		return ERROR;
	}

	HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, SET);

	timers_started = 1;

	return SUCCESS;
}

// Fill the LUT table with sine values
void Generate_LUT(uint16_t lut_size, uint16_t max_value)
{
	max_value /= 2;

	for (uint16_t i = 0; i < lut_size; i++)
	{
		double angle = 2.0 * M_PI * i / lut_size;  // Use double for all constants

		lut[i] = (uint16_t) ((sin(angle) * max_value) + max_value); // Sine, scaled to uint16_t
	}
}

void Timer_actual_frequency(uint16_t freq_set) //
{
	uint32_t arr_val;
	uint16_t psc_val = 0;
	uint8_t prescaler_mached = 1;

	do
	{
		arr_val = MAIN_CLK_FREQ / (freq_set * (psc_val + 1));
		if (arr_val > 65535)
		{
			psc_val = psc_val + 1;
		}
		else
		{
			prescaler_mached = 0;
		}

	} while (prescaler_mached);

	TIM2->PSC = psc_val;
	TIM2->ARR = arr_val;
}

void VFD_Init(void)
{
	// Generate LUT
	Generate_LUT(LUT_SIZE, htim1.Init.Period);

	// Compute point count
	point_count = sizeof(lut) / sizeof(uint16_t);

	// Stop PWM generators
	Motor_Timers_Stop();
}

VFD_StatusTypeDef VFD_Handler(uint16_t target_frequency, uint8_t set_direction)
{
	static uint8_t stop_mode_flag = 0;
	static int16_t current_frequency = 0;  // Actual motor frequency value
	static uint32_t last_tick = 0;

	if (HAL_GPIO_ReadPin(EMERGENCY_GPIO_Port, EMERGENCY_Pin) == GPIO_PIN_RESET)
	{
		emergency_off = 1;
	}

	// Check if a STOP order came
	if (((set_direction == SET_STOP) && p07_stop_mode) || emergency_off)
	{
		Motor_Timers_Stop();
		stop_mode_flag = 1;
	}
	else if (stop_mode_flag && (vfd_direction == set_direction))
	{
		Motor_Timers_Start();
		stop_mode_flag = 0;
	}

	// Hold the target frequency at 0 while the set direction doesn't match the actual VFD direction
	if (vfd_direction != set_direction)
	{
		target_frequency = 0;
	}

	// Change VFD direction ONLY if the actual frequency is 0
	if (actual_frequency == 0)
	{
		switch (set_direction)
		{
		case SET_STOP:
			vfd_direction = SET_DIR_FWD;
			Motor_Timers_Stop();
			break;
		case SET_DIR_FWD:
			vfd_direction = SET_DIR_FWD;
			Motor_Timers_Start();
			break;
		case SET_DIR_REV:
			vfd_direction = SET_DIR_REV;
			Motor_Timers_Start();
			break;
		default:
			break;
		}
	}

	// Calculate the time-difference between the actual and last tick
	// Get the actual time tick
	uint32_t tick_counter = HAL_GetTick();
	uint32_t tick_diff = tick_counter - last_tick;
	last_tick = tick_counter;

	// Check if the target frequency has changed
	if (target_frequency > current_frequency) // Acceleration
	{
		uint16_t acceleration_step = param[4].parameter_val * tick_diff / 1000;  // Acceleration per tick
		current_frequency += acceleration_step;
		if (current_frequency > target_frequency)
		{
			current_frequency = target_frequency; // If the set-point is exceeded, set it to the set-point.
		}
	}
	else if (target_frequency < current_frequency) // Deceleration
	{
		uint16_t deceleration_step = param[5].parameter_val * tick_diff / 1000;  // Deceleration per tick
		current_frequency -= deceleration_step;
		if (current_frequency < target_frequency)
		{
			current_frequency = target_frequency; // If the set-point falls bellow, set it to the set-point.
		}
	}

	actual_frequency = current_frequency;
	actual_frequency_display = current_frequency;

	return VFD_RUNNING;
}

// Execute all 200us
void Motor_LUT_Stepper(void)
{
	// LUT pointer
	static uint16_t index = 0;

	// PWM CCR Variables
	static uint16_t ph1 = 0;
	static uint16_t ph2 = 0;
	static uint16_t ph3 = 0;

	// 120° Offset
	static uint16_t offset_120deg;
	offset_120deg = num_points / 3;  			// 120° entspricht 1/3 der Periode der vollständigen Tabelle

	// Assign temporary values from LUT
	ph1 = lut[index]; 									// Phase 1 bleibt unverändert, da sie die Referenz ist.
	ph2 = lut[(index + offset_120deg) % num_points]; 	// Phase 2 ist um 120° (also um offset_120deg) verschoben
	ph3 = lut[(index + 2 * offset_120deg) % num_points]; // Phase 3 ist um 240° verschoben (doppelt so viel wie Phase 2)

	// VI curve calculation
	if (actual_frequency < p00_freq_nominal)
	{
		amplitudeDivider = bus_voltage_meas / (((float) actual_frequency / 50.0f) * 230.0f);
	}
	else
	{
		amplitudeDivider = bus_voltage_meas / (float) p01_voltage_nominal;
	}

	// Compute the actual voltage for monitoring
	motor_voltage_temp = bus_voltage_meas / amplitudeDivider;

	// Scale values if using amplitude divider
	if (amplitudeDivider < 1)
	{
		amplitudeDivider = 1;
	}

	// Amplitude divider to reach the desired voltage
	ph1 /= amplitudeDivider;
	ph2 /= amplitudeDivider;
	ph3 /= amplitudeDivider;

	// Set PWM duty cycle by direction
	if (vfd_direction)
	{
		TIM1->CCR1 = ph1;
		TIM1->CCR2 = ph2;
		TIM1->CCR3 = ph3;
	}
	else
	{
		TIM1->CCR1 = ph1;
		TIM1->CCR2 = ph3;
		TIM1->CCR3 = ph2;
	}

	// Reset private watch-dog
/*
	static uint32_t error_trigger_counter = 0;
	if(error_trigger_counter < 50000u)
	{
		__HAL_TIM_SET_COUNTER(&htim14, 0);  // Setzt den Timer-Zähler auf 0
		error_trigger_counter++;
	}
	*/
	__HAL_TIM_SET_COUNTER(&htim14, 0);  // Setzt den Timer-Zähler auf 0

	// Increment LUT pointer
	index += actual_frequency;

	if (index >= point_count)
	{
		index = 0;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim14)
	{
		HAL_GPIO_TogglePin(DEBUG_GPIO_Port, DEBUG_Pin);
		Motor_Timers_Stop();
		emergency_off = 1;
	}
}

/*
 void OC_Boost_Start(uint16_t oc_boost_ms)
 {
 static uint8_t oc_boost_init = 0;
 if(oc_boost_init == 0)
 {
 TIM12->PSC = 42000-1;
 TIM12->ARR = 0xFFFF;  // Max Autoreload-Val
 TIM12->DIER |= TIM_DIER_UIE;  // Update Interrupt aktivieren

 // Timer aktivieren
 TIM12->CR1 |= TIM_CR1_CEN;  // Timer starten
 oc_boost_init = 1;
 }

 TIM12->ARR = oc_boost_ms;
 HAL_TIM_Base_Start_IT(&htim2);
 HAL_GPIO_WritePin(DEBUG_GPIO_Port, DEBUG_Pin, SET);
 }

 void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
 {
 if(htim == &htim12)
 {
 //HAL_GPIO_WritePin(OC_BOOST_GPIO_Port, OC_BOOST_Pin, RESET);
 HAL_GPIO_WritePin(DEBUG_GPIO_Port, DEBUG_Pin, RESET);
 }
 }

 */
