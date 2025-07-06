/*
 * adc.h
 *
 *  Created on: Mar 16, 2025
 *      Author: Michael Adam
 *      E-Mail: michael.adam4@proton.me
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

// Defines
#define VREF_EXT			2.5f
#define ADC_MAX_VAL			4095.0f

// Function Prototypes
uint8_t ADC_Init(void);

uint16_t ADC_Get_V_IN(void);

uint16_t ADC_Get_I_IN(void);

uint16_t ADC_Get_VREF(void);

#endif /* INC_ADC_H_ */
