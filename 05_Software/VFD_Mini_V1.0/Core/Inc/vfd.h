/*
 * vfd.h
 *
 *  Created on: Jan 27, 2025
 *      Author: Michael Adam
 *      E-Mail: michael.adam4@proton.me
 */

#ifndef INC_VFD_H_
#define INC_VFD_H_

// Includes
#include "main.h"

// Enumerators
typedef enum
{
	VFD_RUNNING = 0x00U, VFD_ERROR, VFD_BUSY, VFD_STOP
} VFD_StatusTypeDef;

enum
{
	SET_DIR_FWD = 0x00U, SET_DIR_REV, SET_STOP
};

// Function prototypes
uint8_t Motor_Timers_Start(void);

void Motor_Timers_Stop(void);

void VFD_Init(void);

uint8_t VFD_Handler(uint16_t target_frequency, uint8_t set_direction);

void Motor_LUT_Stepper(void);
/*
void OC_Boost_Start(uint16_t oc_boost_ms);
*/
#endif /* INC_VFD_H_ */
