/*
 * gpio.h
 *
 *  Created on: Mar 29, 2025
 *      Author: Michael Adam
 *      E-Mail: michael.adam4@proton.me
 */

#ifndef INC_GPIO_H_
#define INC_GPIO_H_

enum
{
	RELAY_MODE_ERROR, RELAY_MODE_RUN, RELAY_MODE_OFF
};

void GPIO_Init(void);

void GPIO_Relay_Handler(uint8_t mode);

#endif /* INC_GPIO_H_ */
