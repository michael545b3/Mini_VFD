/*
 * menu.h
 *
 *  Created on: Mar 16, 2025
 *      Author: Michael Adam
 *      E-Mail: michael.adam4@proton.me
 */

#ifndef INC_MENU_H_
#define INC_MENU_H_

void Menu_Encoder_Handler(uint8_t encoder_mode);

void Menu_Init(void);

void Menu_Handler(uint8_t menu_mode_enabled);

#endif /* INC_MENU_H_ */
