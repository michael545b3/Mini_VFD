/*
 * dogm132.h
 *
 * Created on: Jan 15, 2022
 * Author: Michael Adam <michael.adam4@proton.me>
 *
 * This library is a driver for the EA DOGL128 Display
 */

#ifndef INC_DOGL128_H_
#define INC_DOGL128_H_

// Includes
#include <stdint.h>
#include <stdio.h>

#include "main.h"

// Defines
#define CMD_SET_DISP_START_LINE 	0x40
#define CMD_SET_PAGE  				0xB0

#define CMD_SET_COLUMN_UPPER  		0x10
#define CMD_SET_COLUMN_LOWER  		0x04

#define CMD_SET_ADC_NORMAL  		0xA0
#define CMD_SET_ADC_REVERSE 		0xA1

#define CMD_SET_DISP_NORMAL 		0xA6
#define CMD_SET_DISP_REVERSE 		0xA7

#define CMD_SET_ALLPTS_NORMAL 		0xA4
#define CMD_SET_ALLPTS_ON  			0xA5
#define CMD_SET_BIAS_9 				0xA2
#define CMD_SET_BIAS_7 				0xA3

#define CMD_RMW  					0xE0
#define CMD_RMW_CLEAR 				0xEE
#define CMD_INTERNAL_RESET  		0xE2
#define CMD_SET_COM_NORMAL  		0xC0
#define CMD_SET_COM_REVERSE  		0xC8
#define CMD_SET_POWER_CONTROL  		0x28 | 0x4
#define CMD_SET_RESISTOR_RATIO  	0x20
#define CMD_SET_VOLUME_FIRST  		0x81
#define CMD_SET_VOLUME_SECOND  		0x00
#define CMD_SET_STATIC_OFF  		0xAC
#define CMD_SET_STATIC_ON  			0xAD
#define CMD_SET_STATIC_REG  		0x0
#define CMD_SET_BOOSTER_FIRST  		0xF8
#define CMD_SET_BOOSTER_234  		0x00
#define CMD_SET_BOOSTER_5  			0x01
#define CMD_SET_BOOSTER_6  			0x03
#define CMD_NOP  					0xE3
#define CMD_TEST  					0xF0

// DOGL128 Width in Pixels
#ifndef DISPLAY_WIDTH
#define DISPLAY_WIDTH				128
#endif

// DOGL128 LCD height in Pixels
#ifndef DISPLAY_HEIGHT
#define DISPLAY_HEIGHT          	64
#endif

// Display Initialization
void DOGL128_Init(void);

// DOGL128 Write Data
void DOGL128_Write_Data(uint8_t *data);

// DOGL128 Write Command
void DOGL128_Write_CMD(uint8_t data);

// DOGL128 Invert Display
void DOGL128_Invert_Display(void);

// DOGL128 Set Contrast
void DOGL128_Set_Contrast(uint8_t contrast);

// DOGL128 Display ON
void DOGL128_ON(void);

// DOGL128 Display OFF
void DOGL128_OFF(void);

#endif /* INC_DOGL128_H_ */
