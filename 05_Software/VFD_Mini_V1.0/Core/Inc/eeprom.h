/*
 * eeprom.h
 *
 *  Created on: Feb 17, 2024
 *      Author: Michael Adam
 *      E-Mail: michael.adam4@proton.me
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

// Includes
#include <stdbool.h>
#include "main.h"

// Defines
// Addresses
#define 	M24C02_ADDR_0		0x50 << 1
#define 	M24C02_ADDR_1		0x51 << 1
#define 	M24C02_ADDR_2		0x52 << 1
#define 	M24C02_ADDR_3		0x53 << 1
#define 	M24C02_ADDR_4		0x54 << 1
#define 	M24C02_ADDR_5		0x55 << 1
#define 	M24C02_ADDR_6		0x56 << 1
#define 	M24C02_ADDR_7		0x57 << 1

// Registers

// Enumerators

// Function Prototypes
ErrorStatus M24C0X_Init(uint8_t device_addr);

ErrorStatus M24C0X_Write_2Byte(uint8_t device_addr, uint8_t addr, uint8_t lsb, uint8_t msb);

ErrorStatus M24C0X_Write_Byte(uint8_t device_addr, uint8_t addr, uint8_t data);

uint8_t M24C0X_Read_Byte(uint8_t device_addr, uint8_t addr);

uint16_t M24C0X_Read_Word(uint8_t device_addr, uint8_t addr);

ErrorStatus M24C0X_Write_Word(uint8_t device_addr, uint8_t addr, uint16_t data);

ErrorStatus M24C0X_Write_Float(uint8_t device_addr, uint8_t addr, float data);

float M24C0X_Read_Float(uint8_t device_addr, uint8_t addr);

#endif /* INC_EEPROM_H_ */
