/*
 * eeprom.c
 *
 *  Created on: Feb 17, 2024
 *      Author: Michael Adam
 *      E-Mail: michael.adam4@proton.me
 */

// Includes
#include "main.h"
#include "eeprom.h"
#include "string.h"

// Defines
#define EEPROM_I2C_HANDLER			hi2c3
#define EEPROM_COM_MAX_DELAY		100

// Variables
extern I2C_HandleTypeDef EEPROM_I2C_HANDLER;

// Local Function Prototypes
uint32_t byte_array_to_uint32(uint8_t *p_bytes)
{
	return ((uint32_t) p_bytes[0]) | (((uint32_t) p_bytes[1]) << 8) | (((uint32_t) p_bytes[2]) << 16) | (((uint32_t) p_bytes[3]) << 24);
}

float byte_array_to_float(uint8_t *p_bytes)
{
	uint32_t data = byte_array_to_uint32(p_bytes);
	return *((float*) &data);
}

void float_to_byte_array(uint8_t *p_bytes, float value)
{
	union
	{
		float a;
		uint8_t bytes[4];
	} thing;
	thing.a = value;

	memcpy(p_bytes, thing.bytes, 4);
}

ErrorStatus M24C0X_Init(uint8_t device_addr)
{
	if (HAL_I2C_IsDeviceReady(&EEPROM_I2C_HANDLER, device_addr, 5, EEPROM_COM_MAX_DELAY) != HAL_OK)
	{
		/* Return false */
		return ERROR;
	}

	return SUCCESS;
}

ErrorStatus M24C0X_Write_2Byte(uint8_t device_addr, uint8_t addr, uint8_t lsb, uint8_t msb) // Processing time 380us
{
	uint8_t buf[3];

	buf[0] = addr;
	buf[1] = msb;
	buf[2] = lsb;

	if (HAL_I2C_Master_Transmit(&EEPROM_I2C_HANDLER, device_addr, buf, 3, EEPROM_COM_MAX_DELAY) != HAL_OK)
		return ERROR;

	return SUCCESS;
}

ErrorStatus M24C0X_Write_Byte(uint8_t device_addr, uint8_t addr, uint8_t data)
{
	uint8_t buf[2];
	buf[0] = addr;
	buf[1] = data;

	if (HAL_I2C_Master_Transmit(&EEPROM_I2C_HANDLER, device_addr, buf, 2, EEPROM_COM_MAX_DELAY) != HAL_OK)
		return ERROR;
	///while (HAL_I2C_IsDeviceReady(&EEPROM_I2C_HANDLER, device_addr << 1, 10, EEPROM_COM_MAX_DELAY) != HAL_OK);


	return SUCCESS;
}

uint8_t M24C0X_Read_Byte(uint8_t device_addr, uint8_t addr)
{
	uint8_t buf[1];
	uint8_t rxbuf[1];

	buf[0] = addr;
	HAL_I2C_Master_Transmit(&EEPROM_I2C_HANDLER, device_addr, buf, 1, EEPROM_COM_MAX_DELAY);

	HAL_I2C_Master_Receive(&EEPROM_I2C_HANDLER, device_addr, rxbuf, 1, EEPROM_COM_MAX_DELAY);

	return rxbuf[0];
}

uint16_t M24C0X_Read_Word(uint8_t device_addr, uint8_t addr) // Processing time 500us
{
	uint16_t word;
	uint8_t rx_buf[2];
	uint8_t tx_buf[2];

	tx_buf[0] = addr;

	HAL_I2C_Master_Transmit(&EEPROM_I2C_HANDLER, device_addr, tx_buf, 1, EEPROM_COM_MAX_DELAY);

	HAL_I2C_Master_Receive(&EEPROM_I2C_HANDLER, device_addr, rx_buf, 2, EEPROM_COM_MAX_DELAY);

	word = ((uint16_t) rx_buf[0] << 8) | rx_buf[1];

	return word;
}

ErrorStatus M24C0X_Write_Word(uint8_t device_addr, uint8_t addr, uint16_t data)
{
	uint8_t buf[3];
	buf[0] = addr;
	buf[1] = data >> 8;
	buf[2] = data & 0xFF;

	if (HAL_I2C_Master_Transmit(&EEPROM_I2C_HANDLER, device_addr, buf, 3, EEPROM_COM_MAX_DELAY) != HAL_OK)
		return ERROR;
	// Wait until EEPROM is ready (ACK polling)
	while (HAL_I2C_IsDeviceReady(&EEPROM_I2C_HANDLER, device_addr << 1, 10, EEPROM_COM_MAX_DELAY) != HAL_OK);

	return SUCCESS;
}

ErrorStatus M24C0X_Write_Float(uint8_t device_addr, uint8_t addr, float data)
{
	uint8_t buf[5];
	float_to_byte_array(buf, data);
	if (HAL_I2C_Master_Transmit(&EEPROM_I2C_HANDLER, device_addr, buf, 5, EEPROM_COM_MAX_DELAY) != HAL_OK)
		return ERROR;

	return SUCCESS;
}

float M24C0X_Read_Float(uint8_t device_addr, uint8_t addr)
{
	uint8_t buf[4];

	buf[0] = addr;

	HAL_I2C_Master_Transmit(&EEPROM_I2C_HANDLER, device_addr, buf, 1, EEPROM_COM_MAX_DELAY);

	HAL_I2C_Master_Receive(&EEPROM_I2C_HANDLER, device_addr, buf, 4, EEPROM_COM_MAX_DELAY);

	float value = byte_array_to_float(buf);

	return value;
}
