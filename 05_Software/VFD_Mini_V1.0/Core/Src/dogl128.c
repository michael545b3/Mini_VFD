/*
 * dogm132.c
 *
 * Created on: Jan 15, 2022
 * Author: Michael Adam <michael.adam4@proton.me>
 *
 * This library is a driver for the EA DOGL128 Display
 */

// Includes
#include "dogl128.h"
#include "main.h"

// Defines

// ************************************************************************
// NO CHANGES ABOVE THIS LINE
// ************************************************************************
#define RST_PIN_EN		0

#define A0_PORT			DISP_RS_GPIO_Port
#define A0_PIN			DISP_RS_Pin
#define CS_PORT			SPI_CS_GPIO_Port
#define CS_PIN			SPI_CS_Pin
#if RST_PIN_EN
#define RST_PORT		RST_GPIO_Port
#define RST_PIN		RST_Pin
#endif //RST_PIN_EN
#define HSPI_DISP		hspi3
// ************************************************************************
// NO CHANGES BELLOW THIS LINE
// ************************************************************************

// Typedefs
extern SPI_HandleTypeDef HSPI_DISP;

// Variables

// Local Function Prototypes
void DOGL128_ON(void);
void DOGL128_OFF(void);
void DOGL128_Write_CMD(uint8_t cmd);

// Functions

// Display Initialization
void DOGL128_Init(void)
{
	const uint8_t Initialization[] =
	{ CMD_SET_DISP_START_LINE,
	CMD_SET_ADC_REVERSE,
	CMD_SET_COM_NORMAL, 0xA6, 0xA2, 0x2F, 0xF8, 0x00, 0x27, 0x81, 0x10, 0xAC, 0x00, 0xAF };
	/*
	 uint8_t init_DOGM128[INITLEN] = 		{0x40, 0xA1, 0xC0, 0xA6, 0xA2, 0x2F, 0xF8, 0x00, 0x27, 0x81, 0x16, 0xAC, 0x00, 0xAF};
	 uint8_t init_DOGL128[INITLEN] = 		{0x40, 0xA1, 0xC0, 0xA6, 0xA2, 0x2F, 0xF8, 0x00, 0x27, 0x81, 0x10, 0xAC, 0x00, 0xAF};
	 uint8_t init_DOGL128[INITLEN] = 		{0x40, 0xA1, 0xC0, 0xA6, 0xA2, 0x2F, 0xF8, 0x00, 0x23, 0x81, 0x1F, 0xAC, 0x00, 0xAF};
	 uint8_t init_DOGS102[INITLEN_DOGS102] = {0x40, 0xA1, 0xC0, 0xA4, 0xA6, 0xA2, 0x2F, 0x27, 0x81, 0x10, 0xFA, 0x90, 0xAF};
	 */

#if RST_PIN_EN
	// Reset Display
	HAL_Delay(50);
	HAL_GPIO_WritePin(RST_PORT, RST_PIN, RESET);		// RES_DISPLAY
	HAL_Delay(20);
	HAL_GPIO_WritePin(RST_PORT, RST_PIN, SET);			// RES_DISPLAY
#endif //RST_PIN_EN

	HAL_Delay(10);

	for (uint8_t z = 0; z < 14; z++)
	{
		DOGL128_Write_CMD(Initialization[z]);
	}
}

// DOGL128 Write Data
void DOGL128_Write_Data(uint8_t *data)
{
	static uint8_t buffer[3] =
	{ 0, CMD_SET_COLUMN_UPPER, CMD_SET_COLUMN_LOWER };

	for (uint8_t p = 0; p < 8; p++)
	{
		// Transmit Commands
		HAL_GPIO_WritePin(A0_PORT, A0_PIN, RESET);		// A0
		HAL_GPIO_WritePin(CS_PORT, CS_PIN, RESET);		// CS
		buffer[0] = CMD_SET_PAGE + p;
		HAL_SPI_Transmit(&HSPI_DISP, buffer, 3, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(CS_PORT, CS_PIN, SET);		// CS

		// Transmit Data
		HAL_GPIO_WritePin(A0_PORT, A0_PIN, SET);
		HAL_GPIO_WritePin(CS_PORT, CS_PIN, RESET);		// CS
		HAL_SPI_Transmit(&HSPI_DISP, &data[DISPLAY_WIDTH * p], 128, 10000);
		HAL_GPIO_WritePin(CS_PORT, CS_PIN, SET);			// CS
	}
}

// DOGL128 Invert Display
void DOGL128_Invert_Display(void)
{
	DOGL128_Write_CMD(0x0A);			// ADC Reverse
	DOGL128_Write_CMD(0xC8);			// Reverse COM31 - COM0
}

// DOGL128 Set Contrast
void DOGL128_Set_Contrast(uint8_t contrast)
{
	if (contrast < 0x40)
	{
		DOGL128_Write_CMD(0x81); 		// Set Contrast Control Register
		DOGL128_Write_CMD(contrast);	// Set Contrast Value
	}
}

// DOGL128 Display ON
void DOGL128_ON(void)
{
	DOGL128_Write_CMD(0xAF);			// LCD Display ON Command
}

// DOGL128 Display OFF
void DOGL128_OFF(void)
{
	DOGL128_Write_CMD(0xAE);			// LCD Display OFF Command
}

// DOGL128 Write Command
void DOGL128_Write_CMD(uint8_t cmd)
{
	HAL_GPIO_WritePin(A0_PORT, A0_PIN, RESET);		// A0
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, RESET);		// CS

	HAL_SPI_Transmit(&HSPI_DISP, &cmd, 1, HAL_MAX_DELAY);
	while (HAL_SPI_GetState(&HSPI_DISP) != HAL_SPI_STATE_READY);

	HAL_GPIO_WritePin(CS_PORT, CS_PIN, SET);			// CS
}
