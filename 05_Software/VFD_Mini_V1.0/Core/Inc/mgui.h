/**
 * Original author:  Tilen Majerle <tilen@majerle.eu>
 * Modification for STM32: Michael Adam <michael.adam4@proton.me>
 * 27.01.2022
 */
#ifndef Display_H
#define Display_H 100

// Includes
#include "main.h"

#include "fonts.h"

#include "stdlib.h"
#include "string.h"

// Color Enumeration
typedef enum
{
	Display_COLOR_BLACK = 0x00, /*!< Black color, no pixel */
	Display_COLOR_WHITE = 0x01 /*!< Pixel is set. Color depends on LCD */
} Display_COLOR_t;

uint8_t Display_Init(void);

/** 
 * @brief  Updates buffer from internal RAM to LCD
 * @note   This function must be called each time you do some changes to LCD, to update buffer from RAM to LCD
 * @param  None
 * @retval None
 */
void Display_UpdateScreen(void);

/**
 * @brief  Toggles pixels invertion inside internal RAM
 * @note   @ref Display_UpdateScreen() must be called after that in order to see updated LCD screen
 * @param  None
 * @retval None
 */
void Display_ToggleInvert(void);

/** 
 * @brief  Fills entire LCD with desired color
 * @note   @ref Display_UpdateScreen() must be called after that in order to see updated LCD screen
 * @param  Color: Color to be used for screen fill. This parameter can be a value of @ref Display_COLOR_t enumeration
 * @retval None
 */
void Display_Fill(Display_COLOR_t Color);

/**
 * @brief  Draws pixel at desired location
 * @note   @ref Display_UpdateScreen() must called after that in order to see updated LCD screen
 * @param  x: X location. This parameter can be a value between 0 and Display_WIDTH - 1
 * @param  y: Y location. This parameter can be a value between 0 and Display_HEIGHT - 1
 * @param  color: Color to be used for screen fill. This parameter can be a value of @ref Display_COLOR_t enumeration
 * @retval None
 */
void Display_DrawPixel(uint16_t x, uint16_t y, Display_COLOR_t color);

/**
 * @brief  Sets cursor pointer to desired location for strings
 * @param  x: X location. This parameter can be a value between 0 and Display_WIDTH - 1
 * @param  y: Y location. This parameter can be a value between 0 and Display_HEIGHT - 1
 * @retval None
 */
void Display_GotoXY(uint16_t x, uint16_t y);

/**
 * @brief  Puts character to internal RAM
 * @note   @ref Display_UpdateScreen() must be called after that in order to see updated LCD screen
 * @param  ch: Character to be written
 * @param  *Font: Pointer to @ref FontDef_t structure with used font
 * @param  color: Color used for drawing. This parameter can be a value of @ref Display_COLOR_t enumeration
 * @retval Character written
 */
char Display_Putc(char ch, FontDef_t *Font, Display_COLOR_t color);

/**
 * @brief  Puts string to internal RAM
 * @note   @ref Display_UpdateScreen() must be called after that in order to see updated LCD screen
 * @param  *str: String to be written
 * @param  *Font: Pointer to @ref FontDef_t structure with used font
 * @param  color: Color used for drawing. This parameter can be a value of @ref Display_COLOR_t enumeration
 * @retval Zero on success or character value when function failed
 */
char Display_Puts(char *str, FontDef_t *Font, Display_COLOR_t color);

/**
 * @brief  Draws line on LCD
 * @note   @ref Display_UpdateScreen() must be called after that in order to see updated LCD screen
 * @param  x0: Line X start point. Valid input is 0 to Display_WIDTH - 1
 * @param  y0: Line Y start point. Valid input is 0 to Display_HEIGHT - 1
 * @param  x1: Line X end point. Valid input is 0 to Display_WIDTH - 1
 * @param  y1: Line Y end point. Valid input is 0 to Display_HEIGHT - 1
 * @param  c: Color to be used. This parameter can be a value of @ref Display_COLOR_t enumeration
 * @retval None
 */
void Display_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, Display_COLOR_t c);

/**
 * @brief  Draws rectangle on LCD
 * @note   @ref Display_UpdateScreen() must be called after that in order to see updated LCD screen
 * @param  x: Top left X start point. Valid input is 0 to Display_WIDTH - 1
 * @param  y: Top left Y start point. Valid input is 0 to Display_HEIGHT - 1
 * @param  w: Rectangle width in units of pixels
 * @param  h: Rectangle height in units of pixels
 * @param  c: Color to be used. This parameter can be a value of @ref Display_COLOR_t enumeration
 * @retval None
 */
void Display_DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, Display_COLOR_t c);

/**
 * @brief  Draws filled rectangle on LCD
 * @note   @ref Display_UpdateScreen() must be called after that in order to see updated LCD screen
 * @param  x: Top left X start point. Valid input is 0 to Display_WIDTH - 1
 * @param  y: Top left Y start point. Valid input is 0 to Display_HEIGHT - 1
 * @param  w: Rectangle width in units of pixels
 * @param  h: Rectangle height in units of pixels
 * @param  c: Color to be used. This parameter can be a value of @ref Display_COLOR_t enumeration
 * @retval None
 */
void Display_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, Display_COLOR_t c);

/**
 * @brief  Draws triangle on LCD
 * @note   @ref Display_UpdateScreen() must be called after that in order to see updated LCD screen
 * @param  x1: First coordinate X location. Valid input is 0 to Display_WIDTH - 1
 * @param  y1: First coordinate Y location. Valid input is 0 to Display_HEIGHT - 1
 * @param  x2: Second coordinate X location. Valid input is 0 to Display_WIDTH - 1
 * @param  y2: Second coordinate Y location. Valid input is 0 to Display_HEIGHT - 1
 * @param  x3: Third coordinate X location. Valid input is 0 to Display_WIDTH - 1
 * @param  y3: Third coordinate Y location. Valid input is 0 to Display_HEIGHT - 1
 * @param  c: Color to be used. This parameter can be a value of @ref Display_COLOR_t enumeration
 * @retval None
 */
void Display_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, Display_COLOR_t color);

/**
 * @brief  Draws circle to STM buffer
 * @note   @ref Display_UpdateScreen() must be called after that in order to see updated LCD screen
 * @param  x: X location for center of circle. Valid input is 0 to Display_WIDTH - 1
 * @param  y: Y location for center of circle. Valid input is 0 to Display_HEIGHT - 1
 * @param  r: Circle radius in units of pixels
 * @param  c: Color to be used. This parameter can be a value of @ref Display_COLOR_t enumeration
 * @retval None
 */
void Display_DrawCircle(int16_t x0, int16_t y0, int16_t r, Display_COLOR_t c);

/**
 * @brief  Draws filled circle to STM buffer
 * @note   @ref Display_UpdateScreen() must be called after that in order to see updated LCD screen
 * @param  x: X location for center of circle. Valid input is 0 to Display_WIDTH - 1
 * @param  y: Y location for center of circle. Valid input is 0 to Display_HEIGHT - 1
 * @param  r: Circle radius in units of pixels
 * @param  c: Color to be used. This parameter can be a value of @ref Display_COLOR_t enumeration
 * @retval None
 */
void Display_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, Display_COLOR_t c);

/**
 * @brief  Writes single byte to slave
 * @param  *I2Cx: I2C used
 * @param  address: 7 bit slave address, left aligned, bits 7:1 are used, LSB bit is not used
 * @param  reg: register to write to
 * @param  data: data to be written
 * @retval None
 */

/**
 * @brief  Writes multi bytes to slave
 * @param  *I2Cx: I2C used
 * @param  address: 7 bit slave address, left aligned, bits 7:1 are used, LSB bit is not used
 * @param  reg: register to write to
 * @param  *data: pointer to data array to write it to slave
 * @param  count: how many bytes will be written
 * @retval None
 */
void Display_Write_Data(uint8_t *data);

/**
 * @brief  Draws the Bitmap
 * @param  X:  X location to start the Drawing
 * @param  Y:  Y location to start the Drawing
 * @param  *bitmap : Pointer to the bitmap
 * @param  W : width of the image
 * @param  H : Height of the image
 * @param  color : 1-> white/blue, 0-> black
 */
void Display_DrawBitmap(int16_t x, int16_t y, const unsigned char *bitmap, int16_t w, int16_t h, uint16_t color);

// Print float
void Print_Float(float value, FontDef_t *Font, uint16_t x_position, uint16_t y_position);

// clear the display
void Display_Clear(void);

#endif
