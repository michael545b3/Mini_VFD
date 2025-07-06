/**
 * Original author:  Tilen Majerle <tilen@majerle.eu>
 * Modification for STM32: Michael Adam <michael.adam4@proton.me>
 * 27.01.2022
 */

// Includes
#include <dogl128.h>
#include <string.h>

#include "main.h"
#include "fonts.h"
#include "mgui.h"

// Absolute Value
#define ABS(x)   ((x) > 0 ? (x) : -(x))

// Data Buffer
static uint8_t Frame_Buffer[(DISPLAY_WIDTH * DISPLAY_HEIGHT) / 8];

// Private Display structure
typedef struct
{
	uint16_t CurrentX;
	uint16_t CurrentY;
	uint8_t Inverted;
	uint8_t Initialized;
} Display_t;

// Private variable
uint8_t display_tx_rdy = 0;

static Display_t Display;

// Defines
#define Display_RIGHT_HORIZONTAL_SCROLL              0x26
#define Display_LEFT_HORIZONTAL_SCROLL               0x27
#define Display_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL 0x29
#define Display_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL  0x2A
#define Display_DEACTIVATE_SCROLL                    0x2E // Stop scroll
#define Display_ACTIVATE_SCROLL                      0x2F // Start scroll
#define Display_SET_VERTICAL_SCROLL_AREA             0xA3 // Set scroll range

#define Display_NORMALDISPLAY       0xA6
#define Display_INVERTDISPLAY       0xA7

// Functions

void Display_DrawBitmap(int16_t x, int16_t y, const unsigned char *bitmap, int16_t w, int16_t h, uint16_t color)
{

	int16_t byteWidth = (w + 7) / 8; // Bitmap scanline pad = whole byte
	uint8_t byte = 0;

	for (int16_t j = 0; j < h; j++, y++)
	{
		for (int16_t i = 0; i < w; i++)
		{
			if (i & 7)
			{
				byte <<= 1;
			}
			else
			{
				byte = (*(const unsigned char*) (&bitmap[j * byteWidth + i / 8]));
			}
			if (byte & 0x80)
				Display_DrawPixel(x + i, y, color);
		}
	}
}

uint8_t Display_Init(void)
{
	// Flush Buffer
	for (uint16_t z = 0; z < ((DISPLAY_WIDTH * DISPLAY_HEIGHT) / 8); z++)
	{
		Frame_Buffer[z] = 0x00;
	}

	// Put Display Initialization Function HERE:
	DOGL128_Init();

	// Clear Screen
	Display_Fill(Display_COLOR_BLACK);

	// Update screen
	Display_UpdateScreen();

	// Set default values
	Display.CurrentX = 0;
	Display.CurrentY = 0;

	// Initialized OK
	Display.Initialized = 1;

	// Return OK
	return 1;
}

void Display_UpdateScreen(void)
{
	DOGL128_Write_Data(Frame_Buffer);
}

void Display_ToggleInvert(void)
{
	uint16_t i;

	/* Toggle invert */
	Display.Inverted = !Display.Inverted;

	/* Do memory toggle */
	for (i = 0; i < sizeof(Frame_Buffer); i++)
	{
		Frame_Buffer[i] = ~Frame_Buffer[i];
	}
}

void Display_Fill(Display_COLOR_t color)
{
	/* Set memory */
	memset(Frame_Buffer, (color == Display_COLOR_BLACK) ? 0x00 : 0xFF, sizeof(Frame_Buffer));
}

void Display_DrawPixel(uint16_t x, uint16_t y, Display_COLOR_t color)
{
	if (x >= DISPLAY_WIDTH || y >= DISPLAY_HEIGHT)
	{
		/* Error */
		return;
	}

	/* Check if pixels are inverted */
	if (Display.Inverted)
	{
		color = (Display_COLOR_t) !color;
	}

	/* Set color */
	if (color == Display_COLOR_WHITE)
	{
		Frame_Buffer[x + (y / 8) * DISPLAY_WIDTH] |= 1 << (y % 8);
	}
	else
	{
		Frame_Buffer[x + (y / 8) * DISPLAY_WIDTH] &= ~(1 << (y % 8));
	}
}

void Display_GotoXY(uint16_t x, uint16_t y)
{
	/* Set write pointers */
	Display.CurrentX = x;
	Display.CurrentY = y;
}

char Display_Putc(char ch, FontDef_t *Font, Display_COLOR_t color)
{
	uint32_t i, b, j;

	/* Check available space in LCD */
	if (
	DISPLAY_WIDTH <= (Display.CurrentX + Font->FontWidth) ||
	DISPLAY_HEIGHT <= (Display.CurrentY + Font->FontHeight))
	{
		/* Error */
		return 0;
	}

	/* Go through font */
	for (i = 0; i < Font->FontHeight; i++)
	{
		b = Font->data[(ch - 32) * Font->FontHeight + i];
		for (j = 0; j < Font->FontWidth; j++)
		{
			if ((b << j) & 0x8000)
			{
				Display_DrawPixel(Display.CurrentX + j, (Display.CurrentY + i), (Display_COLOR_t) color);
			}
			else
			{
				Display_DrawPixel(Display.CurrentX + j, (Display.CurrentY + i), (Display_COLOR_t) !color);
			}
		}
	}

	/* Increase pointer */
	Display.CurrentX += Font->FontWidth;

	/* Return character written */
	return ch;
}

char Display_Puts(char *str, FontDef_t *Font, Display_COLOR_t color)
{
	/* Write characters */
	while (*str)
	{
		/* Write character by character */
		if (Display_Putc(*str, Font, color) != *str)
		{
			/* Return error */
			return *str;
		}

		/* Increase string pointer */
		str++;
	}

	/* Everything OK, zero should be returned */
	return *str;
}

void Display_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, Display_COLOR_t c)
{
	int16_t dx, dy, sx, sy, err, e2, i, tmp;

	/* Check for overflow */
	if (x0 >= DISPLAY_WIDTH)
	{
		x0 = DISPLAY_WIDTH - 1;
	}
	if (x1 >= DISPLAY_WIDTH)
	{
		x1 = DISPLAY_WIDTH - 1;
	}
	if (y0 >= DISPLAY_HEIGHT)
	{
		y0 = DISPLAY_HEIGHT - 1;
	}
	if (y1 >= DISPLAY_HEIGHT)
	{
		y1 = DISPLAY_HEIGHT - 1;
	}

	dx = (x0 < x1) ? (x1 - x0) : (x0 - x1);
	dy = (y0 < y1) ? (y1 - y0) : (y0 - y1);
	sx = (x0 < x1) ? 1 : -1;
	sy = (y0 < y1) ? 1 : -1;
	err = ((dx > dy) ? dx : -dy) / 2;

	if (dx == 0)
	{
		if (y1 < y0)
		{
			tmp = y1;
			y1 = y0;
			y0 = tmp;
		}

		if (x1 < x0)
		{
			tmp = x1;
			x1 = x0;
			x0 = tmp;
		}

		/* Vertical line */
		for (i = y0; i <= y1; i++)
		{
			Display_DrawPixel(x0, i, c);
		}

		/* Return from function */
		return;
	}

	if (dy == 0)
	{
		if (y1 < y0)
		{
			tmp = y1;
			y1 = y0;
			y0 = tmp;
		}

		if (x1 < x0)
		{
			tmp = x1;
			x1 = x0;
			x0 = tmp;
		}

		/* Horizontal line */
		for (i = x0; i <= x1; i++)
		{
			Display_DrawPixel(i, y0, c);
		}

		/* Return from function */
		return;
	}

	while (1)
	{
		Display_DrawPixel(x0, y0, c);
		if (x0 == x1 && y0 == y1)
		{
			break;
		}
		e2 = err;
		if (e2 > -dx)
		{
			err -= dy;
			x0 += sx;
		}
		if (e2 < dy)
		{
			err += dx;
			y0 += sy;
		}
	}
}

void Display_DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, Display_COLOR_t c)
{
	/* Check input parameters */
	if (x >= DISPLAY_WIDTH || y >= DISPLAY_HEIGHT)
	{
		/* Return error */
		return;
	}

	/* Check width and height */
	if ((x + w) >= DISPLAY_WIDTH)
	{
		w = DISPLAY_WIDTH - x;
	}
	if ((y + h) >= DISPLAY_HEIGHT)
	{
		h = DISPLAY_HEIGHT - y;
	}

	/* Draw 4 lines */
	Display_DrawLine(x, y, x + w, y, c); /* Top line */
	Display_DrawLine(x, y + h, x + w, y + h, c); /* Bottom line */
	Display_DrawLine(x, y, x, y + h, c); /* Left line */
	Display_DrawLine(x + w, y, x + w, y + h, c); /* Right line */
}

void Display_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, Display_COLOR_t c)
{
	uint8_t i;

	/* Check input parameters */
	if (x >= DISPLAY_WIDTH || y >= DISPLAY_HEIGHT)
	{
		/* Return error */
		return;
	}

	/* Check width and height */
	if ((x + w) >= DISPLAY_WIDTH)
	{
		w = DISPLAY_WIDTH - x;
	}
	if ((y + h) >= DISPLAY_HEIGHT)
	{
		h = DISPLAY_HEIGHT - y;
	}

	/* Draw lines */
	for (i = 0; i <= h; i++)
	{
		/* Draw lines */
		Display_DrawLine(x, y + i, x + w, y + i, c);
	}
}

void Display_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, Display_COLOR_t color)
{
	/* Draw lines */
	Display_DrawLine(x1, y1, x2, y2, color);
	Display_DrawLine(x2, y2, x3, y3, color);
	Display_DrawLine(x3, y3, x1, y1, color);
}

void Display_DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, Display_COLOR_t color)
{
	int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0, yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0, curpixel = 0;

	deltax = ABS(x2 - x1);
	deltay = ABS(y2 - y1);
	x = x1;
	y = y1;

	if (x2 >= x1)
	{
		xinc1 = 1;
		xinc2 = 1;
	}
	else
	{
		xinc1 = -1;
		xinc2 = -1;
	}

	if (y2 >= y1)
	{
		yinc1 = 1;
		yinc2 = 1;
	}
	else
	{
		yinc1 = -1;
		yinc2 = -1;
	}

	if (deltax >= deltay)
	{
		xinc1 = 0;
		yinc2 = 0;
		den = deltax;
		num = deltax / 2;
		numadd = deltay;
		numpixels = deltax;
	}
	else
	{
		xinc2 = 0;
		yinc1 = 0;
		den = deltay;
		num = deltay / 2;
		numadd = deltax;
		numpixels = deltay;
	}

	for (curpixel = 0; curpixel <= numpixels; curpixel++)
	{
		Display_DrawLine(x, y, x3, y3, color);

		num += numadd;
		if (num >= den)
		{
			num -= den;
			x += xinc1;
			y += yinc1;
		}
		x += xinc2;
		y += yinc2;
	}
}

void Display_DrawCircle(int16_t x0, int16_t y0, int16_t r, Display_COLOR_t c)
{
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	Display_DrawPixel(x0, y0 + r, c);
	Display_DrawPixel(x0, y0 - r, c);
	Display_DrawPixel(x0 + r, y0, c);
	Display_DrawPixel(x0 - r, y0, c);

	while (x < y)
	{
		if (f >= 0)
		{
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		Display_DrawPixel(x0 + x, y0 + y, c);
		Display_DrawPixel(x0 - x, y0 + y, c);
		Display_DrawPixel(x0 + x, y0 - y, c);
		Display_DrawPixel(x0 - x, y0 - y, c);

		Display_DrawPixel(x0 + y, y0 + x, c);
		Display_DrawPixel(x0 - y, y0 + x, c);
		Display_DrawPixel(x0 + y, y0 - x, c);
		Display_DrawPixel(x0 - y, y0 - x, c);
	}
}

void Display_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, Display_COLOR_t c)
{
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	Display_DrawPixel(x0, y0 + r, c);
	Display_DrawPixel(x0, y0 - r, c);
	Display_DrawPixel(x0 + r, y0, c);
	Display_DrawPixel(x0 - r, y0, c);
	Display_DrawLine(x0 - r, y0, x0 + r, y0, c);

	while (x < y)
	{
		if (f >= 0)
		{
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		Display_DrawLine(x0 - x, y0 + y, x0 + x, y0 + y, c);
		Display_DrawLine(x0 + x, y0 - y, x0 - x, y0 - y, c);

		Display_DrawLine(x0 + y, y0 + x, x0 - y, y0 + x, c);
		Display_DrawLine(x0 + y, y0 - x, x0 - y, y0 - x, c);
	}
}

void Print_Float(float value, FontDef_t *Font, uint16_t x_position, uint16_t y_position)
{
	char buf_value[16]; // Buffer für maximal 6 Ziffern plus Punkt und Nullterminator
	int16_t int_part = (int16_t) value;  // Whole number
	int16_t dec_part = (int16_t) ((value - int_part) * 100);  // Decimal number

	// Format the number as string
	snprintf(buf_value, sizeof(buf_value), "%3d.%02d", int_part, dec_part);

	// Goto position
	Display_GotoXY(x_position, y_position);

	// Characterwise print
	for (uint16_t i = 0; buf_value[i] != '\0'; i++)
	{
		Display_Putc(buf_value[i], Font, Display_COLOR_WHITE);
	}
}

void Display_Clear(void)
{
	Display_Fill(0);
	Display_UpdateScreen();
}
