#pragma once

#include <stdint.h>

typedef enum {
	PORTRAIT 			= 0,
	LANDSCAPE 			= 1,
	PORTRAIT_MIRROR 	= 2,
	LANDSCAPE_MIRROR 	= 3
} lcdOrientationTypeDef;

class LcdDisplay {
  public:
	virtual ~LcdDisplay() {};
	virtual void setBackLight(uint8_t backlight) = 0;
	virtual void displayOff() = 0;
	virtual void displayOn() = 0;
	virtual void drawPixel(uint16_t x, uint16_t y, uint16_t color) = 0;
	virtual void drawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint32_t color) = 0;
	virtual void fillRGB(uint16_t color) = 0;
	virtual void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t fillcolor) = 0;
	virtual void setOrientation(lcdOrientationTypeDef value) = 0;
	virtual uint16_t readID(void) = 0;
	virtual void setDisplayWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) = 0;
};
