#pragma once

#include "Lcd/lcd.h"
#include "stm32f4xx_hal.h"
#include <stdbool.h>

class ILI9341Lcd : public LcdDisplay {
  public:

	ILI9341Lcd(TIM_HandleTypeDef *htim);
	void setBackLight(uint8_t backlight) override;
	void displayOff() override;
	void displayOn() override;
	void drawPixel(uint16_t x, uint16_t y, uint16_t color) override;
	void drawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint32_t color) override;
	void fillRGB(uint16_t color) override;
	void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t fillcolor) override;
	void setOrientation(lcdOrientationTypeDef value) override;
	uint16_t readID(void) override;
	void setDisplayWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) override;

	static uint16_t rgb888torgb565(uint8_t red, uint8_t green, uint8_t blue);
  private:
	bool backLightOn = false;
	TIM_HandleTypeDef *backLightTim;

	uint16_t width;
	uint16_t height;
	lcdOrientationTypeDef orientation;

	uint8_t lcdPortraitConfig;
	uint8_t lcdLandscapeConfig;
	uint8_t lcdPortraitMirrorConfig;
	uint8_t lcdLandscapeMirrorConfig;

	void init(void);
	uint8_t lcdBuildMemoryAccessControlConfig(bool rowAddressOrder, bool columnAddressOrder,
	                                bool rowColumnExchange, bool verticalRefreshOrder);
};
