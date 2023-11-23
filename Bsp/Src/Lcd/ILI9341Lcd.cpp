#include "ILI9341Lcd.h"
#include "ili9341/ili9341.h"

#define TIMER_MAX               4799

typedef enum {
  MemoryAccessControlNormalOrder,
  MemoryAccessControlReverseOrder
} MemoryAccessControlRefreshOrder;


ILI9341Lcd::ILI9341Lcd(TIM_HandleTypeDef *htim) {
	this->backLightTim = htim;
	this->backLightOn = false;
	this->width = ILI9341_LCD_PIXEL_WIDTH;
	this->height = ILI9341_LCD_PIXEL_HEIGHT;
	this->orientation = PORTRAIT;

	lcdPortraitConfig = lcdBuildMemoryAccessControlConfig(
	                                                  MemoryAccessControlNormalOrder,		// rowAddressOrder
	                                                  MemoryAccessControlReverseOrder,	    // columnAddressOrder
	                                                  MemoryAccessControlNormalOrder,		// rowColumnExchange
	                                                  MemoryAccessControlNormalOrder);		// verticalRefreshOrder

	lcdLandscapeConfig = lcdBuildMemoryAccessControlConfig(
	                                                  MemoryAccessControlNormalOrder,		// rowAddressOrder
	                                                  MemoryAccessControlNormalOrder,		// columnAddressOrder
	                                                  MemoryAccessControlReverseOrder,	    // rowColumnExchange
	                                                  MemoryAccessControlNormalOrder);		// verticalRefreshOrder

	lcdPortraitMirrorConfig = lcdBuildMemoryAccessControlConfig(
													  MemoryAccessControlReverseOrder,	    // rowAddressOrder
													  MemoryAccessControlNormalOrder,		// columnAddressOrder
													  MemoryAccessControlNormalOrder,		// rowColumnExchange
			                                          MemoryAccessControlNormalOrder);		// verticalRefreshOrder

	lcdLandscapeMirrorConfig = lcdBuildMemoryAccessControlConfig(
	                                                  MemoryAccessControlReverseOrder,	    // rowAddressOrder
	                                                  MemoryAccessControlReverseOrder,	    // columnAddressOrder
	                                                  MemoryAccessControlReverseOrder,	    // rowColumnExchange
	                                                  MemoryAccessControlNormalOrder);		// verticalRefreshOrder

	LCD_IO_Init();
	init();
}

uint16_t ILI9341Lcd::readID(void) {
    LCD_IO_WriteReg(LCD_READ_ID4);

    uint8_t buffer[LCD_READ_ID4_SIZE] = {};
    uint8_t index = 0;
    while (index < LCD_READ_ID4_SIZE) {
	    buffer[index] = LCD_IO_ReadData();
	    index++;
    }

    return buffer[1] << 8 | buffer[2];
}

void ILI9341Lcd::setBackLight(uint8_t backlight) {
	if (backlight > 0) {
		if (!backLightOn) {
			HAL_TIM_PWM_Start(backLightTim, TIM_CHANNEL_1);
			backLightOn = true;
		}

		uint16_t value =  (TIMER_MAX / 100) * backlight;
		__HAL_TIM_SetCompare(backLightTim, TIM_CHANNEL_1, value);

	} else {
		HAL_TIM_PWM_Stop(backLightTim, TIM_CHANNEL_1);
		backLightOn = false;
	}
}

void ILI9341Lcd::displayOff() {
	LCD_IO_WriteReg(LCD_DISPLAY_OFF);
}

void ILI9341Lcd::displayOn() {
	LCD_IO_WriteReg(LCD_DISPLAY_ON);
}

void ILI9341Lcd::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
	setDisplayWindow(x, y, x+w-1, y+h-1);
	int dimensions = w * h;
	while(dimensions--) {
		LCD_IO_WriteData16(color);
	}
}

void ILI9341Lcd::fillRGB(uint16_t color) {
	fillRect(0, 0, ILI9341_LCD_PIXEL_WIDTH-1, ILI9341_LCD_PIXEL_HEIGHT-1, color);
}

void ILI9341Lcd::setOrientation(lcdOrientationTypeDef value) {

	orientation = value;

	LCD_IO_WriteReg(LCD_MAC);

	switch (orientation) {
		case PORTRAIT:
			LCD_IO_WriteData(lcdPortraitConfig);
			width = ILI9341_LCD_PIXEL_WIDTH;
			height = ILI9341_LCD_PIXEL_HEIGHT;
			break;
		case PORTRAIT_MIRROR:
			LCD_IO_WriteData(lcdPortraitMirrorConfig);
			width = ILI9341_LCD_PIXEL_WIDTH;
			height = ILI9341_LCD_PIXEL_HEIGHT;
			break;
		case LANDSCAPE:
			LCD_IO_WriteData(lcdLandscapeConfig);
			width = ILI9341_LCD_PIXEL_HEIGHT;
			height = ILI9341_LCD_PIXEL_WIDTH;
			break;
		case LANDSCAPE_MIRROR:
			LCD_IO_WriteData(lcdLandscapeMirrorConfig);
			width = ILI9341_LCD_PIXEL_HEIGHT;
			height = ILI9341_LCD_PIXEL_WIDTH;
			break;
		default:
			break;
	}
}

void ILI9341Lcd::drawPixel(uint16_t x, uint16_t y, uint16_t color) {

    if (x >= width || y >= height) {
        return;
    }
	setDisplayWindow(x, y, x, y);
	LCD_IO_WriteData16(color);
}

void ILI9341Lcd::setDisplayWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  uint32_t t;
  t = (x0 << 16) | x1;
  LCD_IO_WriteReg(LCD_COLUMN_ADDR);
  LCD_IO_WriteData(t >> 24);
  LCD_IO_WriteData(t >> 16);
  LCD_IO_WriteData(t >> 8);
  LCD_IO_WriteData(t);

  t = (y0 << 16) | y1;
  LCD_IO_WriteReg(LCD_PAGE_ADDR);
  LCD_IO_WriteData(t >> 24);
  LCD_IO_WriteData(t >> 16);
  LCD_IO_WriteData(t >> 8);
  LCD_IO_WriteData(t);

  LCD_IO_WriteReg(LCD_GRAM);
}

void ILI9341Lcd::drawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint32_t color) {

    if (x1 >= width || x2 >= width || x1 > x2) {
        return;
    }
    if (y1 >= height || y2 >= height || y1 > y2) {
        return;
    }

	// Bresenham's algorithm

	  int16_t D;
	  // deltas
	  int16_t delta_x, delta_y;
	  // steps
	  int16_t trace_x = 1, trace_y = 1;

	  // delta x
	  delta_x = x2 - x1;
	  // delta y
	  delta_y = y2 - y1;

	  // check if x2 > x1
	  if (delta_x < 0) {
	    // negate delta x
	    delta_x = -delta_x;
	    // negate step x
	    trace_x = -trace_x;
	  }

	  // check if y2 > y1
	  if (delta_y < 0) {
	    // negate detla y
	    delta_y = -delta_y;
	    // negate step y
	    trace_y = -trace_y;
	  }

	  // Bresenham condition for m < 1 (dy < dx)
	  if (delta_y < delta_x) {
	    // calculate determinant
	    D = (delta_y << 1) - delta_x;
	    // draw first pixel
	    drawPixel(x1, y1, color);
	    // check if x1 equal x2
	    while (x1 != x2) {
	      // update x1
	      x1 += trace_x;
	      // check if determinant is positive
	      if (D >= 0) {
	        // update y1
	        y1 += trace_y;
	        // update determinant
	        D -= 2*delta_x;
	      }
	      // update deteminant
	      D += 2*delta_y;
	      // draw next pixel
	      drawPixel(x1, y1, color);
	    }
	  // for m > 1 (dy > dx)
	  } else {
	    // calculate determinant
	    D = delta_y - (delta_x << 1);
	    // draw first pixel
	    drawPixel(x1, y1, color);
	    // check if y2 equal y1
	    while (y1 != y2) {
	      // update y1
	      y1 += trace_y;
	      // check if determinant is positive
	      if (D <= 0) {
	        // update y1
	        x1 += trace_x;
	        // update determinant
	        D += 2*delta_y;
	      }
	      // update deteminant
	      D -= 2*delta_x;
	      // draw next pixel
	      drawPixel(x1, y1, color);
	    }
	  }
}

uint8_t ILI9341Lcd::lcdBuildMemoryAccessControlConfig(bool rowAddressOrder, bool columnAddressOrder,
                        bool rowColumnExchange, bool verticalRefreshOrder) {

	uint8_t value 				= 0;
	if(verticalRefreshOrder) value 	|= LCD_MADCTL_ML;
	if(rowColumnExchange) value 	|= LCD_MADCTL_MV;
	if(columnAddressOrder) value 	|= LCD_MADCTL_MX;
	if(rowAddressOrder) value 		|= LCD_MADCTL_MY;
    return value;
}

void ILI9341Lcd::init(void)
{
  /* Initialize ILI9341 low level bus layer ----------------------------------*/
  LCD_IO_Init();

  // SOFTWARE RESET
  LCD_IO_WriteReg(0x01);
  LCD_Delay(1000);

  // POWER CONTROL A
  LCD_IO_WriteReg(LCD_POWERA);
  LCD_IO_WriteData(0x39);
  LCD_IO_WriteData(0x2C);
  LCD_IO_WriteData(0x00);
  LCD_IO_WriteData(0x34);
  LCD_IO_WriteData(0x02);

  // POWER CONTROL B
  LCD_IO_WriteReg(LCD_POWERB);
  LCD_IO_WriteData(0x00);
  LCD_IO_WriteData(0xC1);
  LCD_IO_WriteData(0x30);

  // DRIVER TIMING CONTROL A
  LCD_IO_WriteReg(LCD_DTCA);
  LCD_IO_WriteData(0x85);
  LCD_IO_WriteData(0x00);
  LCD_IO_WriteData(0x78);

  // DRIVER TIMING CONTROL B
  LCD_IO_WriteReg(LCD_DTCB);
  LCD_IO_WriteData(0x00);
  LCD_IO_WriteData(0x00);

  // POWER ON SEQUENCE CONTROL
  LCD_IO_WriteReg(LCD_POWER_SEQ);
  LCD_IO_WriteData(0x64);
  LCD_IO_WriteData(0x03);
  LCD_IO_WriteData(0x12);
  LCD_IO_WriteData(0x81);

  // PUMP RATIO CONTROL
  LCD_IO_WriteReg(LCD_PRC);
  LCD_IO_WriteData(0x20);

  // POWER CONTROL,VRH[5:0]
  LCD_IO_WriteReg(LCD_POWER1);
  LCD_IO_WriteData(0x23); // 0x10

  // POWER CONTROL,SAP[2:0];BT[3:0]
  LCD_IO_WriteReg(LCD_POWER2);
  LCD_IO_WriteData(0x10);

  // VCM CONTROL
  LCD_IO_WriteReg(LCD_VCOM1);
  LCD_IO_WriteData(0x3e); // 0x45
  LCD_IO_WriteData(0x28); // 0x15

  // VCM CONTROL 2
  LCD_IO_WriteReg(LCD_VCOM2);
  LCD_IO_WriteData(0x86); // 0x90

  // MEMORY ACCESS CONTROL
  LCD_IO_WriteReg(LCD_MAC);
  LCD_IO_WriteData(0x48);

  // PIXEL FORMAT
  LCD_IO_WriteReg(LCD_PIXEL_FORMAT);
  LCD_IO_WriteData(0x55);

  // FRAME RATIO CONTROL, STANDARD RGB COLOR
  LCD_IO_WriteReg(LCD_FRMCTR1);
  LCD_IO_WriteData(0x00);
  LCD_IO_WriteData(0x18); // 0x1B

  LCD_IO_WriteReg(LCD_DFC);
  LCD_IO_WriteData(0x08); // 0x0A
  LCD_IO_WriteData(0x82); // 0xA2
  LCD_IO_WriteData(0x27);
  LCD_IO_WriteData(0x04); // ?

  // 3GAMMA FUNCTION DISABLE
  LCD_IO_WriteReg(LCD_3GAMMA_EN);
  LCD_IO_WriteData(0x00);

  // GAMMA CURVE SELECTED
  LCD_IO_WriteReg(LCD_GAMMA);
  LCD_IO_WriteData(0x01);

  // POSITIVE GAMMA CORRECTION
  LCD_IO_WriteReg(LCD_PGAMMA);
  LCD_IO_WriteData(0x0F);
  LCD_IO_WriteData(0x31); // 0x29
  LCD_IO_WriteData(0x2B); // 0x24
  LCD_IO_WriteData(0x0C);
  LCD_IO_WriteData(0x0E);
  LCD_IO_WriteData(0x08); // 0x09
  LCD_IO_WriteData(0x4E);
  LCD_IO_WriteData(0xF1); // 0x78
  LCD_IO_WriteData(0x37); // 0x3C
  LCD_IO_WriteData(0x07); // 0x09
  LCD_IO_WriteData(0x10); // 0x13
  LCD_IO_WriteData(0x03); // 0x05
  LCD_IO_WriteData(0x0E); // 0x17
  LCD_IO_WriteData(0x09); // 0x11
  LCD_IO_WriteData(0x00);

  // NEGATIVE GAMMA CORRECTION
  LCD_IO_WriteReg(LCD_NGAMMA);
  LCD_IO_WriteData(0x00);
  LCD_IO_WriteData(0x0E); // 0x16
  LCD_IO_WriteData(0x14); // 0x1B
  LCD_IO_WriteData(0x03); // 0x04
  LCD_IO_WriteData(0x11);
  LCD_IO_WriteData(0x07);
  LCD_IO_WriteData(0x31);
  LCD_IO_WriteData(0xC1); // 0x33
  LCD_IO_WriteData(0x48); // 0x42
  LCD_IO_WriteData(0x08); // 0x05
  LCD_IO_WriteData(0x0F); // 0x0C
  LCD_IO_WriteData(0x0C); // 0x0A
  LCD_IO_WriteData(0x31); // 0x28
  LCD_IO_WriteData(0x36); // 0x2F
  LCD_IO_WriteData(0x0F);

  // FOR RGB INTERFACE
  //LCD_IO_WriteReg(LCD_RGB_INTERFACE);
  //ili9341_WriteData(0xC2);

  //LCD_IO_WriteReg(LCD_INTERFACE);
  //ili9341_WriteData(0x01);
  //ili9341_WriteData(0x00);
  //ili9341_WriteData(0x06);

  LCD_IO_WriteReg(LCD_SLEEP_OUT);
  LCD_Delay(200);
  LCD_IO_WriteReg(LCD_DISPLAY_ON);
  LCD_IO_WriteReg(LCD_GRAM);
}

uint16_t ILI9341Lcd::rgb888torgb565(uint8_t red, uint8_t green, uint8_t blue) {
    uint16_t r = red >> 3;
    uint16_t g = green >> 2;
    uint16_t b = blue >> 3;
    return (uint16_t) ((r << 11) | (g << 5) | b);
}
