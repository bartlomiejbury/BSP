#include "stm32f4xx_hal.h"

extern SPI_HandleTypeDef hspi1;

//CHIP SELECT PIN AND PORT, STANDARD GPIO
#define LCD_CS_PORT								GPIOD
#define LCD_CS_PIN								GPIO_PIN_14

//DATA COMMAND PIN AND PORT, STANDARD GPIO
#define LCD_DC_PORT								GPIOD
#define LCD_DC_PIN								GPIO_PIN_15


void LCD_IO_Init(void) {
	HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_SET);
}

void LCD_IO_WriteReg(uint8_t Reg) {
	HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &Reg, 1, 10);
	HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);
}

uint8_t LCD_IO_ReadData()
{
	HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);
	uint8_t readvalue = 0;
	HAL_SPI_Receive(&hspi1, &readvalue, 1, 10);
	HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);
    return readvalue;
}

uint16_t LCD_IO_ReadData16() {
	HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);
	uint8_t readvalue[2] = {};
	HAL_SPI_Receive(&hspi1, readvalue, 2, 10);
	HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);
    return readvalue[0] << 8 | readvalue[1];
}

void LCD_IO_WriteData16(uint16_t RegValue) {
	HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &RegValue, 2, 10);
	HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);
}

void LCD_IO_WriteData(uint8_t RegValue) {
	HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &RegValue, 1, 10);
	HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);
}

void LCD_IO_WriteDataMultiple(uint8_t * pData, int NumItems) {
    while (NumItems--) {
    	LCD_IO_WriteData(*pData);
	    pData++;
    }
}

void LCD_IO_ReadDataMultiple(uint8_t * pData, int NumItems) {
	int index = 0;
    while (index < NumItems) {
    	pData[index] = LCD_IO_ReadData(*pData);
    	index++;
    }
}

void LCD_IO_WriteDataMultiple16(uint16_t * pData, int NumItems) {
    while (NumItems--) {
    	LCD_IO_WriteData16(*pData);
	    pData++;
    }
}

void LCD_IO_ReadDataMultiple16(uint16_t * pData, int NumItems) {
	int index = 0;
    while (index < NumItems) {
    	pData[index] = LCD_IO_ReadData16(*pData);
    	index++;
    }
}

void LCD_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}
