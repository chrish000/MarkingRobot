/*
 * lcd_user_init.c
 *
 *  Created on: May 12, 2025
 *      Author: chris
 */

#include "lcd_user_init.h"
#include "main.h"

#define LCD_CS_PIN LCD_CS_Pin
#define LCD_CS_PORT LCD_CS_GPIO_Port
#define LCD_DATA_PIN LCD_EN_Pin
#define LCD_DATA_PORT LCD_EN_GPIO_Port
#define LCD_SCK_PIN LCD_D4_Pin
#define LCD_SCK_PORT LCD_D4_GPIO_Port

u8g2_t u8g2;

uint8_t u8x8_gpio_and_delay_stm32(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
		void *arg_ptr) {
	switch (msg) {
	case U8X8_MSG_DELAY_MILLI:
		HAL_Delay(arg_int);
		break;

	case U8X8_MSG_DELAY_NANO:			// delay arg_int * 1 nano second
		__NOP();
		break;

	case U8X8_MSG_DELAY_100NANO:		// delay arg_int * 100 nano seconds
		for (volatile int i = 0; i < 20; i++) {
			__NOP();
		}
		break;

	case U8X8_MSG_GPIO_CS:
		HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_Pin, (GPIO_PinState) arg_int);
		break;

	case U8X8_MSG_GPIO_SPI_DATA:
		HAL_GPIO_WritePin(LCD_DATA_PORT, LCD_DATA_PIN, (GPIO_PinState) arg_int);
		break;

	case U8X8_MSG_GPIO_SPI_CLOCK:
		HAL_GPIO_WritePin(LCD_SCK_PORT, LCD_SCK_PIN, (GPIO_PinState) arg_int);
		break;

	default:
		return 0;
	}
	return 1;
}

void MX_U8G2_Init(void) {
	u8g2_Setup_st7920_s_128x64_f(&u8g2,
	U8G2_R0, u8x8_byte_4wire_sw_spi, u8x8_gpio_and_delay_stm32);

	u8g2_InitDisplay(&u8g2);
	u8g2_SetPowerSave(&u8g2, 0);
}
