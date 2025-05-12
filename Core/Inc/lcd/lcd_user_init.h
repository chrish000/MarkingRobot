/*
 * lcd_user_init.h
 *
 *  Created on: May 12, 2025
 *      Author: chris
 */

#ifndef INC_LCD_LCD_USER_INIT_H_
#define INC_LCD_LCD_USER_INIT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx.h"
#include "u8g2.h"
#include "pins.h"

extern u8g2_t u8g2;

void MX_U8G2_Init(void);
uint8_t u8x8_gpio_and_delay_stm32(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
		void *arg_ptr);

#ifdef __cplusplus
}
#endif

#endif /* INC_LCD_LCD_USER_INIT_H_ */
