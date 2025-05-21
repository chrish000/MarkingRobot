/*
 * button.h
 *
 *  Created on: Apr 25, 2025
 *      Author: chris
 */

#ifndef INC_BUZZER_BUTTON_H_
#define INC_BUZZER_BUTTON_H_

#include "pins.h"
#include "stm32h7xx_hal.h"


void LCD_enableButton() {
	HAL_NVIC_EnableIRQ(LCD_BTN_EXTI);
}

void LCD_disableButton() {
	HAL_NVIC_DisableIRQ(LCD_BTN_EXTI);
}

void LCD_enableEncoder() {
	HAL_NVIC_EnableIRQ(LCD_ENCA_EXTI);
	//HAL_NVIC_EnableIRQ(LCD_ENCB_EXTI);
}

void LCD_disableEncoder() {
	HAL_NVIC_DisableIRQ(LCD_ENCA_EXTI);
	HAL_NVIC_DisableIRQ(LCD_ENCB_EXTI);
}

#endif /* INC_BUZZER_BUTTON_H_ */
