/**
 ******************************************************************************
 * @file           : pins.h
 * @brief          : Enthaelt alle zur Steuerung genutzen Handler und Pinbelegungen
 * @author         : Chris Hauser
 ******************************************************************************
 * @attention
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#ifndef INC_PINS_H_
#define INC_PINS_H_

#include "main.h"
#include "config.h"

struct Pin {
//X
#define X_STEP_PORT X_STEP_GPIO_Port
#define X_STEP_PIN X_STEP_Pin
#define X_DIR_PORT X_DIR_GPIO_Port
#define X_DIR_PIN X_DIR_Pin
#define X_EN_PORT X_EN_GPIO_Port
#define X_EN_PIN X_EN_Pin
	TIM_HandleTypeDef *TIM_Motor_X = &htim23;
	DMA_HandleTypeDef *TIM_DMA_ARR_X = &hdma_tim23_ch1;
	DMA_HandleTypeDef *TIM_DMA_BSRR_X = &hdma_tim23_ch2;
	UART_HandleTypeDef *TMC_UART_address_X = &huart2;
	CRC_HandleTypeDef *CRC_Handle_X = &hcrc;

// Y
#define Y_STEP_PORT Z_STEP_GPIO_Port
#define Y_STEP_PIN Z_STEP_Pin
#define Y_DIR_PORT Z_DIR_GPIO_Port
#define Y_DIR_PIN Z_DIR_Pin
#define Y_EN_PORT Z_EN_GPIO_Port
#define Y_EN_PIN Z_EN_Pin
	TIM_HandleTypeDef *TIM_Motor_Y = &htim24;
	DMA_HandleTypeDef *TIM_DMA_ARR_Y = &hdma_tim24_ch1;
	DMA_HandleTypeDef *TIM_DMA_BSRR_Y = &hdma_tim24_ch2;
	UART_HandleTypeDef *TMC_UART_address_Y = &huart8;
	CRC_HandleTypeDef *CRC_Handle_Y = &hcrc;

//ENDSTOP
#define X_STOP_PIN X_STOP_Pin
#define Y_STOP_PIN Y_STOP_Pin
#define X_STOP_PORT X_STOP_GPIO_Port
#define Y_STOP_PORT Y_STOP_GPIO_Port
#define PWRDET_PIN PWRDET_Pin
#define PRESSURE_PIN PRESSURE_Pin
#define PRESSURE_PORT PRESSURE_GPIO_Port
#define X_STOP_EXTI X_STOP_EXTI_IRQn
#define Y_STOP_EXTI Y_STOP_EXTI_IRQn
#define PWRDET_EXTI PWRDET_EXTI_IRQn

//PRINTHEAD
	TIM_HandleTypeDef *TIM_Printhead = &htim3;
#define TIM_PrintheadChannel TIM_CHANNEL_1

// FAN
#define FAN0_PORT FAN0_GPIO_Port
#define FAN0_PIN FAN0_Pin
#define FAN1_PORT FAN1_GPIO_Port
#define FAN1_PIN FAN1_Pin
#define FAN2_PORT FAN2_GPIO_Port
#define FAN2_PIN FAN2_Pin

//HE0
	TIM_HandleTypeDef *TIM_HE0 = &htim2;
#define TIM_HE0Channel TIM_CHANNEL_2

// BATTERY
	ADC_HandleTypeDef *ADC_Handle = &hadc1;
	TIM_HandleTypeDef *ADC_TIM = &htim8;
#define BATTERY_ALARM_PORT PWRDET_GPIO_Port
#define BATTERY_ALARM_PIN PWRDET_Pin

// PRESSURE
#define LOW_PRESSURE_PORT PRESSURE_GPIO_Port
#define LOW_PRESSURE_PIN PRESSURE_Pin

//BUZZER
	TIM_HandleTypeDef *TIM_BUZZER = &htim4;
#define BUZZER_PORT BUZZER_GPIO_Port
#define BUZZER_PIN BUZZER_Pin

//BUTTON
#define LCD_BTN_PORT LCD_BTN_GPIO_Port
#define LCD_BTN_PIN LCD_BTN_Pin
#define LCD_ENCA_PORT LCD_ENCA_GPIO_Port
#define LCD_ENCA_PIN LCD_ENCA_Pin
#define LCD_ENCB_PORT LCD_ENCB_GPIO_Port
#define LCD_ENCB_PIN LCD_ENCB_Pin
#define LCD_BTN_EXTI LCD_BTN_EXTI_IRQn
#define LCD_ENCA_EXTI LCD_ENCA_EXTI_IRQn
#define LCD_ENCB_EXTI LCD_ENCB_EXTI_IRQn
};

#endif /* INC_PINS_H_ */
