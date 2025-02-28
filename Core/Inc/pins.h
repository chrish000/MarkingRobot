/*
 * pins.h
 *
 *  Created on: Feb 11, 2025
 *      Author: chris
 */

#ifndef INC_PINS_H_
#define INC_PINS_H_

#include "main.h"

struct Pin {
	TIM_HandleTypeDef *TIM_MotorMaster = &htim2;
//X
#define X_STEP_PORT X_STEP_GPIO_Port
#define X_STEP_PIN X_STEP_Pin
#define X_DIR_PORT X_DIR_GPIO_Port
#define X_DIR_PIN X_DIR_Pin
//Y
#define Y_STEP_PORT Z_STEP_GPIO_Port
#define Y_STEP_PIN Z_STEP_Pin
#define Y_DIR_PORT Z_DIR_GPIO_Port
#define Y_DIR_PIN Z_DIR_Pin
//PRINTHEAD
	TIM_HandleTypeDef *TIM_Printhead = &htim3;
#define TIM_PrintheadChannel TIM_CHANNEL_1
//FAN
#define FAN0_PORT FAN0_GPIO_Port
#define FAN0_PIN FAN0_Pin
//ADC
	ADC_HandleTypeDef *ADC_Handle = &hadc1;
	TIM_HandleTypeDef *ADC_TIM = &htim8;
//GPIO
#define LOW_PRESSURE_PORT PRESSURE_GPIO_Port
#define LOW_PRESSURE_PIN PRESSURE_Pin
#define BATTERY_ALARM_PORT PWRDET_GPIO_Port
#define BATTERY_ALARM_PIN PWRDET_Pin
};

#endif /* INC_PINS_H_ */
