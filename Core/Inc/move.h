/**
 ******************************************************************************
 * @file           : move.h
 * @brief          :
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
#ifndef MOVE_H
#define MOVE_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stepper.h"
#include "parser.h"
#include "printhead.h"
#include "pins.h"
#include "config.h"

class Robot {
public:
	Robot() :
			motorMaster(pins), printhead(pins.TIM_Printhead,
			TIM_PrintheadChannel), ADC_Handle(pins.ADC_Handle), ADC_TIM(
					pins.ADC_TIM) {
	}

	Pin pins;
	MotorManager motorMaster;
	Printhead printhead;

	uint16_t ADC_BatteryVoltage = 0;
	float_t batteryVoltage = 0;
	bool lowAirPressure = false;
	bool batteryAlarm = false;

	void init();
	bool moveToPos(float_t newX, float_t newY, float_t speed = DEFAULT_SPEED,
			float_t accel = DEFAULT_ACCEL, bool printing = false);
	bool moveLin(float_t distance, float_t speed = DEFAULT_SPEED,
			float_t accel = DEFAULT_ACCEL, bool printing = false);
	bool moveRot(float_t degrees, float_t speed = DEFAULT_SPEED, float_t accel =
			DEFAULT_ACCEL);
	bool moveToHome();
	void resetPos();
	void setPos(float_t newOrientation, float_t newX, float_t newY);

private:
	ADC_HandleTypeDef *ADC_Handle;
	TIM_HandleTypeDef *ADC_TIM;

	float_t posX = 0;
	float_t posY = 0;
	float_t speed = DEFAULT_SPEED;
	float_t accel = DEFAULT_ACCEL;
	float_t orientation = 0; //0°-360°
};

#endif /* MOVE_H */
