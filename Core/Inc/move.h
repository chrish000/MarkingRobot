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
#include "stepper.h"
#include "printhead.h"
#include "pins.h"

#include <array>

class Robot {
public:
	Robot(Pin pins) :
			motorMaster(pins.TIM_MotorMaster), motorX(X_STEP_PORT, X_STEP_PIN,
			X_DIR_PORT, X_DIR_PIN), motorY(Y_STEP_PORT, Y_STEP_PIN,
			Y_DIR_PORT, Y_DIR_PIN), printhead(pins.TIM_Printhead,
			TIM_PrintheadChannel), ADC_Handle(pins.ADC_Handle) {
	}

	MotorManager motorMaster;
	StepperMotor motorX;
	StepperMotor motorY;
	Printhead printhead;

	struct adcData {
		std::array<uint16_t, 2> array;
		uint16_t &batteryVoltage = array[0];
		uint16_t &airPressure = array[1];
	} adcVal;

	void init();
	bool moveToPos(float_t newX, float_t newY, float_t speed = DEFAULT_SPEED,
			float_t accel = DEFAULT_ACCEL, bool printing = false);

private:
	ADC_HandleTypeDef *ADC_Handle;

	float_t posX = 0;
	float_t posY = 0;
	float_t speed = DEFAULT_SPEED;
	const float_t minSpeed = 1;
	const float_t maxSpeed = 10000;
	float_t accel = DEFAULT_ACCEL;
	const float_t minAccel = 1;
	const float_t maxAccel = 10000;
	float_t orientation = 0; //0°-360°

	bool moveLin(float_t distance, float_t speed, float_t accel, bool printing);
	bool moveRot(float_t degrees, float_t speed, float_t accel);
};

#endif /* MOVE_H */
