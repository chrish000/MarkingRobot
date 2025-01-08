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

class Robot {
public:
	Robot(TIM_HandleTypeDef *htim) :
			motorMaster(htim) /*, motorX(&motorMaster), motorY(&motorMaster)*/{
	}

	MotorManager motorMaster;
	StepperMotor motorX;
	StepperMotor motorY;

	void init();
	bool moveToPos(float_t newX, float_t newY, float_t speed = DEFAULT_SPEED,
			float_t accel = DEFAULT_ACCEL);

private:
	float_t posX = 0;
	float_t posY = 0;
	float_t speed = DEFAULT_SPEED;
	static constexpr float_t minSpeed = 1;	//TODO constrain einbauen
	static constexpr float_t maxSpeed = 10000;
	float_t accel = DEFAULT_ACCEL;
	static constexpr float_t minAccel = 1;
	static constexpr float_t maxAccel = 10000;
	float_t orientation = 0; //0°-360°

	bool moveLin(float_t distance, float_t speed, float_t accel);
	bool moveRot(float_t degrees, float_t speed, float_t accel);
};

#endif /* MOVE_H */
