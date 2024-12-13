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
#ifndef INC_MOVE_H_
#define INC_MOVE_H_

#include "stepper.h"

class Robot {
public:

	StepperMotor motorX;
	StepperMotor motorY;
	SharedInterval intervalBuf;
	void moveToPos(float newX, float newY, float speed, float accel);
	void moveToPos(float newX, float newY);

private:

	short posX = 0;
	short posY = 0;
	float speed = DEFAULT_SPEED; //in mm/s
	float accel = DEFAULT_ACCEL;
	short orientation = 0; //0°-360°

	void moveLin(float distance, float speed, float accel);
	void moveRot(float degrees, float speed, float accel);
};

#endif /* INC_MOVE_H_ */
