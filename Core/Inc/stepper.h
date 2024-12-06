/**
 ******************************************************************************
 * @file           : stepper.h
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
#ifndef INC_STEPPER_H_
#define INC_STEPPER_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h"

class StepperMotor {
public:
	void setStepPin(GPIO_TypeDef *inputStepPort, uint16_t inputStepPin);
	void setDirPin(GPIO_TypeDef *inputDirPort, uint16_t inputDirPin);
	void setStepDir(bool status);
	bool getStepDir();
	void handleStep();
	void setActive(bool status);
	bool getActive();
	void setTargetPos(uint32_t target);

private:
	uint16_t stepPin;
	GPIO_TypeDef *stepPort;
	uint16_t dirPin;
	GPIO_TypeDef *dirPort;

	bool stepDir;
	volatile bool steppingActive = false;
	uint32_t currentPosition;
	uint32_t targetPosition;

	void step();
};

class SharedIntervalBuffer {
private:
	const static uint16_t size = 20000;
	uint32_t buffer[size];
	uint16_t head = 0;
	uint16_t tail = 0;

	uint32_t speed = DEFAULT_SPEED;
	const static uint8_t minSpeed = 1;
	const static uint8_t maxSpeed = 100;
	uint32_t accel = DEFAULT_ACCEL;
	const static uint16_t minAccel = 1;
	const static uint16_t maxAccel = 10000;

	uint16_t intervalPointer = 0;

public:
	const static uint8_t stepIntervalDefault = 9;
	void setSpeed(float_t speed);
	void setAccel(float_t accel);
	void calculateIntervals(float_t speed, float_t accel, float_t distance);
	uint16_t popInterval();
};
#define F_TIM 1000000 //1MHz

#endif /* INC_STEPPER_H_ */
