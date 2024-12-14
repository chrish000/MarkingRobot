/**
 ******************************************************************************
 * @file           : stepper.cpp
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
#include "stepper.h"
#include "utils.h"
#include <algorithm>

//TODO Error-Handling

void StepperMotor::setStepPin(GPIO_TypeDef *inputStepPort,
		uint16_t inputStepPin) {
	stepPin = inputStepPin;
	stepPort = inputStepPort;
}

void StepperMotor::setDirPin(GPIO_TypeDef *inputDirPort, uint16_t inputDirPin) {
	dirPin = inputDirPin;
	dirPort = inputDirPort;
}

void StepperMotor::setStepDir(bool status) {
	stepDir = status;
	if (dirPort != nullptr) {
		if (stepDir == true)
			HAL_GPIO_WritePin(dirPort, dirPin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(dirPort, dirPin, GPIO_PIN_RESET);
	}
}

void SharedInterval::setSpeed(float_t mmPerSecond) {
	speed = std::clamp(mmPerSecond, minSpeed, maxSpeed);

}

void SharedInterval::setAccel(float_t mmPerSecond2) {
	accel = std::clamp(mmPerSecond2, minAccel, maxAccel);
}

void SharedInterval::setDistance(float_t newDistance) {
	if(newDistance != 0)
		distance = newDistance;
}

void SharedInterval::setParam(float_t newSpeed, float_t newAccel, float_t newDistance) {
	setSpeed(newSpeed);
	setAccel(newAccel);
	setDistance(newDistance);
}

void SharedInterval::resetStepCount() {
	stepCount = 0;
}

void StepperMotor::setActive(bool status) {
	steppingActive = status;
}

bool StepperMotor::getActive() {
	return steppingActive;
}

bool StepperMotor::getStepDir() {
	return stepDir;
}

void StepperMotor::setTargetPos(uint32_t target) {
	targetPosition = target;
}

void StepperMotor::step() {
	if (stepPort != nullptr)
		HAL_GPIO_TogglePin(stepPort, stepPin);	//TODO TMC2209:double edge
}

void StepperMotor::handleStep() {
	if (steppingActive) {
		step();
		currentPosition++;
		if (currentPosition == targetPosition) {
			steppingActive = false;
			currentPosition = 0;
		}
	}
}

uint32_t SharedInterval::popInterval() {	//TODO optimieren
	// 1. Beschleunigung
	if (currentSpeed < speed && stepCount < distance / 2) {
		interval = 1.0f / currentSpeed;
		if (currentSpeed == 0) {
			interval = sqrt(2.0f / accel); // Anfangsphase ohne Geschwindigkeit
		}
		currentSpeed += accel * interval;
		accelStepCount = stepCount;
	}

	// 2. Konstante Geschwindigkeit
	else if (stepCount < distance - accelStepCount) {
		interval = 1.0f / speed;
	}

	// 3. Abbremsphase
	else if (stepCount < distance) {
		interval = 1.0f / currentSpeed;
		currentSpeed -= accel * interval;
		if (currentSpeed < 0) {
			currentSpeed = 0; // Sicherheit: Keine negative Geschwindigkeit
		}
	}

	stepCount++;

	return (uint32_t) (interval * F_TIM);
}
