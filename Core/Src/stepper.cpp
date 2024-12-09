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
//#include "circular_buffer.h"
//#include "bresenham.h"

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

void SharedIntervalBuffer::setSpeed(float_t mmPerSecond) {
	speed = constrain(mmPerSecond, minSpeed, maxSpeed);

}

void SharedIntervalBuffer::setAccel(float_t mmPerSecond2) {
	accel = constrain(mmPerSecond2, minAccel, maxAccel);
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

void SharedIntervalBuffer::calculateIntervals(float_t speedInput,
		float_t accelInput, float_t distance) {
	head = 0;
	tail = 0;
	setSpeed(speedInput);
	setAccel(accelInput);
	float_t current_velocity = 0.0f; // Aktuelle Geschwindigkeit (Schritte/s)
	float_t time_interval;          // Zeitintervall zwischen Schritten (s)
	uint16_t step_count = 0;           // Zähler für Schritte

	// 1. Beschleunigungsphase
	while (current_velocity < speed && step_count < distance / 2) {
		time_interval = 1.0f / current_velocity;
		if (current_velocity == 0) {
			time_interval = sqrt(2.0f / accel); // Anfangsphase ohne Geschwindigkeit
		}

		buffer[step_count++] = (uint32_t) (time_interval * F_TIM);
		current_velocity += accel * time_interval;
	}

	uint16_t accel_count = step_count;

	// 2. Konstante Geschwindigkeit
	while (step_count < distance - accel_count) {
		time_interval = 1.0f / speed;
		buffer[step_count++] = (uint32_t) (time_interval * F_TIM);
	}

	// 3. Abbremsphase
	while (step_count < distance) {
		time_interval = 1.0f / current_velocity;
		buffer[step_count++] = (uint32_t) (time_interval * F_TIM);
		current_velocity -= accel * time_interval;
		if (current_velocity < 0)
			current_velocity = 0; // Sicherheit: Keine negative Geschwindigkeit
	}

	tail = step_count; // Gesamtanzahl der berechneten Intervalle

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

uint16_t SharedIntervalBuffer::popInterval() {
	if (head == tail) {
		return 0; // Leer
	}
	uint16_t value = buffer[head];
	head = (head + 1) % size;
	return value;
}
