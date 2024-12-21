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
#ifndef STEPPER_H
#define STEPPER_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"

class MotorManager {
public:
	MotorManager(TIM_HandleTypeDef *htim) :
			htim(htim) {
	}

	TIM_HandleTypeDef *htim;

	static constexpr uint8_t stepIntervalDefault = 9;
	void setSpeed(float_t speed);
	void setAccel(float_t accel);
	void setDistance(float_t distance);
	void setParam(float_t speed, float_t accel, float_t distance);
	void resetStepCount();
	uint32_t calcInterval();
	void startTimer();
	void stopTimer();
	bool getTimerState();

private:
	float_t speed = DEFAULT_SPEED;
	static constexpr float_t minSpeed = 1;
	static constexpr float_t maxSpeed = 10000;
	float_t accel = DEFAULT_ACCEL;
	static constexpr float_t minAccel = 1;
	static constexpr float_t maxAccel = 10000;
	float_t distance;
	uint32_t stepCount = 0;	// Zähler für Schritte
	uint16_t accelStepCount = 0;
	float_t currentSpeed = 0.0f; // Aktuelle Geschwindigkeit (Schritte/s)
	float_t interval = stepIntervalDefault;	// Zeitintervall zwischen Schritten (s)

};

class StepperMotor {
public:
	StepperMotor(MotorManager *stepperMotorParent) :
			parent(*stepperMotorParent) {
	}

	void setStepPin(GPIO_TypeDef *inputStepPort, uint16_t inputStepPin);
	void setDirPin(GPIO_TypeDef *inputDirPort, uint16_t inputDirPin);
	void setStepDir(bool status);
	bool getStepDir();
	void handleStep();
	void setTargetPos(uint32_t target);

private:
	MotorManager &parent;

	uint16_t stepPin;
	GPIO_TypeDef *stepPort;
	uint16_t dirPin;
	GPIO_TypeDef *dirPort;

	bool direction;
	uint32_t currentPosition;
	uint32_t targetPosition;

	void step();
};

#define F_TIM 1000000 //1MHz

#endif /* STEPPER_H */
