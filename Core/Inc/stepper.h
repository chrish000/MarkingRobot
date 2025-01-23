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
#include "ringbuffer.hpp"

enum {
	forward = 1, reverse = 0
};

class MotorManager {
public:
	MotorManager(TIM_HandleTypeDef *htim) :
			htim(htim) {
	}

	TIM_HandleTypeDef *htim;

	struct moveCommands {
		float_t speed;
		float_t accel;
		uint32_t stepDistance;
		bool directionX;
		bool directionY;
	};
	struct stepCmd {
		uint32_t interval;
		bool directionX;
		bool directionY;
	};
	const static size_t buffer_size_move = 16;	//size = n-1 elements
	//TODO Der Buffer stepBuf ist zu klein für lange, schnelle Bewegungen
	const static size_t buffer_size_step = 64;	//size = n-1 elements
	jnk0le::Ringbuffer<moveCommands, buffer_size_move, 0, 32> moveBuf;
	jnk0le::Ringbuffer<stepCmd, buffer_size_step,0 ,32> stepBuf;

	static constexpr uint8_t stepIntervalDefault = 9;
	void setSpeed(float_t speed);
	void setAccel(float_t accel);
	void setDistance(float_t distance);
	void setParam(float_t speed, float_t accel, float_t distance);
	//void inline resetStepCount();
	bool calcInterval();
	void startTimer();
	void stopTimer();
	bool getTimerState();

private:

	moveCommands *moveCmdCalcBuf;

	struct intervalCalc {
		uint32_t stepCnt = 0;	// Zähler für Schritte
		uint32_t accelStepCnt = 0;
		float_t currentSpeed = 0.0f; // Aktuelle Geschwindigkeit (Schritte/s)
		float_t interval = stepIntervalDefault;	// Zeitintervall zwischen Schritten (s)
	} intervalCalc;
	bool timerActiveFlag = 0;
	struct stepCmd trapezoid(moveCommands*);
};

class StepperMotor {
public:
	/*
	 StepperMotor(MotorManager *stepperMotorParent) :
	 parent(*stepperMotorParent) {
	 }
	 */

	void setStepPin(GPIO_TypeDef *inputStepPort, uint16_t inputStepPin);
	void setDirPin(GPIO_TypeDef *inputDirPort, uint16_t inputDirPin);
	void setStepDir(bool status);
	bool getStepDir();
	void handleStep();
	void setTargetPos(uint32_t target);

private:
	//MotorManager &parent;

	uint16_t stepPin;
	GPIO_TypeDef *stepPort;
	uint16_t dirPin;
	GPIO_TypeDef *dirPort;

	bool direction;
	//uint32_t currentPosition;
	//uint32_t targetPosition;

	void step();
};

#define F_TIM 1000000 //1MHz

#endif /* STEPPER_H */
