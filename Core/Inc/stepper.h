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
		float_t speed;	//Schritte pro s
		float_t accel;	//Schritte pro s^2
		uint32_t stepDistance;
		bool directionX;
		bool directionY;
	};

	struct stepCmd {
		uint32_t interval;	//Dauer in ns
		bool directionX;
		bool directionY;
	};

	const static size_t buffer_size_move = 16;	//size = n-1 elements
	const static size_t buffer_size_step = 64;	//size = n-1 elements
	jnk0le::Ringbuffer<moveCommands, buffer_size_move, 0, 32> moveBuf;
	jnk0le::Ringbuffer<stepCmd, buffer_size_step, 0, 32> stepBuf;

	static constexpr uint8_t stepIntervalDefault = 9;
	bool calcInterval();
	void startTimer();
	void stopTimer();
	bool getTimerState();

private:

	moveCommands *moveCmdCalcBuf;

	struct intervalCalcStruct {
		uint32_t stepCnt = 0;	// Zähler für Schritte
		uint32_t accelStepCnt = 0;
		float_t currentSpeed = 0.0f; // Aktuelle Geschwindigkeit (Schritte/s)
		float_t interval = stepIntervalDefault;	// Zeitintervall zwischen Schritten (ns)
	} intervalCalc;

	bool timerActiveFlag = 0;
	float_t bezier_t = 0;
	struct stepCmd trapezoid(moveCommands*);
	struct stepCmd bezier(moveCommands*);
};

class StepperMotor {
public:

	void setStepPin(GPIO_TypeDef *inputStepPort, uint16_t inputStepPin);
	void setDirPin(GPIO_TypeDef *inputDirPort, uint16_t inputDirPin);
	void setStepDir(bool status);
	bool getStepDir();
	void handleStep();

private:

	uint16_t stepPin;
	GPIO_TypeDef *stepPort;
	uint16_t dirPin;
	GPIO_TypeDef *dirPort;

	bool direction;

	void step();
};

#define F_TIM 1000000 //1MHz

#endif /* STEPPER_H */
