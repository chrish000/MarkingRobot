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
#include "TMC2209.h"

/* Defines -------------------------------------------------------------------*/
#define F_TIM 1000000 //1MHz
#define V_MIN (STEPS_PER_MM * 0.5) //Mindestgeschwindigkeit in steps/s (= x.x mm/s), MUSS mindestens *0.2 sein!

enum class Direction : bool {
	Forward = true, Reverse = false
};

class MotorManager {
public:
	//Instruktor
	MotorManager(TIM_HandleTypeDef *htim) :
			htim(htim), moveCmdCalcBuf(new moveCommands { }) {
	}
	//Destruktor
	~MotorManager() {
		delete moveCmdCalcBuf;
	}

	TIM_HandleTypeDef *htim;

	struct moveCommands {
		float_t speed;	//Schritte pro s
		float_t accel;	//Schritte pro s^2
		uint32_t stepDistance;
		Direction directionX;
		Direction directionY;
		bool printigMove;
	};

	struct stepCmd {
		uint32_t interval;	//Dauer in ns
		Direction directionX;
		Direction directionY;
		bool printigMove;
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
		float_t timeAccel = 0.0f;
		float_t currentSpeed = 0.0f; // Aktuelle Geschwindigkeit (Schritte/s)
		float_t currentAccel = 0.0f;
		float_t interval = stepIntervalDefault;	// Zeitintervall zwischen Schritten (ns)
	} calc;

	bool timerActiveFlag = 0;
	const uint8_t bezierFactor = 30; //Verschiebung der Kontrollpubnkte in % (Abflachung)
	float_t bezierT = 0;
	struct stepCmd trapezoid(moveCommands*);
	struct stepCmd bezier(moveCommands*);
	void calculateTrapezoidAccelerationParameters(moveCommands*);
};

class StepperMotor {
public:
	//Instruktor
	StepperMotor(GPIO_TypeDef *stepPort, uint16_t stepPin,
			GPIO_TypeDef *dirPort, uint16_t dirPin,
			UART_HandleTypeDef *TMC_UART_address,
			GPIO_TypeDef *hardware_enable_port, uint16_t hardware_enable_pin) :
			tmc(TMC_UART_address, hardware_enable_port, hardware_enable_pin), stepPort(
					stepPort), stepPin(stepPin), dirPort(dirPort), dirPin(
					dirPin) {
	}
	//Destruktor
	~StepperMotor() {

	}

	TMC2209 tmc;

	void setStepDir(Direction dir);
	Direction getStepDir();
	void handleStep();

private:
	GPIO_TypeDef *stepPort;
	uint16_t stepPin;
	GPIO_TypeDef *dirPort;
	uint16_t dirPin;

	Direction direction = Direction::Forward;

	void step();
};

#endif /* STEPPER_H */
