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
#define F_TIM 275000000 //1MHz
#define V_MIN (STEPS_PER_MM * 0.5) //Mindestgeschwindigkeit in steps/s (= x.x mm/s), MUSS mindestens *0.2 sein!

enum class Direction : bool {
	Forward = true, Reverse = false
};

class MotorManager {
public:
	//Konstruktor
	MotorManager(TIM_HandleTypeDef *htim);
	//Destruktor
	~MotorManager();

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
			uint16_t interval:13;	//Dauer in ns
			uint16_t directionX:1;
			uint16_t directionY:1;
			uint16_t printigMove:1;
		};

	const static size_t buffer_size_move = 16;	//size = n-1 elements
	const static size_t buffer_size_step = 128;	//size = n-1 elements
	jnk0le::Ringbuffer<moveCommands, buffer_size_move, false, 32> moveBuf;
	static jnk0le::Ringbuffer<stepCmd, buffer_size_step, false, 32>* stepBuf;

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
		float_t interval = 1/V_MIN;	// Zeitintervall zwischen Schritten (ns)
		float_t time = 1/V_MIN;
	} calc;

	bool timerActiveFlag = 0;
	const uint8_t bezierFactor = 0; //Verschiebung der Kontrollpubnkte in % (Abflachung) //BUG: Kurve entwickelt sich in falsche Richtung
	float_t bezierT = 0;
	struct stepCmd trapezoid(moveCommands*);
	struct stepCmd bezier(moveCommands*);
	struct stepCmd sinus(moveCommands*);
	void calculateTrapezoidAccelerationParameters(moveCommands*);
	void calculateSinusAccelerationParameters(moveCommands*);
};

class StepperMotor {
public:
	//Instruktor
	StepperMotor(GPIO_TypeDef *stepPort, uint16_t stepPin,
			GPIO_TypeDef *dirPort, uint16_t dirPin,
			UART_HandleTypeDef *TMC_UART_address,
			GPIO_TypeDef *hardware_enable_port, uint16_t hardware_enable_pin, bool inverse_motor_dir) :
			tmc(TMC_UART_address, hardware_enable_port, hardware_enable_pin), stepPort(
					stepPort), stepPin(stepPin), dirPort(dirPort), dirPin(
					dirPin), inverseMotorDirection(inverse_motor_dir) {
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
	bool inverseMotorDirection;

	void step();
};

#endif /* STEPPER_H */
