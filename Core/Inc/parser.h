/**
 ******************************************************************************
 * @file           : parser.h
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
#ifndef PARSER_H
#define PARSER_H

/* Includes ------------------------------------------------------------------*/
#include "ringbuffer.hpp"
#include "stepper.h"
#include "config.h"
#include "pins.h"

/* Defines -------------------------------------------------------------------*/
#define F_TIM 275000000 //275MHz
#define V_MIN (STEPS_PER_MM * 0.5) //Mindestgeschwindigkeit in steps/s (= x.x mm/s), MUSS mindestens *0.2 sein!

enum class Direction : bool {
	Forward = true, Reverse = false
};

class MotorManager {
public:
	//Konstruktor
	MotorManager(Pin pins);
	//Destruktor
	~MotorManager();

	StepperMotor motorX;
	StepperMotor motorY;

	struct moveCommands {
		float_t speed;	//Schritte pro s
		float_t accel;	//Schritte pro s^2
		uint32_t stepDistance;
		Direction directionX;
		Direction directionY;
		bool printigMove;
	};

	const static size_t buffer_size_move = 8;	//size = n-1 elements
	jnk0le::Ringbuffer<moveCommands, buffer_size_move, false, 32> moveBuf;

	bool calcInterval();
	void resetCalc();

private:

	struct intervalCalcStruct {
		uint32_t stepCnt = 0;	// Zähler für Schritte
		uint32_t accelStepCnt = 0;
		float_t currentSpeed = 0.0f; // Aktuelle Geschwindigkeit (Schritte/s)
		float_t currentAccel = 0.0f;
		float_t interval = 1 / V_MIN;	// Zeitintervall zwischen Schritten (ns)
#if	defined(ACCEL_CURVE_SINUS) || defined(ACCEL_CURVE_BEZIER)
		float_t timeAccel = 0.0f;
		float_t time = 1/V_MIN;
		float_t bezierT = 0;
#endif
	};

	intervalCalcStruct calcX;
	intervalCalcStruct calcY;
	moveCommands moveCmdCalcBufX;
	moveCommands moveCmdCalcBufY;

#if	defined(ACCEL_CURVE_BEZIER)
	const uint8_t bezierFactor = 0; //Verschiebung der Kontrollpubnkte in % (Abflachung) //BUG: Kurve entwickelt sich in falsche Richtung

#endif
	float_t trapezoid(moveCommands*, intervalCalcStruct*);
	void calculateTrapezoidAccelerationParameters(moveCommands*,
			intervalCalcStruct*);
#if defined(ACCEL_CURVE_BEZIER)
	float_t bezier(moveCommands*);
#endif
#if defined(ACCEL_CURVE_SINUS)
	float_t sinus(moveCommands*);
	void calculateSinusAccelerationParameters(moveCommands*);
#endif

	void calcRoutine(moveCommands*, intervalCalcStruct*, StepperMotor*);
};

#endif /* PARSER_H */
