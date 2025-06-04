/**
 ******************************************************************************
 * @file           : stepper.cpp
 * @brief          : Schrittberechungen
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

/* Includes ------------------------------------------------------------------*/
#include "parser.h"
#include "utils.h"
#include <math.h>
#include <limits>
#include <climits>
#include <algorithm>

#if MOTOR_XY_RATIO > 10000
    #define MOTOR_RATIO (2.0 - (MOTOR_XY_RATIO * 0.0001))
#else
    #define MOTOR_RATIO (MOTOR_XY_RATIO * 0.0001f)
#endif

// Konstruktor
MotorManager::MotorManager(Pin pins) :
		motorX(pins.TIM_Motor_X, pins.TIM_DMA_ARR_X, pins.TIM_DMA_BSRR_X,
		X_STEP_PORT, X_STEP_PIN, X_DIR_PORT, X_DIR_PIN, pins.TMC_UART_address_X,
				pins.CRC_Handle_X, X_EN_PORT, X_EN_PIN), motorY(
				pins.TIM_Motor_Y, pins.TIM_DMA_ARR_Y, pins.TIM_DMA_BSRR_Y,
				Y_STEP_PORT, Y_STEP_PIN, Y_DIR_PORT, Y_DIR_PIN,
				pins.TMC_UART_address_Y, pins.CRC_Handle_Y, Y_EN_PORT,
				Y_EN_PIN) {
}

// Destruktor
MotorManager::~MotorManager() = default;


void MotorManager::resetCalc() {
	calcX = intervalCalcStruct { };
	calcY = intervalCalcStruct { };
}

/**
 * @brief Schreibt die berechneten Schrittintervalle in den zugehörigen Puffer
 * @param None
 * @retval true wenn Speicherung erfolgereich
 */
void MotorManager::calcRoutine(moveCommands *mCmdBuf, intervalCalcStruct *calc,
		StepperMotor *M) {
	while (!M->stepBuf.isFull() && calc->stepCnt < mCmdBuf->stepDistance) {
		StepperMotor::stepCmd sCmd;

#if defined(ACCEL_CURVE_TRAPEZOID)
		sCmd.interval = trapezoid(mCmdBuf, calc);
#elif defined(ACCEL_CURVE_BEZIER)
		sCmd.interval = bezier(mCmdBuf, calc);
#elif defined(ACCEL_CURVE_SINUS)
		sCmd.interval = sinus(mCmdBuf, calc);
#endif

		bool directionMask =
				(M == &motorX) ?
						(bool) mCmdBuf->directionX : (bool) mCmdBuf->directionY;
		sCmd.gpioMask = (directionMask ? M->dirPin : (M->dirPin << 16))
				| (M->stepFlag ? (M->stepPin << 16) : M->stepPin);

		M->stepFlag = !M->stepFlag;
		M->stepBuf.insert(sCmd);
	}
}

/**
 * @brief Uebernimmt die Berechnung und das Handling der Schrittintervalle
 * @param None
 * @retval None
 */
bool MotorManager::calcInterval() {
	if (moveBuf.isEmpty())
		return false; 	//Berechne falls Daten vorhanden

	moveCmdFinishedFlag = false;
	moveCmdCalcBufX = moveCmdCalcBufY = *moveBuf.peek();

#if MOTOR_XY_RATIO > 10000
    moveCmdCalcBufX.accel = moveCmdCalcBufY.accel * MOTOR_RATIO;
    moveCmdCalcBufX.speed = moveCmdCalcBufY.speed * MOTOR_RATIO;
    moveCmdCalcBufX.stepDistance = moveCmdCalcBufY.stepDistance * MOTOR_RATIO;
#else
    moveCmdCalcBufY.accel = moveCmdCalcBufX.accel * MOTOR_RATIO;
    moveCmdCalcBufY.speed = moveCmdCalcBufX.speed * MOTOR_RATIO;
    moveCmdCalcBufY.stepDistance = moveCmdCalcBufX.stepDistance * MOTOR_RATIO;
#endif

	calcRoutine(&moveCmdCalcBufX, &calcX, &motorX);
	calcRoutine(&moveCmdCalcBufY, &calcY, &motorY);

	//Berechnung abgeschlossen
	if (calcX.stepCnt == moveCmdCalcBufX.stepDistance
			&& calcY.stepCnt == moveCmdCalcBufY.stepDistance) {

		resetCalc();

		if (moveBuf.remove()) {
			if (!motorX.timerActiveFlag)
				motorX.startTimer();
			if (!motorY.timerActiveFlag)
				motorY.startTimer();
			moveCmdFinishedFlag = true;
			posBuf.remove();
			return true;
		} else {
			ErrorCode = MOVE_BUF;
			Error_Handler();
			return false; //niemals erreicht
		}
	}
	//Wenn X voll oder fertig und Y voll oder fertig
	else if ((motorX.stepBuf.isFull()
			|| calcX.stepCnt == moveCmdCalcBufX.stepDistance)
			&& (motorY.stepBuf.isFull()
					|| calcY.stepCnt == moveCmdCalcBufY.stepDistance)) {
		printFlag = moveCmdCalcBufX.printingMove;
		if (!motorX.timerActiveFlag)
			motorX.startTimer();
		if (!motorY.timerActiveFlag)
			motorY.startTimer();
		return true;
	}
	return false;
}

#if defined(ACCEL_CURVE_TRAPEZOID)
/**
 * @brief Berechnet das Intervall für den nächsten Schritt als lineare Beschleunigung
 * @param None
 * @retval Intervallwert
 */
float_t MotorManager::trapezoid(moveCommands *moveCmd,
		intervalCalcStruct *calc) {
	if (calc->accelStepCnt == 0) {
		calculateTrapezoidAccelerationParameters(moveCmd, calc);
		calc->currentSpeed = V_MIN;
	}

	calc->currentAccel =
			(calc->stepCnt <= calc->accelStepCnt) ? moveCmd->accel :
			(calc->stepCnt < moveCmd->stepDistance - calc->accelStepCnt) ?
					0 : -moveCmd->accel;

	calc->currentSpeed += calc->currentAccel * calc->interval;
	calc->interval = 1.0 / calc->currentSpeed;
	calc->stepCnt++;

	return (calc->interval * F_TIM);
}

/**
 * @brief Berechnet die nötigen Parameter fuer ein Trapezprofil und passt ggf Geschwindigkeit an
 * @param moveCmd Pointer zu Berechnungsbefehl
 * @retval None
 */
void MotorManager::calculateTrapezoidAccelerationParameters(
		moveCommands *moveCmd, intervalCalcStruct *calc) {

	float_t speedSquared = moveCmd->speed * moveCmd->speed;
	float_t vMinSquared = V_MIN * V_MIN;
	calc->accelStepCnt = static_cast<uint32_t>((speedSquared - vMinSquared)
			/ (2.0 * moveCmd->accel) + 0.5); //+0.5 für korrektes runden ohne roundf()

	if (calc->accelStepCnt > moveCmd->stepDistance * 0.5) {
		calc->accelStepCnt = moveCmd->stepDistance * 0.5;
		moveCmd->speed = sqrtf(2 * moveCmd->accel * calc->accelStepCnt);
	}
}
#endif

#if defined(ACCEL_CURVE_BEZIER)
float_t MotorManager::bezier(moveCommands *moveCmd) {
	if (calc.accelStepCnt == 0) { //Berechne Schrittzahl für Beschleunigung
		calculateTrapezoidAccelerationParameters(moveCmd);
		calc.timeAccel = (moveCmd->speed - V_MIN) / moveCmd->accel;
		calc.interval = 1.0f / V_MIN;
	}

	// 1. Beschleunigungsphase
	if (calc.stepCnt <= calc.accelStepCnt) {
		const float_t P0 = V_MIN;			//Startgeschwindigkeit
		const float_t P1 = V_MIN + moveCmd->speed * (bezierFactor *0.01f);//Kontrollpunkt 1
		const float_t P2 = moveCmd->speed
				- moveCmd->speed * (bezierFactor *0.01f);	//Kontrollpunkt 2
		const float_t P3 = moveCmd->speed;	//Zielgeschwindigkeit

		bezierT += calc.interval / calc.timeAccel;
		float_t velocity = eval_bezier(P0, P1, P2, P3, bezierT); //Geschwindigkeit in steps/s
		calc.interval = 1.0f / fmax(velocity, V_MIN);
		if (calc.stepCnt == calc.accelStepCnt)
			bezierT = 0;
	}

	// 2. Phase konstanter Geschwindigkeit
	else if (calc.stepCnt < moveCmd->stepDistance - calc.accelStepCnt) {
		if (bezierT > 0)
			bezierT = 0;
		calc.interval = 1.0f / moveCmd->speed;
	}

	// 3. Abbremsphase
	else if (calc.stepCnt < moveCmd->stepDistance) {
		const float_t P0 = moveCmd->speed;			//Startgeschwindigkeit
		const float_t P1 = moveCmd->speed
				- moveCmd->speed * (bezierFactor *0.01f);	//Kontrollpunkt 1
		const float_t P2 = V_MIN + moveCmd->speed * (bezierFactor *0.01f);//Kontrollpunkt 2
		const float_t P3 = V_MIN;					//Zielgeschwindigkeit

		bezierT += calc.interval / calc.timeAccel;
		float_t velocity = eval_bezier(P0, P1, P2, P3, bezierT); //Geschwindigkeit in steps/s
		calc.interval = 1.0f / fmax(velocity, V_MIN);
		if (calc.stepCnt == moveCmd->stepDistance) {
			bezierT = 0;
			calc.accelStepCnt = 0;
			calc.timeAccel = 0;
		}
	}

	calc.stepCnt++;

	return (calc.interval * F_TIM);
}
#endif

#if defined(ACCEL_CURVE_SINUS)
float_t MotorManager::sinus(moveCommands *moveCmd) {
	if (calc.accelStepCnt == 0) {
		calculateSinusAccelerationParameters(moveCmd);
	}
	float_t a_max = moveCmd->accel, v_max = moveCmd->speed;

	// 1. Beschleunigungsphase
	if (calc.stepCnt <= calc.accelStepCnt) {
		//calc.currentSpeed = (v_max + V_MIN)/2-(v_max-V_MIN)/2*cos(2*a_max/v_max*calc.time);
		calc.currentAccel = a_max * (v_max - V_MIN) / v_max * sin(2 * a_max / v_max * calc.time);
	}

	// 2. Phase konstanter Geschwindigkeit
	else if (calc.stepCnt < moveCmd->stepDistance - calc.accelStepCnt) {
		calc.currentAccel = 0;
		calc.time = 0;
		calc.currentSpeed = moveCmd->speed;
	}

	// 3. Abbremsphase
	else if (calc.stepCnt < moveCmd->stepDistance) {
		//calc.currentSpeed = (v_max + V_MIN)/2+(v_max-V_MIN)/2*cos(2*a_max/v_max*calc.time);
		calc.currentAccel = -a_max * (v_max - V_MIN) / v_max	* sin(2 * a_max / v_max * calc.time);

		if (calc.stepCnt == moveCmd->stepDistance) {
			calc.accelStepCnt = 0;
			calc.time = 0;
		}
	}
	calc.currentSpeed += calc.currentAccel * calc.interval;
	calc.interval = 1.0f / calc.currentSpeed;
	calc.time += calc.interval;

	calc.stepCnt++;

	return (calc.interval * F_TIM);
}

void MotorManager::calculateSinusAccelerationParameters(moveCommands *moveCmd) {//TODO accelStepCnt berechnung stimmt noch nicht
	float_t stepCnt = 0, speed = 0, accel = 0, time = 1 / V_MIN, interval = 1
			/ V_MIN;
	float_t a_max = moveCmd->accel, v_max = moveCmd->speed;
	for (; speed < v_max; stepCnt++) {
		accel = a_max * (v_max - V_MIN) / v_max * sin(2 * a_max / v_max * time);
		speed += accel * interval;
		interval = 1 / speed;
		time += interval;
	}
	calc.accelStepCnt = stepCnt;
	//calc.accelStepCnt = M_PI*pow(v_max-V_MIN,2)/(2*a_max);
	if(calc.accelStepCnt > moveCmd->stepDistance *0.5f){
		calc.accelStepCnt = moveCmd->stepDistance *0.5f;
		moveCmd->speed = sqrt(2*a_max*moveCmd->stepDistance/M_PI)-V_MIN;
	}
}
#endif
