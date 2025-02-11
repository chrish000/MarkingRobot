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

/* Includes ------------------------------------------------------------------*/
#include "stepper.h"
#include "utils.h"
#include <math.h>

/* MotorManager --------------------------------------------------------------*/

/**
 * @brief Berechnet das Zeitintervall zwischen Schritten basierend auf Geschwindigkeit und Beschleunigung
 * @param None
 * @retval Zeitintervall in Mikrosekunden
 */
bool MotorManager::calcInterval() {
	if (moveBuf.isEmpty() == false) {	//Berechne falls Daten vorhanden
		moveCmdCalcBuf = moveBuf.peek();
		//Berechne solange, bis Puffer voll oder Berechung abgeschlossen
		while (stepBuf.isFull() == false
				&& calc.stepCnt < moveCmdCalcBuf->stepDistance) {
#if defined(ACCEL_CURVE_TRAPEZOID)
			stepBuf.insert(trapezoid(moveCmdCalcBuf));
#elif defined(ACCEL_CURVE_BEZIER)
			stepBuf.insert(bezier(moveCmdCalcBuf));
#elif defined(ACCEL_CURVE_JERK)
			stepBuf.insert(jerk(moveCmdCalcBuf));
#endif
		}

		//Berechnung abgeschlossen
		if (calc.stepCnt == moveCmdCalcBuf->stepDistance) {
			calc.stepCnt = 0;
			bezierT = 0;
			if (moveBuf.remove()) {
				if (timerActiveFlag == false)
					startTimer();
				return true;
			} else {
				ErrorCode = MOVE_BUF;
				Error_Handler();
				return false; //niemals erreicht
			}
		}

		//Berechnung nicht abgeschlossen aber Puffer voll
		else if (stepBuf.isFull()
				&& calc.stepCnt < moveCmdCalcBuf->stepDistance) {
			if (timerActiveFlag == false)
				startTimer();
			return true;
		}

		//Berechnung nicht abgeschlossen und Puffer nicht voll
		else if (!stepBuf.isFull()
				&& calc.stepCnt >= moveCmdCalcBuf->stepDistance) {
			ErrorCode = STEP_BUF;
			Error_Handler();
			return false; //niemals erreicht
		}

	} else
		//Keine Daten für Berechnung vorhanden
		return false;
	return false;
}

/**
 * @brief Startet den Timer in der Klasse MotorManager mit dem Standardwert
 * @param None
 * @retval None
 */
void MotorManager::startTimer() {
	if (htim != nullptr) {
		htim->Instance->ARR = stepIntervalDefault;
		timerActiveFlag = true;
		HAL_TIM_Base_Start_IT(htim);
	}
}

/**
 * @brief Stoppt den Timer in der Klasse MotorManager und setzt den Standardwert des Zeahlers
 * @param None
 * @retval None
 */
void MotorManager::stopTimer() {
	if (htim != nullptr) {
		htim->Instance->ARR = stepIntervalDefault;
		timerActiveFlag = false;
		HAL_TIM_Base_Stop_IT(htim);
	}
}

/**
 * @brief Gibt den aktuellen Status des Timers in der Klasse MotorManager zurück
 * @param None
 * @retval true wenn der Timer aktiv ist, false sonst oder bei Fehler
 */
bool MotorManager::getTimerState() {
	HAL_TIM_StateTypeDef state = HAL_TIM_Base_GetState(htim);
	return (state == HAL_TIM_STATE_READY || state == HAL_TIM_STATE_BUSY) ? 1 : 0;
}

/**
 * @brief Berechnet das Intervall für den nächsten Schritt als lineare Beschleunigung
 * @param None
 * @retval Intervall mit der Sktuktur stepCmd
 */
//TODO Endposition
MotorManager::stepCmd MotorManager::trapezoid(moveCommands *moveCmd) {
	if (calc.accelStepCnt == 0) { //Berechne Schrittzahl für Beschleunigung
		calc.accelStepCnt = roundf(
				(pow(moveCmd->speed, 2) - pow(V_MIN, 2))
						/ (2.0f * moveCmd->accel));
		if (calc.accelStepCnt > moveCmd->stepDistance / 2) {
			calc.accelStepCnt = moveCmd->stepDistance / 2;
			moveCmd->speed = sqrt(2 * moveCmd->accel * calc.accelStepCnt);
		}
		calc.currentSpeed = V_MIN;
	} else {

		// 1. Beschleunigungsphase
		if (calc.stepCnt <= calc.accelStepCnt) {
			calc.currentAccel = moveCmd->accel;
		}

		// 2. Phase konstanter Geschwindigkeit
		else if (calc.stepCnt < moveCmd->stepDistance - calc.accelStepCnt) {
			calc.currentAccel = 0;
		}

		// 3. Abbremsphase
		else if (calc.stepCnt < moveCmd->stepDistance) {
			calc.currentAccel = -moveCmd->accel;
		}

		calc.currentSpeed += calc.currentAccel * calc.interval;
	}
	calc.interval = 1.0f / calc.currentSpeed;

	calc.stepCnt++;

	MotorManager::stepCmd stepCmd { stepCmd.interval = calc.interval * F_TIM,
			stepCmd.directionX = moveCmd->directionX, stepCmd.directionY =
					moveCmd->directionY, stepCmd.printigMove =
					moveCmd->printigMove };
	return stepCmd;
}

/**
 * @brief Berechnet das Intervall für den nächsten Schritt als S-Kurve
 * @param None
 * @retval Intervall mit der Sktuktur stepCmd
 */
//TODO Bugfix Werte
MotorManager::stepCmd MotorManager::bezier(moveCommands *moveCmd) {
	if (calc.accelStepCnt == 0) { //Berechne Schrittzahl für Beschleunigung
		calc.accelStepCnt = roundf(
				(pow(moveCmd->speed, 2) - pow(V_MIN, 2))
						/ (2.0f * moveCmd->accel));
		if (calc.accelStepCnt > moveCmd->stepDistance / 2) {
			calc.accelStepCnt = moveCmd->stepDistance / 2;
			moveCmd->speed = sqrt(2 * moveCmd->accel * calc.accelStepCnt);
		}
		calc.timeAccel = (moveCmd->speed - V_MIN) / moveCmd->accel;
	}

	// 1. Beschleunigungsphase
	if (calc.stepCnt <= calc.accelStepCnt) {
		const float_t P0 = V_MIN;			//Startgeschwindigkeit
		const float_t P1 = V_MIN + moveCmd->speed * (bezierFactor / 100);//Kontrollpunkt 1
		const float_t P2 = moveCmd->speed
				- moveCmd->speed * (bezierFactor / 100);	//Kontrollpunkt 2
		const float_t P3 = moveCmd->speed;	//Zielgeschwindigkeit

		bezierT = bezierT * calc.timeAccel + calc.interval / calc.timeAccel;
		float_t velocity = eval_bezier(P0, P1, P2, P3, bezierT); //Geschwindigkeit in steps/s
		calc.interval = 1.0f / velocity;
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
		const float_t P1 = V_MIN + moveCmd->speed * (bezierFactor / 100);//Kontrollpunkt 1
		const float_t P2 = moveCmd->speed
				- moveCmd->speed * (bezierFactor / 100);	//Kontrollpunkt 2
		const float_t P3 = V_MIN;					//Zielgeschwindigkeit

		bezierT = bezierT * calc.timeAccel + calc.interval / calc.timeAccel;
		float_t velocity = eval_bezier(P0, P1, P2, P3, bezierT); //Geschwindigkeit in steps/s
		calc.interval = 1.0f / velocity;
		if (calc.stepCnt == moveCmd->stepDistance)
			bezierT = 0;
	}

	calc.stepCnt++;

	MotorManager::stepCmd stepCmd { stepCmd.interval = calc.interval * F_TIM,
			stepCmd.directionX = moveCmd->directionX, stepCmd.directionY =
					moveCmd->directionY, stepCmd.printigMove =
					moveCmd->printigMove };
	return stepCmd;
}

/* StepperMotor --------------------------------------------------------------*/

/**
 * @brief Setzt die Drehrichtung des Motors
 * @param status true für Vorwärts, false für Rückwärts
 * @retval None
 */
void StepperMotor::setStepDir(bool status) {
	direction = status;
	if (dirPort != nullptr) {
		if (direction == forward)
			HAL_GPIO_WritePin(dirPort, dirPin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(dirPort, dirPin, GPIO_PIN_RESET);
	}
}

/**
 * @brief Gibt die aktuelle Drehrichtung zurück
 * @param None
 * @retval true wenn Vorwärts, false wenn Rückwärts
 */
bool StepperMotor::getStepDir() {
	return direction;
}

/**
 * @brief Fuehrt Schritte aus und überprüft das Erreichen der Zielposition
 * @param None
 * @retval None
 */
void StepperMotor::handleStep() {
	step();
}

/**
 * @brief Führt einen Schritt aus, indem der Step-Pin toggelt
 * @param None
 * @retval None
 */
void StepperMotor::step() {
	if (stepPort != nullptr)
		HAL_GPIO_TogglePin(stepPort, stepPin); // double-edge des Schrittmotortreibers muss aktiv sein
}
