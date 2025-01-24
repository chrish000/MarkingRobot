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
				&& intervalCalc.stepCnt < moveCmdCalcBuf->stepDistance) {
//#if defined(ACCEL_CURVE_TRAPEZOID)
			stepBuf.insert(trapezoid(moveCmdCalcBuf));
//#elif defined(ACCEL_CURVE_BEZIER)
//			stepBuf.insert(bezier(moveCmdCalcBuf));
//#endif
		}
		//Berechnung abgeschlossen
		if (intervalCalc.stepCnt == moveCmdCalcBuf->stepDistance) {
			intervalCalc.stepCnt = 0;
#ifdef ACCEL_CURVE_BEZIER
			bezier_t = 0;
#endif
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
				&& intervalCalc.stepCnt < moveCmdCalcBuf->stepDistance) {
			if (timerActiveFlag == false)
				startTimer();
			return true;
		}

		//Berechnung nicht abgeschlossen und Puffer nicht voll
		else if (!stepBuf.isFull()
				&& intervalCalc.stepCnt >= moveCmdCalcBuf->stepDistance) {
			ErrorCode = STEP_BUF;
			Error_Handler();
			return false; //niemals erreicht
		}
	} else
		//Keine Daten für Berechnung vorhanden
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
		//resetStepCount();
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
 * @brief Berechnet das Intervall für den nächsten Schritt
 * @param None
 * @retval Intervall mit der Sktuktur stepCmd
 */
MotorManager::stepCmd MotorManager::trapezoid(moveCommands *moveCmd) {
	// 1. Beschleunigungsphase
	if (intervalCalc.currentSpeed < moveCmd->speed
			&& intervalCalc.stepCnt < moveCmd->stepDistance / 2) {
		if (intervalCalc.currentSpeed == 0) {
			intervalCalc.interval = sqrt(2.0f / moveCmd->accel); // Anfangsphase ohne Geschwindigkeit
		} else
			intervalCalc.interval = 1.0f / intervalCalc.currentSpeed;
		intervalCalc.currentSpeed += moveCmd->accel * intervalCalc.interval;
		intervalCalc.accelStepCnt = intervalCalc.stepCnt;
	}

	// 2. Phase konstanter Geschwindigkeit
	else if (intervalCalc.stepCnt
			< moveCmd->stepDistance - intervalCalc.accelStepCnt) {
		intervalCalc.interval = 1.0f / moveCmd->speed;
	}

	// 3. Abbremsphase
	else if (intervalCalc.stepCnt < moveCmd->stepDistance) {
		intervalCalc.interval = 1.0f / intervalCalc.currentSpeed;
		intervalCalc.currentSpeed -= moveCmdCalcBuf->accel
				* intervalCalc.interval;
		if (intervalCalc.currentSpeed < 0) {
			intervalCalc.currentSpeed = 0; // Sicherheit: Keine negative Geschwindigkeit
		}
	}

	intervalCalc.stepCnt++;

	MotorManager::stepCmd stepCmd { stepCmd.interval = intervalCalc.interval
			* F_TIM, stepCmd.directionX = moveCmd->directionX,
			stepCmd.directionY = moveCmd->directionY };
	return stepCmd;
}

MotorManager::stepCmd MotorManager::bezier(moveCommands *moveCmd) {
	if (intervalCalc.accelStepCnt == 0) { //berechne die Schrittzahl für die Beschleunigung
		MotorManager::intervalCalcStruct intervalCalcTemp;
		while (intervalCalcTemp.currentSpeed < moveCmd->speed
				&& intervalCalcTemp.accelStepCnt < moveCmd->stepDistance / 2) {
			intervalCalcTemp.accelStepCnt++;
			if (intervalCalcTemp.currentSpeed == 0) {
				intervalCalcTemp.interval = sqrt(2.0f / moveCmd->accel); // Anfangsphase ohne Geschwindigkeit
			} else
				intervalCalcTemp.interval = 1.0f
						/ intervalCalcTemp.currentSpeed;
			intervalCalcTemp.currentSpeed += moveCmd->accel
					* intervalCalcTemp.interval;
		}
		intervalCalc.accelStepCnt = intervalCalcTemp.accelStepCnt;
	}

	// 1. Beschleunigungsphase
	if (bezier_t <= 1) {
		const float_t P0 = 0.0f;			//Startgeschwindigkeit
		const float_t P1 = 0.0f;			//Kontrollpunkt 1
		const float_t P2 = moveCmd->speed;	//Kontrollpunkt 2
		const float_t P3 = moveCmd->speed;	//Zielgeschwindigkeit

		float_t velocity = eval_bezier(P0, P1, P2, P3, bezier_t);
		if (velocity > 0.0f)
			intervalCalc.interval = 1.0f / velocity;
		else
			Error_Handler();

		bezier_t += 1 / moveCmd->stepDistance;
	}

	// 2. Phase konstanter Geschwindigkeit
	else if (intervalCalc.stepCnt
			< moveCmd->stepDistance - intervalCalc.accelStepCnt) {
		if (bezier_t > 0)
			bezier_t = 0;
		intervalCalc.interval = 1.0f / moveCmd->speed;
	}

	// 3. Abbremsphase
	else if (intervalCalc.stepCnt < moveCmd->stepDistance) {
		const float_t P0 = moveCmd->speed;			//Startgeschwindigkeit
		const float_t P1 = moveCmd->speed;			//Kontrollpunkt 1
		const float_t P2 = 0.0f;	//Kontrollpunkt 2
		const float_t P3 = 0.0f;	//Zielgeschwindigkeit

		float_t velocity = eval_bezier(P0, P1, P2, P3, bezier_t);
		if (velocity > 0.0f)
			intervalCalc.interval = 1.0f / velocity;
		else
			Error_Handler();

		bezier_t += 1 / moveCmd->stepDistance;
	}

	intervalCalc.stepCnt++;

	MotorManager::stepCmd stepCmd { stepCmd.interval = intervalCalc.interval
			* F_TIM, stepCmd.directionX = moveCmd->directionX,
			stepCmd.directionY = moveCmd->directionY };
	return stepCmd;
}

/* StepperMotor --------------------------------------------------------------*/
/**
 * @brief Setzt den Step-Pin (Port und Pin-Nummer)
 * @param inputStepPort GPIO-Port des Step-Pins
 * @param inputStepPin Pin-Nummer des Step-Pins
 * @retval None
 */
void StepperMotor::setStepPin(GPIO_TypeDef *inputStepPort,
		uint16_t inputStepPin) {
	stepPin = inputStepPin;
	stepPort = inputStepPort;
}

/**
 * @brief Setzt den Richtungs-Pin (Port und Pin-Nummer)
 * @param inputDirPort GPIO-Port des Richtungs-Pins
 * @param inputDirPin Pin-Nummer des Richtungs-Pins
 * @retval None
 */
void StepperMotor::setDirPin(GPIO_TypeDef *inputDirPort, uint16_t inputDirPin) {
	dirPin = inputDirPin;
	dirPort = inputDirPort;
}

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
