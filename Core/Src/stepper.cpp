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
#if defined(ACCEL_CURVE_TRAPEZOID)
			stepBuf.insert(trapezoid(moveCmdCalcBuf));
#elif defined(ACCEL_CURVE_BEZIER)
			stepBuf.insert(bezier(moveCmdCalcBuf));
#endif
		}

		//Berechnung abgeschlossen
		if (intervalCalc.stepCnt == moveCmdCalcBuf->stepDistance) {
			intervalCalc.stepCnt = 0;
			bezier_t = 0;
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
MotorManager::stepCmd MotorManager::trapezoid(moveCommands *moveCmd) {
	if (intervalCalc.accelStepCnt == 0) { //Berechne Schrittzahl für Beschleunigung
		intervalCalc.accelStepCnt = roundf(
				sqr(moveCmd->speed) / (2.0f * moveCmd->accel));
		if (intervalCalc.accelStepCnt > moveCmd->stepDistance / 2) {
			intervalCalc.accelStepCnt = moveCmd->stepDistance / 2;
			moveCmd->speed = sqrt(
					2 * moveCmd->accel * intervalCalc.accelStepCnt);
		}
	}

	// 1. Beschleunigungsphase
	if (intervalCalc.stepCnt < intervalCalc.accelStepCnt) {
		const float_t P0 = V_MIN;			//Startgeschwindigkeit
		const float_t P1 = V_MIN + moveCmd->speed * (bezier_factor / 100);//Kontrollpunkt 1
		const float_t P2 = moveCmd->speed
				- moveCmd->speed * (bezier_factor / 100);	//Kontrollpunkt 2
		const float_t P3 = moveCmd->speed;	//Zielgeschwindigkeit

		bezier_t += 1.0f / intervalCalc.accelStepCnt;
		float_t velocity = eval_bezier(P0, P1, P2, P3, bezier_t); //Geschwindigkeit in steps/s
		intervalCalc.interval = 1.0f / velocity;
	} else if (intervalCalc.stepCnt = intervalCalc.accelStepCnt) {
		const float_t P0 = V_MIN;			//Startgeschwindigkeit
		const float_t P1 = V_MIN + moveCmd->speed * (bezier_factor / 100);//Kontrollpunkt 1
		const float_t P2 = moveCmd->speed
				- moveCmd->speed * (bezier_factor / 100);	//Kontrollpunkt 2
		const float_t P3 = moveCmd->speed;	//Zielgeschwindigkeit

		bezier_t += 1.0f / intervalCalc.accelStepCnt;
		float_t velocity = eval_bezier(P0, P1, P2, P3, bezier_t); //Geschwindigkeit in steps/s
		intervalCalc.interval = 1.0f / velocity;
		bezier_t = 0;
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
		const float_t P0 = moveCmd->speed;	//Startgeschwindigkeit
		const float_t P1 = V_MIN;			//Zielgeschwindigkeit

		bezier_t += 1.0f / intervalCalc.accelStepCnt;
		float_t velocity = interp(P0, P1, bezier_t); //Geschwindigkeit in steps/s
		intervalCalc.interval = 1.0f / velocity;
	}

	intervalCalc.stepCnt++;

	MotorManager::stepCmd stepCmd { stepCmd.interval = intervalCalc.interval
			* F_TIM, stepCmd.directionX = moveCmd->directionX,
			stepCmd.directionY = moveCmd->directionY };
	return stepCmd;
}

/**
 * @brief Berechnet das Intervall für den nächsten Schritt als S-Kurve
 * @param None
 * @retval Intervall mit der Sktuktur stepCmd
 */
MotorManager::stepCmd MotorManager::bezier(moveCommands *moveCmd) {
	if (intervalCalc.accelStepCnt == 0) { //Berechne Schrittzahl für Beschleunigung
		intervalCalc.accelStepCnt = roundf(
				sqr(moveCmd->speed) / (2.0f * moveCmd->accel));
		if (intervalCalc.accelStepCnt > moveCmd->stepDistance / 2) {
			intervalCalc.accelStepCnt = moveCmd->stepDistance / 2;
			moveCmd->speed = sqrt(
					2 * moveCmd->accel * intervalCalc.accelStepCnt);
		}
	}

	// 1. Beschleunigungsphase
	if (intervalCalc.stepCnt < intervalCalc.accelStepCnt) {
		const float_t P0 = V_MIN;			//Startgeschwindigkeit
		const float_t P1 = V_MIN + moveCmd->speed * (bezier_factor / 100);//Kontrollpunkt 1
		const float_t P2 = moveCmd->speed
				- moveCmd->speed * (bezier_factor / 100);	//Kontrollpunkt 2
		const float_t P3 = moveCmd->speed;	//Zielgeschwindigkeit

		bezier_t += 1.0f / intervalCalc.accelStepCnt;
		float_t velocity = eval_bezier(P0, P1, P2, P3, bezier_t); //Geschwindigkeit in steps/s
		intervalCalc.interval = 1.0f / velocity;
	} else if (intervalCalc.stepCnt = intervalCalc.accelStepCnt) {
		const float_t P0 = V_MIN;			//Startgeschwindigkeit
		const float_t P1 = V_MIN + moveCmd->speed * (bezier_factor / 100);//Kontrollpunkt 1
		const float_t P2 = moveCmd->speed
				- moveCmd->speed * (bezier_factor / 100);	//Kontrollpunkt 2
		const float_t P3 = moveCmd->speed;	//Zielgeschwindigkeit

		bezier_t += 1.0f / intervalCalc.accelStepCnt;
		float_t velocity = eval_bezier(P0, P1, P2, P3, bezier_t); //Geschwindigkeit in steps/s
		intervalCalc.interval = 1.0f / velocity;
		bezier_t = 0;
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
		const float_t P1 = V_MIN + moveCmd->speed * (bezier_factor / 100);//Kontrollpunkt 1
		const float_t P2 = moveCmd->speed
				- moveCmd->speed * (bezier_factor / 100);	//Kontrollpunkt 2
		const float_t P3 = V_MIN;					//Zielgeschwindigkeit

		bezier_t += 1.0f / intervalCalc.accelStepCnt;
		float_t velocity = eval_bezier(P0, P1, P2, P3, bezier_t); //Geschwindigkeit in steps/s
		intervalCalc.interval = 1.0f / velocity;
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
