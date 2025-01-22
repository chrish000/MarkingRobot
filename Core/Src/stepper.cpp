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
#include <algorithm>

/* MotorManager --------------------------------------------------------------*/
/**
 * @brief Setzt die Geschwindigkeit in mm/s mit Begrenzung auf minSpeed und maxSpeed
 * @param mmPerSecond Geschwindigkeit in mm/s
 * @retval None
 */
/*
 void MotorManager::setSpeed(float_t mmPerSecond) {
 speed = std::clamp(mmPerSecond, minSpeed, maxSpeed);
 }
 */

/**
 * @brief Setzt die Beschleunigung in mm/s^2 mit Begrenzung auf minAccel und maxAccel
 * @param mmPerSecond2 Beschleunigung in mm/s^2
 * @retval None
 */
/*
 void MotorManager::setAccel(float_t mmPerSecond2) {
 accel = std::clamp(mmPerSecond2, minAccel, maxAccel);
 }
 */

/**
 * @brief Setzt die Distanz, falls sie nicht null ist
 * @param newDistance Neue Distanz in Einheiten
 * @retval None
 */
/*
 void MotorManager::setDistance(float_t newDistance) {
 if (newDistance != 0)
 distance = newDistance;
 }
 */

/**
 * @brief Setzt alle relevanten Parameter: Geschwindigkeit, Beschleunigung und Distanz
 * @param newSpeed Neue Geschwindigkeit in mm/s
 * @param newAccel Neue Beschleunigung in mm/s^2
 * @param newDistance Neue Distanz in Einheiten
 * @retval None
 */
/*
 void MotorManager::setParam(float_t newSpeed, float_t newAccel,
 float_t newDistance) {
 setSpeed(newSpeed);
 setAccel(newAccel);
 setDistance(newDistance);
 }
 */

/**
 * @brief Zurücksetzen des Schrittzählers
 * @param None
 * @retval None
 */
/*
void MotorManager::resetStepCount() {
	intervalCalc.stepCnt = 0;
}
*/

MotorManager::stepCmd MotorManager::trapezoid(moveCommands *moveCmd) {
	// 1. Beschleunigungsphase
	if (intervalCalc.currentSpeed < moveCmd->speed
			&& intervalCalc.stepCnt < moveCmd->stepDistance / 2) {
		intervalCalc.interval = 1.0f / intervalCalc.currentSpeed;
		if (intervalCalc.currentSpeed == 0) {
			intervalCalc.interval = sqrt(2.0f / moveCmd->accel); // Anfangsphase ohne Geschwindigkeit
		}
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

			stepBuf.insert(trapezoid(moveCmdCalcBuf));
		}
		//Berechnung abgeschlossen
		if (intervalCalc.stepCnt == moveCmdCalcBuf->stepDistance) {
			intervalCalc.stepCnt = 0;
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
		if (stepBuf.isFull()
				&& intervalCalc.stepCnt < moveCmdCalcBuf->stepDistance) {
			if (timerActiveFlag == false)
				startTimer();
			return true;
		}

		//Berechnung nicht abgeschlossen und Puffer nicht voll
		/*
		else {
			ErrorCode = STEP_BUF;
			Error_Handler();
			return false; //niemals erreicht
		}
		*/
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
	/*
	 currentPosition++;
	 if (currentPosition == targetPosition) {
	 parent.stopTimer();
	 currentPosition = 0;
	 }
	 */
}

/**
 * @brief Setzt die Zielposition
 * @param target Zielposition in Schritten
 * @retval None
 */
/*
 void StepperMotor::setTargetPos(uint32_t target) {
 targetPosition = target;
 }
 */

/**
 * @brief Führt einen Schritt aus, indem der Step-Pin toggelt
 * @param None
 * @retval None
 */
void StepperMotor::step() {
	if (stepPort != nullptr)
		HAL_GPIO_TogglePin(stepPort, stepPin); // double-edge des Schrittmotortreibers muss aktiv sein
}
