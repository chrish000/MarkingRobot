/**
 ******************************************************************************
 * @file           : move.c
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
#include <math.h>
#include "utils.h"
#include "move.h"

/**
 * @brief Lineare Bewegung des Roboters
 * @param distance Distanz in mm
 * @param speed Geschwindigkeit in mm/s
 * @param accel Beschleunigung in mm/s^2
 * @retval None
 */
void Robot::moveLin(float_t distance, float_t speed, float_t accel) {
	if (distance != 0) //Prüfen ob Distanz nicht 0 ist
			{
		uint32_t steps = fabs(distance * STEPS_PER_MM); //Schritte berechnen
		bool direction = (distance > 0) ? 1 : 0; //Richtung bestimmen TODO Distanz kann nie negativ sein mit aktueller Berechnung

		motorX.setStepDir(direction);
		motorY.setStepDir(!direction);
		motorMaster.setParam(speed * STEPS_PER_MM, accel * STEPS_PER_MM, steps);
		motorX.setTargetPos(steps);
		motorY.setTargetPos(steps);
		HAL_TIM_Base_Start_IT(motorMaster.htim);
	}
}

/**
 * @brief Rotationsbewegung des Roboters
 * @param degrees Drehwinkel in Grad
 * @param speed Geschwindigkeit in mm/s
 * @param accel Beschleunigung in mm/s^2
 * @retval None
 */
void Robot::moveRot(float_t degrees, float_t speed, float_t accel) {
	if (degrees != 0) //Prüfen ob Distanz nicht 0 ist
			{
		uint32_t steps = fabs(degrees * STEPS_PER_DEG); //Schritte berechnen
		bool direction = (degrees > 0) ? 1 : 0; //Richtung bestimmen

		motorX.setStepDir(!direction);
		motorY.setStepDir(!direction);
		motorMaster.setParam(speed * STEPS_PER_MM, accel * STEPS_PER_MM, steps);
		motorX.setTargetPos(steps);
		motorY.setTargetPos(steps);
		HAL_TIM_Base_Start_IT(motorMaster.htim);
		orientation += degrees;
	}
}

/**
 * @brief Berechnung des Drehwinkels für eine Zielposition
 * @param newX Zielposition X-Koordinate
 * @param newY Zielposition Y-Koordinate
 * @param oldX Aktuelle X-Koordinate
 * @param oldY Aktuelle Y-Koordinate
 * @param oldOrientation Aktuelle Orientierung in Grad
 * @retval Berechneter Drehwinkel in Grad
 */
float_t calcTurn(float_t newX, float_t newY, float_t oldX, float_t oldY,
		float_t oldOrientation) {
	float_t target = atan2(newY - oldY, newX - oldX) * 180 / M_PI; //Zielwinkel berechnen
	target = fmod(target + 360, 360.0); //Zielwinkel normalisieren auf [0, 360]
	float_t turn = target - oldOrientation; //Drehwinkel berechnen
	turn = fmod(turn + 360, 360.0); //Drehwinkel normalisieren auf [0, 360]
	if (turn > 180) {
		turn -= 360; // Wenn der Winkel größer als 180 ist, den negativen kleineren Winkel verwenden
	}
	return turn;
}

/**
 * @brief Berechnung der Distanz zwischen zwei Punkten
 * @param newX Zielposition X-Koordinate
 * @param newY Zielposition Y-Koordinate
 * @param oldX Aktuelle X-Koordinate
 * @param oldY Aktuelle Y-Koordinate
 * @retval Berechnete Distanz in mm
 */
float_t calcDistance(float_t newX, float_t newY, float_t oldX, float_t oldY) {
	return sqrt(sqr(newX - oldX) + sqr(newY - oldY));
}

/**
 * @brief Bewegung des Roboters zu einer Zielposition
 * @param newX Zielposition X-Koordinate
 * @param newY Zielposition Y-Koordinate
 * @param newSpeed Geschwindigkeit in mm/s
 * @param newAccel Beschleunigung in mm/s^2
 * @retval None
 */
void Robot::moveToPos(float_t newX, float_t newY, float_t newSpeed,
		float_t newAccel) {
	if (!(newX == posX && newY == posY)) {
		speed = newSpeed;	//Aktuelle Geschwindigkeit speichern
		accel = newAccel;
		float_t turn = calcTurn(newX, newY, posX, posY, orientation);
		if (turn != 0) {
			moveRot(turn, speed, accel);
			while (motorMaster.getTimerState())
				//TODO optimieren
				;
		}
		moveLin(calcDistance(newX, newY, posX, posY), speed, accel);
		while (motorMaster.getTimerState())
			//TODO optimieren
			;
		posX = newX;
		posY = newY;
	}
}

