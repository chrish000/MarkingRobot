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
#include <math.h>
#include "utils.h"
#include "move.h"

void Robot::moveLin(float distance, float speed, float accel) {
	if (distance != 0) //Prüfen ob Distanz nicht 0 ist
			{
		uint32_t steps = fabs(distance * StepsPerMM); //Schritte berechnen
		bool direction = (distance > 0) ? 1 : 0; //Richtung bestimmen TODO Distanz kann nie negativ sein mit aktueller Berechnung

		motorX.setStepDir(direction);
		motorY.setStepDir(!direction);
		intervalBuf.calculateIntervals(speed, accel, steps);
		motorX.setTargetPos(steps);
		motorY.setTargetPos(steps);
		motorX.setActive(true);
		motorY.setActive(true);
	}
}

void Robot::moveRot(float degrees, float speed, float accel) {
	if (degrees != 0) //Prüfen ob Distanz nicht 0 ist
			{
		uint32_t steps = fabs(degrees * StepsPerDeg); //Schritte berechnen
		bool direction = (degrees > 0) ? 1 : 0; //Richtung bestimmen

		motorX.setStepDir(!direction);
		motorY.setStepDir(!direction);
		intervalBuf.calculateIntervals(speed, accel, steps);
		motorX.setTargetPos(steps);
		motorY.setTargetPos(steps);
		motorX.setActive(true);
		motorY.setActive(true);
		orientation += degrees;
	}
}

float calcTurn(float newX, float newY, float oldX, float oldY,
		float oldOrientation) {
	float target = atan2(newY - oldY, newX - oldX) * 180 / M_PI; //Zielwinkel berechnen
	target = fmod(target + 360, 360.0); //Zielwinkel normalisieren auf [0, 360]
	float turn = target - oldOrientation; //Drehwinkel berechnen
	turn = fmod(turn + 360, 360.0); //Drehwinkel normalisieren auf [0, 360]
	if (turn > 180) {
		turn -= 360; // Wenn der Winkel größer als 180 ist, den negativen kleineren Winkel verwenden
	}
	return turn;
}

float calcDistance(float newX, float newY, float oldX, float oldY) {
	return sqrt(sqr(newX - oldX) + sqr(newY - oldY));
}

void Robot::moveToPos(float newX, float newY, float newSpeed, float newAccel) {
	if (!(newX == posX && newY == posY)) {
		speed = newSpeed;	//Aktuelle Geschwindigkeit speichern
		accel = newAccel;
		float turn = calcTurn(newX, newY, posX, posY, orientation);
		if (turn != 0) {
			moveRot(turn, speed, accel);
			while (motorX.getActive() || motorY.getActive())
				;
		}
		moveLin(calcDistance(newX, newY, posX, posY), speed, accel);
		while (motorX.getActive() || motorY.getActive())
			;
		posX = newX;
		posY = newY;
	}
}

void Robot::moveToPos(float newX, float newY) {
	if (!(newX == posX && newY == posY)) {
		float turn = calcTurn(newX, newY, posX, posY, orientation);
		if (turn != 0) {
			moveRot(turn, speed, accel);
			while (motorX.getActive() || motorY.getActive())
				;
		}
		moveLin(calcDistance(newX, newY, posX, posY), speed, accel);
		while (motorX.getActive() || motorY.getActive())
			;
		posX = newX;
		posY = newY;
	}
}

