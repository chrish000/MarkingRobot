/*
 * move.c
 *
 *  Created on: Jul 29, 2024
 *      Author: chris
 */
#include "main.h"

void Step_X(uint32_t steps, uint8_t dir)
{
	StepsToMove = steps;

	// Richtung setzen
	if (dir == 1) // Vorwärts
		HAL_GPIO_WritePin(X_DIR_GPIO_Port, X_DIR_Pin, GPIO_PIN_SET);
	else // Rückwärts
		HAL_GPIO_WritePin(X_DIR_GPIO_Port, X_DIR_Pin, GPIO_PIN_RESET);
}

void Step_Z(uint32_t steps, uint8_t dir)
{
	StepsToMove = steps;

	// Richtung setzen
	if (dir == 1) // Vorwärts
		HAL_GPIO_WritePin(Z_DIR_GPIO_Port, Z_DIR_Pin, GPIO_PIN_SET);
	else // Rückwärts
		HAL_GPIO_WritePin(Z_DIR_GPIO_Port, Z_DIR_Pin, GPIO_PIN_RESET);
}

void Move_Lin(float Distance)
{
	if (Distance != 0) //Prüfen ob Distanz nicht 0 ist
	{
		uint32_t steps = fabs(Distance * StepsPerMM); //Schritte berechnen
		uint8_t direction = (Distance > 0) ? 1 : 0; //Richtung bestimmen

		Step_X(steps, direction);
		Step_Z(steps, !direction);
		MovementActive = true;
	}
}

void Move_Rot(float Degrees)
{
	if (Degrees != 0) //Prüfen ob Distanz nicht 0 ist
	{
		uint32_t steps = fabs(Degrees * StepsPerDeg); //Schritte berechnen
		uint8_t direction = (Degrees > 0) ? 1 : 0; //Richtung bestimmen

		Step_X(steps, !direction);
		Step_Z(steps, !direction);
		MovementActive = true;
		Orientation += Degrees;
	}
}

float sqr(float x) { return x*x; }

float CalcTurn(float NewX, float NewY, float OldX, float OldY, float OldOrientation)
{
	float target = atan2(NewY-OldY, NewX-OldX) * 180/M_PI;//Zielwinkel berechnen
	target = fmod(target + 360, 360.0);//Zielwinkel normalisieren auf [0, 360]
	float turn = target - OldOrientation;//Drehwinkel berechnen
	turn = fmod(turn + 360, 360.0);//Drehwinkel normalisieren auf [0, 360]
	if(turn > 180)
	{
		turn -= 360; // Wenn der Winkel größer als 180 ist, den negativen kleineren Winkel verwenden
	}
	return turn;
}

float CalcDistance(float NewX, float NewY, float OldX, float OldY)
{
	return sqrt(sqr(NewX - OldX)+sqr(NewY - OldY));
}

void Move_To_Pos(float NextPosition[2])
{
	if(!(NextPosition[0] == PosX && NextPosition[1] == PosY))
	{
		//TODO Logik für Bewegung zu nächstem Punkt
		Move_Rot(CalcTurn(NextPosition[0], NextPosition[1], PosX, PosY, Orientation));
		while(MovementActive);
		Move_Lin(CalcDistance(NextPosition[0], NextPosition[1], PosX, PosY));
		while(MovementActive);
		PosX = NextPosition[0];
		PosY = NextPosition[1];
	}
}
