/*
 * move.c
 *
 *  Created on: Jul 25, 2024
 *      Author: chris
 */
#include "main.h"

void Step_X(uint32_t StepCount, uint8_t Dir)
{
	PWMStepX = 0;
	PWMCounterX = 0; // Reset Counter
	TargetStepsX = StepCount; // Set Target Steps
	HAL_GPIO_WritePin(X_DIR_GPIO_Port, X_DIR_Pin, (Dir == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	PWMEnabledX = true; // Start Bewegung
}

uint8_t Move_To_Pos(uint16_t NextPosition[2])
{
	if(NextPosition[0] != PosX && NextPosition[1] != PosY)
	{
		//TODO Logik für Bewegung zu nächstem Punkt
	}
	return true;
}

uint8_t Move_Linear(float Distance)
{
	if (Distance != 0) //Prüfen ob Distanz nicht 0 ist
	{
		uint32_t steps = (uint32_t)(Distance * StepsPerMM); //Schritte berechnen
		uint8_t direction = (Distance > 0) ? 1 : 0; //Richtung bestimmen

		Step_X(steps, direction);
		//Step_Z(steps, direction);
	}
	return true;
}

uint8_t Move_Degrees(float Degrees)
{
	return true;
}


