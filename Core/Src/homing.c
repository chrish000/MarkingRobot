/*
 * homing.c
 *
 *  Created on: Jul 25, 2024
 *      Author: chris
 */
#include "main.h"
#include "homing.h"
#include "move.h"

/**
  * @brief  Handle Timeout while homing.
  *
  * @note
  *
  * @param  none
  *
  * @retval None
  */
void handleTimeout()
{
	//TODO: ERROR
}

/**
  * @brief  Homing-Routine
  *
  * @note
  *
  * @param  none
  *
  * @retval None
  */
void home()
{
	/*TODO Der Roboter soll eine bestimmte Stecke fahren. Wenn innerhalb dieser Stecke ein Endschalter
	 * geschalten hat, so soll der Roboter anhalten und sich drehen, bis beide Endschalter gleichzeitig schalten.
	 * Wenn die Strecke gefahren wurde und kein Endschalter wurde getroffen, soll ein ERROR
	 * ausgegeben werden. */
	while(1)
	{
		if(1 /* Wenn die definierte Distanz gefahren wurde */)
		{
			handleTimeout();
			break;
		}
		//Kein Endschalter
		if(!(HAL_GPIO_ReadPin(X_MIN_GPIO_Port, X_MIN_Pin) || HAL_GPIO_ReadPin(Z_MIN_GPIO_Port, Z_MIN_Pin)))
		{
			// TODO move
		}
		//X_MIN zuerst
		else if(HAL_GPIO_ReadPin(X_MIN_GPIO_Port, X_MIN_Pin) && !HAL_GPIO_ReadPin(Z_MIN_GPIO_Port, Z_MIN_Pin))
		{
			Move_Degrees(1);
			break;
		}
		//Z_MIN zuerst
		else if(!HAL_GPIO_ReadPin(X_MIN_GPIO_Port, X_MIN_Pin) && HAL_GPIO_ReadPin(Z_MIN_GPIO_Port, Z_MIN_Pin))
		{
			Move_Degrees(-1);
			break;
		}
		//Beide Endschater
		else if(HAL_GPIO_ReadPin(X_MIN_GPIO_Port, X_MIN_Pin) && HAL_GPIO_ReadPin(Z_MIN_GPIO_Port, Z_MIN_Pin))
		{

		}
	}

}
