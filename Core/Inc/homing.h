/*

 * homing.h

 *

 *  Created on: Feb 10, 2025

 *      Author: Chris Hauser

 */
#ifndef INC_HOMING_H_
#define INC_HOMING_H_

#define MAX_HOMING_DIST 200 //mm
#define MAX_HOMING_TRY 3
#define MAX_HOMING_TIMEOUT HAL_MAX_DELAY //ms
#define HOMING_SPEED 100 //mm/s
#define HOMING_MAX_FAULT 1 //deg
#define	SENSOR_DIST 460.5f //mm
#define HOMING_OFFSET_X 140 //mm	(von Roboter Rahmen aussen hinten zu Duese)
#define HOMING_OFFSET_Y 298 //mm	(von Roboter Antrieb aussen seitlich zu Duese)

#include "move.h"
#include "parser.h"
#include "pins.h"

typedef enum {
	HOMING_FINISHED = 0x00,
	HOMING_ERROR = 0x01,
	HOMING_BUSY = 0x02,
	HOMING_TIMEOUT = 0x03,
	HOMING_TRY = 0x04
} HOMING_StatusTypeDef;

bool homingActive;
uint32_t Dist, xDist, yDist;
bool xFlag = false, yFlag = false;

uint8_t phase = 0;
uint32_t timeout = 0xffffffff;
uint8_t counterTry = 0;

void enableSensors() {
	homingActive = true;
	xFlag = yFlag = false;
	HAL_GPIO_WritePin(FAN1_PORT, FAN1_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(FAN2_PORT, FAN2_PIN, GPIO_PIN_SET);
	HAL_NVIC_EnableIRQ(X_STOP_EXTI);
	HAL_NVIC_EnableIRQ(Y_STOP_EXTI);
}
void disableSensors() {
	homingActive = false;
	HAL_GPIO_WritePin(FAN1_PORT, FAN1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(FAN2_PORT, FAN2_PIN, GPIO_PIN_RESET);
	HAL_NVIC_DisableIRQ(X_STOP_EXTI);
	HAL_NVIC_DisableIRQ(Y_STOP_EXTI);
}
void stopMotors(Robot *rob) {
	rob->motorMaster.motorX.stopTimer();
	rob->motorMaster.motorY.stopTimer();
	rob->motorMaster.motorX.stepBuf.consumerClear();
	rob->motorMaster.motorY.stepBuf.consumerClear();
	rob->motorMaster.moveBuf.consumerClear();
	//rob->motorMaster.resetCalc();
	rob->resetPos();
}

void adjustDistWithMotorRatio() {	//TODO überprüfen
	float_t motorRatio = MOTOR_XY_RATIO * 0.01f;
	if (motorRatio > 1.0f) {
		motorRatio = 2.0f - motorRatio;
		xDist = xDist * motorRatio;
	} else {
		yDist = yDist * motorRatio;
	}
}

HOMING_StatusTypeDef home(Robot *rob) {

	if (HAL_GetTick() - timeout > MAX_HOMING_TIMEOUT)
		return HOMING_TIMEOUT;
	else if (counterTry > MAX_HOMING_TRY)
		return HOMING_TRY;

	switch (phase) {
	case 0:	//Vorbereiten
		disableSensors();
		phase = 1;
		break;
	case 1:	//Y abtasten (Rotation)
		if (!rob->motorMaster.motorX.timerActiveFlag) {
			Dist = xDist = yDist = 0;
			enableSensors();
			rob->moveLin(-MAX_HOMING_DIST, HOMING_SPEED);
			timeout = HAL_GetTick();
			phase = 2;
		}
		break;
	case 2:	//Y abgetastet (Rotation)
		if (xFlag && yFlag) {
			timeout = 0xffffffff;
			stopMotors(rob);
			disableSensors();

			//Distanzen an Radgrößen anpassen
			adjustDistWithMotorRatio();

			//Fehler berechnen
			float_t errorDistance = (fmax(xDist, yDist) - fmin(xDist, yDist))
					* MM_PER_STEP;
			float_t error = atan(errorDistance / SENSOR_DIST);
			error = (error * 180) / M_PI;	//Rad zu Deg

			//Fehler anpassen
			if (error > HOMING_MAX_FAULT) { //erneut Abtasten
				rob->moveLin(errorDistance, HOMING_SPEED);
				(xDist > yDist) ?
						rob->moveRot(error, HOMING_SPEED) :
						rob->moveRot(-error, HOMING_SPEED);
				counterTry++;
				phase = 1;
			} else { //fortfahren
				rob->moveLin(errorDistance, HOMING_SPEED);
				(xDist > yDist) ?
						rob->moveRot(error, HOMING_SPEED) :
						rob->moveRot(-error, HOMING_SPEED);
				rob->moveLin(HOMING_OFFSET_Y + 50, HOMING_SPEED);
				phase = 3;
				counterTry = 0;
			}
		}
		break;
	case 3: //Y nullen
		if (!rob->motorMaster.motorX.timerActiveFlag) {
			Dist = xDist = yDist = 0;
			enableSensors();
			rob->moveLin(-MAX_HOMING_DIST, HOMING_SPEED);
			timeout = HAL_GetTick();
			phase = 4;
		}
		break;
	case 4: //Y genullt
		if (xFlag || yFlag) {
			timeout = 0xffffffff;
			stopMotors(rob);
			disableSensors();

			rob->moveLin(HOMING_OFFSET_Y + 50, HOMING_SPEED);
			phase = 5;
		}
		break;
	case 5: //X vorbereiten
		rob->moveRot(-90, HOMING_SPEED);
		phase = 6;
		break;
	case 6:	//X nullen
		if (!rob->motorMaster.motorX.timerActiveFlag) {
			Dist = xDist = yDist = 0;
			enableSensors();
			rob->moveLin(-MAX_HOMING_DIST, HOMING_SPEED);
			timeout = HAL_GetTick();
			phase = 7;
		}
		break;
	case 7:	//X genullt
		if (xFlag || yFlag) {
			timeout = 0xffffffff;
			stopMotors(rob);
			disableSensors();

			rob->moveLin(HOMING_OFFSET_Y + 50, HOMING_SPEED);
			phase = 8;
		}
		break;
	case 8:	//Position anpassen und beenden
		rob->setPos(0, 0, 0);
		phase = 0;
		return HOMING_FINISHED;
		break;
	}
	return HOMING_BUSY;

}
#endif /* INC_HOMING_H_ */
