
#ifndef HOMING_H
#define HOMING_H

#include "move.h"
#include "parser.h"
#include "pins.h"
#include "config.h"

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
uint32_t timeout = 0;
bool timeoutActive = false;
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
	//Motoren stoppen
	rob->motorMaster.motorX.stopTimer();
	rob->motorMaster.motorY.stopTimer();
	//Alle Puffer leeren
	rob->motorMaster.moveBuf.consumerClear();
	rob->motorMaster.motorX.stepBuf.consumerClear();
	rob->motorMaster.motorY.stepBuf.consumerClear();
	rob->motorMaster.resetCalc();
	//rob->resetPos();
}

bool movementFinished(Robot *rob) {
	bool status = true;
	//status &= !rob->motorMaster.motorX.timerActiveFlag;
	status &= rob->motorMaster.moveBuf.isEmpty();
	status &= rob->motorMaster.motorX.stepBuf.isEmpty();
	return status;
}

void adjustDistWithMotorRatio() {
	float_t motorRatio = MOTOR_XY_RATIO * 0.01f;
	if (motorRatio > 1.0f) {
		motorRatio = 2.0f - motorRatio;
		xDist = xDist * motorRatio;
	} else {
		yDist = yDist * motorRatio;
	}
}

HOMING_StatusTypeDef home(Robot *rob) {

	if (timeoutActive)
		if (HAL_GetTick() - timeout > MAX_HOMING_TIMEOUT)
			return HOMING_TIMEOUT;
	if (counterTry > MAX_HOMING_TRY)
		return HOMING_TRY;

	switch (phase) {
	case 0:	//Vorbereiten
		disableSensors();
#ifdef MOVE_TO_HOME_BEFORE_HOMING
		rob->moveToHome();
#endif
		rob->moveRot(90, DEFAULT_SPEED, DEFAULT_ACCEL);
		phase = 1;
		break;
	case 1:	//Y abtasten (Rotation)
		if (movementFinished(rob)) {
			Dist = xDist = yDist = 0;
			enableSensors();
			rob->moveLin(-MAX_HOMING_DIST, HOMING_SPEED_PROBING, HOMING_ACCEL);
			timeout = HAL_GetTick();
			timeoutActive = true;
			phase = 2;
		}
		break;
	case 2:	//Y abgetastet (Rotation)
		if (xFlag && yFlag) {
			timeoutActive = false;
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
			rob->moveLin(errorDistance + DIST_BETWEEN_PROBING, HOMING_SPEED_PROBING,
			HOMING_ACCEL);
			(xDist < yDist) ?
					rob->moveRot(error, HOMING_SPEED_PROBING, HOMING_ACCEL) :
					rob->moveRot(-error, HOMING_SPEED_PROBING, HOMING_ACCEL);

			if (error > HOMING_MAX_FAULT) { //erneut Abtasten
				counterTry++;
				phase = 1;
			} else { //fortfahren
				phase = 3;
				counterTry = 0;
			}
		}
		break;
	case 3: //Y nullen
		if (movementFinished(rob)) {
			timeoutActive = false;
			Dist = xDist = yDist = 0;
			enableSensors();
			rob->moveLin(-MAX_HOMING_DIST, HOMING_SPEED_PROBING, HOMING_ACCEL);
			timeout = HAL_GetTick();
			timeoutActive = true;
			phase = 4;
		}
		break;
	case 4: //Y genullt
		if (xFlag || yFlag) {
			timeoutActive = false;
			stopMotors(rob);
			disableSensors();

			rob->moveLin(HOMING_OFFSET_Y - HOMING_OFFSET_X + 70, HOMING_SPPED_MOVING, HOMING_ACCEL);
			phase = 5;
		}
		break;
	case 5: //X vorbereiten
		rob->moveRot(-90, HOMING_SPPED_MOVING);
		phase = 6;
		break;
	case 6:	//X nullen
		if (movementFinished(rob)) {
			timeoutActive = false;
			Dist = xDist = yDist = 0;
			enableSensors();
			rob->moveLin(-MAX_HOMING_DIST, HOMING_SPEED_PROBING, HOMING_ACCEL);
			timeout = HAL_GetTick();
			timeoutActive = true;
			phase = 7;
		}
		break;
	case 7:	//X genullt
		if (xFlag || yFlag) {
			timeoutActive = false;
			stopMotors(rob);
			disableSensors();

			rob->moveLin(HOMING_OFFSET_Y - HOMING_OFFSET_X + 70, HOMING_SPPED_MOVING, HOMING_ACCEL);
			phase = 8;
		}
		break;
	case 8:	//Position anpassen und beenden
		rob->setPos(0, 0, 0);
		phase = 9;
		break;
	case 9:
		phase = 0;
		return HOMING_FINISHED;
		break;
	}
	return HOMING_BUSY;

}
#endif /* HOMING_H */
