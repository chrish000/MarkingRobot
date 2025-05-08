/**
 ******************************************************************************
 * @file           : move.c
 * @brief          : Befehlsberechnung
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
#include "utils.h"
#include "move.h"
#include <algorithm>

#define DEG_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEG (180.0 / M_PI)

void Robot::resetPos() {
	orientation = posX = posY = 0;
	totalDistSinceHoming = 0;
}
void Robot::setPos(float_t newOrientation, float_t newX, float_t newY) {
	orientation = newOrientation;
	posX = newX;
	posY = newY;
}

float_t Robot::getPosX() {
	return posX;
}
float_t Robot::getPosY() {
	return posY;
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
	float_t target = atan2(newY - oldY, newX - oldX) * RAD_TO_DEG; // Zielwinkel berechnen
	target = fmod(target + 360, 360.0);	// Zielwinkel normalisieren auf [0, 360]
	oldOrientation = fmod(oldOrientation + 360, 360.0); // aktuelle Orientierung normalisieren auf [0, 360]
	float_t turn = target - oldOrientation;				// Drehwinkel berechnen
	turn = fmod(turn + 360, 360.0);		// Drehwinkel normalisieren auf [0, 360]
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
	return sqrt(pow(newX - oldX, 2) + pow(newY - oldY, 2));
}

MotorManager::position calcNewPos(float_t distance, float_t orientation,
		float_t oldX, float_t oldY) {
	MotorManager::position newPos;
	newPos.x = oldX + distance * cos(DEG_TO_RAD * DEG_TO_RAD);
	newPos.y = oldY + distance * sin(DEG_TO_RAD * DEG_TO_RAD);
	newPos.orient = orientation;
	return newPos;
}

/* Robot ---------------------------------------------------------------------*/
/**
 * @brief Funktion zur Initialisierun des Roboters
 * @param None
 * @retval None
 */
void Robot::init() {
	HAL_NVIC_DisableIRQ(X_STOP_EXTI);
	HAL_NVIC_DisableIRQ(Y_STOP_EXTI);

	motorMaster.moveBuf.consumerClear();
	motorMaster.motorX.init();
	motorMaster.motorY.init();
	motorMaster.motorX.tmc.setup();
	motorMaster.motorX.tmc.setMicrostepsPerStep(MICROSTEPS);
	motorMaster.motorY.tmc.setup();
	motorMaster.motorY.tmc.setMicrostepsPerStep(MICROSTEPS);

	printhead.init();

	HAL_GPIO_WritePin(FAN0_PORT, FAN0_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(FAN1_PORT, FAN1_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(FAN2_PORT, FAN2_PIN, GPIO_PIN_SET);

	HAL_ADCEx_Calibration_Start(ADC_Handle, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED); // ADC Kalibrieren
	HAL_ADC_Start_DMA(ADC_Handle, (uint32_t*) &ADC_BatteryVoltage, 1); // ADC starten
	HAL_TIM_Base_Start(ADC_TIM);			// Timer für ADC ausloesung starten

	Buzzer_InitTypeDef buzzerConfig;
	buzzerConfig.timer = pins.TIM_BUZZER;
	buzzerConfig.timerClockFreqHz = HAL_RCC_GetPCLK2Freq(); // NOTE: this should be freq of timer, not frequency of peripheral clock
	Buzzer_Init(&hbuzzer, &buzzerConfig);
	Buzzer_Start(&hbuzzer);
}

/**
 * @brief Bewegung des Roboters zu einer Zielposition
 * @param newX Zielposition X-Koordinate
 * @param newY Zielposition Y-Koordinate
 * @param newSpeed Geschwindigkeit in mm/s
 * @param newAccel Beschleunigung in mm/s^2
 * @retval None
 */
bool Robot::moveToPos(MoveParams param) {
	float_t newX = param.x.value_or(posX);
	float_t newY = param.y.value_or(posY);
	float_t newSpeed = param.speed.value_or(speed);
	float_t newAccel = param.accel.value_or(accel);
	bool printing = (bool) param.printing.value_or(false);

	if (!(newX == posX && newY == posY)) {
		speed = std::clamp(newSpeed, 0.0f, (float_t) MAX_SPEED); //Neue Geschwindigkeit speichern
		accel = std::clamp(newAccel, 0.0f, (float_t) MAX_ACCEL);
		float_t turn = calcTurn(newX, newY, posX, posY, orientation);
		if (turn != 0) {
			if (motorMaster.moveBuf.writeAvailable() >= 2) {
				if (moveRot(turn, DEFAULT_SPEED, accel) == false)
					return false;
				if (moveLin(calcDistance(newX, newY, posX, posY), speed, accel,
						printing) == false)
					return false;
				posX = newX;
				posY = newY;
				return true;
			} else
				return false;
		} else {
			if (motorMaster.moveBuf.writeAvailable() != 0) {
				if (moveLin(calcDistance(newX, newY, posX, posY), speed, accel,
						printing) == false)
					return false;
				posX = newX;
				posY = newY;
				return true;
			} else
				return false;
		}
	}
	return false;
}

/**
 * @brief Lineare Bewegung des Roboters
 * @param distance Distanz in mm
 * @param speed Geschwindigkeit in mm/s
 * @param accel Beschleunigung in mm/s^2
 * @retval None
 */
bool Robot::moveLin(float_t distance, float_t speed, float_t accel,
		bool printing) {
	if (distance == 0) //Prüfen ob Distanz 0 ist
		return false;
	uint32_t steps = fabs(distance * STEPS_PER_MM); //Schritte berechnen
	bool direction = (distance > 0) ? 1 : 0; //Richtung bestimmen
#ifdef REVERSE_BOTH_MOTOR_DIRECTION
	direction = !direction;
#endif
	MotorManager::moveCommands cmd;
	cmd.speed = speed * STEPS_PER_MM;
	cmd.accel = accel * STEPS_PER_MM;
	cmd.stepDistance = steps;
	if (Inverse_Motor_X_Dir)
		cmd.directionX = (direction ? Direction::Reverse : Direction::Forward);
	else
		cmd.directionX = (direction ? Direction::Forward : Direction::Reverse);

	if (Inverse_Motor_Y_Dir)
		cmd.directionY = (!direction ? Direction::Reverse : Direction::Forward);
	else
		cmd.directionY = (!direction ? Direction::Forward : Direction::Reverse);
	cmd.printigMove = printing;

	totalDistSinceHoming += distance;
	motorMaster.posBuf.insert(calcNewPos(distance, orientation, posX, posY));
	return motorMaster.moveBuf.insert(cmd);
}

/**
 * @brief Rotationsbewegung des Roboters
 * @param degrees Drehwinkel in Grad
 * @param speed Geschwindigkeit in mm/s
 * @param accel Beschleunigung in mm/s^2
 * @retval None
 */
bool Robot::moveRot(float_t degrees, float_t speed, float_t accel) {
	if (degrees == 0) //Prüfen ob Distanz 0 ist
		return false;
	uint32_t steps = fabs(degrees * STEPS_PER_DEG); //Schritte berechnen
	bool direction = (degrees > 0) ? 1 : 0; //Richtung bestimmen
#ifdef REVERSE_BOTH_MOTOR_DIRECTION
	direction = !direction;
#endif
	MotorManager::moveCommands cmd;
	cmd.speed = speed * STEPS_PER_MM;
	cmd.accel = accel * STEPS_PER_MM;
	cmd.stepDistance = steps;
	if (Inverse_Motor_X_Dir)
		cmd.directionX = (direction ? Direction::Reverse : Direction::Forward);
	else
		cmd.directionX = (direction ? Direction::Forward : Direction::Reverse);

	if (Inverse_Motor_Y_Dir)
		cmd.directionY = (direction ? Direction::Reverse : Direction::Forward);
	else
		cmd.directionY = (direction ? Direction::Forward : Direction::Reverse);
	cmd.printigMove = false;

	orientation += degrees;
	orientation = fmod(orientation + 360, 360.0); //Drehwinkel normalisieren auf [0, 360]

	motorMaster.posBuf.insert( { posX, posY, orientation });
	return motorMaster.moveBuf.insert(cmd);
}

bool Robot::moveToHome() {
	if (posX == 0 && posY == 0 && orientation == 0) //steht bereits auf Position
		return true;

	if (motorMaster.moveBuf.writeAvailable() < 5) //Nicht genug Platz im Bewegungspuffer
		return false;

	MoveParams param;
	param.x = 500;
	param.y = 500;
	param.speed = DEFAULT_SPEED;
	param.accel = DEFAULT_ACCEL;
	param.printing = false;

	bool status = true;
	status &= moveToPos(param);
	status &= moveRot(-(orientation - 45));
	status &= moveLin(-707.107);
	status &= moveRot(-45);

	if (status == true)
		setPos(0, 0, 0);

	return status;
}
