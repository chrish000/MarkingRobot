/* USER CODE .hGIN Header */
/**
 ******************************************************************************
 * @file           : TMC2209.cpp
 * @brief          : TMC2209 functions
 ******************************************************************************
 * @attention
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

//TODO CoolStep???
/* Includes ------------------------------------------------------------------*/
#include "TMC2209.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/

void TMC2209::setup() {

}

void TMC2209::enable() {
	HAL_GPIO_WritePin(this.HardwareEnablePort, this.HardwareEnablePin,
			GPIO_PIN_RESET);
}

void TMC2209::disable() {
	HAL_GPIO_WritePin(this.HardwareEnablePort, this.HardwareEnablePin,
			GPIO_PIN_SET);
}

void TMC2209::setMicrosteps(uint16_t microstepsPerStep) {

}

uint8_t TMC2209::percentToCurrentSetting(uint8_t percent) {
	uint8_t constrained_percent = constrain(percent, PERCENT_MIN, PERCENT_MAX);
	uint8_t current_setting = map(constrained_percent, PERCENT_MIN, PERCENT_MAX,
			CURRENT_SETTING_MIN, CURRENT_SETTING_MAX);
	return current_setting;
}

uint8_t TMC2209::currentSettingToPercent(uint8_t current_setting) {
	uint8_t percent = map(current_setting, CURRENT_SETTING_MIN,
			CURRENT_SETTING_MAX, PERCENT_MIN, PERCENT_MAX);
	return percent;
}

void TMC2209::setRunCurrent(uint8_t percentage) {

}

void TMC2209::setHoldCurrent(uint8_t percentage) {

}

void TMC2209::setHoldDelay(uint8_t percent) {

}

void TMC2209::doubleEdge(uint8_t state) {

}

void TMC2209::inverseMotorDirection(uint8_t state) {

}

void TMC2209::setStandstillMode(uint8_t state) {

}

void TMC2209::AutomaticCurrentScaling(uint8_t state) {

}

void TMC2209::moveAtVelocity(int32_t microstepsPerPeriod) {

}

void TMC2209::moveUsingStepDirInterface() {

}

void TMC2209::stealthChop(uint8_t state) {

}

uint8_t TMC2209::getInterfaceTransmissionCounter() {
	return read(ADDRESS_IFCNT);
}

uint16_t TMC2209::getMicrostepCounter() {
	return read(ADDRESS_MSCNT);
}
