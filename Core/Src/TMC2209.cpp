/******************************************************************************
 * @file           : TMC2209.cpp
 * @brief          : TMC2209 class and functions for communicating via UART.
 * @author		   : Chris Hauser
 ******************************************************************************
 * @attention
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 * This TMC2209 Library is meant to be used with a STM32H7.
 *
 ******************************************************************************
 */
#include "TMC2209.h"
#include "utils.h"
#include <cstring>

/**
 * @brief  Constructor for the TMC2209 class, initializes default settings.
 */
TMC2209::TMC2209() {
	cool_step_enabled_ = false;
	data_received_flag = false;
	TMC2209_status = TMC_OK;
}

/**
 * @brief  Array that contains pre-calculated CRC-Values for the initialization-process
 */
constexpr uint8_t TMC2209::precomputedCRC[16];

/**
 * @brief  Configures the TMC2209 communication interface.
 * @param  None
 * @retval None
 */
void TMC2209::setup() {
	HAL_HalfDuplex_EnableReceiver(UART_address);
	HAL_UART_Receive_DMA(UART_address, rxBufferRaw,
			WRITE_READ_REPLY_DATAGRAM_SIZE);
	initialize();
}

/* Unidirectional methods ----------------------------------------------------*/

/**
 * @brief  Enables the TMC2209 driver by setting the enable pin and configuring the chopper settings.
 * @param  None
 * @retval None
 */
void TMC2209::enable() {
	HAL_GPIO_WritePin(hardware_enable_port, hardware_enable_pin,
			GPIO_PIN_RESET); //Treiber aktivieren
	chopper_config_.toff = toff_;
	writeStoredChopperConfig();
}

/**
 * @brief  Disables the TMC2209 driver by setting the enable pin and disabling the chopper.
 * @param  None
 * @retval None
 */
void TMC2209::disable() {

	HAL_GPIO_WritePin(hardware_enable_port, hardware_enable_pin, GPIO_PIN_SET); //Treiber deaktivieren
	chopper_config_.toff = TOFF_DISABLE;
	writeStoredChopperConfig();
}

/**
 * @brief  Sets the number of microsteps per full step for the TMC2209 driver.
 * @param  microsteps_per_step: Desired microstep resolution. (1, 2, 4, 8...256)
 * @retval None
 */
void TMC2209::setMicrostepsPerStep(uint16_t microsteps_per_step) {
	switch (microsteps_per_step) {
	case 1:
		chopper_config_.mres = MRES_001;
		break;
	case 2:
		chopper_config_.mres = MRES_002;
		break;
	case 4:
		chopper_config_.mres = MRES_004;
		break;
	case 8:
		chopper_config_.mres = MRES_008;
		break;
	case 16:
		chopper_config_.mres = MRES_016;
		break;
	case 32:
		chopper_config_.mres = MRES_032;
		break;
	case 64:
		chopper_config_.mres = MRES_064;
		break;
	case 128:
		chopper_config_.mres = MRES_128;
		break;
	case 256:
		chopper_config_.mres = MRES_256;
		break;
	}
	writeStoredChopperConfig();
}

/**
 * @brief  Configures the run current for the TMC2209 driver.
 * @param  runCurrent: Desired run current value in mA, constrained within valid limits.
 * @retval None
 */

void TMC2209::setRunCurrent(uint16_t runCurrent) {
	runCurrent = constrain(runCurrent, CURRENT_SETTING_MIN,
			CURRENT_SETTING_MAX);
	driver_current_.irun = map(runCurrent, CURRENT_SETTING_MIN,
			CURRENT_SETTING_MAX, RUN_CURRENT_SETTING_MIN,
			RUN_CURRENT_SETTING_MAX);
	writeStoredDriverCurrent();
}

/**
 * @brief  Configures the hold current for the TMC2209 driver.
 * @param  holdCurrent: Desired hold current value in mA, constrained within valid limits.
 * @retval None
 */

void TMC2209::setHoldCurrent(uint16_t holdCurrent) {
	holdCurrent = constrain(holdCurrent, CURRENT_SETTING_MIN,
			CURRENT_SETTING_MAX);
	driver_current_.ihold = map(holdCurrent, CURRENT_SETTING_MIN,
			CURRENT_SETTING_MAX, RUN_CURRENT_SETTING_MIN,
			RUN_CURRENT_SETTING_MAX);
	writeStoredDriverCurrent();
}

/**
 * @brief  Sets the hold delay percentage for the TMC2209 driver.
 * @param  holdDelayPercent: Percentage of hold delay, constrained within valid limits.
 * @retval None
 */

void TMC2209::setHoldDelay(uint8_t holdDelayPercent) {
	holdDelayPercent = constrain(holdDelayPercent, PERCENT_MIN, PERCENT_MAX);
	driver_current_.iholddelay = map(holdDelayPercent, PERCENT_MIN, PERCENT_MAX,
			HOLD_DELAY_MIN, HOLD_DELAY_MAX);
	writeStoredDriverCurrent();
}

/**
 * @brief  Sets all current-related values for the TMC2209 driver.
 * @param  runCurrent: Running current, constrained within valid limits.
 * @param  holdCurrent: Holding current, constrained within valid limits.
 * @param  holdDelayPercent: Hold delay percentage, constrained within valid limits.
 * @retval None
 */
void TMC2209::setAllCurrentValues(uint16_t runCurrent, uint16_t holdCurrent,
		uint8_t holdDelayPercent) {
	runCurrent = constrain(runCurrent, CURRENT_SETTING_MIN,
			CURRENT_SETTING_MAX);
	driver_current_.irun = map(runCurrent, CURRENT_SETTING_MIN,
			CURRENT_SETTING_MAX, RUN_CURRENT_SETTING_MIN,
			RUN_CURRENT_SETTING_MAX);
	holdCurrent = constrain(holdCurrent, CURRENT_SETTING_MIN,
			CURRENT_SETTING_MAX);
	driver_current_.ihold = map(holdCurrent, CURRENT_SETTING_MIN,
			CURRENT_SETTING_MAX, RUN_CURRENT_SETTING_MIN,
			RUN_CURRENT_SETTING_MAX);
	holdDelayPercent = constrain(holdDelayPercent, PERCENT_MIN, PERCENT_MAX);
	driver_current_.iholddelay = map(holdDelayPercent, PERCENT_MIN, PERCENT_MAX,
			HOLD_DELAY_MIN, HOLD_DELAY_MAX);
	writeStoredDriverCurrent();
}

/**
 * @brief  Enables double-edge step pulses for the TMC2209 driver.
 * @param  None
 * @retval None
 */
void TMC2209::enableDoubleEdge() {
	chopper_config_.double_edge = DOUBLE_EDGE_ENABLE;
	writeStoredChopperConfig();
}

/**
 * @brief  Disables double-edge step pulses for the TMC2209 driver.
 * @param  None
 * @retval None
 */
void TMC2209::disableDoubleEdge() {
	chopper_config_.double_edge = DOUBLE_EDGE_DISABLE;
	writeStoredChopperConfig();
}

/**
 * @brief  Enables inverse motor direction by setting the shaft bit in the global configuration.
 * @param  None
 * @retval None
 */
void TMC2209::enableInverseMotorDirection() {
	global_config_.shaft = 1;
	writeStoredGlobalConfig();
}

/**
 * @brief  Disables inverse motor direction by clearing the shaft bit in the global configuration.
 * @param  None
 * @retval None
 */
void TMC2209::disableInverseMotorDirection() {
	global_config_.shaft = 0;
	writeStoredGlobalConfig();
}

/**
 * @brief  Sets the standstill mode by configuring the freewheel bit in the PWM configuration.
 * @param  mode: The desired standstill mode (e.g., freewheel or brake).
 * @retval None
 */
void TMC2209::setStandstillMode(TMC2209::StandstillMode mode) {
	pwm_config_.freewheel = mode;
	writeStoredPwmConfig();
}

/**
 * @brief  Enables automatic current scaling by setting the corresponding bit in the PWM configuration.
 * @param  None
 * @retval None
 */
void TMC2209::enableAutomaticCurrentScaling() {
	pwm_config_.pwm_autoscale = STEPPER_DRIVER_FEATURE_ON;
	writeStoredPwmConfig();
}

/**
 * @brief  Disables automatic current scaling by clearing the corresponding bit in the PWM configuration.
 * @param  None
 * @retval None
 */
void TMC2209::disableAutomaticCurrentScaling() {
	pwm_config_.pwm_autoscale = STEPPER_DRIVER_FEATURE_OFF;
	writeStoredPwmConfig();
}

/**
 * @brief  Enables automatic gradient adaptation by setting the corresponding bit in the PWM configuration.
 * @param  None
 * @retval None
 */
void TMC2209::enableAutomaticGradientAdaptation() {
	pwm_config_.pwm_autograd = STEPPER_DRIVER_FEATURE_ON;
	writeStoredPwmConfig();
}

/**
 * @brief  Disables automatic gradient adaptation by clearing the corresponding bit in the PWM configuration.
 * @param  None
 * @retval None
 */
void TMC2209::disableAutomaticGradientAdaptation() {
	pwm_config_.pwm_autograd = STEPPER_DRIVER_FEATURE_OFF;
	writeStoredPwmConfig();
}

/**
 * @brief  Sets the PWM offset by configuring the PWM amplitude in the PWM configuration.
 * @param  pwm_amplitude: The desired PWM amplitude value.
 * @retval None
 */
void TMC2209::setPwmOffset(uint8_t pwm_amplitude) {
	pwm_config_.pwm_offset = pwm_amplitude;
	writeStoredPwmConfig();
}

/**
 * @brief  Sets the PWM gradient by configuring the corresponding bit in the PWM configuration.
 * @param  pwm_amplitude: The desired PWM gradient value.
 * @retval None
 */
void TMC2209::setPwmGradient(uint8_t pwm_amplitude) {
	pwm_config_.pwm_grad = pwm_amplitude;
	writeStoredPwmConfig();
}

/**
 * @brief  Sets the power-down delay by writing the specified value to the TPOWERDOWN register.
 * @param  power_down_delay: The desired power-down delay value.
 * @retval None
 */
void TMC2209::setPowerDownDelay(uint8_t power_down_delay) {
	write(ADDRESS_TPOWERDOWN, power_down_delay);
}

/**
 * @brief  Sets the reply delay by writing the specified value to the REPLYDELAY register.
 * @param  reply_delay: The desired reply delay value (0 to REPLY_DELAY_MAX).
 * @retval None
 */
void TMC2209::setReplyDelay(uint8_t reply_delay) {
	if (reply_delay > REPLY_DELAY_MAX) {
		reply_delay = REPLY_DELAY_MAX;
	}
	ReplyDelay reply_delay_data;
	reply_delay_data.bytes = 0;
	reply_delay_data.replydelay = reply_delay;
	write(ADDRESS_REPLYDELAY, reply_delay_data.bytes);
}

/**
 * @brief  Moves the motor at a specified velocity by writing the value to the VACTUAL register.
 * @param  microsteps_per_period: The number of microsteps per period (velocity).
 * @retval None
 */
void TMC2209::moveAtVelocity(int32_t microsteps_per_period) {
	write(ADDRESS_VACTUAL, microsteps_per_period);
}

/**
 * @brief  Configures the motor to move using the Step/Dir interface by writing to the VACTUAL register.
 * @param  None
 * @retval None
 */
void TMC2209::moveUsingStepDirInterface() {
	write(ADDRESS_VACTUAL, VACTUAL_STEP_DIR_INTERFACE);
}

/**
 * @brief  Enables StealthChop mode by configuring the global configuration register.
 * @param  None
 * @retval None
 */
void TMC2209::enableStealthChop() {
	global_config_.enable_spread_cycle = 0;
	writeStoredGlobalConfig();
}

/**
 * @brief  Disables StealthChop mode by setting the global configuration to enable SpreadCycle.
 * @param  None
 * @retval None
 */
void TMC2209::disableStealthChop() {
	global_config_.enable_spread_cycle = 1;
	writeStoredGlobalConfig();
}

/**
 * @brief  Sets the CoolStep duration threshold for the driver.
 * @param  duration_threshold: The threshold value to set for CoolStep duration.
 * @retval None
 */
void TMC2209::setCoolStepDurationThreshold(uint32_t duration_threshold) {
	write(ADDRESS_TCOOLTHRS, duration_threshold);
}

/**
 * @brief  Sets the StealthChop duration threshold for the driver.
 * @param  duration_threshold: The threshold value to set for StealthChop duration.
 * @retval None
 */
void TMC2209::setStealthChopDurationThreshold(uint32_t duration_threshold) {
	write(ADDRESS_TPWMTHRS, duration_threshold);
}

/**
 * @brief  Sets the StallGuard threshold for the driver.
 * @param  stall_guard_threshold: The threshold value to set for StallGuard.
 * @retval None
 */
void TMC2209::setStallGuardThreshold(uint8_t stall_guard_threshold) {
	write(ADDRESS_SGTHRS, stall_guard_threshold);
}

/**
 * @brief  Enables the CoolStep feature with specified lower and upper threshold values.
 * @param  lower_threshold: The lower threshold for CoolStep.
 * @param  upper_threshold: The upper threshold for CoolStep.
 * @retval None
 */
void TMC2209::enableCoolStep(uint8_t lower_threshold, uint8_t upper_threshold) {
	cool_config_.semin = constrain(lower_threshold, SEMIN_MIN, SEMIN_MAX);
	cool_config_.semax = constrain(upper_threshold, SEMAX_MIN, SEMAX_MAX);
	write(ADDRESS_COOLCONF, cool_config_.bytes);
	cool_step_enabled_ = true;
}

/**
 * @brief  Disables the CoolStep feature.
 * @param  None
 * @retval None
 */
void TMC2209::disableCoolStep() {
	cool_config_.semin = SEMIN_OFF;
	write(ADDRESS_COOLCONF, cool_config_.bytes);
	cool_step_enabled_ = false;
}

/**
 * @brief  Sets the CoolStep current increment value.
 * @param  current_increment: The desired current increment setting.
 * @retval None
 */
void TMC2209::setCoolStepCurrentIncrement(CurrentIncrement current_increment) {
	cool_config_.seup = current_increment;
	write(ADDRESS_COOLCONF, cool_config_.bytes);
}

/**
 * @brief  Sets the CoolStep measurement count value.
 * @param  measurement_count: The desired measurement count setting.
 * @retval None
 */
void TMC2209::setCoolStepMeasurementCount(MeasurementCount measurement_count) {
	cool_config_.sedn = measurement_count;
	write(ADDRESS_COOLCONF, cool_config_.bytes);
}

/**
 * @brief  Enables analog current scaling for the driver.
 * @param  None
 * @retval None
 */
void TMC2209::enableAnalogCurrentScaling() {
	global_config_.i_scale_analog = 1;
	writeStoredGlobalConfig();
}

/**
 * @brief  Disables analog current scaling for the driver.
 * @param  None
 * @retval None
 */
void TMC2209::disableAnalogCurrentScaling() {
	global_config_.i_scale_analog = 0;
	writeStoredGlobalConfig();
}

/**
 * @brief  Configures the driver to use external sense resistors for current sensing.
 * @param  None
 * @retval None
 */
void TMC2209::useExternalSenseResistors() {
	global_config_.internal_rsense = 0;
	writeStoredGlobalConfig();
}

/**
 * @brief  Configures the driver to use internal sense resistors for current sensing.
 * @param  None
 * @retval None
 */
void TMC2209::useInternalSenseResistors() {
	global_config_.internal_rsense = 1;
	writeStoredGlobalConfig();
}
/* Bidirectional methods -----------------------------------------------------*/

/**
 * @brief  Reads the version of the TMC2209 driver.
 * @param  None
 * @retval uint8_t The version of the driver.
 */
uint8_t TMC2209::getVersion() {
	Input input;
	input.bytes = read(ADDRESS_IOIN);

	return input.version;
}

/**
 * @brief  Checks if the TMC2209 driver is communicating correctly.
 * @param  None
 * @retval bool True if communication is successful, false otherwise.
 */
bool TMC2209::isCommunicating() {
	return (getVersion() == VERSION);
}

/**
 * @brief  Checks if the TMC2209 is both set up and communicating correctly.
 * @param  None
 * @retval bool True if the driver is set up and communication is successful, false otherwise.
 */
bool TMC2209::isSetupAndCommunicating() {
	return serialOperationMode();
}

/**
 * @brief  Checks if the TMC2209 is communicating but not yet fully set up.
 * @param  None
 * @retval bool True if the driver is communicating but not set up, false otherwise.
 */
bool TMC2209::isCommunicatingButNotSetup() {
	return (isCommunicating() && (not isSetupAndCommunicating()));
}

/**
 * @brief  Checks if the hardware of the TMC2209 is disabled.
 * @param  None
 * @retval bool True if the hardware is disabled, false otherwise.
 */
bool TMC2209::hardwareDisabled() {
	Input input;
	input.bytes = read(ADDRESS_IOIN);

	return input.enn;
}

/**
 * @brief  Retrieves the number of microsteps per step based on the current chopper configuration.
 * @param  None
 * @retval uint16_t The number of microsteps per step.
 */
uint16_t TMC2209::getMicrostepsPerStep() {
	uint16_t microsteps_per_step_exponent;
	switch (chopper_config_.mres) {
	case MRES_001: {
		microsteps_per_step_exponent = 0;
		break;
	}
	case MRES_002: {
		microsteps_per_step_exponent = 1;
		break;
	}
	case MRES_004: {
		microsteps_per_step_exponent = 2;
		break;
	}
	case MRES_008: {
		microsteps_per_step_exponent = 3;
		break;
	}
	case MRES_016: {
		microsteps_per_step_exponent = 4;
		break;
	}
	case MRES_032: {
		microsteps_per_step_exponent = 5;
		break;
	}
	case MRES_064: {
		microsteps_per_step_exponent = 6;
		break;
	}
	case MRES_128: {
		microsteps_per_step_exponent = 7;
		break;
	}
	case MRES_256:
	default: {
		microsteps_per_step_exponent = 8;
		break;
	}
	}
	return 1 << microsteps_per_step_exponent;
}

/**
 * @brief  Retrieves the current settings of the TMC2209 driver.
 * @param  None
 * @retval TMC2209::Settings The current settings of the TMC2209 driver.
 */
TMC2209::Settings TMC2209::getSettings() {
	Settings settings;
	settings.is_communicating = isCommunicating();

	if (settings.is_communicating) {
		readAndStoreRegisters();

		settings.is_setup = global_config_.pdn_disable;
		settings.software_enabled = (chopper_config_.toff > TOFF_DISABLE);
		settings.microsteps_per_step = getMicrostepsPerStep();
		settings.inverse_motor_direction_enabled = global_config_.shaft;
		settings.stealth_chop_enabled = not global_config_.enable_spread_cycle;
		settings.standstill_mode = pwm_config_.freewheel;
		settings.irun = driver_current_.irun;
		settings.irun_register_value = driver_current_.irun;
		settings.ihold = driver_current_.ihold;
		settings.ihold_register_value = driver_current_.ihold;
		settings.iholddelay_percent = holdDelaySettingToPercent(
				driver_current_.iholddelay);
		settings.iholddelay_register_value = driver_current_.iholddelay;
		settings.automatic_current_scaling_enabled = pwm_config_.pwm_autoscale;
		settings.automatic_gradient_adaptation_enabled =
				pwm_config_.pwm_autograd;
		settings.pwm_offset = pwm_config_.pwm_offset;
		settings.pwm_gradient = pwm_config_.pwm_grad;
		settings.cool_step_enabled = cool_step_enabled_;
		settings.analog_current_scaling_enabled = global_config_.i_scale_analog;
		settings.internal_sense_resistors_enabled =
				global_config_.internal_rsense;
	} else {
		settings.is_setup = false;
		settings.software_enabled = false;
		settings.microsteps_per_step = 0;
		settings.inverse_motor_direction_enabled = false;
		settings.stealth_chop_enabled = false;
		settings.standstill_mode = pwm_config_.freewheel;
		settings.irun = 0;
		settings.irun_register_value = 0;
		settings.ihold = 0;
		settings.ihold_register_value = 0;
		settings.iholddelay_percent = 0;
		settings.iholddelay_register_value = 0;
		settings.automatic_current_scaling_enabled = false;
		settings.automatic_gradient_adaptation_enabled = false;
		settings.pwm_offset = 0;
		settings.pwm_gradient = 0;
		settings.cool_step_enabled = false;
		settings.analog_current_scaling_enabled = false;
		settings.internal_sense_resistors_enabled = false;
	}

	return settings;
}

/**
 * @brief  Retrieves the current status of the TMC2209 driver.
 * @param  None
 * @retval TMC2209::Status The current status of the TMC2209 driver.
 */
TMC2209::Status TMC2209::getStatus() {
	DriveStatus drive_status;
	drive_status.bytes = 0;
	drive_status.bytes = read(ADDRESS_DRV_STATUS);

	return drive_status.status;
}

/**
 * @brief  Retrieves the global status of the TMC2209 driver.
 * @param  None
 * @retval TMC2209::GlobalStatus The global status of the TMC2209 driver.
 */
TMC2209::GlobalStatus TMC2209::getGlobalStatus() {
	GlobalStatusUnion global_status_union;
	global_status_union.bytes = 0;
	global_status_union.bytes = read(ADDRESS_GSTAT);

	return global_status_union.global_status;
}

/**
 * @brief  Clears the reset flag in the global status register of the TMC2209.
 * @param  None
 * @retval None
 */
void TMC2209::clearReset() {
	GlobalStatusUnion global_status_union;
	global_status_union.bytes = 0;
	global_status_union.global_status.reset = 1;
	write(ADDRESS_GSTAT, global_status_union.bytes);
}

/**
 * @brief  Clears the drive error flag in the global status register of the TMC2209.
 * @param  None
 * @retval None
 */
void TMC2209::clearDriveError() {
	GlobalStatusUnion global_status_union;
	global_status_union.bytes = 0;
	global_status_union.global_status.drv_err = 1;
	write(ADDRESS_GSTAT, global_status_union.bytes);
}

/**
 * @brief  Retrieves the interface transmission counter from the TMC2209.
 * @param  None
 * @retval uint8_t  The current value of the interface transmission counter.
 */
uint8_t TMC2209::getInterfaceTransmissionCounter() {
	return read(ADDRESS_IFCNT);
}

/**
 * @brief  Retrieves the interstep duration from the TMC2209.
 * @param  None
 * @retval uint32_t  The current value of the interstep duration.
 */
uint32_t TMC2209::getInterstepDuration() {
	return read(ADDRESS_TSTEP);
}

/**
 * @brief  Retrieves the StallGuard result from the TMC2209.
 * @param  None
 * @retval uint16_t  The current value of the StallGuard result.
 */
uint16_t TMC2209::getStallGuardResult() {
	return read(ADDRESS_SG_RESULT);
}

/**
 * @brief  Retrieves the PWM scale sum from the TMC2209.
 * @param  None
 * @retval uint8_t  The current PWM scale sum value.
 */
uint8_t TMC2209::getPwmScaleSum() {
	PwmScale pwm_scale;
	pwm_scale.bytes = read(ADDRESS_PWM_SCALE);

	return pwm_scale.pwm_scale_sum;
}

/**
 * @brief  Retrieves the PWM scale auto value from the TMC2209.
 * @param  None
 * @retval int16_t  The current PWM scale auto value.
 */
int16_t TMC2209::getPwmScaleAuto() {
	PwmScale pwm_scale;
	pwm_scale.bytes = read(ADDRESS_PWM_SCALE);

	return pwm_scale.pwm_scale_auto;
}

/**
 * @brief  Retrieves the PWM offset auto value from the TMC2209.
 * @param  None
 * @retval uint8_t  The current PWM offset auto value.
 */
uint8_t TMC2209::getPwmOffsetAuto() {
	PwmAuto pwm_auto;
	pwm_auto.bytes = read(ADDRESS_PWM_AUTO);

	return pwm_auto.pwm_offset_auto;
}

/**
 * @brief  Retrieves the PWM gradient auto value from the TMC2209.
 * @param  None
 * @retval uint8_t  The current PWM gradient auto value.
 */
uint8_t TMC2209::getPwmGradientAuto() {
	PwmAuto pwm_auto;
	pwm_auto.bytes = read(ADDRESS_PWM_AUTO);

	return pwm_auto.pwm_gradient_auto;
}

/**
 * @brief  Retrieves the microstep counter value from the TMC2209.
 * @param  None
 * @retval uint16_t  The current microstep counter value.
 */
uint16_t TMC2209::getMicrostepCounter() {
	return read(ADDRESS_MSCNT);
}

/* Private methods -----------------------------------------------------------*/

/**
 * @brief  Initializes the TMC2209 driver by setting operation mode, clearing errors, and configuring settings.
 */
void TMC2209::initialize() {
	init_flag = 1;
	setOperationModeToSerial();
	setRegistersToDefaults();
	clearDriveError();

	minimizeMotorCurrent();
	disable();
	disableAutomaticCurrentScaling();
	disableAutomaticGradientAdaptation();
	init_flag = 0;
	precomputedCRCIndex = 0;
}

/**
 * @brief  Sets the operation mode of the TMC2209 to serial communication mode by configuring global settings.
 * @param  None
 * @retval None
 */
void TMC2209::setOperationModeToSerial() {

	global_config_.bytes = 0;
	global_config_.i_scale_analog = 0;
	global_config_.pdn_disable = 1;
	global_config_.mstep_reg_select = 1;
	global_config_.multistep_filt = 1;

	writeStoredGlobalConfig();
}

/**
 * @brief  Initializes the TMC2209 registers to their default values.
 * @param  None
 * @retval None
 */
void TMC2209::setRegistersToDefaults() {
	driver_current_.bytes = 0;
	driver_current_.ihold = IHOLD_DEFAULT;
	driver_current_.irun = IRUN_DEFAULT;
	driver_current_.iholddelay = IHOLDDELAY_DEFAULT;
	write(ADDRESS_IHOLD_IRUN, driver_current_.bytes);

	chopper_config_.bytes = CHOPPER_CONFIG_DEFAULT;
	chopper_config_.tbl = TBL_DEFAULT;
	chopper_config_.hend = HEND_DEFAULT;
	chopper_config_.hstart = HSTART_DEFAULT;
	chopper_config_.toff = TOFF_DEFAULT;
	write(ADDRESS_CHOPCONF, chopper_config_.bytes);

	pwm_config_.bytes = PWM_CONFIG_DEFAULT;
	write(ADDRESS_PWMCONF, pwm_config_.bytes);

	cool_config_.bytes = COOLCONF_DEFAULT;
	write(ADDRESS_COOLCONF, cool_config_.bytes);

	write(ADDRESS_TPOWERDOWN, TPOWERDOWN_DEFAULT);
	write(ADDRESS_TPWMTHRS, TPWMTHRS_DEFAULT);
	write(ADDRESS_VACTUAL, VACTUAL_DEFAULT);
	write(ADDRESS_TCOOLTHRS, TCOOLTHRS_DEFAULT);
	write(ADDRESS_SGTHRS, SGTHRS_DEFAULT);
	write(ADDRESS_COOLCONF, COOLCONF_DEFAULT);
}

/**
 * @brief  Reads and stores the global, chopper, and PWM configuration registers.
 * @param  None
 * @retval None
 */
void TMC2209::readAndStoreRegisters() {
	global_config_.bytes = readGlobalConfigBytes();
	chopper_config_.bytes = readChopperConfigBytes();
	pwm_config_.bytes = readPwmConfigBytes();
}

/**
 * @brief  Checks if the serial operation mode is enabled by reading the global configuration register.
 * @param  None
 * @retval True if serial operation mode is enabled, false otherwise.
 */
bool TMC2209::serialOperationMode() {
	GlobalConfig global_config;
	global_config.bytes = readGlobalConfigBytes();

	return global_config.pdn_disable;
}

/**
 * @brief  Minimizes the motor current by setting both the run and hold current to the minimum value.
 * @param  None
 * @retval None
 */
void TMC2209::minimizeMotorCurrent() {
	driver_current_.irun = CURRENT_SETTING_MIN;
	driver_current_.ihold = CURRENT_SETTING_MIN;
	writeStoredDriverCurrent();
}

template<typename Datagram>
/**
 * @brief  Sends a datagram via UART using DMA.
 * @param  datagram: The datagram structure containing the data to be transmitted.
 * @param  datagram_size: The size of the datagram to be send.
 * @retval None
 */
void TMC2209::sendDatagram(Datagram &datagram, uint8_t datagram_size) {
	HAL_HalfDuplex_EnableTransmitter(UART_address);
	HAL_UART_Transmit_DMA(UART_address, (uint8_t*) &datagram, datagram_size);
	//EnableReciver in TxCpltCallback
}

/**
 * @brief  Writes data to a specific register of the TMC2209.
 * @param  register_address: The address of the register to write to.
 * @param  data: The data to write to the register.
 * @retval None
 */
void TMC2209::write(uint8_t register_address, uint32_t data) {
	WriteReadReplyDatagram write_datagram;
	write_datagram.bytes = 0;
	write_datagram.sync = SYNC;
	write_datagram.serial_address = DEFAULT_SERIAL_ADDRESS;
	write_datagram.register_address = register_address;
	write_datagram.rw = RW_WRITE;
	write_datagram.data = reverseData(data, DATA_SIZE);
	if (init_flag) {
		write_datagram.crc = precomputedCRC[precomputedCRCIndex];
		precomputedCRCIndex++;
	} else
		write_datagram.crc = HAL_CRC_Calculate(&hcrc,
				(uint32_t*) &write_datagram,
				WRITE_READ_REPLY_DATAGRAM_SIZE - 1);

	sendDatagram(write_datagram, WRITE_READ_REPLY_DATAGRAM_SIZE);

	uint32_t sent_timeout = HAL_GetTick() + SEND_TIMEOUT;
	while (!data_sent_flag) {
		if (HAL_GetTick() > sent_timeout) {
			TMC2209_status = TMC_TIMEOUT;
			Error_Handler();
		}
	}
	data_sent_flag = false;
	TMC2209_status = TMC_OK;
}
/**
 * @brief  Reads data from a specific register of the TMC2209.
 * @param  register_address: The address of the register to read from.
 * @retval uint32_t: The data read from the register.
 */
uint32_t TMC2209::read(uint8_t register_address) {
	ReadRequestDatagram read_request_datagram;
	read_request_datagram.bytes = 0;
	read_request_datagram.sync = SYNC;
	read_request_datagram.serial_address = DEFAULT_SERIAL_ADDRESS;
	read_request_datagram.register_address = register_address;
	read_request_datagram.rw = RW_READ;
	if (init_flag) {
		read_request_datagram.crc = precomputedCRC[precomputedCRCIndex];
		precomputedCRCIndex++;
	} else
		read_request_datagram.crc = HAL_CRC_Calculate(&hcrc,
				(uint32_t*) &read_request_datagram,
				READ_REQUEST_DATAGRAM_SIZE - 1);

	sendDatagram(read_request_datagram, READ_REQUEST_DATAGRAM_SIZE);

	uint32_t recive_timeout = HAL_GetTick() + READ_REPLY_TIMEOUT;
	while (!data_received_flag) {
		if (HAL_GetTick() > recive_timeout) {
			TMC2209_status = TMC_TIMEOUT;
			Error_Handler();
		}
	}

	uint8_t crc_check = HAL_CRC_Calculate(&hcrc, (uint32_t*) &rxBufferRaw,
			WRITE_READ_REPLY_DATAGRAM_SIZE - 1);
	if (crc_check == rxBufferRaw[7]) {
		WriteReadReplyDatagram read_reply_datagram;
		memcpy(&read_reply_datagram.bytes, rxBufferRaw,
				WRITE_READ_REPLY_DATAGRAM_SIZE);
		read_reply_datagram.data = reverseData(read_reply_datagram.data,
				DATA_SIZE);
		data_received_flag = false;
		TMC2209_status = TMC_OK;
		if (read_reply_datagram.register_address == register_address)
			return read_reply_datagram.data;
		else {
			TMC2209_status = TMC_UART_ERROR;
			//Error_Handler();
		}
	} else {
		TMC2209_status = TMC_CRC_ERROR;
		//Error_Handler();
	}
	//Never reached
	return 0;
}

/**
 * @brief  Converts the hold delay setting to a percentage value.
 * @param  hold_delay_setting: The hold delay setting to convert.
 * @retval uint8_t: The corresponding percentage value.
 */
uint8_t TMC2209::holdDelaySettingToPercent(uint8_t hold_delay_setting) {
	uint8_t percent = map(hold_delay_setting, HOLD_DELAY_MIN, HOLD_DELAY_MAX,
			PERCENT_MIN, PERCENT_MAX);
	return percent;
}

/**
 * @brief  Writes the stored global configuration to the driver.
 * @param  None
 * @retval None
 */
void TMC2209::writeStoredGlobalConfig() {
	write(ADDRESS_GCONF, global_config_.bytes);
}

/**
 * @brief  Reads the global configuration bytes from the driver.
 * @param  None
 * @retval uint32_t  The global configuration bytes.
 */
uint32_t TMC2209::readGlobalConfigBytes() {
	return read(ADDRESS_GCONF);
}

/**
 * @brief  Writes the stored driver current configuration to the driver.
 * @param  None
 * @retval None
 */
void TMC2209::writeStoredDriverCurrent() {
	write(ADDRESS_IHOLD_IRUN, driver_current_.bytes);

	if (driver_current_.irun >= SEIMIN_UPPER_CURRENT_LIMIT) {
		cool_config_.seimin = SEIMIN_UPPER_SETTING;
	} else {
		cool_config_.seimin = SEIMIN_LOWER_SETTING;
	}
	if (cool_step_enabled_) {
		write(ADDRESS_COOLCONF, cool_config_.bytes);
	}
}

/**
 * @brief  Writes the stored chopper configuration to the driver.
 * @param  None
 * @retval None
 */
void TMC2209::writeStoredChopperConfig() {
	write(ADDRESS_CHOPCONF, chopper_config_.bytes);
}

/**
 * @brief  Reads the chopper configuration bytes from the driver.
 * @param  None
 * @retval Chopper configuration data as a 32-bit value.
 */
uint32_t TMC2209::readChopperConfigBytes() {
	return read(ADDRESS_CHOPCONF);
}

/**
 * @brief  Writes the stored PWM configuration to the driver.
 * @param  None
 * @retval None
 */
void TMC2209::writeStoredPwmConfig() {
	write(ADDRESS_PWMCONF, pwm_config_.bytes);
}

/**
 * @brief  Reads the PWM configuration bytes from the driver.
 * @param  None
 * @retval uint32_t: The PWM configuration bytes.
 */
uint32_t TMC2209::readPwmConfigBytes() {
	return read(ADDRESS_PWMCONF);
}
