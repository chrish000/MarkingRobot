// ----------------------------------------------------------------------------
// TMC2209.cpp
//
// Modified from Peter Polidoro
// peter@polidoro.io
// https://github.com/janelia-arduino/TMC2209
// ----------------------------------------------------------------------------
#include "TMC2209.h"
#include "utils.h"

TMC2209::TMC2209() {
	cool_step_enabled_ = false;
}

void TMC2209::setup(UART_HandleTypeDef serial_address) {
	HAL_HalfDuplex_EnableReceiver(&serial_address);
	HAL_UARTEx_ReceiveToIdle_DMA(&serial_address,
			(uint8_t*) &write_read_reply_datagram,
			WRITE_READ_REPLY_DATAGRAM_SIZE);

	initialize();
}

// unidirectional methods

void TMC2209::enable() {
	HAL_GPIO_WritePin(hardware_enable_port, hardware_enable_pin,
			GPIO_PIN_RESET); //Treiber aktivieren
	chopper_config_.toff = toff_;
	writeStoredChopperConfig();
}
void TMC2209::disable() {

	HAL_GPIO_WritePin(hardware_enable_port, hardware_enable_pin, GPIO_PIN_SET); //Treiber deaktivieren
	chopper_config_.toff = TOFF_DISABLE;
	writeStoredChopperConfig();
}

void TMC2209::setMicrostepsPerStep(uint16_t microsteps_per_step) {
	chopper_config_.mres = microsteps_per_step;
	writeStoredChopperConfig();
}

void TMC2209::setRunCurrent(uint16_t runCurrent) {
	runCurrent = constrain(runCurrent, CURRENT_SETTING_MIN, CURRENT_SETTING_MAX);
	driver_current_.irun = map(runCurrent, CURRENT_SETTING_MIN, CURRENT_SETTING_MAX, RUN_CURRENT_SETTING_MIN, RUN_CURRENT_SETTING_MAX);
	writeStoredDriverCurrent();
}

void TMC2209::setHoldCurrent(uint16_t holdCurrent) {
	holdCurrent = constrain(holdCurrent, CURRENT_SETTING_MIN, CURRENT_SETTING_MAX);
	driver_current_.ihold = map(holdCurrent, CURRENT_SETTING_MIN, CURRENT_SETTING_MAX, RUN_CURRENT_SETTING_MIN, RUN_CURRENT_SETTING_MAX);
	writeStoredDriverCurrent();
}

void TMC2209::setHoldDelay(uint8_t holdDelayPercent) {
	holdDelayPercent = constrain(holdDelayPercent, PERCENT_MIN, PERCENT_MAX);
	driver_current_.iholddelay = map(holdDelayPercent, PERCENT_MIN, PERCENT_MAX, HOLD_DELAY_MIN, HOLD_DELAY_MAX);
	writeStoredDriverCurrent();
}

void TMC2209::setAllCurrentValues(uint16_t runCurrent, uint16_t holdCurrent, uint8_t holdDelayPercent) {
	runCurrent = constrain(runCurrent, CURRENT_SETTING_MIN, CURRENT_SETTING_MAX);
	driver_current_.irun = map(runCurrent, CURRENT_SETTING_MIN, CURRENT_SETTING_MAX, RUN_CURRENT_SETTING_MIN, RUN_CURRENT_SETTING_MAX);
	holdCurrent = constrain(holdCurrent, CURRENT_SETTING_MIN, CURRENT_SETTING_MAX);
	driver_current_.ihold = map(holdCurrent, CURRENT_SETTING_MIN, CURRENT_SETTING_MAX, RUN_CURRENT_SETTING_MIN, RUN_CURRENT_SETTING_MAX);
	holdDelayPercent = constrain(holdDelayPercent, PERCENT_MIN, PERCENT_MAX);
	driver_current_.iholddelay = map(holdDelayPercent, PERCENT_MIN, PERCENT_MAX, HOLD_DELAY_MIN, HOLD_DELAY_MAX);
	writeStoredDriverCurrent();
}

void TMC2209::enableDoubleEdge() {
	chopper_config_.double_edge = DOUBLE_EDGE_ENABLE;
	writeStoredChopperConfig();
}

void TMC2209::disableDoubleEdge() {
	chopper_config_.double_edge = DOUBLE_EDGE_DISABLE;
	writeStoredChopperConfig();
}

void TMC2209::enableInverseMotorDirection() {
	global_config_.shaft = 1;
	writeStoredGlobalConfig();
}

void TMC2209::disableInverseMotorDirection() {
	global_config_.shaft = 0;
	writeStoredGlobalConfig();
}

void TMC2209::setStandstillMode(TMC2209::StandstillMode mode) {
	pwm_config_.freewheel = mode;
	writeStoredPwmConfig();
}

void TMC2209::enableAutomaticCurrentScaling() {
	pwm_config_.pwm_autoscale = STEPPER_DRIVER_FEATURE_ON;
	writeStoredPwmConfig();
}

void TMC2209::disableAutomaticCurrentScaling() {
	pwm_config_.pwm_autoscale = STEPPER_DRIVER_FEATURE_OFF;
	writeStoredPwmConfig();
}

void TMC2209::enableAutomaticGradientAdaptation() {
	pwm_config_.pwm_autograd = STEPPER_DRIVER_FEATURE_ON;
	writeStoredPwmConfig();
}

void TMC2209::disableAutomaticGradientAdaptation() {
	pwm_config_.pwm_autograd = STEPPER_DRIVER_FEATURE_OFF;
	writeStoredPwmConfig();
}

void TMC2209::setPwmOffset(uint8_t pwm_amplitude) {
	pwm_config_.pwm_offset = pwm_amplitude;
	writeStoredPwmConfig();
}

void TMC2209::setPwmGradient(uint8_t pwm_amplitude) {
	pwm_config_.pwm_grad = pwm_amplitude;
	writeStoredPwmConfig();
}

void TMC2209::setPowerDownDelay(uint8_t power_down_delay) {
	write(ADDRESS_TPOWERDOWN, power_down_delay);
}

void TMC2209::setReplyDelay(uint8_t reply_delay) {
	if (reply_delay > REPLY_DELAY_MAX) {
		reply_delay = REPLY_DELAY_MAX;
	}
	ReplyDelay reply_delay_data;
	reply_delay_data.bytes = 0;
	reply_delay_data.replydelay = reply_delay;
	write(ADDRESS_REPLYDELAY, reply_delay_data.bytes);
}

void TMC2209::moveAtVelocity(int32_t microsteps_per_period) {
	write(ADDRESS_VACTUAL, microsteps_per_period);
}

void TMC2209::moveUsingStepDirInterface() {
	write(ADDRESS_VACTUAL, VACTUAL_STEP_DIR_INTERFACE);
}

void TMC2209::enableStealthChop() {
	global_config_.enable_spread_cycle = 0;
	writeStoredGlobalConfig();
}

void TMC2209::disableStealthChop() {
	global_config_.enable_spread_cycle = 1;
	writeStoredGlobalConfig();
}

void TMC2209::setCoolStepDurationThreshold(uint32_t duration_threshold) {
	write(ADDRESS_TCOOLTHRS, duration_threshold);
}

void TMC2209::setStealthChopDurationThreshold(uint32_t duration_threshold) {
	write(ADDRESS_TPWMTHRS, duration_threshold);
}

void TMC2209::setStallGuardThreshold(uint8_t stall_guard_threshold) {
	write(ADDRESS_SGTHRS, stall_guard_threshold);
}

void TMC2209::enableCoolStep(uint8_t lower_threshold, uint8_t upper_threshold) {
	cool_config_.semin = constrain(lower_threshold, SEMIN_MIN, SEMIN_MAX);
	cool_config_.semax = constrain(upper_threshold, SEMAX_MIN, SEMAX_MAX);
	write(ADDRESS_COOLCONF, cool_config_.bytes);
	cool_step_enabled_ = true;
}

void TMC2209::disableCoolStep() {
	cool_config_.semin = SEMIN_OFF;
	write(ADDRESS_COOLCONF, cool_config_.bytes);
	cool_step_enabled_ = false;
}

void TMC2209::setCoolStepCurrentIncrement(CurrentIncrement current_increment) {
	cool_config_.seup = current_increment;
	write(ADDRESS_COOLCONF, cool_config_.bytes);
}

void TMC2209::setCoolStepMeasurementCount(MeasurementCount measurement_count) {
	cool_config_.sedn = measurement_count;
	write(ADDRESS_COOLCONF, cool_config_.bytes);
}

void TMC2209::enableAnalogCurrentScaling() {
	global_config_.i_scale_analog = 1;
	writeStoredGlobalConfig();
}

void TMC2209::disableAnalogCurrentScaling() {
	global_config_.i_scale_analog = 0;
	writeStoredGlobalConfig();
}

void TMC2209::useExternalSenseResistors() {
	global_config_.internal_rsense = 0;
	writeStoredGlobalConfig();
}

void TMC2209::useInternalSenseResistors() {
	global_config_.internal_rsense = 1;
	writeStoredGlobalConfig();
}
// bidirectional methods

uint8_t TMC2209::getVersion() {
	Input input;
	input.bytes = read(ADDRESS_IOIN);

	return input.version;
}

bool TMC2209::isCommunicating() {
	return (getVersion() == VERSION);
}

bool TMC2209::isSetupAndCommunicating() {
	return serialOperationMode();
}

bool TMC2209::isCommunicatingButNotSetup() {
	return (isCommunicating() && (not isSetupAndCommunicating()));
}

bool TMC2209::hardwareDisabled() {
	Input input;
	input.bytes = read(ADDRESS_IOIN);

	return input.enn;
}
//################################################################
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
		settings.irun_percent = currentSettingToPercent(driver_current_.irun);
		settings.irun_register_value = driver_current_.irun;
		settings.ihold_percent = currentSettingToPercent(driver_current_.ihold);
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
		settings.irun_percent = 0;
		settings.irun_register_value = 0;
		settings.ihold_percent = 0;
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

TMC2209::Status TMC2209::getStatus() {
	DriveStatus drive_status;
	drive_status.bytes = 0;
	drive_status.bytes = read(ADDRESS_DRV_STATUS);
	return drive_status.status;
}

TMC2209::GlobalStatus TMC2209::getGlobalStatus() {
	GlobalStatusUnion global_status_union;
	global_status_union.bytes = 0;
	global_status_union.bytes = read(ADDRESS_GSTAT);
	return global_status_union.global_status;
}

void TMC2209::clearReset() {
	GlobalStatusUnion global_status_union;
	global_status_union.bytes = 0;
	global_status_union.global_status.reset = 1;
	write(ADDRESS_GSTAT, global_status_union.bytes);
}

void TMC2209::clearDriveError() {
	GlobalStatusUnion global_status_union;
	global_status_union.bytes = 0;
	global_status_union.global_status.drv_err = 1;
	write(ADDRESS_GSTAT, global_status_union.bytes);
}

uint8_t TMC2209::getInterfaceTransmissionCounter() {
	return read(ADDRESS_IFCNT);
}

uint32_t TMC2209::getInterstepDuration() {
	return read(ADDRESS_TSTEP);
}

uint16_t TMC2209::getStallGuardResult() {
	return read(ADDRESS_SG_RESULT);
}

uint8_t TMC2209::getPwmScaleSum() {
	PwmScale pwm_scale;
	pwm_scale.bytes = read(ADDRESS_PWM_SCALE);

	return pwm_scale.pwm_scale_sum;
}

int16_t TMC2209::getPwmScaleAuto() {
	PwmScale pwm_scale;
	pwm_scale.bytes = read(ADDRESS_PWM_SCALE);

	return pwm_scale.pwm_scale_auto;
}

uint8_t TMC2209::getPwmOffsetAuto() {
	PwmAuto pwm_auto;
	pwm_auto.bytes = read(ADDRESS_PWM_AUTO);

	return pwm_auto.pwm_offset_auto;
}

uint8_t TMC2209::getPwmGradientAuto() {
	PwmAuto pwm_auto;
	pwm_auto.bytes = read(ADDRESS_PWM_AUTO);

	return pwm_auto.pwm_gradient_auto;
}

uint16_t TMC2209::getMicrostepCounter() {
	return read(ADDRESS_MSCNT);
}

// private
void TMC2209::initialize() {
	setOperationModeToSerial();
	setRegistersToDefaults();
	clearDriveError();

	minimizeMotorCurrent();
	disable();
	disableAutomaticCurrentScaling();
	disableAutomaticGradientAdaptation();
}

void TMC2209::setOperationModeToSerial() {

	global_config_.bytes = 0;
	global_config_.i_scale_analog = 0;
	global_config_.pdn_disable = 1;
	global_config_.mstep_reg_select = 1;
	global_config_.multistep_filt = 1;

	writeStoredGlobalConfig();
}

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

void TMC2209::readAndStoreRegisters() {
	global_config_.bytes = readGlobalConfigBytes();
	chopper_config_.bytes = readChopperConfigBytes();
	pwm_config_.bytes = readPwmConfigBytes();
}

bool TMC2209::serialOperationMode() {
	GlobalConfig global_config;
	global_config.bytes = readGlobalConfigBytes();

	return global_config.pdn_disable;
}

void TMC2209::minimizeMotorCurrent() {
	driver_current_.irun = CURRENT_SETTING_MIN;
	driver_current_.ihold = CURRENT_SETTING_MIN;
	writeStoredDriverCurrent();
}

/*
uint32_t TMC2209::reverseData(uint32_t data) {
	uint32_t reversed_data = 0;
	uint8_t right_shift;
	uint8_t left_shift;
	for (uint8_t i = 0; i < DATA_SIZE; ++i) {
		right_shift = (DATA_SIZE - i - 1) * BITS_PER_BYTE;
		left_shift = i * BITS_PER_BYTE;
		reversed_data |= ((data >> right_shift) & BYTE_MAX_VALUE) << left_shift;
	}
	return reversed_data;
}
 */

template<typename Datagram>
void TMC2209::sendDatagram(Datagram &datagram, uint8_t datagram_size) {
	HAL_UART_Transmit_DMA(&serial_address, (uint8_t*) &datagram, datagram_size);
}

void TMC2209::write(uint8_t register_address, uint32_t data) {
	WriteReadReplyDatagram write_datagram;
	write_datagram.bytes = 0;
	write_datagram.sync = SYNC;
	write_datagram.serial_address = DEFAULT_SERIAL_ADDRESS;
	write_datagram.register_address = register_address;
	write_datagram.rw = RW_WRITE;
	//write_datagram.data = reverseData(data);
	write_datagram.data = data;
	write_datagram.crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) &write_datagram,
			WRITE_READ_REPLY_DATAGRAM_SIZE - 1);

	sendDatagram(write_datagram, WRITE_READ_REPLY_DATAGRAM_SIZE);
}

uint32_t TMC2209::read(uint8_t register_address) {
	ReadRequestDatagram read_request_datagram;
	read_request_datagram.bytes = 0;
	read_request_datagram.sync = SYNC;
	read_request_datagram.serial_address = DEFAULT_SERIAL_ADDRESS;
	read_request_datagram.register_address = register_address;
	read_request_datagram.rw = RW_READ;
	read_request_datagram.crc = HAL_CRC_Calculate(&hcrc,
			(uint32_t*) &read_request_datagram, READ_REQUEST_DATAGRAM_SIZE - 1);

	sendDatagram(read_request_datagram, READ_REQUEST_DATAGRAM_SIZE);

	/*
	 uint32_t reply_delay = 0;
	 while ((serialAvailable() < WRITE_READ_REPLY_DATAGRAM_SIZE)
	 and (reply_delay < REPLY_DELAY_MAX_MICROSECONDS)) {
	 delayMicroseconds(REPLY_DELAY_INC_MICROSECONDS);
	 reply_delay += REPLY_DELAY_INC_MICROSECONDS;
	 }

	 if (reply_delay >= REPLY_DELAY_MAX_MICROSECONDS) {
	 return 0;
	 }

	 uint64_t byte;
	 uint8_t byte_count = 0;
	 WriteReadReplyDatagram read_reply_datagram;
	 read_reply_datagram.bytes = 0;
	 for (uint8_t i = 0; i < WRITE_READ_REPLY_DATAGRAM_SIZE; ++i) {
	 byte = serialRead();
	 read_reply_datagram.bytes |= (byte << (byte_count++ * BITS_PER_BYTE));
	 }

	 return reverseData(read_reply_datagram.data);
	 */
	return 0x12345678;
}

uint8_t TMC2209::holdDelaySettingToPercent(uint8_t hold_delay_setting) {
	uint8_t percent = map(hold_delay_setting, HOLD_DELAY_MIN, HOLD_DELAY_MAX,
			PERCENT_MIN, PERCENT_MAX);
	return percent;
}

void TMC2209::writeStoredGlobalConfig() {
	write(ADDRESS_GCONF, global_config_.bytes);
}

uint32_t TMC2209::readGlobalConfigBytes() {
	return read(ADDRESS_GCONF);
}

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

void TMC2209::writeStoredChopperConfig() {
	write(ADDRESS_CHOPCONF, chopper_config_.bytes);
}

uint32_t TMC2209::readChopperConfigBytes() {
	return read(ADDRESS_CHOPCONF);
}

void TMC2209::writeStoredPwmConfig() {
	write(ADDRESS_PWMCONF, pwm_config_.bytes);
}

uint32_t TMC2209::readPwmConfigBytes() {
	return read(ADDRESS_PWMCONF);
}
