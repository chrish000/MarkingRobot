/******************************************************************************
 * @file           : TMC2209.h
 * @brief          : Header for TMC2209.c file.
 *                   This file contains the common defines of the application.
 * @author		   : Chris Hauser
 ******************************************************************************
 * @attention
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 * This TMC2209 Library is meant to be used with a STM32H7
 *
 ******************************************************************************
 */
#ifndef TMC2209_H
#define TMC2209_H

#include "main.h"

/**
 * @brief  TMC Status structures definition
 */
enum TMC_StatusTypeDef{
	TMC_OK = 0x00,
	TMC_ERROR = 0x01,
	TMC_CRC_ERROR = 0x02,
	TMC_UART_ERROR = 0x03,
	TMC_TIMEOUT = 0x04
};

class TMC2209 {
public:

	TMC2209(UART_HandleTypeDef *UART_address,
			GPIO_TypeDef *hardware_enable_port, uint16_t hardware_enable_pin) :
			UART_address(UART_address), hardware_enable_port(
					hardware_enable_port), hardware_enable_pin(
					hardware_enable_pin) {
		cool_step_enabled_ = false;
		data_received_flag = false;
		TMC2209_status = TMC_OK;
	}

	UART_HandleTypeDef *UART_address;
	GPIO_TypeDef *hardware_enable_port;
	uint16_t hardware_enable_pin;
	TMC_StatusTypeDef TMC2209_status;

	void setup();

	void setOperationModeToSerial();

	/* Unidirectional methods ----------------------------------------------------*/

	// driver must be enabled before use. it is disabled by default
	void enable();
	void disable();

	// valid values = 1,2,4,8,...128,256, other values get rounded down
	void setMicrostepsPerStep(uint16_t microsteps_per_step);

	void setRunCurrent(uint16_t runCurrent);

	void setHoldCurrent(uint16_t holdCurrent);

	void setHoldDelay(uint8_t holdDelayPercent);	// range 0-100

	void setAllCurrentValues(uint16_t runCurrent, uint16_t holdCurrent,
			uint8_t holdDelayPercent);

	void enableDoubleEdge();
	void disableDoubleEdge();

	void enableInverseMotorDirection();
	void disableInverseMotorDirection();

	enum StandstillMode		//Stand still option when motor current is zero
	{
		NORMAL = 0,			//Normal operation
		FREEWHEELING = 1,		//Freewheeling
		STRONG_BRAKING = 2,	//Coil shorted using LS drivers
		BRAKING = 3,			//Coil shorted using HS drivers
	};
	void setStandstillMode(StandstillMode mode);

	void enableAutomaticCurrentScaling();
	void disableAutomaticCurrentScaling();
	void enableAutomaticGradientAdaptation();
	void disableAutomaticGradientAdaptation();

	void setPwmOffset(uint8_t pwm_amplitude);	// range 0-255

	void setPwmGradient(uint8_t pwm_amplitude);	// range 0-255

	void setPowerDownDelay(uint8_t power_down_delay);// default = 20, mimimum of 2 for StealthChop auto tuning

	constexpr static uint8_t REPLY_DELAY_MAX = 15;	//*8 bit times
	void setReplyDelay(uint8_t delay);// mimimum of 2 when using multiple serial addresses in bidirectional communication

	void moveAtVelocity(int32_t microsteps_per_period);	//in +-(2^23)-1 [μsteps/t]
	void moveUsingStepDirInterface();

	void enableStealthChop();
	void disableStealthChop();

	/* When the velocity exceeds the limit set, the driver switches to SpreadCycle
	 * Inputformat is the actual measured time between two 1/256 microsteps derived
	 * from the step frequency in units of 1/fCLK. */
	void setStealthChopDurationThreshold(uint32_t duration_threshold);

	void setStallGuardThreshold(uint8_t stall_guard_threshold);	//Detection threshold for stall

	// lower_threshold: min = 1, max = 15
	// upper_threshold: min = 0, max = 15, 0-2 recommended
	void enableCoolStep(uint8_t lower_threshold = 1,
			uint8_t upper_threshold = 0);
	void disableCoolStep();
	enum CurrentIncrement {
		CURRENT_INCREMENT_1 = 0,
		CURRENT_INCREMENT_2 = 1,
		CURRENT_INCREMENT_4 = 2,
		CURRENT_INCREMENT_8 = 3,
	};
	void setCoolStepCurrentIncrement(CurrentIncrement current_increment);
	enum MeasurementCount {
		MEASUREMENT_COUNT_32 = 0,
		MEASUREMENT_COUNT_8 = 1,
		MEASUREMENT_COUNT_2 = 2,
		MEASUREMENT_COUNT_1 = 3,
	};
	void setCoolStepMeasurementCount(MeasurementCount measurement_count);
	void setCoolStepDurationThreshold(uint32_t duration_threshold);

	void enableAnalogCurrentScaling();
	void disableAnalogCurrentScaling();

	void useExternalSenseResistors();
	void useInternalSenseResistors();

	/* Bidirectional methods -----------------------------------------------------*/
	uint8_t getVersion();

	// if driver is not communicating, check power and communication connections
	bool isCommunicating();

	// check to make sure TMC2209 is properly setup and communicating
	bool isSetupAndCommunicating();

	// driver may be communicating but not setup if driver power is lost then
	// restored after setup so that defaults are loaded instead of setup options
	bool isCommunicatingButNotSetup();

	// driver may also be disabled by the hardware enable input pin
	// this pin must be grounded or disconnected before driver may be enabled
	bool hardwareDisabled();
	uint16_t getMicrostepsPerStep();

	//Datagram
	constexpr static uint8_t WRITE_READ_REPLY_DATAGRAM_SIZE = 8;
	volatile bool data_received_flag = false;
	volatile bool data_sent_flag = false;
	uint8_t rxBufferRaw[8];

	struct Settings {
		bool is_communicating;
		bool is_setup;
		bool software_enabled;
		uint16_t microsteps_per_step;
		bool inverse_motor_direction_enabled;
		bool stealth_chop_enabled;
		uint8_t standstill_mode;
		uint16_t irun;
		uint8_t irun_register_value;
		uint8_t ihold;
		uint8_t ihold_register_value;
		uint8_t iholddelay_percent;
		uint8_t iholddelay_register_value;
		bool automatic_current_scaling_enabled;
		bool automatic_gradient_adaptation_enabled;
		uint8_t pwm_offset;
		uint8_t pwm_gradient;
		bool cool_step_enabled;
		bool analog_current_scaling_enabled;
		bool internal_sense_resistors_enabled;
	};
	Settings getSettings();

	struct Status {
		uint32_t over_temperature_warning :1;
		uint32_t over_temperature_shutdown :1;
		uint32_t short_to_ground_a :1;
		uint32_t short_to_ground_b :1;
		uint32_t low_side_short_a :1;
		uint32_t low_side_short_b :1;
		uint32_t open_load_a :1;
		uint32_t open_load_b :1;
		uint32_t over_temperature_120c :1;
		uint32_t over_temperature_143c :1;
		uint32_t over_temperature_150c :1;
		uint32_t over_temperature_157c :1;
		uint32_t reserved0 :4;
		uint32_t current_scaling :5;
		uint32_t reserved1 :9;
		uint32_t stealth_chop_mode :1;
		uint32_t standstill :1;
	};
	constexpr static uint8_t CURRENT_SCALING_MAX = 31;
	Status getStatus();

	struct GlobalStatus {
		uint32_t reset :1;
		uint32_t drv_err :1;
		uint32_t uv_cp :1;
		uint32_t reserved :29;
	};
	GlobalStatus getGlobalStatus();
	void clearReset();
	void clearDriveError();

	uint8_t getInterfaceTransmissionCounter();

	uint32_t getInterstepDuration();

	uint16_t getStallGuardResult();

	uint8_t getPwmScaleSum();
	int16_t getPwmScaleAuto();
	uint8_t getPwmOffsetAuto();
	uint8_t getPwmGradientAuto();

	uint16_t getMicrostepCounter();

private:

	//Initialization of TMC
	volatile bool init_flag = 0;
	static constexpr uint8_t precomputedCRC[] = {//fixed order in initialize()
			0xdf, 0x97, 0x06, 0x45, 0x1f, 0x0f, 0x0e, 0x34, 0x8d, 0x45, 0x19,
					0x57, 0xd9, 0xf6, 0x0c, 0x45, 0xe7, 0x81, 0xdf, 0x97, 0x06,
					0x45, 0x1f, 0x0f, 0x0e, 0x34, 0x8d, 0x45, 0x19, 0x57, 0xd9,
					0xf6, 0x0c, 0x45, 0xe7, 0x81 };
	uint8_t precomputedCRCIndex = 0;
	void initialize();

	// Serial Settings
	constexpr static uint8_t BYTE_MAX_VALUE = 0xFF;
	constexpr static uint8_t BITS_PER_BYTE = 8;

	constexpr static uint8_t STEPPER_DRIVER_FEATURE_OFF = 0;
	constexpr static uint8_t STEPPER_DRIVER_FEATURE_ON = 1;

	// Datagrams
	const static uint8_t DATA_SIZE = 4;	//Number of Bytes stored in 'data' from WriteReadReplyDatagram
	union WriteReadReplyDatagram {
		struct {
			uint64_t sync :4;
			uint64_t reserved :4;
			uint64_t serial_address :8;
			uint64_t register_address :7;
			uint64_t rw :1;
			uint64_t data :32;
			uint64_t crc :8;
		};
		uint64_t bytes;
	};

	constexpr static uint8_t SYNC = 0x05;
	constexpr static uint8_t RW_READ = 0;
	constexpr static uint8_t RW_WRITE = 1;
	constexpr static uint8_t DEFAULT_SERIAL_ADDRESS = 0x00;
	constexpr static uint8_t READ_REPLY_SERIAL_ADDRESS = 0xFF;

	constexpr static uint8_t READ_REQUEST_DATAGRAM_SIZE = 4;
	union ReadRequestDatagram {
		struct {
			uint32_t sync :4;
			uint32_t reserved :4;
			uint32_t serial_address :8;
			uint32_t register_address :7;
			uint32_t rw :1;
			uint32_t crc :8;
		};
		uint32_t bytes;
	};

	// General Configuration Registers
	constexpr static uint8_t ADDRESS_GCONF = 0x00;
	union GlobalConfig {
		struct {
			uint32_t i_scale_analog :1;
			uint32_t internal_rsense :1;
			uint32_t enable_spread_cycle :1;
			uint32_t shaft :1;
			uint32_t index_otpw :1;
			uint32_t index_step :1;
			uint32_t pdn_disable :1;
			uint32_t mstep_reg_select :1;
			uint32_t multistep_filt :1;
			uint32_t test_mode :1;
			uint32_t reserved :22;
		};
		uint32_t bytes;
	};
	GlobalConfig global_config_;

	constexpr static uint8_t ADDRESS_GSTAT = 0x01;
	union GlobalStatusUnion {
		struct {
			GlobalStatus global_status;
		};
		uint32_t bytes;
	};

	constexpr static uint8_t ADDRESS_IFCNT = 0x02;

	constexpr static uint8_t ADDRESS_REPLYDELAY = 0x03;
	union ReplyDelay {
		struct {
			uint32_t reserved_0 :8;
			uint32_t replydelay :4;
			uint32_t reserved_1 :20;
		};
		uint32_t bytes;
	};

	constexpr static uint8_t ADDRESS_IOIN = 0x06;
	union Input {
		struct {
			uint32_t enn :1;
			uint32_t reserved_0 :1;
			uint32_t ms1 :1;
			uint32_t ms2 :1;
			uint32_t diag :1;
			uint32_t reserved_1 :1;
			uint32_t pdn_serial :1;
			uint32_t step :1;
			uint32_t spread_en :1;
			uint32_t dir :1;
			uint32_t reserved_2 :14;
			uint32_t version :8;
		};
		uint32_t bytes;
	};
	constexpr static uint8_t VERSION = 0x21;	//0x21=first version of the IC

	// Velocity Dependent Driver Feature Control Register Set
	constexpr static uint8_t ADDRESS_IHOLD_IRUN = 0x10;
	union DriverCurrent {
		struct {
			uint32_t ihold :5;
			uint32_t reserved_0 :3;
			uint32_t irun :5;
			uint32_t reserved_1 :3;
			uint32_t iholddelay :4;
			uint32_t reserved_2 :12;
		};
		uint32_t bytes;
	};
	DriverCurrent driver_current_;
	constexpr static uint8_t PERCENT_MIN = 0;
	constexpr static uint8_t PERCENT_MAX = 100;
	const uint16_t CURRENT_SETTING_MIN = 0;		//mA
	const uint16_t CURRENT_SETTING_MAX = 2000;	//mA
	const uint8_t RUN_CURRENT_SETTING_MIN = 0;
	const uint8_t RUN_CURRENT_SETTING_MAX = 31;
	const uint8_t HOLD_DELAY_MIN = 0;
	const uint8_t HOLD_DELAY_MAX = 15;
	constexpr static uint8_t IHOLD_DEFAULT = 16;
	constexpr static uint8_t IRUN_DEFAULT = 31;
	constexpr static uint8_t IHOLDDELAY_DEFAULT = 1;

	constexpr static uint8_t ADDRESS_TPOWERDOWN = 0x11;
	constexpr static uint8_t TPOWERDOWN_DEFAULT = 20;

	constexpr static uint8_t ADDRESS_TSTEP = 0x12;

	constexpr static uint8_t ADDRESS_TPWMTHRS = 0x13;
	constexpr static uint32_t TPWMTHRS_DEFAULT = 0;

	constexpr static uint8_t ADDRESS_VACTUAL = 0x22;
	constexpr static int32_t VACTUAL_DEFAULT = 0;
	constexpr static int32_t VACTUAL_STEP_DIR_INTERFACE = 0;

	// CoolStep and StallGuard Control Register Set
	constexpr static uint8_t ADDRESS_TCOOLTHRS = 0x14;
	constexpr static uint8_t TCOOLTHRS_DEFAULT = 0;
	constexpr static uint8_t ADDRESS_SGTHRS = 0x40;
	constexpr static uint8_t SGTHRS_DEFAULT = 0;
	constexpr static uint8_t ADDRESS_SG_RESULT = 0x41;

	constexpr static uint8_t ADDRESS_COOLCONF = 0x42;
	constexpr static uint8_t COOLCONF_DEFAULT = 0;
	union CoolConfig {
		struct {
			uint32_t semin :4;
			uint32_t reserved_0 :1;
			uint32_t seup :2;
			uint32_t reserved_1 :1;
			uint32_t semax :4;
			uint32_t reserved_2 :1;
			uint32_t sedn :2;
			uint32_t seimin :1;
			uint32_t reserved_3 :16;
		};
		uint32_t bytes;
	};
	CoolConfig cool_config_;
	bool cool_step_enabled_;
	constexpr static uint8_t SEIMIN_UPPER_CURRENT_LIMIT = 20;//minimum current for smart current control (20 == 19/32 of max irun)
	constexpr static uint8_t SEIMIN_LOWER_SETTING = 0;
	constexpr static uint8_t SEIMIN_UPPER_SETTING = 1;
	constexpr static uint8_t SEMIN_OFF = 0;
	constexpr static uint8_t SEMIN_MIN = 1;
	constexpr static uint8_t SEMIN_MAX = 15;
	constexpr static uint8_t SEMAX_MIN = 0;
	constexpr static uint8_t SEMAX_MAX = 15;

	// Microstepping Control Register Set
	constexpr static uint8_t ADDRESS_MSCNT = 0x6A;
	constexpr static uint8_t ADDRESS_MSCURACT = 0x6B;

	// Driver Register Set
	constexpr static uint8_t ADDRESS_CHOPCONF = 0x6C;
	union ChopperConfig {
		struct {
			uint32_t toff :4;
			uint32_t hstart :3;
			uint32_t hend :4;
			uint32_t reserved_0 :4;
			uint32_t tbl :2;
			uint32_t vsense :1;
			uint32_t reserved_1 :6;
			uint32_t mres :4;
			uint32_t interpolation :1;
			uint32_t double_edge :1;
			uint32_t diss2g :1;
			uint32_t diss2vs :1;
		};
		uint32_t bytes;
	};
	ChopperConfig chopper_config_;
	constexpr static uint32_t CHOPPER_CONFIG_DEFAULT = 0x10000053;
	constexpr static uint8_t TBL_DEFAULT = 0b10;
	constexpr static uint8_t HEND_DEFAULT = 0;
	constexpr static uint8_t HSTART_DEFAULT = 5;
	constexpr static uint8_t TOFF_DEFAULT = 3;
	constexpr static uint8_t TOFF_DISABLE = 0;
	uint8_t toff_ = TOFF_DEFAULT;
	constexpr static uint8_t MRES_256 = 0b0000;
	constexpr static uint8_t MRES_128 = 0b0001;
	constexpr static uint8_t MRES_064 = 0b0010;
	constexpr static uint8_t MRES_032 = 0b0011;
	constexpr static uint8_t MRES_016 = 0b0100;
	constexpr static uint8_t MRES_008 = 0b0101;
	constexpr static uint8_t MRES_004 = 0b0110;
	constexpr static uint8_t MRES_002 = 0b0111;
	constexpr static uint8_t MRES_001 = 0b1000;
	constexpr static uint8_t DOUBLE_EDGE_DISABLE = 0;
	constexpr static uint8_t DOUBLE_EDGE_ENABLE = 1;

	constexpr static size_t MICROSTEPS_PER_STEP_MIN = 1;
	constexpr static size_t MICROSTEPS_PER_STEP_MAX = 256;

	constexpr static uint8_t ADDRESS_DRV_STATUS = 0x6F;
	union DriveStatus {
		struct {
			Status status;
		};
		uint32_t bytes;
	};

	constexpr static uint8_t ADDRESS_PWMCONF = 0x70;
	union PwmConfig {
		struct {
			uint32_t pwm_offset :8;
			uint32_t pwm_grad :8;
			uint32_t pwm_freq :2;
			uint32_t pwm_autoscale :1;
			uint32_t pwm_autograd :1;
			uint32_t freewheel :2;
			uint32_t reserved :2;
			uint32_t pwm_reg :4;
			uint32_t pwm_lim :4;
		};
		uint32_t bytes;
	};
	PwmConfig pwm_config_;
	constexpr static uint32_t PWM_CONFIG_DEFAULT = 0xC10D0024;
	constexpr static uint8_t PWM_OFFSET_MIN = 0;
	constexpr static uint8_t PWM_OFFSET_MAX = 255;
	constexpr static uint8_t PWM_OFFSET_DEFAULT = 0x24;
	constexpr static uint8_t PWM_GRAD_MIN = 0;
	constexpr static uint8_t PWM_GRAD_MAX = 255;
	constexpr static uint8_t PWM_GRAD_DEFAULT = 0x14;

	union PwmScale {
		struct {
			uint32_t pwm_scale_sum :8;
			uint32_t reserved_0 :8;
			uint32_t pwm_scale_auto :9;
			uint32_t reserved_1 :7;
		};
		uint32_t bytes;
	};
	constexpr static uint8_t ADDRESS_PWM_SCALE = 0x71;

	union PwmAuto {
		struct {
			uint32_t pwm_offset_auto :8;
			uint32_t reserved_0 :8;
			uint32_t pwm_gradient_auto :8;
			uint32_t reserved_1 :8;
		};
		uint32_t bytes;
	};
	constexpr static uint8_t ADDRESS_PWM_AUTO = 0x72;

	//void setOperationModeToSerial();

	void setRegistersToDefaults();
	void readAndStoreRegisters();

	bool serialOperationMode();

	void minimizeMotorCurrent();

	template<typename Datagram>
	uint8_t calculateCrc(Datagram &datagram, uint8_t datagram_size);
	template<typename Datagram>
	void sendDatagramUnidirectional(Datagram &datagram, uint8_t datagram_size);
	template<typename Datagram>
	void sendDatagram(Datagram &datagram, uint8_t datagram_size);

	const uint16_t SEND_TIMEOUT = 10000;	//ms
	void write(uint8_t register_address, uint32_t data);
	const uint16_t READ_REPLY_TIMEOUT = 10000;	//ms
	uint32_t read(uint8_t register_address);

	uint8_t holdDelaySettingToPercent(uint8_t hold_delay_setting);

	uint8_t pwmAmplitudeToPwmAmpl(uint8_t pwm_amplitude);
	uint8_t pwmAmplitudeToPwmGrad(uint8_t pwm_amplitude);

	void writeStoredGlobalConfig();
	uint32_t readGlobalConfigBytes();
	void writeStoredDriverCurrent();
	void writeStoredChopperConfig();
	uint32_t readChopperConfigBytes();
	void writeStoredPwmConfig();
	uint32_t readPwmConfigBytes();
};

#endif
