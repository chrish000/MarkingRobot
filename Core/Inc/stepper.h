/**
 ******************************************************************************
 * @file           : stepper.h
 * @brief          :
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
#ifndef STEPPER_H
#define STEPPER_H

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "ringbuffer.hpp"
#include "TMC2209.h"

/* Defines -------------------------------------------------------------------*/
class StepperMotor {
public:
	//Instruktor
	StepperMotor(TIM_HandleTypeDef *TIM_Motor, DMA_HandleTypeDef *TIM_DMA_ARR,
			DMA_HandleTypeDef *TIM_DMA_BSRR, GPIO_TypeDef *stepPort,
			uint16_t stepPin, GPIO_TypeDef *dirPort, uint16_t dirPin,
			UART_HandleTypeDef *TMC_UART_address,
			CRC_HandleTypeDef *TMC_CRC_Handle,
			GPIO_TypeDef *hardware_enable_port, uint16_t hardware_enable_pin);
	//Destruktor
	~StepperMotor();

	TMC2209 tmc;

	TIM_HandleTypeDef *TIM_Motor;
	DMA_HandleTypeDef *TIM_DMA_ARR;
	DMA_HandleTypeDef *TIM_DMA_BSRR;
	GPIO_TypeDef *stepPort;
	uint16_t stepPin;
	GPIO_TypeDef *dirPort;
	uint16_t dirPin;

	struct stepCmd {
		uint32_t interval;	//Dauer in ns
		uint32_t gpioMask;
	} StepCmdBuffer;

	const static size_t buffer_size_step = 1024;	//size = n-1 elements
	static constexpr uint16_t timARRDefault = 1000;

	jnk0le::Ringbuffer<stepCmd, buffer_size_step, false, 64> stepBuf;

	bool timerActiveFlag = 0;
	bool stepFlag = 0;

	void init();
	void startTimer();
	void stopTimer();
	HAL_TIM_StateTypeDef getTimerState();

private:

};

#endif /* STEPPER_H */
