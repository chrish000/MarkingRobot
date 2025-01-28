/**
 ******************************************************************************
 * @file           : printhead.h
 * @brief          : Header for main.c file.
 *                   This file contains the functions and variavles to operate the printhead.
 ******************************************************************************
 * @attention
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#ifndef PRINTHEAD_H_
#define PRINTHEAD_H_

#include "main.h"

class Printhead {
public:
	Printhead(TIM_HandleTypeDef *htim, uint32_t timChannel) :
			htim(htim), channel(timChannel) {
		init();
	}

	struct param {
		uint16_t period = PRINTHEAD_PERIOD;
		uint8_t dutyCycle = PRINTHEAD_DUTY_CYCLE;
	}param;
	void setParam(uint16_t newPeriod = PRINTHEAD_PERIOD, uint8_t newDutyCycle =
			PRINTHEAD_DUTY_CYCLE) {
		param.period = newPeriod;
		param.dutyCycle = newDutyCycle;
		htim->Instance->ARR = param.period;
		htim->Instance->CCR1 = param.dutyCycle;
	}
	void start() {
		HAL_TIM_PWM_Start(htim, channel);
	}
	void stop() {
		HAL_TIM_PWM_Stop(htim, channel);
	}

private:

	TIM_HandleTypeDef *htim;
	uint32_t channel;

	void init() {
		setParam();
	}
};

#endif /* PRINTHEAD_H_ */
