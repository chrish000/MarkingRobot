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
#include <algorithm>

class Printhead {
public:
	Printhead(TIM_HandleTypeDef *htim, uint32_t timChannel) :
			htim(htim), channel(timChannel) {
	}

	struct param {
		uint16_t period = PRINTHEAD_PERIOD;
		uint8_t dutyCycle = std::clamp(PRINTHEAD_DUTY_CYCLE, 0, 100);
	} param;

	void setParam(uint16_t newPeriod = PRINTHEAD_PERIOD, uint8_t newDutyCycle =
	PRINTHEAD_DUTY_CYCLE) {
		param.period = newPeriod;
		param.dutyCycle = newDutyCycle;
		htim->Instance->ARR = (uint32_t)(param.period * 10 - 1);
		htim->Instance->CCR1 = (uint32_t)(param.dutyCycle * param.period * 10 / 100 - 1);
	}
	void start() {
		HAL_TIM_PWM_Start(htim, channel);
		active = true;
	}
	void stop() {
		HAL_TIM_PWM_Stop(htim, channel);
		active = false;
	}
	bool isActive() {
		return active;
	}
	void init() {
		stop();
		setParam();
	}

private:
	TIM_HandleTypeDef *htim;
	uint32_t channel;

	bool active = false;
};

#endif /* PRINTHEAD_H_ */
