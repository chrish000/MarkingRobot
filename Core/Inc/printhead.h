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
#include <limits>
#include <algorithm>

class Printhead
{
public:
	Printhead(TIM_HandleTypeDef *htim, uint32_t timChannel) : htim(htim), channel(timChannel)
	{
	}

	struct param
	{
		uint16_t period = PRINTHEAD_PERIOD;
		uint8_t dutyCycle = PRINTHEAD_DUTY_CYCLE;
	} param;

	void setParam(uint16_t newPeriod = PRINTHEAD_PERIOD, uint8_t newDutyCycle =
															 PRINTHEAD_DUTY_CYCLE)
	{
		param.period = newPeriod;
		param.dutyCycle = std::clamp(newDutyCycle, 0, 100);
		htim->Instance->ARR = std::clamp((uint16_t)(param.period * 10 - 1),
										 std::numeric_limits<uint16_t>::min(),
										 std::numeric_limits<uint16_t>::max());
		htim->Instance->CCR1 = std::clamp((uint16_t)(param.dutyCycle * param.period * 10 / 100 - 1),
										  std::numeric_limits<uint16_t>::min(),
										  std::numeric_limits<uint16_t>::max());
	}
	void start()
	{
		HAL_TIM_PWM_Start(htim, channel);
		active = true;
	}
	void stop()
	{
		HAL_TIM_PWM_Stop(htim, channel);
		active = false;
	}
	bool isActive()
	{
		return active;
	}
	void init()
	{
		stop();
		setParam();
	}

private:
	TIM_HandleTypeDef *htim;
	uint32_t channel;

	bool active = false;
};

#endif /* PRINTHEAD_H_ */
