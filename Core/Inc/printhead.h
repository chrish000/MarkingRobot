/**
 ******************************************************************************
 * @file           : printhead.h
 * @brief          : Spritzapparatansteuerung
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

#ifndef PRINTHEAD_H_
#define PRINTHEAD_H_

#include "main.h"
#include "config.h"
#include <limits>
#include <algorithm>

#define CLEAN_PULSETIME 300 //ms
#define CLEAN_PULSEWIDTH 80 //%
#define CLEAN_TOTAL_TIME 10 //s

class Printhead {
public:
	Printhead(TIM_HandleTypeDef *htim, uint32_t timChannel) :
			htim(htim), channel(timChannel) {
	}

	struct param {
		uint16_t period = PRINTHEAD_PERIOD;
		uint8_t dutyCycle = PRINTHEAD_DUTY_CYCLE;
	} param;

	void setParam(uint16_t newPeriod = PRINTHEAD_PERIOD, uint8_t newDutyCycle =
	PRINTHEAD_DUTY_CYCLE) {
		param.period = newPeriod;
		param.dutyCycle = std::clamp(newDutyCycle, (uint8_t) 0, (uint8_t) 100);
		htim->Instance->ARR = std::clamp((uint16_t) (param.period * 10 - 1),
				std::numeric_limits<uint16_t>::min(),
				std::numeric_limits<uint16_t>::max());
		htim->Instance->CCR1 = std::clamp(
				(uint16_t) (param.dutyCycle * param.period * 10 / 100 - 1),
				std::numeric_limits<uint16_t>::min(),
				std::numeric_limits<uint16_t>::max());
	}
	void start() {
		HAL_TIM_PWM_Start(htim, channel);
		active = true;
	}
	void stop() {
		HAL_TIM_PWM_Stop(htim, channel);
		active = false;
		timerClean = 0;
	}
	bool isActive() {
		return active;
	}
	void init() {
		stop();
		setParam();
	}

	//return true wenn fertig, false sonst
	bool clean() {
		if (timerClean == 0) {
			timerClean = HAL_GetTick();
			setParam(CLEAN_PULSETIME, CLEAN_PULSEWIDTH);
			start();
		}
		if (HAL_GetTick() > timerClean + CLEAN_TOTAL_TIME * 1000) {
			stop();
			timerClean = 0;
			return true;
		}
		return false;
	}

private:
	TIM_HandleTypeDef *htim;
	uint32_t channel;

	bool active = false;
	uint32_t timerClean = 0;
};

#endif /* PRINTHEAD_H_ */
