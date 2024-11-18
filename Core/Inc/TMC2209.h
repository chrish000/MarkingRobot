/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : TMC2209.h
 * @brief          : Header for TMC2209.cpp
 ******************************************************************************
 * @attention
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TMC2209_H
#define __TMC2209_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
//#include "stm32h7xx_hal.h"

/* Exported constants --------------------------------------------------------*/
/* Exported classes ----------------------------------------------------------*/
class TMC2209 {
public:
	UART_HandleTypeDef UART;
	GPIO_TypeDef HardwareEnablePort;
	uint16_t HardwareEnablePin;
	uint8_t StandstillMode;
};
/* Exported macro ------------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/
/* Private defines -----------------------------------------------------------*/

#endif /* __TMC2209_H */
