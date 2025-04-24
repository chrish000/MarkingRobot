/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
	extern ADC_HandleTypeDef hadc1;
	extern CRC_HandleTypeDef hcrc;

	extern TIM_HandleTypeDef htim2;
	extern TIM_HandleTypeDef htim3;
	extern TIM_HandleTypeDef htim23;
	extern TIM_HandleTypeDef htim24;
	extern DMA_HandleTypeDef hdma_tim23_ch1;
	extern DMA_HandleTypeDef hdma_tim23_ch2;
	extern DMA_HandleTypeDef hdma_tim24_ch1;
	extern DMA_HandleTypeDef hdma_tim24_ch2;

	extern UART_HandleTypeDef huart8;
	extern UART_HandleTypeDef huart2;
	extern TIM_HandleTypeDef htim8;

	typedef enum
	{
		NONE = 0x00,
		LOW_VOLTAGE = 0x01,
		MOVE_BUF = 0x10,
		STEP_BUF = 0x11
	} ERROR_HandleCode;
	extern ERROR_HandleCode ErrorCode;
	extern uint8_t printFlag;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void DMA_Callback(DMA_HandleTypeDef *hdma);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Z_STEP_Pin GPIO_PIN_2
#define Z_STEP_GPIO_Port GPIOE
#define Z_DIR_Pin GPIO_PIN_3
#define Z_DIR_GPIO_Port GPIOE
#define PWRDET_Pin GPIO_PIN_15
#define PWRDET_GPIO_Port GPIOC
#define PWRDET_EXTI_IRQn EXTI15_10_IRQn
#define Z_STOP_Pin GPIO_PIN_0
#define Z_STOP_GPIO_Port GPIOC
#define Z_STOP_EXTI_IRQn EXTI0_IRQn
#define X_STOP_Pin GPIO_PIN_1
#define X_STOP_GPIO_Port GPIOC
#define X_STOP_EXTI_IRQn EXTI1_IRQn
#define PRESSURE_Pin GPIO_PIN_2
#define PRESSURE_GPIO_Port GPIOC
#define PRESSURE_EXTI_IRQn EXTI2_IRQn
#define BAT_VOLTAGE_Pin GPIO_PIN_0
#define BAT_VOLTAGE_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_5
#define BUZZER_GPIO_Port GPIOC
#define X_DIR_Pin GPIO_PIN_3
#define X_DIR_GPIO_Port GPIOD
#define X_STEP_Pin GPIO_PIN_4
#define X_STEP_GPIO_Port GPIOD
#define X_UART_Pin GPIO_PIN_5
#define X_UART_GPIO_Port GPIOD
#define X_EN_Pin GPIO_PIN_6
#define X_EN_GPIO_Port GPIOD
#define HE0_PWM_Pin GPIO_PIN_3
#define HE0_PWM_GPIO_Port GPIOB
#define HE1_PWM_Pin GPIO_PIN_4
#define HE1_PWM_GPIO_Port GPIOB
#define FAN2_Pin GPIO_PIN_5
#define FAN2_GPIO_Port GPIOB
#define FAN1_Pin GPIO_PIN_6
#define FAN1_GPIO_Port GPIOB
#define FAN0_Pin GPIO_PIN_7
#define FAN0_GPIO_Port GPIOB
#define Z_EN_Pin GPIO_PIN_0
#define Z_EN_GPIO_Port GPIOE
#define Z_UART_Pin GPIO_PIN_1
#define Z_UART_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
