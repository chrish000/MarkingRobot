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
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

  /* Private includes ----------------------------------------------------------*/
  /* USER CODE BEGIN Includes */

  /* USER CODE END Includes */

  /* Exported types ------------------------------------------------------------*/
  /* USER CODE BEGIN ET */
  extern CRC_HandleTypeDef hcrc;
  /* USER CODE END ET */

  /* Exported constants --------------------------------------------------------*/
  /* USER CODE BEGIN EC */
  extern volatile float PosX; // in mm
  extern volatile float PosY; // in mm

  extern volatile uint32_t PWMStepX; // Zähler für die X PWM-Impulse
  extern volatile uint16_t PWMCounterX;
  extern volatile uint32_t TargetStepsX;

  extern volatile uint8_t PWMEnabledX; // Variable zum Ein-/Ausschalten der X-PWM
  /* USER CODE END EC */

  /* Exported macro ------------------------------------------------------------*/
  /* USER CODE BEGIN EM */

  /* USER CODE END EM */

  /* Exported functions prototypes ---------------------------------------------*/
  void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Z_STEP_Pin GPIO_PIN_2
#define Z_STEP_GPIO_Port GPIOE
#define Z_DIR_Pin GPIO_PIN_3
#define Z_DIR_GPIO_Port GPIOE
#define PWRDET_Pin GPIO_PIN_15
#define PWRDET_GPIO_Port GPIOC
#define PWRDET_EXTI_IRQn EXTI15_10_IRQn
#define Z_MIN_Pin GPIO_PIN_0
#define Z_MIN_GPIO_Port GPIOC
#define X_MIN_Pin GPIO_PIN_1
#define X_MIN_GPIO_Port GPIOC
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
#define Z_EN_Pin GPIO_PIN_0
#define Z_EN_GPIO_Port GPIOE
#define Z_UART_Pin GPIO_PIN_1
#define Z_UART_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
#define MICROSTEPS 256
#define STEPS_PER_MM 181.952  // 5.684 alt		// "/(256/MICROSTEPS)"
#define STEPS_PER_DEG 825.666 // 25.793 alt
#define RUN_CURRENT_DEFAULT 200
#define HOLD_CURRENT_DEFAULT 500
  /* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
