/*
 * pins.c
 *
 *  Created on: May 12, 2025
 *      Author: chris
 */

#include "pins.h"

//X
TIM_HandleTypeDef *TIM_Motor_X = &htim23;
DMA_HandleTypeDef *TIM_DMA_ARR_X = &hdma_tim23_ch1;
DMA_HandleTypeDef *TIM_DMA_BSRR_X = &hdma_tim23_ch2;
UART_HandleTypeDef *TMC_UART_address_X = &huart2;
CRC_HandleTypeDef *CRC_Handle_X = &hcrc;

// Y
TIM_HandleTypeDef *TIM_Motor_Y = &htim24;
DMA_HandleTypeDef *TIM_DMA_ARR_Y = &hdma_tim24_ch1;
DMA_HandleTypeDef *TIM_DMA_BSRR_Y = &hdma_tim24_ch2;
UART_HandleTypeDef *TMC_UART_address_Y = &huart8;
CRC_HandleTypeDef *CRC_Handle_Y = &hcrc;

//PRINTHEAD
TIM_HandleTypeDef *TIM_Printhead = &htim3;

//HE0
TIM_HandleTypeDef *TIM_HE0 = &htim2;

// BATTERY
ADC_HandleTypeDef *ADC_Handle = &hadc1;
TIM_HandleTypeDef *ADC_TIM = &htim8;

//BUZZER
TIM_HandleTypeDef *TIM_BUZZER = &htim4;
