/**
 ******************************************************************************
 * @file           : stepper.cpp
 * @brief          : Motorsteuerung
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

/* Includes ------------------------------------------------------------------*/
#include "stepper.h"

// Konstruktor
StepperMotor::StepperMotor(TIM_HandleTypeDef *TIM_Motor,
		DMA_HandleTypeDef *TIM_DMA_ARR, DMA_HandleTypeDef *TIM_DMA_BSRR,
		GPIO_TypeDef *stepPort, uint16_t stepPin, GPIO_TypeDef *dirPort,
		uint16_t dirPin, UART_HandleTypeDef *TMC_UART_address,
		CRC_HandleTypeDef *TMC_CRC_Handle, GPIO_TypeDef *hardware_enable_port,
		uint16_t hardware_enable_pin) :

		tmc(TMC_UART_address, TMC_CRC_Handle, hardware_enable_port,
				hardware_enable_pin), TIM_Motor(TIM_Motor), TIM_DMA_ARR(
				TIM_DMA_ARR), TIM_DMA_BSRR(TIM_DMA_BSRR), stepPort(stepPort), stepPin(
				stepPin), dirPort(dirPort), dirPin(dirPin) {
}

// Destruktor
StepperMotor::~StepperMotor() {
	stopTimer();
}

 uint32_t data[2] = { X_STEP_Pin, X_STEP_Pin << 16 };
void StepperMotor::init() {
	stepBuf.consumerClear();

	TIM_Motor->Instance->ARR = timARRDefault;
	__HAL_TIM_ENABLE_DMA(TIM_Motor, TIM_DMA_CC1);
	__HAL_TIM_ENABLE_DMA(TIM_Motor, TIM_DMA_CC2);
	HAL_DMA_RegisterCallback(TIM_DMA_BSRR, HAL_DMA_XFER_CPLT_CB_ID, DMA_Callback);
}

/**
 * @brief Startet den Timer in der Klasse MotorManager mit dem Standardwert
 * @param None
 * @retval None
 */
void StepperMotor::startTimer() {
	assert(TIM_Motor != nullptr && "Timer Handle darf nicht NULL sein!");
	TIM_Motor->Instance->ARR = timARRDefault;
	stepBuf.remove(&StepCmdBuffer);
	HAL_DMA_Start(TIM_DMA_ARR, (uint32_t) &StepCmdBuffer.interval,
			(uint32_t) &TIM_Motor->Instance->ARR, 1);
	HAL_DMA_Start_IT(TIM_DMA_BSRR, (uint32_t) &StepCmdBuffer.gpioMask,
			(uint32_t) &stepPort->BSRR, 2);
	timerActiveFlag = true;
	HAL_TIM_Base_Start(TIM_Motor);
}

/**
 * @brief Stoppt den Timer in der Klasse MotorManager und setzt den Standardwert des Zeahlers
 * @param None
 * @retval None
 */
void StepperMotor::stopTimer() {
	assert(TIM_Motor != nullptr && "Timer Handle darf nicht NULL sein!");
	TIM_Motor->Instance->ARR = timARRDefault;
	timerActiveFlag = false;
	HAL_TIM_Base_Stop(TIM_Motor);
}

/**
 * @brief Gibt den aktuellen Status des Timers zurück
 * @param None
 * @retval HAL state
 */
HAL_TIM_StateTypeDef StepperMotor::getTimerState() {
	return HAL_TIM_Base_GetState(TIM_Motor);
}
