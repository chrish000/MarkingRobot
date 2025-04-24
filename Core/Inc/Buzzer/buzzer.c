#include "buzzer.h"

void Buzzer_Play(Buzzer_HandleTypeDef *handle, uint32_t noteFreq,
		uint8_t duration, uint8_t BPM) {
	Buzzer_Note(handle, noteFreq);
	HAL_Delay(60000 / BPM * 4 / duration);
}

void Buzzer_Note(Buzzer_HandleTypeDef *handle, uint32_t noteFreq) {
	if (noteFreq > 0) {
		handle->Init.timer->Instance->ARR = handle->Init.timerClockFreqHz
				/ (handle->Init.timer->Init.Prescaler + 1) / noteFreq / 2;
	} else
		handle->Init.timer->Instance->ARR = 0;
}

void Buzzer_NoNote(Buzzer_HandleTypeDef *handle) {
	Buzzer_Note(handle, 0);
}

void Buzzer_Init(Buzzer_HandleTypeDef *handle, Buzzer_InitTypeDef *config) {
	handle->Init = *config;
}

void Buzzer_Start(Buzzer_HandleTypeDef *handle) {
	Buzzer_NoNote(handle);
	HAL_TIM_Base_Start_IT(handle->Init.timer);
}

void Buzzer_Stop(Buzzer_HandleTypeDef *handle) {
	Buzzer_NoNote(handle);
	HAL_TIM_Base_Stop_IT(handle->Init.timer);
}
