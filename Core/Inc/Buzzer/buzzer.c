#include "buzzer.h"

static note_t* _song = NULL;
static size_t _songSize = 0;
static size_t _currentIndex = 0;
static uint8_t _bpm = 120;
static uint8_t _isPlaying = 0;

void Buzzer_Play_Song_Blocking(Buzzer_HandleTypeDef *handle, note_t* song, size_t songSize, uint8_t BPM) {
	for (size_t i = 0; i < songSize; i++) {
		Buzzer_Play_Blocking(handle, song[i].pitch, song[i].duration, BPM);
	}
}

void Buzzer_Play_Blocking(Buzzer_HandleTypeDef *handle, uint32_t noteFreq,
		uint8_t duration, uint8_t BPM) {
	Buzzer_Note(handle, noteFreq);
	HAL_Delay(60000 / BPM * 4 / duration);
}

void Buzzer_Play_Song(Buzzer_HandleTypeDef *handle, note_t* song, size_t songSize, uint8_t BPM) {
	if (!_isPlaying) {
		_song = song;
		_songSize = songSize;
		_bpm = BPM;
		_currentIndex = 0;
		_isPlaying = 1;
		Buzzer_Play(handle, _song[_currentIndex].pitch, _song[_currentIndex].duration, _bpm);
	} else {
		uint32_t now = HAL_GetTick();
		if (now - handle->clock >= handle->clockPeriod) {
			_currentIndex++;
			if (_currentIndex >= _songSize) {
				_isPlaying = 0;
				Buzzer_NoNote(handle);
				return;
			}
			Buzzer_Play(handle, _song[_currentIndex].pitch, _song[_currentIndex].duration, _bpm);
		}
	}
}



void Buzzer_Play(Buzzer_HandleTypeDef *handle, uint32_t noteFreq,
		uint8_t duration, uint8_t BPM) {
	Buzzer_Note(handle, noteFreq);

	handle->clockPeriod = (60000 / BPM * 4 / duration);
	handle->clock = HAL_GetTick();
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
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
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
	_isPlaying = 0;
	HAL_TIM_Base_Stop_IT(handle->Init.timer);
}
