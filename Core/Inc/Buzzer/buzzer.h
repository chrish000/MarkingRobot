#include "main.h"

#ifndef BUZZER_H
#define BUZZER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	uint16_t pitch;
	uint16_t duration;
} note_t;

typedef struct {
	TIM_HandleTypeDef *timer;
	__IO uint32_t channel;
	uint32_t timerClockFreqHz;
} Buzzer_InitTypeDef;

typedef struct {
	Buzzer_InitTypeDef Init;
	uint8_t value;
	uint16_t blinkFreqHz;
	uint16_t clock;
	uint16_t clockPeriod;
} Buzzer_HandleTypeDef;

void Buzzer_Play_Song_Blocking(Buzzer_HandleTypeDef *handle, note_t* song, size_t songSize, uint8_t BPM);
void Buzzer_Play_Blocking(Buzzer_HandleTypeDef *handle, uint32_t noteFreq,
		uint8_t duration, uint8_t BPM);
void Buzzer_Play_Song(Buzzer_HandleTypeDef *handle, note_t* song, size_t songSize, uint8_t BPM);
void Buzzer_Play(Buzzer_HandleTypeDef *handle, uint32_t noteFreq,
		uint8_t duration, uint8_t BPM);
void Buzzer_Note(Buzzer_HandleTypeDef *handle, uint32_t noteFreq);
void Buzzer_NoNote(Buzzer_HandleTypeDef *handle);
void Buzzer_Init(Buzzer_HandleTypeDef *handle, Buzzer_InitTypeDef *config);
void Buzzer_Start(Buzzer_HandleTypeDef *handle);
void Buzzer_Stop(Buzzer_HandleTypeDef *handle);

#ifdef __cplusplus
}
#endif

#endif /* BUZZER_H */
