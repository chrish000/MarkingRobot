/*
 * menue.h
 *
 *  Created on: May 12, 2025
 *      Author: chris
 */

#ifndef INC_LCD_MENUE_H_
#define INC_LCD_MENUE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "u8g2.h"

extern u8g2_t u8g2;

typedef enum : uint8_t{
	logo = 0,
	lesen_von_sd = 1,
	duese_reinigen = 2,
	reinigung_zurueck = 3,
	reinigung_starten = 4,
	reinigung_laeuft = 5,
	sd_nicht_erkannt = 6,
	sd_warten = 7,
	sd_datei_0 = 10,
	sd_datei_1 = 11,
	sd_datei_2 = 12,
	sd_datei_3 = 13,
	datei_auswaehlen_zurueck = 20,
	datei_auswaehlen_starten = 21,
	roboter_ausrichten_zurueck = 22,
	roboter_ausrichten_starten = 23,
	markieren_laeuft = 30,
	markieren_beendet = 31,
	druck_gering_abbrechen = 32,
	druck_gering_starten = 33,
	referenzieren_gescheitert_abbrechen = 34,
	referenzieren_gescheitert_starten = 35,
	druck_abbrechen_bestaetigen_zurueck = 36,
	druck_abbrechen_bestaetigen_starten = 37,
	referenzieren_abbrechen_bestaetigen_zurueck = 38,
	referenzieren_abbrechen_bestaetigen_starten = 39,
	akku_leer = 40,
} screen;

typedef enum : int8_t {
	undefined = 0, prev = -1, next = 1, selected = 3
} menuDir;

extern menuDir menuIndex;
extern screen activeScreen;

void MX_U8G2_Init(void);
uint8_t u8x8_gpio_and_delay_stm32(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
		void *arg_ptr);

void DisplayRoutine(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_LCD_MENUE_H_ */
