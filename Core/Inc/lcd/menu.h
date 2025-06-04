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

typedef enum {
	logo,
	lesen_von_sd,
	duese_reinigen,
	reinigung_zurueck,
	reinigung_starten,
	reinigung_laeuft,
	sd_nicht_erkannt,
	sd_warten,
	sd_datei_0,
	sd_datei_1,
	sd_datei_2,
	sd_datei_3,
	sd_zurueck,
	datei_auswaehlen_zurueck,
	datei_auswaehlen_starten,
	roboter_ausrichten_zurueck,
	roboter_ausrichten_starten,
	markieren_laeuft,
	markieren_beendet,
	druck_gering_abbrechen,
	druck_gering_starten,
	referenzieren_gescheitert_abbrechen,
	referenzieren_gescheitert_starten,
	druck_abbrechen_bestaetigen_zurueck,
	druck_abbrechen_bestaetigen_starten,
	referenzieren_abbrechen_bestaetigen_zurueck,
	referenzieren_abbrechen_bestaetigen_starten,
	akku_leer,
} Screen;

typedef enum {
	undefined, prev, next, selected
} MenuEnc;

extern volatile MenuEnc menuIndex;
extern Screen activeScreen;
extern Screen lastScreen;

void MX_U8G2_Init(void);
uint8_t u8x8_gpio_and_delay_stm32(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
		void *arg_ptr);
void UpdateStatusBarFast(void);
void DisplayRoutine(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_LCD_MENUE_H_ */
