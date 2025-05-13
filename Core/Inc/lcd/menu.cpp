/*
 * menue.c
 *
 *  Created on: May 12, 2025
 *      Author: chris
 */

#include "main.h"
#include "menu.h"
#include "move.h"
#include "lcd_bitmaps.h"

#define LCD_CS_PIN LCD_CS_Pin
#define LCD_CS_PORT LCD_CS_GPIO_Port
#define LCD_DATA_PIN LCD_EN_Pin
#define LCD_DATA_PORT LCD_EN_GPIO_Port
#define LCD_SCK_PIN LCD_D4_Pin
#define LCD_SCK_PORT LCD_D4_GPIO_Port

#define DISPLAY_LOGO_TIME 1000 //ms

//extern Robot robi;
u8g2_t u8g2;
screen activeScreen = logo;
uint32_t screenTimer = 0;
menuDir menuIndex = undefined;

void inline StatusScreen() {
	char buffer[7];

	u8g2_DrawStr(&u8g2, 0, 10, "Akku");
	snprintf(buffer, sizeof(buffer), "%.1f V", robi.batteryVoltage);
	u8g2_DrawStr(&u8g2, 0, 20, buffer);
	snprintf(buffer, sizeof(buffer), "%i %%", robi.batteryPercentage);
	u8g2_DrawStr(&u8g2, 0, 30, buffer);

	u8g2_DrawStr(&u8g2, 99, 10, "Druck");
	if (HAL_GPIO_ReadPin(PRESSURE_PORT, PRESSURE_PIN)
						== GPIO_PIN_RESET)
		u8g2_DrawStr(&u8g2, 99, 20, "n.OK");
	else
		u8g2_DrawStr(&u8g2, 99, 20, "OK");
}

void DisplayRoutine() {
	u8g2_ClearBuffer(&u8g2);

	switch (activeScreen) {
	case logo:
		u8g2_DrawXBM(&u8g2, 0, 0, SHL_Logo_width, SHL_Logo_height, SHL_Logo_bits);
		if (screenTimer == 0)
			screenTimer = HAL_GetTick();
		if (HAL_GetTick() > screenTimer + DISPLAY_LOGO_TIME) {
			activeScreen = lesen_von_sd;
			screenTimer = 0;
		}
		break;
	case lesen_von_sd:
		// Code für den Bildschirm "Lesen von SD-Karte"
		StatusScreen();
		//TODO
		break;
	case duese_reinigen:
		// Code für den Bildschirm "Düse reinigen"
		break;
	case reinigung_zurueck:
		// Code für die Rückkehr vom Bildschirm "Reinigung"
		break;
	case reinigung_starten:
		// Code zum Starten der Reinigung
		break;
	case reinigung_laeuft:
		// Code für den Bildschirm "Reinigung läuft"
		u8g2_DrawXBM(&u8g2, 0, 0, Uhr_width, Uhr_height,
				Uhr_bits);
		break;
	case sd_nicht_erkannt:
		// Code für den Bildschirm "SD-Karte nicht erkannt"
		break;
	case sd_datei_1:
		// Code für die Anzeige der ersten SD-Datei
		break;
	case sd_datei_2:
		// Code für die Anzeige der zweiten SD-Datei
		break;
	case sd_datei_3:
		// Code für die Anzeige der dritten SD-Datei
		break;
	case sd_datei_4:
		// Code für die Anzeige der vierten SD-Datei
		break;
	case sd_datei_5:
		// Code für die Anzeige der fünften SD-Datei
		break;
	case datei_auswaehlen_zurueck:
		// Code für die Rückkehr vom Bildschirm "Datei auswählen"
		break;
	case datei_auswaehlen_starten:
		// Code zum Starten der Dateiauswahl
		break;
	case roboter_ausrichten_zurueck:
		// Code für die Rückkehr vom Bildschirm "Roboter ausrichten"
		u8g2_DrawXBM(&u8g2, 0, 0, Referenzieren_width, Referenzieren_height,
				Referenzieren_bits);
		break;
	case roboter_ausrichten_starten:
		// Code zum Starten des Ausrichtens des Roboters
		u8g2_DrawXBM(&u8g2, 0, 0, Referenzieren_width, Referenzieren_height,
				Referenzieren_bits);
		break;
	case markieren_laeuft:
		// Code für den Bildschirm "Markieren läuft"
		u8g2_DrawXBM(&u8g2, 0, 0, Uhr_width, Uhr_height,
				Uhr_bits);
		break;
	case markieren_beendet:
		// Code für den Bildschirm "Markieren beendet"
		u8g2_DrawXBM(&u8g2, 0, 0, Flagge_width, Flagge_height,
				Flagge_bits);
		break;
	case druck_gering_abbrechen:
		// Code zum Abbrechen bei geringem Druck
		break;
	case druck_gering_starten:
		// Code zum Starten bei geringem Druck
		break;
	case referenzieren_gescheitert_abbrechen:
		// Code zum Abbrechen, wenn das Referenzieren fehlgeschlagen ist
		break;
	case referenzieren_gescheitert_starten:
		// Code zum Starten des Referenzierens nach einem Fehler
		break;
	case akku_leer:
		// Code für den Bildschirm "Akku leer"
		u8g2_DrawXBM(&u8g2, 0, 0, Batterie_width, Batterie_height,
				Batterie_bits);
		break;
	default:
		// Optional: Code für den Fall, dass aktiveScreen einen ungültigen Wert hat
		break;
	}

	u8g2_SendBuffer(&u8g2);
}

uint8_t u8x8_gpio_and_delay_stm32(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
		void *arg_ptr) {
	switch (msg) {
	case U8X8_MSG_DELAY_MILLI:
		HAL_Delay(arg_int);
		break;

	case U8X8_MSG_DELAY_NANO:			// delay arg_int * 1 nano second
		__NOP();
		break;

	case U8X8_MSG_DELAY_100NANO:		// delay arg_int * 100 nano seconds
		for (volatile int i = 0; i < 20; i++) {
			__NOP();
		}
		break;

	case U8X8_MSG_GPIO_CS:
		HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_Pin, (GPIO_PinState) arg_int);
		break;

	case U8X8_MSG_GPIO_SPI_DATA:
		HAL_GPIO_WritePin(LCD_DATA_PORT, LCD_DATA_PIN, (GPIO_PinState) arg_int);
		break;

	case U8X8_MSG_GPIO_SPI_CLOCK:
		HAL_GPIO_WritePin(LCD_SCK_PORT, LCD_SCK_PIN, (GPIO_PinState) arg_int);
		break;

	default:
		return 0;
	}
	return 1;
}

void MX_U8G2_Init(void) {
	u8g2_Setup_st7920_s_128x64_f(&u8g2,
	U8G2_R0, u8x8_byte_4wire_sw_spi, u8x8_gpio_and_delay_stm32);

	u8g2_InitDisplay(&u8g2);
	u8g2_SetPowerSave(&u8g2, 0);

	//User Interface
	u8g2_ClearDisplay(&u8g2);
	u8g2_ClearBuffer(&u8g2);
	u8g2_SetFont(&u8g2, u8g2_font_6x12_mr);
}
