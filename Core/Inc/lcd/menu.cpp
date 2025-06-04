/*
 * menue.c
 *
 *  Created on: May 12, 2025
 *      Author: chris
 */

#include "main.h"
#include "menu.h"
#include "move.h"
#include "sd.h"
//#include "homing.h"
#include "lcd_bitmaps.h"
#include <cstring>

#define LCD_CS_PIN LCD_CS_Pin
#define LCD_CS_PORT LCD_CS_GPIO_Port
#define LCD_DATA_PIN LCD_EN_Pin
#define LCD_DATA_PORT LCD_EN_GPIO_Port
#define LCD_SCK_PIN LCD_D4_Pin
#define LCD_SCK_PORT LCD_D4_GPIO_Port

#define DISPLAY_LOGO_TIME 1000 //ms

#define FILE_NAME_BUFFER_SIZE 4
#define MAX_FILE_NAME_LENGTH 20

u8g2_t u8g2;
Screen activeScreen = logo;
Screen lastScreen = activeScreen;
uint32_t screenTimer = 0;
volatile MenuEnc menuIndex = undefined;

DIR dir;
FILINFO fileNameBuf[FILE_NAME_BUFFER_SIZE] = { 0 };
uint8_t selectedFile = 0;

void vorgangAbbrechen() {
	robi.sd.closeCurrentFile();
	robi.printhead.stop();
	robi.printingFlag = false;
	robi.motorMaster.moveBuf.consumerClear();
	robi.motorMaster.posBuf.consumerClear();
	robi.motorMaster.motorX.stepBuf.consumerClear();
	robi.motorMaster.motorY.stepBuf.consumerClear();
	robi.motorMaster.motorX.tmc.disable();
	robi.motorMaster.motorY.tmc.disable();
	distSequence = 0;
	homingSequence = 0;
	airSequence = 0;
	readFromSD = true;
	homingRoutine = false;
	robi.isHomedFlag = false;
	PressureAlarm = false;
	robi.totalDistSinceHoming = 0;
}

void UpdateStatusBarFast() {
	static float lastVoltage = -1.0f;
	static uint8_t lastPercentage = 255;
	static uint8_t lastPressureState = 1;

	float voltage = robi.batteryVoltage;
	uint8_t percentage = robi.batteryPercentage;
	uint8_t pressureState = lowPressure;

	// Nur aktualisieren, wenn sich Werte geändert haben
	if (voltage == lastVoltage && percentage == lastPercentage
			&& pressureState == lastPressureState)
		return;

	// Bildschirmbereich löschen (nur den relevanten Teil!)
	//u8g2_SetDrawColor(&u8g2, 0);  // Hintergrundfarbe (löschen)
	//u8g2_DrawBox(&u8g2, 0, 10, 23, 35);
	//u8g2_SetDrawColor(&u8g2, 1);  // Vordergrundfarbe (schreiben)

	// Akku-Spannung
	char buffer[5];
	snprintf(buffer, sizeof(buffer), "%2.1f", voltage);
	u8g2_DrawStr(&u8g2, 0, 20, buffer);

	// Ladezustand
	snprintf(buffer, sizeof(buffer), "%2d", percentage);
	u8g2_DrawStr(&u8g2, 0, 30, buffer);

	// Druckstatus
	u8g2_DrawStr(&u8g2, 99, 20, (pressureState == 1) ? "n.OK" : "OK  ");

	u8g2_SendBuffer(&u8g2);

	// Werte speichern für nächsten Vergleich
	lastVoltage = voltage;
	lastPercentage = percentage;
	lastPressureState = pressureState;
}

void inline StatusScreen() {
	char buffer[7];

	u8g2_DrawStr(&u8g2, 0, 10, "Akku");
	snprintf(buffer, sizeof(buffer), "%2.1f V", robi.batteryVoltage);
	u8g2_DrawStr(&u8g2, 0, 20, buffer);
	snprintf(buffer, sizeof(buffer), "%2d %%", robi.batteryPercentage);
	u8g2_DrawStr(&u8g2, 0, 30, buffer);

	u8g2_DrawStr(&u8g2, 99, 10, "Druck");
	if (HAL_GPIO_ReadPin(PRESSURE_PORT, PRESSURE_PIN) == GPIO_PIN_RESET)
		u8g2_DrawStr(&u8g2, 99, 20, "n.OK");
	else
		u8g2_DrawStr(&u8g2, 99, 20, "OK");
}

void drawFileName(u8g2_t *u8g2, int x, int y, const char *str, int n) {
	char buffer[n + 1];  // +1 für Nullterminator

	int len = strlen(str);
	if (len <= n) {
		// Passt komplett, einfach kopieren
		strncpy(buffer, str, n);
		buffer[len] = '\0';
	} else {
		// Kürzen und durch '...' ersetzen
		if (n >= 3) {
			strncpy(buffer, str, n - 3);
			buffer[n - 3] = '.';
			buffer[n - 2] = '.';
			buffer[n - 1] = '.';
			buffer[n] = '\0';
		} else {
			// Weniger als 3 Zeichen verfügbar – nur Punkte
			for (int i = 0; i < n; i++)
				buffer[i] = '.';
			buffer[n] = '\0';
		}
	}

	u8g2_DrawStr(u8g2, x, y, buffer);
}

void DisplayRoutine() {
	u8g2_ClearBuffer(&u8g2);
	lastScreen = activeScreen;
	switch (activeScreen) {
	case logo:
		u8g2_DrawXBM(&u8g2, 0, 0, SHL_Logo_width, SHL_Logo_height,
				SHL_Logo_bits);
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
		u8g2_DrawStr(&u8g2, 2, 48, "Lesen von SD");
		u8g2_DrawStr(&u8g2, 2, 59, "Duese reinigen");
		u8g2_DrawFrame(&u8g2, 0, 37, 95, 14);
		if (menuIndex == selected) {
			activeScreen = sd_warten;
			menuIndex = undefined;
		} else if (menuIndex == next) {
			activeScreen = duese_reinigen;
			menuIndex = undefined;
		}
		break;
	case duese_reinigen:
		// Code für den Bildschirm "Düse reinigen"
		StatusScreen();
		u8g2_DrawStr(&u8g2, 2, 48, "Lesen von SD");
		u8g2_DrawStr(&u8g2, 2, 59, "Duese reinigen");
		u8g2_DrawFrame(&u8g2, 0, 50, 95, 14);
		if (menuIndex == selected) {
			activeScreen = reinigung_zurueck;
			menuIndex = undefined;
		} else if (menuIndex == prev) {
			activeScreen = lesen_von_sd;
			menuIndex = undefined;
		}
		break;
	case reinigung_zurueck:
		// Code für die Rückkehr vom Bildschirm "Reinigung"
		u8g2_DrawStr(&u8g2, 10, 20, "Reinigungsprogramm");
		u8g2_DrawStr(&u8g2, 2, 61, "zurueck");
		u8g2_DrawFrame(&u8g2, 0, 52, 45, 12);
		u8g2_DrawStr(&u8g2, 85, 61, "starten");
		if (menuIndex == selected) {
			activeScreen = lesen_von_sd;
			menuIndex = undefined;
		} else if (menuIndex == next) {
			activeScreen = reinigung_starten;
			menuIndex = undefined;
		}
		break;
	case reinigung_starten:
		// Code zum Starten der Reinigung
		u8g2_DrawStr(&u8g2, 10, 20, "Reinigungsprogramm");
		u8g2_DrawStr(&u8g2, 2, 61, "zurueck");
		u8g2_DrawStr(&u8g2, 85, 61, "starten");
		u8g2_DrawFrame(&u8g2, 83, 52, 45, 12);
		if (menuIndex == selected) {
			activeScreen = reinigung_laeuft;
			menuIndex = undefined;
		} else if (menuIndex == prev) {
			activeScreen = reinigung_zurueck;
			menuIndex = undefined;
		}
		break;
	case reinigung_laeuft:
		// Code für den Bildschirm "Reinigung läuft"
		u8g2_DrawStr(&u8g2, 15, 10, "Reinigung laeuft");
		u8g2_DrawXBM(&u8g2, 50, 20, Uhr_width, Uhr_height, Uhr_bits);
		u8g2_DrawStr(&u8g2, 2, 61, "abbrechen");
		u8g2_DrawFrame(&u8g2, 0, 52, 57, 12);
		if (robi.printhead.clean()) {
			activeScreen = reinigung_starten;
			menuIndex = undefined;
		}
		if (menuIndex == selected) {
			robi.printhead.stop();
			activeScreen = reinigung_zurueck;
			menuIndex = undefined;
		}

		break;
	case sd_nicht_erkannt:
		// Code für den Bildschirm "SD-Karte nicht erkannt"
		u8g2_DrawStr(&u8g2, 20, 20, "SD nicht erkannt");
		u8g2_DrawStr(&u8g2, 2, 61, "zurueck");
		u8g2_DrawFrame(&u8g2, 0, 52, 45, 12);
		if (menuIndex == selected) {
			activeScreen = lesen_von_sd;
			menuIndex = undefined;
		}
		break;
	case sd_warten:
		u8g2_DrawXBM(&u8g2, 50, 20, Uhr_width, Uhr_height, Uhr_bits);
		u8g2_SendBuffer(&u8g2);  //Hier direkt schicken für korrekte Anzeige
		if (!HAL_GPIO_ReadPin(SD_DET_GPIO_Port, SD_DET_Pin)) {
			if (!robi.sd.initFlag)
				robi.sd.init();
			if (robi.sd.getFilesInDir(&dir, "/", fileNameBuf,
			FILE_NAME_BUFFER_SIZE) == false) {
				activeScreen = sd_nicht_erkannt;
			} else
				activeScreen = sd_zurueck;
		} else {
			activeScreen = sd_nicht_erkannt;
		}
		menuIndex = undefined;
		break;
	case sd_zurueck: {
		u8g2_DrawStr(&u8g2, 2, 10, "< zurueck");
		int y = 24;
		for (int i = 0; i < FILE_NAME_BUFFER_SIZE; ++i) {
			drawFileName(&u8g2, 2, y, fileNameBuf[i].fname,
			MAX_FILE_NAME_LENGTH);
			y += 14;
		}
		u8g2_DrawFrame(&u8g2, 0, 0, 128, 14);
		if (menuIndex == selected) {
			activeScreen = lesen_von_sd;
			menuIndex = undefined;
		} else if (menuIndex == next && fileNameBuf[0].fname[0] != '\0') {
			activeScreen = sd_datei_0;
			menuIndex = undefined;
		}
		break;
	}
	case sd_datei_0: {
		// Code für die Anzeige der ersten SD-Datei
		u8g2_DrawStr(&u8g2, 2, 10, "< zurueck");
		int y = 24;
		for (int i = 0; i < FILE_NAME_BUFFER_SIZE; ++i) {
			drawFileName(&u8g2, 2, y, fileNameBuf[i].fname,
			MAX_FILE_NAME_LENGTH);
			y += 14;
		}
		u8g2_DrawFrame(&u8g2, 0, 14, 128, 14);
		if (menuIndex == selected) {
			selectedFile = 0;
			activeScreen = datei_auswaehlen_zurueck;
			menuIndex = undefined;
		} else if (menuIndex == next && fileNameBuf[1].fname[0] != '\0') {
			activeScreen = sd_datei_1;
			menuIndex = undefined;
		} else if (menuIndex == prev) {
			activeScreen = sd_zurueck;
			menuIndex = undefined;
		}
		break;
	}
	case sd_datei_1: {
		// Code für die Anzeige der zweiten SD-Datei
		u8g2_DrawStr(&u8g2, 2, 10, "< zurueck");
		int y = 24;
		for (int i = 0; i < FILE_NAME_BUFFER_SIZE; ++i) {
			drawFileName(&u8g2, 2, y, fileNameBuf[i].fname,
			MAX_FILE_NAME_LENGTH);
			y += 14;
		}
		u8g2_DrawFrame(&u8g2, 0, 28, 128, 14);
		if (menuIndex == selected) {
			selectedFile = 1;
			activeScreen = datei_auswaehlen_zurueck;
			menuIndex = undefined;
		} else if (menuIndex == next && fileNameBuf[2].fname[0] != '\0') {
			activeScreen = sd_datei_2;
			menuIndex = undefined;
		} else if (menuIndex == prev) {
			activeScreen = sd_datei_0;
			menuIndex = undefined;
		}
		break;
	}
	case sd_datei_2: {
		// Code für die Anzeige der dritten SD-Datei
		u8g2_DrawStr(&u8g2, 2, 10, "< zurueck");
		int y = 24;
		for (int i = 0; i < FILE_NAME_BUFFER_SIZE; ++i) {
			drawFileName(&u8g2, 2, y, fileNameBuf[i].fname,
			MAX_FILE_NAME_LENGTH);
			y += 14;
		}
		u8g2_DrawFrame(&u8g2, 0, 42, 128, 14);

		if (menuIndex == selected) {
			selectedFile = 2;
			activeScreen = datei_auswaehlen_zurueck;
			menuIndex = undefined;
		} else if (menuIndex == next && fileNameBuf[3].fname[0] != '\0') {
			activeScreen = sd_datei_3;
			menuIndex = undefined;
		} else if (menuIndex == prev) {
			activeScreen = sd_datei_1;
			menuIndex = undefined;
		}
		break;
	}
	case sd_datei_3: {
		// Code für die Anzeige der vierten SD-Datei
		u8g2_DrawStr(&u8g2, 2, 10, "< zurueck");
		int y = 24;
		for (int i = 0; i < FILE_NAME_BUFFER_SIZE; ++i) {
			drawFileName(&u8g2, 2, y, fileNameBuf[i].fname,
			MAX_FILE_NAME_LENGTH);
			y += 14;
		}
		u8g2_DrawFrame(&u8g2, 0, 56, 128, 14);
		if (menuIndex == selected) {
			selectedFile = 3;
			activeScreen = datei_auswaehlen_zurueck;
			menuIndex = undefined;
		} else if (menuIndex == prev) {
			activeScreen = sd_datei_2;
			menuIndex = undefined;
		}
		break;
	}
	case datei_auswaehlen_zurueck:
		// Code für die Rückkehr vom Bildschirm "Datei auswählen"
		u8g2_DrawStr(&u8g2, 0, 10, "Ausgewaehlt:");
		drawFileName(&u8g2, 0, 24, fileNameBuf[selectedFile].fname,
		MAX_FILE_NAME_LENGTH);
		u8g2_DrawStr(&u8g2, 2, 61, "zurueck");
		u8g2_DrawFrame(&u8g2, 0, 52, 45, 12);
		u8g2_DrawStr(&u8g2, 85, 61, "starten");
		if (menuIndex == selected) {
			activeScreen = sd_datei_0;
			menuIndex = undefined;
		} else if (menuIndex == next) {
			activeScreen = datei_auswaehlen_starten;
			menuIndex = undefined;
		}
		break;
	case datei_auswaehlen_starten:
		// Code zum Starten der Dateiauswahl
		u8g2_DrawStr(&u8g2, 0, 10, "Ausgewaehlt:");
		drawFileName(&u8g2, 0, 24, fileNameBuf[selectedFile].fname,
		MAX_FILE_NAME_LENGTH);
		u8g2_DrawStr(&u8g2, 2, 61, "zurueck");
		u8g2_DrawStr(&u8g2, 85, 61, "starten");
		u8g2_DrawFrame(&u8g2, 83, 52, 45, 12);
		if (menuIndex == selected) {
			activeScreen = roboter_ausrichten_zurueck;
			menuIndex = undefined;
		} else if (menuIndex == prev) {
			activeScreen = datei_auswaehlen_zurueck;
			menuIndex = undefined;
		}
		break;
	case roboter_ausrichten_zurueck:
		// Code für die Rückkehr vom Bildschirm "Roboter ausrichten"
		u8g2_DrawStr(&u8g2, 10, 10, "Roboter ausrichten");
		u8g2_DrawXBM(&u8g2, 50, 14, Referenzieren_width, Referenzieren_height,
				Referenzieren_bits);
		u8g2_DrawStr(&u8g2, 2, 61, "zurueck");
		u8g2_DrawFrame(&u8g2, 0, 52, 45, 12);
		u8g2_DrawStr(&u8g2, 85, 61, "starten");
		if (menuIndex == selected) {
			activeScreen = sd_datei_0;
			menuIndex = undefined;
		} else if (menuIndex == next) {
			activeScreen = roboter_ausrichten_starten;
			menuIndex = undefined;
		}
		break;
	case roboter_ausrichten_starten:
		// Code zum Starten des Ausrichtens des Roboters
		u8g2_DrawStr(&u8g2, 10, 10, "Roboter ausrichten");
		u8g2_DrawXBM(&u8g2, 50, 14, Referenzieren_width, Referenzieren_height,
				Referenzieren_bits);
		u8g2_DrawStr(&u8g2, 2, 61, "zurueck");
		u8g2_DrawStr(&u8g2, 85, 61, "starten");
		u8g2_DrawFrame(&u8g2, 83, 52, 45, 12);
		if (menuIndex == selected) {
			robi.sd.openFile(fileNameBuf[selectedFile].fname);
			robi.printingFlag = true;
			robi.isHomedFlag = false;
			homingFailed = false;
			activeScreen = markieren_laeuft;
			menuIndex = undefined;
		} else if (menuIndex == prev) {
			activeScreen = roboter_ausrichten_zurueck;
			menuIndex = undefined;
		}
		break;
	case markieren_laeuft:
		// Code für den Bildschirm "Markieren läuft"
		StatusScreen();
		u8g2_DrawXBM(&u8g2, 50, 20, Uhr_width, Uhr_height, Uhr_bits);
		u8g2_DrawStr(&u8g2, 15, 61, "Markieren laeuft");
		break;
	case markieren_beendet:
		// Code für den Bildschirm "Markieren beendet"
		StatusScreen();
		u8g2_DrawXBM(&u8g2, 50, 5, Flagge_width, Flagge_height, Flagge_bits);
		u8g2_DrawStr(&u8g2, 15, 48, "Markieren beendet");
		u8g2_DrawStr(&u8g2, 85, 61, "weiter");
		u8g2_DrawFrame(&u8g2, 83, 52, 39, 12);
		if (menuIndex == selected) {
			activeScreen = lesen_von_sd;
			menuIndex = undefined;
		}
		break;
	case druck_gering_abbrechen:
		// Code zum Abbrechen bei geringem Druck
		if (HAL_GPIO_ReadPin(PRESSURE_PORT, PRESSURE_PIN) == GPIO_PIN_RESET) {
			u8g2_DrawStr(&u8g2, 0, 10, "Druck zu gering");
			u8g2_DrawStr(&u8g2, 0, 24, "Auftanken!");
			u8g2_DrawStr(&u8g2, 76, 40, "n.OK");
		} else {
			u8g2_DrawStr(&u8g2, 0, 10, "Druck OK");
			u8g2_DrawStr(&u8g2, 0, 24, "Fortfahren?");
			u8g2_DrawStr(&u8g2, 76, 40, "OK");
		}
		u8g2_DrawStr(&u8g2, 40, 40, "Druck:");

		u8g2_DrawStr(&u8g2, 85, 61, "starten");
		u8g2_DrawStr(&u8g2, 2, 61, "abbrechen");
		u8g2_DrawFrame(&u8g2, 0, 52, 57, 12);
		if (menuIndex == selected) {
			activeScreen = druck_abbrechen_bestaetigen_zurueck;
			menuIndex = undefined;
		} else if (menuIndex == next) {
			activeScreen = druck_gering_starten;
			menuIndex = undefined;
		}
		break;
	case druck_gering_starten:
		// Code zum Starten bei geringem Druck
		if (HAL_GPIO_ReadPin(PRESSURE_PORT, PRESSURE_PIN) == GPIO_PIN_RESET) {
			u8g2_DrawStr(&u8g2, 0, 10, "Druck zu gering");
			u8g2_DrawStr(&u8g2, 0, 24, "Auftanken!");
			u8g2_DrawStr(&u8g2, 76, 40, "n.OK");
		} else {
			u8g2_DrawStr(&u8g2, 0, 10, "Druck OK");
			u8g2_DrawStr(&u8g2, 0, 24, "Fortfahren?");
			u8g2_DrawStr(&u8g2, 76, 40, "OK");
		}
		u8g2_DrawStr(&u8g2, 40, 40, "Druck:");

		u8g2_DrawStr(&u8g2, 85, 61, "starten");
		u8g2_DrawFrame(&u8g2, 83, 52, 45, 12);
		u8g2_DrawStr(&u8g2, 2, 61, "abbrechen");
		if (menuIndex == selected) {
			if (!lowPressure) {
				activeScreen = markieren_laeuft;
			}
			menuIndex = undefined;
		} else if (menuIndex == prev) {
			activeScreen = druck_gering_abbrechen;
			menuIndex = undefined;
		}
		break;
	case druck_abbrechen_bestaetigen_zurueck:
		u8g2_DrawStr(&u8g2, 10, 25, "Vorgang abbrechen?");
		u8g2_DrawStr(&u8g2, 2, 61, "zurueck");
		u8g2_DrawFrame(&u8g2, 0, 52, 45, 12);
		u8g2_DrawStr(&u8g2, 73, 61, "abbrechen");
		if (menuIndex == selected) {
			activeScreen = druck_gering_abbrechen;
			menuIndex = undefined;
		} else if (menuIndex == next) {
			activeScreen = druck_abbrechen_bestaetigen_starten;
			menuIndex = undefined;
		}
		break;
	case druck_abbrechen_bestaetigen_starten:
		u8g2_DrawStr(&u8g2, 10, 25, "Vorgang abbrechen?");
		u8g2_DrawStr(&u8g2, 2, 61, "zurueck");
		u8g2_DrawStr(&u8g2, 73, 61, "abbrechen");
		u8g2_DrawFrame(&u8g2, 71, 52, 57, 12);
		if (menuIndex == selected) {
			vorgangAbbrechen();
			activeScreen = lesen_von_sd;
			menuIndex = undefined;
		} else if (menuIndex == prev) {
			activeScreen = druck_abbrechen_bestaetigen_zurueck;
			menuIndex = undefined;
		}
		break;
	case referenzieren_gescheitert_abbrechen:
		// Code zum Abbrechen, wenn das Referenzieren fehlgeschlagen ist
		u8g2_DrawStr(&u8g2, 5, 10, "Fehler Referenzieren");
		u8g2_DrawStr(&u8g2, 15, 30, "Erneut versuchen?");
		u8g2_DrawStr(&u8g2, 85, 61, "starten");
		u8g2_DrawStr(&u8g2, 2, 61, "abbrechen");
		u8g2_DrawFrame(&u8g2, 0, 52, 57, 12);
		if (menuIndex == selected) {
			activeScreen = referenzieren_abbrechen_bestaetigen_zurueck;
			menuIndex = undefined;
		} else if (menuIndex == next) {
			activeScreen = referenzieren_gescheitert_starten;
			menuIndex = undefined;
		}
		break;
	case referenzieren_gescheitert_starten:
		// Code zum Starten des Referenzierens nach einem Fehler
		u8g2_DrawStr(&u8g2, 5, 10, "Fehler Referenzieren");
		u8g2_DrawStr(&u8g2, 15, 30, "Erneut versuchen?");
		u8g2_DrawStr(&u8g2, 85, 61, "starten");
		u8g2_DrawFrame(&u8g2, 83, 52, 45, 12);
		u8g2_DrawStr(&u8g2, 2, 61, "abbrechen");
		if (menuIndex == selected) {
			activeScreen = roboter_ausrichten_zurueck;
			menuIndex = undefined;
		} else if (menuIndex == prev) {
			activeScreen = referenzieren_gescheitert_abbrechen;
			menuIndex = undefined;
		}
		break;
	case referenzieren_abbrechen_bestaetigen_zurueck:
		u8g2_DrawStr(&u8g2, 10, 25, "Vorgang abbrechen?");
		u8g2_DrawStr(&u8g2, 2, 61, "zurueck");
		u8g2_DrawFrame(&u8g2, 0, 52, 45, 12);
		u8g2_DrawStr(&u8g2, 73, 61, "abbrechen");
		if (menuIndex == selected) {
			activeScreen = referenzieren_gescheitert_abbrechen;
			menuIndex = undefined;
		} else if (menuIndex == next) {
			activeScreen = referenzieren_abbrechen_bestaetigen_starten;
			menuIndex = undefined;
		}
		break;
	case referenzieren_abbrechen_bestaetigen_starten:
		u8g2_DrawStr(&u8g2, 10, 25, "Vorgang abbrechen?");
		u8g2_DrawStr(&u8g2, 2, 61, "zurueck");
		u8g2_DrawStr(&u8g2, 73, 61, "abbrechen");
		u8g2_DrawFrame(&u8g2, 71, 52, 57, 12);
		if (menuIndex == selected) {
			vorgangAbbrechen();
			activeScreen = lesen_von_sd;
			menuIndex = undefined;
		} else if (menuIndex == prev) {
			activeScreen = referenzieren_abbrechen_bestaetigen_zurueck;
			menuIndex = undefined;
		}
		break;
	case akku_leer:
		// Code für den Bildschirm "Akku leer"
		u8g2_DrawStr(&u8g2, 35, 10, "Akku leer");
		u8g2_DrawXBM(&u8g2, 35, 25, Batterie_width, Batterie_height,
				Batterie_bits);
		break;
	default:
		// Optional: Code für den Fall, dass aktiveScreen einen ungültigen Wert hat
		u8g2_DrawStr(&u8g2, 0, 10, "ERROR");
		break;
	}

	u8g2_SendBuffer(&u8g2);

	HAL_NVIC_EnableIRQ(LCD_BTN_EXTI);

	if (activeScreen != lastScreen)
		DisplayRoutine();
}

uint8_t u8x8_gpio_and_delay_stm32(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
		void *arg_ptr) {
	switch (msg) {

	case U8X8_MSG_DELAY_MILLI:
		HAL_Delay(arg_int);
		break;

	case U8X8_MSG_DELAY_NANO:
		__NOP();
		break;

	case U8X8_MSG_DELAY_100NANO:
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		break;

	case U8X8_MSG_GPIO_CS:
		if (arg_int)
			LCD_CS_PORT->BSRR = LCD_CS_PIN;
		else
			LCD_CS_PORT->BSRR = (LCD_CS_PIN << 16);
		break;

	case U8X8_MSG_GPIO_SPI_DATA:
		if (arg_int)
			LCD_DATA_PORT->BSRR = LCD_DATA_PIN;
		else
			LCD_DATA_PORT->BSRR = (LCD_DATA_PIN << 16);
		break;

	case U8X8_MSG_GPIO_SPI_CLOCK:
		if (arg_int)
			LCD_SCK_PORT->BSRR = LCD_SCK_PIN;
		else
			LCD_SCK_PORT->BSRR = (LCD_SCK_PIN << 16);
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
