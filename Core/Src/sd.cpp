#include "sd.h"
#include "string.h"
#include "stdio.h"

/* PUBLIC */
/**
 * @brief Initialisierungsfunktion
 * @param None
 * @retval None
 */
void SD::init() {
	fresult = f_mount(&fs, "/", 1);
	checkFreeSpace();
	if (getResult() != FR_OK)
		initFlag = false;
	initFlag = true;
}

/**
 * @brief Gibt die Stringlänge des Lese-Puffers aus
 * @param None
 * @retval size_t: Length of Buffer
 */
size_t SD::getBufferSize() {
	return strnlen(lineBuffer, BUFFER_SIZE);
}

/**
 * @brief Leert den Lese-Üuffer
 * @param None
 * @retval None
 */
void SD::clearBuffer() {
	memset(lineBuffer, '\0', sizeof(lineBuffer));
}

/**
 * @brief
 * @param None
 * @retval FRESULT aktueller Ergebniszustand
 */
FRESULT SD::getResult() {
	return fresult;
}

/**
 * @brief Liest alle Dateien in Verzeichnis
 * @param dir Pointer auf Verzeichnis
 * @param dirPath String des Verzeichnispfads
 * @param fNameStorage Speicherplatz für gelesene Dateien
 * @param maxFiles Maximal zu lesende Dateien
 * @retval bool: true wenn erfolgreich, false sonst
 */
bool SD::getFilesInDir(DIR *dir, const TCHAR *dirPath, FILINFO *fNameStorage,
		uint8_t maxFiles) {
	if (f_opendir(dir, dirPath) != FR_OK)
		return false;

	FILINFO file;
	strncpy((char*)file.fname, "System Volume Information", sizeof(file.fname));

	for (int i = 0; i < maxFiles; ++i) {
		if (f_readdir(dir, fNameStorage) != FR_OK)
			return false;
		if (fNameStorage->fname[0] != 0 && strcmp((char*)fNameStorage->fname, (char*)file.fname) != 0)
			fNameStorage++;
	}
	fresult = f_closedir(dir);
	return true;
}

/**
 * @brief Öffnet Datei
 * @param const TCHAR*: Poniter zu String des Dateipfadnamens
 * @retval None
 */
void SD::openFile(const TCHAR *path) {
	fresult = f_open(&fil, path, FA_OPEN_EXISTING | FA_READ);
}

/**
 * @brief Öffnet Verzeichnis
 * @param DIR*: Pointer zu Verzeichniss
 * @param const TCHAR*: Poniter zu String des Dateipfadnamens
 * @retval None
 */
void SD::openDIR(DIR *dir, const TCHAR *dirPath) {
	fresult = f_opendir(dir, dirPath);
}

/**
 * @brief Schließt aktuelles Verzeichnis
 * @param None
 * @retval None
 */
void SD::closeCurrentDir() {
	assert(&dir != nullptr && "dir-ptr == nullptr");
	fresult = f_closedir(&dir);
}

/**
 * @brief Schließt aktuell geöffnete Datei
 * @param None
 * @retval None
 */
void SD::closeCurrentFile() {
	assert(&fil != nullptr && "file-ptr == nullptr");
	fresult = f_close(&fil);
}

/**
 * @brief Liest die nächste Zeile in der Datei
 * @param None
 * @retval None
 */
bool SD::readNextLine() {
	assert(&fil != nullptr && "file-ptr == nullptr");
	memset(lineBuffer, '\0', sizeof(lineBuffer));
	return f_gets(lineBuffer, BUFFER_SIZE, &fil);
}

/* PRIVATE */
/**
 * @brief Liest den gesamten und freien Speicherplatz auf der SD
 * @param None
 * @retval None
 */
void SD::checkFreeSpace() {
	f_getfree("", &fre_clust, &pfs);
	total = (uint32_t) ((pfs->n_fatent - 2) * pfs->csize * 0.5);
	free_space = (uint32_t) (fre_clust * pfs->csize * 0.5);
}
