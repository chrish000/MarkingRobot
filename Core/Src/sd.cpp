#include "sd.h"
#include "string.h"
#include "stdio.h"

/* PUBLIC */
/**
 * @brief
 * @param None
 * @retval None
 */
void SD::init() {
	fresult = f_mount(&fs, "/", 1);
	checkFreeSpace();
}

/**
 * @brief
 * @param size_t: Length of Buffer
 * @retval None
 */
size_t SD::getBufferSize() {
	return strnlen(lineBuffer, BUFFER_SIZE);
}

/**
 * @brief
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
 * @brief
 * @param
 * @retval bool: true wenn erfolgreich, false sonst
 */
bool SD::getFilesInDir(DIR *dir, const TCHAR *dirPath, FILINFO *fNameStorage,
		uint8_t maxFiles) {
	if (f_opendir(dir, dirPath) != FR_OK)
		return false;
	// if(f_readdir(dir, fNameStorage) != FR_OK) return false; //Erster Name ist "Sysem Volume Information"
	for (int i = 0; i < maxFiles; ++i) {
		if (f_readdir(dir, fNameStorage) != FR_OK)
			return false;
		if (fNameStorage->fname[0] == 0)
			return false;
		fNameStorage++;
	}
	fresult = f_closedir(dir);
	return true;
}

/**
 * @brief
 * @param const TCHAR*: Poniter zu String des Dateipfadnamens
 * @retval None
 */
void SD::openFile(const TCHAR *path) {
	fresult = f_open(&fil, path, FA_OPEN_EXISTING | FA_READ);
}

/**
 * @brief
 * @param DIR*: Pointer zu Verzeichniss
 * @param const TCHAR*: Poniter zu String des Dateipfadnamens
 * @retval None
 */
void SD::openDIR(DIR *dir, const TCHAR *dirPath) {
	fresult = f_opendir(dir, dirPath);
}

/**
 * @brief
 * @param None
 * @retval None
 */
void SD::closeCurrentDir() {
	assert(&dir != nullptr && "dir-ptr == nullptr");
	fresult = f_closedir(&dir);
}

/**
 * @brief
 * @param None
 * @retval None
 */
void SD::closeCurrentFile() {
	assert(&fil != nullptr && "file-ptr == nullptr");
	fresult = f_close(&fil);
}

/**
 * @brief
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
 * @brief
 * @param None
 * @retval None
 */
void SD::checkFreeSpace() {
	f_getfree("", &fre_clust, &pfs);
	total = (uint32_t) ((pfs->n_fatent - 2) * pfs->csize * 0.5);
	free_space = (uint32_t) (fre_clust * pfs->csize * 0.5);
}
