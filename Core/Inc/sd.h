/**
 ******************************************************************************
 * @file           : sd.h
 * @brief          :
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

#ifndef SD_H_
#define SD_H_

#include "stdint.h"
#include "stddef.h"
#include "fatfs.h"

#define BUFFER_SIZE 128

class SD {
public:
	TCHAR lineBuffer[BUFFER_SIZE];

	void init();
	size_t getBufferSize();
	void clearBuffer();
	FRESULT getResult();
	bool getFilesInDir(DIR *dir, const TCHAR *dirPath, FILINFO *fNameStorage,
			uint8_t maxFiles);
	void openFile(const TCHAR *path);
	void openDIR(DIR *dir, const TCHAR *dirPath);
	void closeCurrentDir();
	void closeCurrentFile();
	bool readNextLine();

private:
	FATFS fs;  // file system
	DIR dir;
	FIL fil; // File
	FILINFO fno;
	FRESULT fresult;  // result
	UINT br, bw;  // File read/write count

	/**** capacity related *****/
	FATFS *pfs;
	DWORD fre_clust;
	uint32_t total, free_space;

	void checkFreeSpace();
};

#endif /* SD_H_ */
