/**
 ******************************************************************************
 * @file           : utils.h
 * @brief          : Header for utils.c file.
 *                   This file contains the common defines of the application.
 * @author		   : Chris Hauser
 ******************************************************************************
 * @attention
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UTILS_H
#define __UTILS_H

/* Exported functions prototypes ---------------------------------------------*/
/**
 * @brief  Maps a value from one range to another range.
 * @param  x: The value to be mapped.
 * @param  in_min: The minimum value of the input range.
 * @param  in_max: The maximum value of the input range.
 * @param  out_min: The minimum value of the output range.
 * @param  out_max: The maximum value of the output range.
 * @retval int: The mapped value in the output range.
 */
inline int map(int x, int in_min, int in_max, int out_min, int out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * @brief  Reverses the byte order of a given 32-bit data word.
 * @param  data: The 32-bit data word whose byte order is to be reversed.
 * @param  data_size: The number of bytes in the data word.
 * @retval unsigned int: The 32-bit data word with reversed byte order.
 */
inline unsigned int reverseData(unsigned int data, unsigned char data_size) {
	const unsigned char BITS_PER_BYTE = 8;
	const unsigned char BYTE_MAX_VALUE = 0xFF;
	unsigned int reversed_data = 0;
	unsigned char right_shift;
	unsigned char left_shift;
	for (unsigned char i = 0; i < data_size; ++i) {
		right_shift = (data_size - i - 1) * BITS_PER_BYTE;
		left_shift = i * BITS_PER_BYTE;
		reversed_data |= ((data >> right_shift) & BYTE_MAX_VALUE) << left_shift;
	}
	return reversed_data;
}

#endif /* __UTILS_H */

