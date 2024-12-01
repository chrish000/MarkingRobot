/**
 ******************************************************************************
 * @file           : utils.c
 * @brief          : Utility functions
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
/* Includes ------------------------------------------------------------------*/
#include "utils.h"


static inline int constrain(int value, int min, int max) {
	if (value < min)
		return min;
	if (value > max)
		return max;
	return value;
}

static inline int map(int x, int in_min, int in_max, int out_min, int out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static inline unsigned int reverseData(unsigned int data, unsigned char data_size) {
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
