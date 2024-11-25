/*
 * utils.c
 *
 *  Created on: Nov 18, 2024
 *      Author: chris
 */

/**
 * @brief  Constrains the input value to be within a specified range.
 * @param  value: The value to be constrained.
 * @param  min: The minimum allowed value.
 * @param  max: The maximum allowed value.
 * @retval int: The constrained value.
 *             If 'value' is less than 'min', it returns 'min'.
 *             If 'value' is greater than 'max', it returns 'max'.
 *             Otherwise, it returns 'value'.
 */
static inline int constrain(int value, int min, int max) {
	if (value < min)
		return min;
	if (value > max)
		return max;
	return value;
}

/**
 * @brief  Maps a value from one range to another range.
 * @param  x: The value to be mapped.
 * @param  in_min: The minimum value of the input range.
 * @param  in_max: The maximum value of the input range.
 * @param  out_min: The minimum value of the output range.
 * @param  out_max: The maximum value of the output range.
 * @retval int: The mapped value in the output range.
 */
static inline int map(int x, int in_min, int in_max, int out_min, int out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * @brief  Reverses the byte order of a given 32-bit data word.
 * @param  data: The 32-bit data word whose byte order is to be reversed.
 * @param  data_size: The number of bytes in the data word.
 * @retval unsigned int: The 32-bit data word with reversed byte order.
 */
static inline unsigned int reverseData(uint32_t data, uint8_t data_size) {
	const unsigned char BITS_PER_BYTE = 8;
	const unsigned char BYTE_MAX_VALUE = 0xFF;
	uint32_t reversed_data = 0;
	uint8_t right_shift;
	uint8_t left_shift;
	for (uint8_t i = 0; i < data_size; ++i) {
		right_shift = (data_size - i - 1) * BITS_PER_BYTE;
		left_shift = i * BITS_PER_BYTE;
		reversed_data |= ((data >> right_shift) & BYTE_MAX_VALUE) << left_shift;
	}
	return reversed_data;
}

