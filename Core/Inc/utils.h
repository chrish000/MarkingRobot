/**
 ******************************************************************************
 * @file           : utils.h
 * @brief          :
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
#ifndef UTILS_H
#define UTILS_H

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
inline int normalize(int x, int in_min, int in_max, int out_min, int out_max) {
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

// Compute the linear interpolation between two real numbers.
static inline float interp(const float a, const float b, const float t) { return (1 - t) * a + t * b; }

/**
 * @brief  Computing a Bézier-Curve.
 * @param  a: starting point.
 * @param  b: first control point.
 * @param  b: second control point.
 * @param  c: last point.
 * @param  t: value for interpolating point on curve.
 * @retval float: computed value.
 */
static inline float eval_bezier(const float a, const float b, const float c, const float d, const float t) {
  const float iab = interp(a, b, t),
              ibc = interp(b, c, t),
              icd = interp(c, d, t),
              iabc = interp(iab, ibc, t),
              ibcd = interp(ibc, icd, t);
  return interp(iabc, ibcd, t);
}
#endif /* UTILS_H */
