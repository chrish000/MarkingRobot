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
 * @brief  Constrains the input value to be within a specified range.
 * @param  value: The value to be constrained.
 * @param  min: The minimum allowed value.
 * @param  max: The maximum allowed value.
 * @retval int: The constrained value.
 *             If 'value' is less than 'min', it returns 'min'.
 *             If 'value' is greater than 'max', it returns 'max'.
 *             Otherwise, it returns 'value'.
 */
static inline int constrain(int value, int min, int max);

/**
 * @brief  Maps a value from one range to another range.
 * @param  x: The value to be mapped.
 * @param  in_min: The minimum value of the input range.
 * @param  in_max: The maximum value of the input range.
 * @param  out_min: The minimum value of the output range.
 * @param  out_max: The maximum value of the output range.
 * @retval int: The mapped value in the output range.
 */
static inline int map(int x, int in_min, int in_max, int out_min, int out_max);

/**
 * @brief  Reverses the byte order of a given 32-bit data word.
 * @param  data: The 32-bit data word whose byte order is to be reversed.
 * @param  data_size: The number of bytes in the data word.
 * @retval unsigned int: The 32-bit data word with reversed byte order.
 */
static inline unsigned int reverseData(unsigned int data,
		unsigned char data_size);

#endif /* __UTILS_H */

