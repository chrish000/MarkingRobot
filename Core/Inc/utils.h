/*
 * utils.c
 *
 *  Created on: Nov 18, 2024
 *      Author: chris
 */

static inline int constrain(int value, int min, int max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

static inline int map(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

