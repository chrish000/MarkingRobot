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
#ifndef UTILS_H
#define UTILS_H

/* Exported functions prototypes ---------------------------------------------*/

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
