/**
 ******************************************************************************
 * @file           : config.h
 * @brief          : Enthält alle Kofigurationsparameter
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

#ifndef CONFIG_H
#define CONFIG_H

/**
 *	###########################################################################
 *	BEWEGUNG
 *	###########################################################################
 */
#define MICROSTEPS 128
#define STEPS_PER_MM 92.847
#define MM_PER_STEP (1.0/STEPS_PER_MM)
#define STEPS_PER_DEG 413.49

#define DEFAULT_SPEED 1200 //in mm/s
#define MAX_SPEED 1200
#define DEFAULT_ACCEL 500
#define MAX_ACCEL 3000


#define RUN_CURRENT_DEFAULT 500
#define HOLD_CURRENT_DEFAULT 100

//#define REVERSE_BOTH_MOTOR_DIRECTION
#define Inverse_Motor_X_Dir false
#define Inverse_Motor_Y_Dir false
#define MOTOR_XY_RATIO 100.14f	//% (<100 -> mehr links; >100 -> mehr rechts)

//Beschleunigungskurve: {Trapezoid; Bezier}
#define ACCEL_CURVE_TRAPEZOID
//#define ACCEL_CURVE_SINUS
//#define ACCEL_CURVE_BEZIER

/**
 *	###########################################################################
 *	MARKIEREINHEIT
 *	###########################################################################
 */
#define PRINTHEAD_PERIOD 500 //ms
#define PRINTHEAD_DUTY_CYCLE 10 //%

#endif /* CONFIG_H */
