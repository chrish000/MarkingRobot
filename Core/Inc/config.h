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
#define MM_PER_STEP (1.0 / STEPS_PER_MM)
#define STEPS_PER_DEG 413

#define DEFAULT_SPEED 1000 // in mm/s
#define MAX_SPEED 1100
#define DEFAULT_ACCEL 1000
#define MAX_ACCEL 2000

//#define DIST_TILL_NEW_HOMING 0xffffffff //mm

#define RUN_CURRENT_DEFAULT 2000
#define HOLD_CURRENT_DEFAULT 100

// #define REVERSE_BOTH_MOTOR_DIRECTION
//#define INVERT_MOTOR_X_DIR //Auskommentiern um zu aktivieren
//#define INVERT_MOTOR_Y_DIR //Auskommentiern um zu aktivieren
#define MOTOR_XY_RATIO 10014 //10014 = 100.14% (<100 -> mehr links; >100 -> mehr rechts)

// Beschleunigungskurve: {Trapezoid; Bezier}
#define ACCEL_CURVE_TRAPEZOID
// #define ACCEL_CURVE_SINUS
// #define ACCEL_CURVE_BEZIER

/**
 *	###########################################################################
 *	MARKIEREINHEIT
 *	###########################################################################
 */
#define PRINTHEAD_PERIOD 500    // ms
#define PRINTHEAD_DUTY_CYCLE 10 //%

#define CLEAN_PULSETIME 300 //ms
#define CLEAN_PULSEWIDTH 80 //%
#define CLEAN_TOTAL_TIME 10 //s

/**
 *	###########################################################################
 *	HOMING
 *	###########################################################################
 */
#define MAX_HOMING_DIST 500     //mm
#define DIST_BETWEEN_PROBING 20 //mm
#define HOMING_MAX_FAULT 0.02   //deg
#define MAX_HOMING_TRY 10
#define MAX_HOMING_TIMEOUT 30000 //ms

#define HOMING_SPEED_PROBING 40 //mm/s
#define HOMING_SPEED_MOVING 150 //mm/s
#define HOMING_ACCEL 75         //mm/s^2

#define SENSOR_DIST 356     //mm
#define NULLPUNKT_OFFSET_X 140 //mm	(von Roboter Rahmen aussen hinten zu Duese)
#define NULLPUNKT_OFFSET_Y 298 //mm	(von Roboter Antrieb aussen seitlich zu Duese)
#define NULLPUNKT_WINKELABSTAND 35 //mm (Abstand von Zeichungs-Nullpunkt und Winkel)
#define MOVE_TO_HOME_BEFORE_HOMING	//Auskommentiern um zu aktivieren

/**
 *	###########################################################################
 *	AKKU
 *	###########################################################################
 */
#define ADC_MAX 65535.0f
#define V_REF 3.3f
#define VOLTAGE_DIVIDER_RATIO 7.6862f
#define MIN_BAT_VOLTAGE 22.2f
#define MAX_BAT_VOLTAGE 25.2f

#endif /* CONFIG_H */
