/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "common/maths.h"
#include "flight/servos.h"

/* Position of servos mounted in inverse direction */
#define INVERSE_SERVO_1 0
#define INVERSE_SERVO_2 2
#define INVERSE_SERVO_3 4

/* Multiplier used to convert radian to +500:-500 range required by servo mixer */
#define SERVO_MULT 318.30988618379067153776752674503

/* The max e min range of Servos in radians */
#define SERVO_MIN DEGREES_TO_RADIANS(-80)
#define SERVO_MAX DEGREES_TO_RADIANS(80)

/* Here you should put Your Platform Values in millimeters */
/* Here you put the length of your servos arm . */
#define LENGTH_SERVO_ARM 15

/* Here you put the length of your rods length. */
#define LENGTH_SERVO_LEG 120

/* Here you put the default Height of your platform. 
 *  * This value should be close to yours rods length.
 *  */
#define PLATFORM_HEIGHT_DEFAULT 115

/* Here you put the radius of the top of your platform. */
#define PLATFORM_TOP_RADIUS 45.5

/* Here you put the radius of the base of your platform. */
#define PLATFORM_BASE_RADIUS 67.5

/* Here you put the angle between two servos axis points */
#define THETA_P_ANGLE DEGREES_TO_RADIANS(67)

/* Here you put the angle between two platform attachment points */
#define THETA_R_ANGLE DEGREES_TO_RADIANS(12)

/* Here you dont need to change*/
#define THETA_ANGLE ((M_PIf/3.0 - THETA_P_ANGLE) / 2.0)

/* Here you put the pulses of each of yours servos respective to their position in horizontal */
#define SERVO_ZERO_POSITION 1550, 1450, 1350, 1600, 1500, 1600

/* Here you put the rotation of servo arms in respect to axis X */
#define BETA_ANGLES M_PIf / 2, -M_PIf / 2, -M_PIf / 6, 5 * M_PIf / 6, -5 * M_PIf / 6, M_PIf / 6

typedef struct stewartPlatform_s {
    double beta[6];
    float alpha[6]; 
    struct fp_vector baseJoint[6], platformJoint[6], legLength[6];
    struct fp_vector translation;
    fp_angles_t rotation;

} stewartPlatform_t;

void resetStewartPlatform(stewartPlatform_t*);
void getStewartPlatformServoInputs(float[6] input, const struct fp_vector, stewartPlatform_t* );

