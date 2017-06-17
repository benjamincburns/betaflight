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

#include "common/maths.h"
#include "flight/stewart_platform.h"

void setRotation(fp_angles_t, stewartPlatform_s*);
void getPlatformJoints(stewartPlatform_s*);

void calcLegLength(stewartPlatform_s*);
void calcAlpha(stewartPlatform_s*);
void calcServoPos(float[6], stewartPlatform_s*);

static struct fp_vector vecOrigin = { 0, 0, 0 };

void resetStewartPlatform(stewartPlatform_s* platform) {
    platform->platformJoint = { { } }
    platform->legLength = { { } }
    platform->translation = { }
    platform->rotation = { }
    platform->alpha = { }
    platform->beta = { BETA_ANGLES }

    baseJoint[0].X = -PLATFORM_BASE_RADIUS * cos_approx(DEGREES_TO_RADIANS(30) - THETA_ANGLE);
    baseJoint[1].X = -PLATFORM_BASE_RADIUS * cos_approx(DEGREES_TO_RADIANS(30) - THETA_ANGLE);
    baseJoint[2].X =  PLATFORM_BASE_RADIUS * sin_approx(THETA_ANGLE);
    baseJoint[3].X =  PLATFORM_BASE_RADIUS * cos_approx(DEGREES_TO_RADIANS(30) + THETA_ANGLE);
    baseJoint[4].X =  PLATFORM_BASE_RADIUS * cos_approx(DEGREES_TO_RADIANS(30) + THETA_ANGLE);
    baseJoint[5].X =  PLATFORM_BASE_RADIUS * sin_approx(THETA_ANGLE);

    baseJoint[0].Y = -PLATFORM_BASE_RADIUS * sin_approx(DEGREES_TO_RADIANS(30) - THETA_ANGLE);
    baseJoint[1].Y =  PLATFORM_BASE_RADIUS * sin_approx(DEGREES_TO_RADIANS(30) - THETA_ANGLE);
    baseJoint[2].Y =  PLATFORM_BASE_RADIUS * cos_approx(THETA_ANGLE);
    baseJoint[3].Y =  PLATFORM_BASE_RADIUS * sin_approx(DEGREES_TO_RADIANS(30) + THETA_ANGLE);
    baseJoint[4].Y = -PLATFORM_BASE_RADIUS * sin_approx(DEGREES_TO_RADIANS(30) + THETA_ANGLE);
    baseJoint[5].Y = -PLATFORM_BASE_RADIUS * cos_approx(THETA_ANGLE);

    platformJoint[0].X = -PLATFORM_TOP_RADIUS * sin_approx(DEGREES_TO_RADIANS(30) + THETA_R_ANGLE / 2);
    platformJoint[1].X = -PLATFORM_TOP_RADIUS * sin_approx(DEGREES_TO_RADIANS(30) + THETA_R_ANGLE / 2);
    platformJoint[2].X = -PLATFORM_TOP_RADIUS * sin_approx(DEGREES_TO_RADIANS(30) - THETA_R_ANGLE / 2);
    platformJoint[3].X =  PLATFORM_TOP_RADIUS * cos_approx(THETA_R_ANGLE / 2);
    platformJoint[4].X =  PLATFORM_TOP_RADIUS * cos_approx(THETA_R_ANGLE / 2);
    platformJoint[5].X = -PLATFORM_TOP_RADIUS * sin_approx(DEGREES_TO_RADIANS(30) - THETA_R_ANGLE / 2);

    platformJoint[0].Y = -PLATFORM_TOP_RADIUS * cos_approx(DEGREES_TO_RADIANS(30) + THETA_R_ANGLE / 2);
    platformJoint[1].Y =  PLATFORM_TOP_RADIUS * cos_approx(DEGREES_TO_RADIANS(30) + THETA_R_ANGLE / 2);
    platformJoint[2].Y =  PLATFORM_TOP_RADIUS * cos_approx(DEGREES_TO_RADIANS(30) - THETA_R_ANGLE / 2);
    platformJoint[3].Y =  PLATFORM_TOP_RADIUS * sin_approx(THETA_R_ANGLE / 2);
    platformJoint[4].Y = -PLATFORM_TOP_RADIUS * sin_approx(THETA_R_ANGLE / 2);
    platformJoint[5].Y = -PLATFORM_TOP_RADIUS * cos_approx(DEGREES_TO_RADIANS(30) - THETA_R_ANGLE / 2);
}

void calcLegLength(stewartPlatform_t* platform) {
    float rotMatrix[3][3] = {};

    buildRotationMatrix(platform->rotation, rotMatrix);

    for (int i = 0; i < 6; i++) {
        platform->legLength[i].X = (rotMatrix[0][0] * platformJoint[i].X) + (rotMatrix[0][1] * platformJoint[i].Y) + (rotMatrix[0][2] * platformJoint[i].Z);
        platform->legLength[i].Y = (rotMatrix[1][0] * platformJoint[i].X) + (rotMatrix[1][1] * platformJoint[i].Y) + (rotMatrix[1][2] * platformJoint[i].Z);
        platform->legLength[i].Z = (rotMatrix[2][0] * platformJoint[i].X) + (rotMatrix[2][1] * platformJoint[i].Y) + (rotMatrix[2][2] * platformJoint[i].Z);

        platform->legLength[i].X += translation.X;
        platform->legLength[i].Y += translation.Y;
        platform->legLength[i].Z += translation.Z;
    }
}

void calcAlpha(stewartPlatform_t* platform) {
    struct fp_vector basePoint, Li;
    double min, max, dist;

    for (int i = 0; i < 6; i++) {
        min = SERVO_MIN;
        max = SERVO_MAX;
        for (int j = 0; j < 20; j++){
            basePoint.X = LENGTH_SERVO_ARM * cos_approx(alpha[i]) * cos_approx(beta[i]) + baseJoint[i].X;
            basePoint.Y = LENGTH_SERVO_ARM * cos_approx(alpha[i]) * sin_approx(beta[i]) + baseJoint[i].Y;
            basePoint.Z = LENGTH_SERVO_ARM * sin_approx(alpha[i]);

            Li.X = legLength[i].X - basePoint.X;
            Li.Y = legLength[i].Y - basePoint.Y;
            Li.Z = legLength[i].Z - basePoint.Z;

            dist = sqrt(Li.X * Li.X + Li.Y * Li.Y + Li.Z * Li.Z);

            if (abs(LENGTH_SERVO_LEG - dist) < 0.01) {
                break;
            }

            if (dist < LENGTH_SERVO_LEG) {
                max = alpha[i];
            }
            else {
                min = alpha[i];
            }
            if (max == SERVO_MIN || min == SERVO_MAX) {
                break;
            }

            alpha[i] = min + (max - min) / 2;
        }
    }
}

void getServoPosition(float[6] input, stewartPlatform_t* platform) {

    // future work: full 6DOF attitude w/ washout filter
    platform->translation = { input[INPUT_RC_AUX1], input[INPUT_RC_AUX2], input[INPUT_RC_THROTTLE] }; // for now...

    platform->rotation.angles = {
        input[INPUT_STABILIZED_ROLL],
        input[INPUT_STABILIZED_PITCH],
        input[INPUT_STABILIZED_YAW]
    };

    calcLegLength(platform);
    calcAlpha(platform);

    for (int i = 0; i < 6; i++) {
        input[INPUT_STEWART_PLATFORM_1 + i] = constrain(servoHorPos[i] + (alpha[i]) *  SERVO_MULT , -500, 500);
    }
}

