/*
 * attitude_calc.h
 *
 *  Created on: Nov 19, 2019
 *      Author: bocaim
 */

#ifndef INC_ATTITUDE_CALC_H_
#define INC_ATTITUDE_CALC_H_

/*===== Structure to hold the data =====*/
typedef struct {
	float ANGLE_RELATIVE_TO_EARTH_X[5]; 	/* This angle is the inclination positive or negative with respect to the calibration axis */
	float ANGLE_RELATIVE_TO_EARTH_Y[5];	/* This angle is the inclination positive or negative with respect to the calibration axis */
	float ANGLE_RELATIVE_TO_EARTH_Z[5];	/* This angle is the inclination positive or negative with respect to the calibration axis */
} gyro_data_t;

typedef struct {
	float GET_GYRO_RAW_X_ANGLE_VEL[5];
	float GET_GYRO_RAW_Y_ANGLE_VEL[5];
	float GET_GYRO_RAW_Z_ANGLE_VEL[5];
}gyro_raw_data_t;

typedef struct {
	float ACCELERATIO_AXIS_X[5];	/* This acceleration is measured in G's */
	float ACCELERATIO_AXIS_Y[5];	/* This acceleration is measured in G's */
	float ACCELERATIO_AXIS_Z[5];	/* This acceleration is measured in G's */
}acc_data_t;

extern float actualRollAngle;
extern float actualPitchAngle;
extern float actualRawYawRate;

extern void calcStateMachine();
extern void calcActualAngle(void);

#endif /* INC_ATTITUDE_CALC_H_ */
