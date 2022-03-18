/*
 * sensors.h
 *
 *  Created on: Nov 18, 2019
 *      Author: bocaim
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

#include "attitude_calc.h"

#define GyroMaskXdataAvail 0x1
#define GyroMaskYdataAvail 0x2
#define GyroMaskZdataAvail 0x4

#define USE_SMOOTHING_FILTER 0

/*=== Initialization struct of the Gyroscope ===*/
extern unsigned char GYRO_COM_TEST_VAL;

extern unsigned char ACC_COM_TEST_VAL;

extern unsigned char GYRO_INIT_VAL[];

extern unsigned char GYRO_FIFO_INIT_VAL;


/*=== Structure where to put the read data from the gyro ===*/
typedef struct gyro_input_struct_tag
{
	unsigned char OUT_TEMP;
	unsigned char STATUS_REG;
	unsigned char OUT_X_L;
	unsigned char OUT_X_H;
	unsigned char OUT_Y_L;
	unsigned char OUT_Y_H;
	unsigned char OUT_Z_L;
	unsigned char OUT_Z_H;
} gyro_input_struct_t;

/*=== Structure where to put the transformed gyro data ===*/
typedef struct gyro_angular_data_tag
{
	float GYRO_RAW_X_ANGLE_VEL;
	float GYRO_RAW_Y_ANGLE_VEL;
	float GYRO_RAW_Z_ANGLE_VEL;
	float GYRO_X_ANGLE_TO_EARTH;
	float GYRO_Y_ANGLE_TO_EARTH;
	float GYRO_Z_ANGLE_TO_EARTH;
	unsigned char GYRO_NEW_DATA_AVAILABLE;
} gyro_angular_data_t;

typedef struct acc_input_struct_tag
{
	unsigned char OUT_X_L;
	unsigned char OUT_X_H;
	unsigned char OUT_Y_L;
	unsigned char OUT_Y_H;
	unsigned char OUT_Z_L;
	unsigned char OUT_Z_H;
} acc_input_struct_t;

typedef struct acc_angular_data_tag
{
	float ACC_RAW_X_ANGLE_VEL;
	float ACC_RAW_Y_ANGLE_VEL;
	float ACC_RAW_Z_ANGLE_VEL;
	float ACC_X_ANGLE_TO_EARTH;
	float ACC_Y_ANGLE_TO_EARTH;
	float ACC_Z_ANGLE_TO_EARTH;
	unsigned char ACC_NEW_DATA_AVAILABLE;
} acc_angular_data_t;

typedef struct mag_input_struct_tag
{
	unsigned char OUT_X_H;
	unsigned char OUT_X_L;
	unsigned char OUT_Z_H;
	unsigned char OUT_Z_L;
	unsigned char OUT_Y_H;
	unsigned char OUT_Y_L;
} mag_input_struct_t;

typedef struct	mag_data_struct_tag
{
	float MILIGAUSS_X_AXIS;
	float MILIGAUSS_Y_AXIS;
	float MILIGAUSS_Z_AXIS;
} mag_data_struct_t;

extern gyro_input_struct_t GYRO_RAW_INPUT_STRUCT;
extern acc_input_struct_t ACC_RAW_INPUT_STRUCT;

extern gyro_angular_data_t GYRO_DATA_STRUCT;
extern acc_angular_data_t ACC_DATA_STRUCT;

extern void gyroCalculateData(void);
extern void accCalculateData(void);
extern void magCalculateData(void);

extern unsigned char HAL_GET_GYRO_RAW_DATA_IF(gyro_raw_data_t* Buffer);
extern unsigned char HAL_GET_ACC_DATA_IF(acc_data_t* Buffer);

#endif /* INC_SENSORS_H_ */
