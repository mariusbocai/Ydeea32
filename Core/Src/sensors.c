/*
 * sensors.c
 *
 *  Created on: Nov 18, 2019
 *      Author: bocaim
 */
#include "sensors.h"
#include "typeDefs.h"

gyro_angular_data_t GYRO_DATA_STRUCT;
acc_angular_data_t ACC_DATA_STRUCT;

unsigned int missX=0, missY=0, missZ=0;

/*=== Function to calculate the angular velocity and the angle of deviation from gyro raw data ===*/
void gyroCalculateData()
{
	signed short tempVar;

	/*=== If new X axis data is available, calculate the raw angular velocity ===*/
	if(GYRO_RAW_INPUT_STRUCT.STATUS_REG & GyroMaskXdataAvail)
	{
		tempVar = (signed short)((GYRO_RAW_INPUT_STRUCT.OUT_X_H<<8) | (GYRO_RAW_INPUT_STRUCT.OUT_X_L));
		GYRO_DATA_STRUCT.GYRO_RAW_X_ANGLE_VEL = (float)(((tempVar)*9)); //mili degrees per second
		GYRO_DATA_STRUCT.GYRO_X_ANGLE_TO_EARTH += (GYRO_DATA_STRUCT.GYRO_RAW_X_ANGLE_VEL/200000);
	}
	else
	{
		missX++;
	}
	if(GYRO_RAW_INPUT_STRUCT.STATUS_REG & GyroMaskYdataAvail)
	{
		tempVar = (signed short)((GYRO_RAW_INPUT_STRUCT.OUT_Y_H<<8) | (GYRO_RAW_INPUT_STRUCT.OUT_Y_L));
		GYRO_DATA_STRUCT.GYRO_RAW_Y_ANGLE_VEL = (float)(((tempVar)*9));
		GYRO_DATA_STRUCT.GYRO_Y_ANGLE_TO_EARTH += GYRO_DATA_STRUCT.GYRO_RAW_Y_ANGLE_VEL/200000;
	}
	else
	{
		missY++;
	}
	if(GYRO_RAW_INPUT_STRUCT.STATUS_REG & GyroMaskZdataAvail)
	{
		tempVar = (signed short)((GYRO_RAW_INPUT_STRUCT.OUT_Z_H<<8) | (GYRO_RAW_INPUT_STRUCT.OUT_Z_L));
		GYRO_DATA_STRUCT.GYRO_RAW_Z_ANGLE_VEL = (float)(((tempVar)*9));
		GYRO_DATA_STRUCT.GYRO_Z_ANGLE_TO_EARTH += GYRO_DATA_STRUCT.GYRO_RAW_Z_ANGLE_VEL/200000;
	}
	else
	{
		missZ++;
	}
}

void accCalculateData()
{
	signed short tempVar;

	tempVar = (signed short)((ACC_RAW_INPUT_STRUCT.OUT_X_H<<8) | (ACC_RAW_INPUT_STRUCT.OUT_X_L));
	ACC_DATA_STRUCT.ACC_RAW_X_ANGLE_VEL = (double)((((double)tempVar)/256)); /*g*/

	tempVar = (signed short)((ACC_RAW_INPUT_STRUCT.OUT_Y_H<<8) | (ACC_RAW_INPUT_STRUCT.OUT_Y_L));
	ACC_DATA_STRUCT.ACC_RAW_Y_ANGLE_VEL = (float)((((float)tempVar)/256)); /*g*/

	tempVar = (signed short)((ACC_RAW_INPUT_STRUCT.OUT_Z_H<<8) | (ACC_RAW_INPUT_STRUCT.OUT_Z_L));
	ACC_DATA_STRUCT.ACC_RAW_Z_ANGLE_VEL = (float)((((float)tempVar)/256)); /*g*/
}

unsigned char HAL_GET_GYRO_RAW_DATA_IF(gyro_raw_data_t* Buffer)
{
	unsigned char retValue;
#if (USE_SMOOTHING_FILTER == 1)
	unsigned char x;
	if(GyroRawQueueIndex == 5)
	{
		for(x = 0; x < 4; x++)
		{
			Buffer->GET_GYRO_RAW_X_ANGLE_VEL[x] = Buffer->GET_GYRO_RAW_X_ANGLE_VEL[x+1];
			Buffer->GET_GYRO_RAW_Y_ANGLE_VEL[x] = Buffer->GET_GYRO_RAW_Y_ANGLE_VEL[x+1];
			Buffer->GET_GYRO_RAW_Z_ANGLE_VEL[x] = Buffer->GET_GYRO_RAW_Z_ANGLE_VEL[x+1];
		}
		Buffer->GET_GYRO_RAW_X_ANGLE_VEL[4] = GYRO_DATA_STRUCT.GYRO_RAW_X_ANGLE_VEL;
		Buffer->GET_GYRO_RAW_Y_ANGLE_VEL[4] = GYRO_DATA_STRUCT.GYRO_RAW_Y_ANGLE_VEL;
		Buffer->GET_GYRO_RAW_Z_ANGLE_VEL[4] = GYRO_DATA_STRUCT.GYRO_RAW_Z_ANGLE_VEL;
	}
	else
	{
		Buffer->GET_GYRO_RAW_X_ANGLE_VEL[GyroRawQueueIndex] = GYRO_DATA_STRUCT.GYRO_RAW_X_ANGLE_VEL;
		Buffer->GET_GYRO_RAW_Y_ANGLE_VEL[GyroRawQueueIndex] = GYRO_DATA_STRUCT.GYRO_RAW_Y_ANGLE_VEL;
		Buffer->GET_GYRO_RAW_Z_ANGLE_VEL[GyroRawQueueIndex] = GYRO_DATA_STRUCT.GYRO_RAW_Z_ANGLE_VEL;
		GyroRawQueueIndex++;
	}
#else
	Buffer->GET_GYRO_RAW_X_ANGLE_VEL[0] = GYRO_DATA_STRUCT.GYRO_RAW_X_ANGLE_VEL;
	Buffer->GET_GYRO_RAW_Y_ANGLE_VEL[0] = GYRO_DATA_STRUCT.GYRO_RAW_Y_ANGLE_VEL;
	Buffer->GET_GYRO_RAW_Z_ANGLE_VEL[0] = GYRO_DATA_STRUCT.GYRO_RAW_Z_ANGLE_VEL;
#endif
	retValue = E_OK;
	return retValue;
}

/*===== API te read the angles provided by the GYRO=====*/
unsigned char HAL_GET_GYRO_DATA_IF(gyro_data_t* Buffer)
{
	unsigned char retValue;
#if (USE_SMOOTHING_FILTER == 1)
	unsigned char x;
#endif
	if((GYRO_RAW_INPUT_STRUCT.STATUS_REG & GyroMaskXdataAvail) && (GYRO_RAW_INPUT_STRUCT.STATUS_REG & GyroMaskYdataAvail) && (GYRO_RAW_INPUT_STRUCT.STATUS_REG & GyroMaskZdataAvail) )
	{
#if (USE_SMOOTHING_FILTER == 1)
		if(GyroQueueIndex == 5)
		{
			for(x = 0; x < 4; x++)
			{
				Buffer->ANGLE_RELATIVE_TO_EARTH_X[x] = Buffer->ANGLE_RELATIVE_TO_EARTH_X[x+1];
				Buffer->ANGLE_RELATIVE_TO_EARTH_Y[x] = Buffer->ANGLE_RELATIVE_TO_EARTH_Y[x+1];
				Buffer->ANGLE_RELATIVE_TO_EARTH_Z[x] = Buffer->ANGLE_RELATIVE_TO_EARTH_Z[x+1];
			}
			Buffer->ANGLE_RELATIVE_TO_EARTH_X[4] = GYRO_DATA_STRUCT.GYRO_X_ANGLE_TO_EARTH;
			Buffer->ANGLE_RELATIVE_TO_EARTH_Y[4] = GYRO_DATA_STRUCT.GYRO_Y_ANGLE_TO_EARTH;
			Buffer->ANGLE_RELATIVE_TO_EARTH_Z[4] = GYRO_DATA_STRUCT.GYRO_Z_ANGLE_TO_EARTH;
		}
		else
		{
			Buffer->ANGLE_RELATIVE_TO_EARTH_X[GyroQueueIndex] = GYRO_DATA_STRUCT.GYRO_X_ANGLE_TO_EARTH;
			Buffer->ANGLE_RELATIVE_TO_EARTH_Y[GyroQueueIndex] = GYRO_DATA_STRUCT.GYRO_Y_ANGLE_TO_EARTH;
			Buffer->ANGLE_RELATIVE_TO_EARTH_Z[GyroQueueIndex] = GYRO_DATA_STRUCT.GYRO_Z_ANGLE_TO_EARTH;
			GyroQueueIndex++;
		}
		retValue = E_OK;
#else
		Buffer->ANGLE_RELATIVE_TO_EARTH_X[0] = GYRO_DATA_STRUCT.GYRO_X_ANGLE_TO_EARTH;
		Buffer->ANGLE_RELATIVE_TO_EARTH_Y[0] = GYRO_DATA_STRUCT.GYRO_Y_ANGLE_TO_EARTH;
		Buffer->ANGLE_RELATIVE_TO_EARTH_Z[0] = GYRO_DATA_STRUCT.GYRO_Z_ANGLE_TO_EARTH;
#endif
	}
	else retValue = E_NOK;
	return retValue;
}

unsigned char HAL_GET_ACC_DATA_IF(acc_data_t* Buffer)
{
#if (USE_SMOOTHING_FILTER == 1)
	unsigned char x;
	if(AccQueueIndex == 5)
		{
			for(x = 0; x < 4; x++)
			{
				Buffer->ACCELERATIO_AXIS_X[x] = Buffer->ACCELERATIO_AXIS_X[x+1];
				Buffer->ACCELERATIO_AXIS_Y[x] = Buffer->ACCELERATIO_AXIS_Y[x+1];
				Buffer->ACCELERATIO_AXIS_Z[x] = Buffer->ACCELERATIO_AXIS_Z[x+1];
			}
			Buffer->ACCELERATIO_AXIS_X[4] = ACC_DATA_STRUCT.ACC_RAW_X_ANGLE_VEL;
			Buffer->ACCELERATIO_AXIS_Y[4] = ACC_DATA_STRUCT.ACC_RAW_Y_ANGLE_VEL;
			Buffer->ACCELERATIO_AXIS_Z[4] = ACC_DATA_STRUCT.ACC_RAW_Z_ANGLE_VEL;
		}
		else
		{
			Buffer->ACCELERATIO_AXIS_X[AccQueueIndex] = ACC_DATA_STRUCT.ACC_RAW_X_ANGLE_VEL;
			Buffer->ACCELERATIO_AXIS_Y[AccQueueIndex] = ACC_DATA_STRUCT.ACC_RAW_Y_ANGLE_VEL;
			Buffer->ACCELERATIO_AXIS_Z[AccQueueIndex] = ACC_DATA_STRUCT.ACC_RAW_Z_ANGLE_VEL;
			AccQueueIndex++;
		}
#else
		Buffer->ACCELERATIO_AXIS_X[0] = ACC_DATA_STRUCT.ACC_RAW_X_ANGLE_VEL;
		Buffer->ACCELERATIO_AXIS_Y[0] = ACC_DATA_STRUCT.ACC_RAW_Y_ANGLE_VEL;
		Buffer->ACCELERATIO_AXIS_Z[0] = ACC_DATA_STRUCT.ACC_RAW_Z_ANGLE_VEL;
#endif
	return E_OK;
}

