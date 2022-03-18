/*
 * attitude_calc.c
 *
 *  Created on: Nov 19, 2019
 *      Author: bocaim
 */

#include "attitude_calc.h"
#include "sensors.h"
#include "typeDefs.h"

unsigned char STATUS_ARMED = 0;
unsigned char autoHover;

/*static*/ float desiredRollAngle; //right = negative
/*static*/ float desiredPitchAngle; //down = negative
/*static*/ float desiredYawRate;
double rollAngleError, pitchAngleError, yawRateError;

const signed short A[5] = {-3, 12, 17, 12, 3};

float smoothAccRollAngle[5];

float actualRollAngle;
float actualPitchAngle;
float actualRawYawRate;

static float accXAngle;
static float accYAngle;

gyro_data_t GYRO_DATA;
acc_data_t ACCELERATION_DATA;
gyro_raw_data_t GET_GYRO_RAW_DATA;

unsigned char STATE = 0;
unsigned char LEAVE_STATE;
unsigned char Orange, Red, Blue, Green;

//void resetStateMachine(void)
//{
//	accXAngle = 0;
//	accYAngle = 0;
//	AccQueueIndex = 0;
//	GyroRawQueueIndex = 0;
//	STATE = stateInit;
//}

/*===== State machine implemented to be able to do tasks in a certain order easier =====*/
void calcStateMachine(void)
{
	switch (STATE)
	{
		case stateInit:
			if (okToLeaveState) STATE = stateCalibration;
			break;
		case stateCalibration:
			if (okToLeaveState) STATE = stateRunning;
			break;
		case stateRunning:
			break;
		default:
			break;
	}
}
//
///*===== Algorithm for detection of ARMED/SAFE status =====*/
//void checkSafetyTriggerCondition(void)
//{
//	static unsigned short CNT_ARM = 0;
//
//	if((INPUT_COMM.THROTTLE_OK)&&(INPUT_COMM.RUDDER_OK))
//	{
//		if((INPUT_COMM.THROTTLE_STICK<100)&&(INPUT_COMM.RUDDER_STICK>700))
//				if (STATUS_ARMED == 0)
//				{
//					CNT_ARM++;
//				}
//		if(CNT_ARM == 250)
//		{
//			/* Arm the multicopter */
//			STATUS_ARMED = 1;
//		}
//		if((INPUT_COMM.THROTTLE_STICK<200)&&(INPUT_COMM.RUDDER_STICK<200))
//		{
//				if (STATUS_ARMED == 1)
//				{
//					CNT_ARM--;
//				}
//		}
//		if(CNT_ARM == 0)
//		{
//			STATUS_ARMED = 0;
//		}
//	}
//}

unsigned char isHoveringDesired()
{
	return 1;
}

/*===== Algorithm for calculation of the desired angle of the craft based on input from sticks =====*/
void calcDesiredAngle(void)
{
	/* First of all scale the desired angles */
	if(0/*INPUT_COMM.ROLL_OK*/)
	{
		//desiredRollAngle = 50 - (float)INPUT_COMM.ROLL_STICK/10;		/* this will alow for a +-50 degrees of roll */
	}
	else
	{
		desiredRollAngle = 0;		/* In case there is no command from Rx, try to level the aircraft */
	}
	if(0/*INPUT_COMM.PITCH_OK*/)
	{
		//desiredPitchAngle = 50 - (float)INPUT_COMM.PITCH_STICK/10;		/* this will alow for a +-50 degrees of pitch */
	}
	else
	{
		desiredPitchAngle = 0;		/* In case there is no command from Rx, try to level the aircraft */
	}
	if(0/*INPUT_COMM.RUDDER_OK*/)
	{
		//desiredYawRate = 500 - (float)INPUT_COMM.RUDDER_STICK;		/* TO CHECK IF CORRECT!!!!!!yaw rate calculation in millidegrees per second */
	}
	else
	{
		desiredYawRate = 0;		/* In case there is no command from Rx, try to level the aircraft */
	}
}

/*===== Algorithm for calculation of the actual angle of the aircraft based on information from sensors =====*/
void calcActualAngle(void)
{
#if (USE_SMOOTHING_FILTER == 1)
	unsigned char localIndex;
#endif
	static double accXAngle_deg;
	static double accYAngle_deg;

	/*first read 5 samples to fill the buffer*/
	if(STATE == stateInit)
	{
		//(void)HAL_GET_GYRO_DATA_IF(&GYRO_DATA);
		(void)HAL_GET_GYRO_RAW_DATA_IF(&GET_GYRO_RAW_DATA); /*call here the function to fill the gyro raw data buffer*/
		(void)HAL_GET_ACC_DATA_IF(&ACCELERATION_DATA);
#if (USE_SMOOTHING_FILTER == 1)
		if(AccQueueIndex == 5)
		{
#endif
			okToLeaveState = 1;
#if (USE_SMOOTHING_FILTER == 1)
		}
#endif
	}
	else if(STATE == stateCalibration)
	{
		/*filter the first 5 samples of the acc sensor*/
		if(HAL_GET_ACC_DATA_IF(&ACCELERATION_DATA) == E_OK)
		{
#if (USE_SMOOTHING_FILTER == 1)
			if(AccQueueIndex == 5)
			{
				for(localIndex = 0; localIndex < 5; localIndex++)
				{
					accXAngle += (float)(A[localIndex]*(ACCELERATION_DATA.ACCELERATIO_AXIS_X[localIndex]));
					accYAngle += (float)(A[localIndex]*(ACCELERATION_DATA.ACCELERATIO_AXIS_Y[localIndex]));
				}
				accXAngle = accXAngle/35;
				accYAngle = accYAngle/35;
				actualPitchAngle = (accXAngle * 57.3); /* 57.3 == 3.14159 * 180 */
				actualRollAngle = (accYAngle * 57.3); /* 57.3 == 3.14159 * 180 */
				okToLeaveState = 1;
			}
#else
			accXAngle = ACCELERATION_DATA.ACCELERATIO_AXIS_X[0];
			accYAngle = ACCELERATION_DATA.ACCELERATIO_AXIS_Y[0];
			actualPitchAngle = (accXAngle * 57.3); /* 57.3 == 3.14159 * 180 */
			actualRollAngle = (accYAngle * 57.3); /* 57.3 == 3.14159 * 180 */
			okToLeaveState = 1;
#endif
		}
	}
	else if(STATE == stateRunning)
	{
		/*Use Savitzky-Golay smoothing filter on the Acc data*/
		if(HAL_GET_ACC_DATA_IF(&ACCELERATION_DATA) == E_OK)
		{
#if (USE_SMOOTHING_FILTER == 1)
			if(AccQueueIndex == 5)
			{
				for(localIndex = 0; localIndex < 5; localIndex++)
				{
					accXAngle += (float)(A[localIndex]*(ACCELERATION_DATA.ACCELERATIO_AXIS_X[localIndex]));
					accYAngle += (float)(A[localIndex]*(ACCELERATION_DATA.ACCELERATIO_AXIS_Y[localIndex]));
				}
				accXAngle = accXAngle/35;
				accYAngle = accYAngle/35;
				accXAngle_deg = (accXAngle * 57.3); /* 57.3 == 3.14159 * 180 */
				accYAngle_deg = (accYAngle * 57.3); /* 57.3 == 3.14159 * 180 */
			}
#else
			accXAngle = ACCELERATION_DATA.ACCELERATIO_AXIS_X[0];
			accYAngle = ACCELERATION_DATA.ACCELERATIO_AXIS_Y[0];
			accXAngle_deg = (accXAngle * 57.3); /* 57.3 == 3.14159 * 180 */
			accYAngle_deg = (accYAngle * 57.3); /* 57.3 == 3.14159 * 180 */
#endif
		}
		if(HAL_GET_GYRO_RAW_DATA_IF(&GET_GYRO_RAW_DATA) == E_OK)
		{
#if (USE_SMOOTHING_FILTER == 1)
			/*Calculate the actual pitch and roll using complementary filter*/
			actualPitchAngle = (float)((0.98)*(actualPitchAngle + (float)(GET_GYRO_RAW_DATA.GET_GYRO_RAW_X_ANGLE_VEL[4]/200000)) + (0.02)*accXAngle_deg);
			actualRollAngle = (0.98)*(actualRollAngle + (float)(GET_GYRO_RAW_DATA.GET_GYRO_RAW_Y_ANGLE_VEL[4]/200000)) + (0.02)*accYAngle_deg;
			actualRawYawRate = GET_GYRO_RAW_DATA.GET_GYRO_RAW_Z_ANGLE_VEL[4]; //instead of "Z" was :X: !!!! error? bug?
#else
			actualPitchAngle = (float)((0.9)*(actualPitchAngle + (float)(GET_GYRO_RAW_DATA.GET_GYRO_RAW_X_ANGLE_VEL[0]/200000)) + (0.1)*accXAngle_deg);
			actualRollAngle = (0.9)*(actualRollAngle + (float)(GET_GYRO_RAW_DATA.GET_GYRO_RAW_Y_ANGLE_VEL[0]/200000)) + (0.1)*accYAngle_deg;
			actualRawYawRate = GET_GYRO_RAW_DATA.GET_GYRO_RAW_Z_ANGLE_VEL[0];

			if(actualPitchAngle> 0)
			{
				Orange = ((unsigned char)actualPitchAngle / 5);
				Blue = 0;
				if (Orange > 10) Orange = 10;
			}
			else
			{
				Blue = ((unsigned char)((-actualPitchAngle)) / 5);
				Orange = 0;
			}
			if(actualRollAngle> 0)
			{
				Green = ((unsigned char)actualRollAngle / 5);
				Red = 0;
				if (Green > 10) Green = 10;
			}
			else
			{
				Red = ((unsigned char)((-actualRollAngle)) / 5);
				Green = 0;
			}

#endif
		}
		rollAngleError = desiredRollAngle - actualRollAngle;
		pitchAngleError = desiredPitchAngle - actualPitchAngle;
	    yawRateError = desiredYawRate - actualRawYawRate;
	}
}

/*===== Now that we know the actual and the desired angle, calculate the error between them =====*/
/*void calcError()
{
	if(INPUT_COMM.THROTTLE_OK)
	{
		setCommandtoAct(AllMotors, INPUT_COMM.THROTTLE_STICK);
	}
}*/

