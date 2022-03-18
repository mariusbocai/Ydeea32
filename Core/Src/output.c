#include "pid.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
//  1\    /2    Forward
//    \  /     /|\
//     \/       |
//     /\       |
//    /  \      |
//  4/    \3    |
// 1 = ccw

extern TIM_HandleTypeDef htim3;

/*Variable declaration*/
SPid rollAxis;
SPid pitchAxis;
SPid yawAxis;

char indexChart;
unsigned short Init_PID;
static signed int commandMot1, commandMot2, commandMot3, commandMot4;

//#define PIDMIX(X,Y,Z) INPUT_COMM.THROTTLE_STICK + rollPIDResult*X + pitchPIDResult*Y + yawPIDResult*Z
#define PIDMIX(X,Y,Z) 1100 + rollPIDResult*X + pitchPIDResult*Y + yawPIDResult*Z

double rollPIDResult, pitchPIDResult, yawPIDResult;
unsigned char selectPidCurrentLoop;

unsigned int limitCommand(signed int commDelta)
{
	signed int retValue = 0;
	retValue = commDelta;
	if(commDelta > 1000)
	{
		retValue = 999;
	}
	else if (commDelta <1)
	{
		retValue = 1;
	}
	return (unsigned int)(4000 - retValue);
}

void PWM_Main()
{
	htim3.Instance->CCR1 = limitCommand(commandMot1);
	htim3.Instance->CCR2 = limitCommand(commandMot2);
	htim3.Instance->CCR3 = limitCommand(commandMot3);
	htim3.Instance->CCR4 = limitCommand(commandMot4);
}


void PID_Main(void)
{
    if(Init_PID == 1)
    {
        commandMot1 = 0;
        commandMot2 = 0;
        commandMot3 = 0;
        commandMot4 = 0;
        Init_PID = 0;
    }
   
    if((1/*STATUS_ARMED*/)&&(1/*INPUT_COMM.THROTTLE_STICK >= 100*/))
    {
			rollPIDResult = UpdatePID(&rollAxis, rollAngleError, actualRollAngle);
			if(rollPIDResult < 0) rollPIDResult = 0;
			pitchPIDResult = UpdatePID(&pitchAxis, pitchAngleError, actualPitchAngle);
			if(pitchPIDResult < 0) pitchPIDResult = 0;
			yawPIDResult = UpdatePID(&yawAxis, yawRateError, actualRawYawRate);
			if(yawPIDResult < 0) yawPIDResult = 0;
			commandMot1 = PIDMIX(+1,+1, +1);
			commandMot2 = PIDMIX(-1,+1, -1);
			commandMot3 = PIDMIX(-1,-1, +1);
			commandMot4 = PIDMIX(+1,-1, -1);
			//setCommandtoAct(Motor1, limitCommand(commandMot1));
			//setCommandtoAct(Motor4, limitCommand(commandMot4));
			//setCommandtoAct(Motor2, limitCommand(commandMot2));
			//setCommandtoAct(Motor3, limitCommand(commandMot3));
    }
	else
	{ /*when the multicopter is not armed, reset continuously all functions*/
		PID_Init();
		rollPIDResult = 0;
		pitchPIDResult = 0;
		//setCommandtoAct(AllMotors, 4000); // all motors forced idle
		//resetStateMachine();
	}
#if(0)
    {
        if(((CLOCK_VAR%10)==0) && (MCU_LOAD<25))
        {
					if(indexChart < 2)
					{
						CHART_OUT_ARRAY[0][0] = &actualPitchAngle;
						CHART_OUT_ARRAY[0][1] = &actualRollAngle;
						CHART_OUT_ARRAY[0][2] = &actualRawYawRate;
					
						CHART_OUT_ARRAY[1][0] = &pitchPIDResult;
						CHART_OUT_ARRAY[1][1] = &pitchPIDResult;
						CHART_OUT_ARRAY[1][2] = &rollPIDResult;
					
						serialSendChar(CHART_OUT_ARRAY[indexChart][0], noNewLine);
						serialSendChar(CHART_OUT_ARRAY[indexChart][1], noNewLine);
						serialSendChar(CHART_OUT_ARRAY[indexChart][2], putNewLine);
					}
					else
					{
						double cm1 = (double)commandMot1;
						double cm2 = (double)commandMot2;
						double cm3 = (double)commandMot3;
						double cm4 = (double)commandMot4;
					
            serialSendChar(&cm1, noNewLine);
						serialSendChar(&cm2, noNewLine);
						serialSendChar(&cm3, noNewLine);
            serialSendChar(&cm4, putNewLine);
						//double tempvariabl = (double)MAG_DATA_STRUCT.MILIGAUSS_X_AXIS;
            //serialSendChar(&tempvariabl, noNewLine);
            //serialSendChar(&MCU_LOAD, putNewLine);
						//double value_cast_x = (double)GYRO_DATA_STRUCT.GYRO_RAW_X_ANGLE_VEL;
						//double value_cast_y = (double)GYRO_DATA_STRUCT.GYRO_RAW_Y_ANGLE_VEL;
						// 		double value_cast_x = (double)ACC_DATA_STRUCT.ACC_RAW_X_ANGLE_VEL;
						// 		double value_cast_y = (double)ACC_DATA_STRUCT.ACC_RAW_Y_ANGLE_VEL;
						// 		double value_cast_z = (double)ACC_DATA_STRUCT.ACC_RAW_Z_ANGLE_VEL;
					}
        }
				else if((MCU_LOAD >= 25))
				{
					/*No data output was sent because runtime is too high; in this case, send some predefined character*/
				}
    }
#endif
} 
