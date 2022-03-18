#include "pid.h"
//#include "uart.h"
#include "output.h"

/*=======================================================*/
/*PID code copied from "PID without a PhD" by Tim Wescott*/
/*=======================================================*/
unsigned short deBug = 1000;

void PID_Init(void)
{
	rollPIDResult = 0;
	pitchPIDResult = 0;
	yawPIDResult = 0;
	selectPidCurrentLoop = 1;
	
	/*Reminder: the gains haven't been calculated in any way! just to have a compilable software!*/
	rollAxis.dState = 0;
	rollAxis.iState = 0;
	rollAxis.iMax = 1000;
	rollAxis.iMin = -1000;
	rollAxis.iGain = 0;//was 0.01
	rollAxis.pGain = 5; //was 2
	rollAxis.dGain = 0;	//was 0.01
	
	pitchAxis.dState = 0;
	pitchAxis.iState = 0;
	pitchAxis.iMax = 1000;
	pitchAxis.iMin = -1000;
	pitchAxis.iGain = 0;//was 0.01
	pitchAxis.pGain = 5;//was 2
	pitchAxis.dGain = 0; //was 0.01
	
	yawAxis.dState = 0;
	yawAxis.iState = 0;
	yawAxis.iMax = 1000;
	yawAxis.iMin = -1000;
	yawAxis.iGain = 0; //0.02
	yawAxis.pGain = 2;
	yawAxis.dGain = 0;  //0.01
}

/*============================================*/
/*PID code that should be called for each axis*/
/*============================================*/
double UpdatePID(SPid * pid, double error, double position)
{
	double pTerm, dTerm, iTerm;
	pTerm = pid->pGain * error;
	// calculate the proportional term
	// calculate the integral state with appropriate limiting
	pid->iState += error;
	if (pid->iState > pid->iMax) 
	{
		pid->iState = pid->iMax;
	}
	else if (pid->iState < pid->iMin)
	{
		pid->iState = pid->iMin;
	}
	iTerm = pid->iGain * pid->iState; // calculate the integral term
	dTerm = pid->dGain * (position - pid->dState);
	pid->dState = position;
	return pTerm + iTerm - dTerm;
}
