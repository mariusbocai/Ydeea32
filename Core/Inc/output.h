#ifndef output_c_included
#define output_c_included

/*Macro definition*/
#define rollAxisId 1
#define pitchAxisId 2
#define yawAxisId 3

/*Variable declaration*/
extern double rollPIDResult;
extern double pitchPIDResult;
extern double yawPIDResult;

extern SPid rollAxis;
extern SPid pitchAxis;
extern SPid yawAxis;

extern unsigned char selectPidCurrentLoop;
extern unsigned char Init_PID;

/*Interface functions*/
extern void PID_Main(void);
extern void resetStateMachine(void);
extern void PWM_Main(void);

/*Declarations needed for Chart*/
extern char indexChart;

#define outGainFactor 1

#endif
