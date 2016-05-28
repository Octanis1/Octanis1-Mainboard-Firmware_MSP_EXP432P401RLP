#include <stdio.h>
#include <stdlib.h>
#include "pid.h"


float pid_update (pid_controler_t *pid, float error)
{
    float pTerm, iTerm, dTerm;
    
    //proportional term
    pTerm = pid->pGain * error;
    
    //integral term
    pid->iState += error;
    if (pid->iState > pid->iMax)
        pid->iState = pid->iMax;
    else if (pid->iState < pid->iMin)
        pid->iState = pid->iMin;

    iTerm = pid->iGain*pid->iState;

    //differential term
    dTerm = pid->dGain*(error - pid->dState);
    pid->dState = error;

    return pTerm + iTerm - dTerm;
}

void pid_init (pid_controler_t* pid, float pGain, float iGain, float dGain, float iMax, float iMin)
{
	pid->pGain = pGain;
	pid->iGain = iGain;
	pid->dGain = dGain;
	pid->iMax = iMax;
	pid->iMin = iMin;
	pid->dState = 0;
	pid->iState = 0;
}
