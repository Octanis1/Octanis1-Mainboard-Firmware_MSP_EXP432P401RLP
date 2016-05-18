#include <stdio.h>
#include <stdlib.h>
#include "pid.h"


float pid_update (pid_controler_t *pid, float error, float position)
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
    dTerm = pid->dGain*(position - pid->dState);
    pid->dState = position;

    return pTerm + iTerm - dTerm;
}
