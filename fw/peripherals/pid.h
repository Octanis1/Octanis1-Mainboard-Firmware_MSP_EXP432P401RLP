#ifndef PID_H
#define PID_H

typedef struct pid_controler_t{
    float pGain;
    float iGain;
    float dGain;
    float dState;
    float iState;
    float iMax;
    float iMin;
}pid_controler_t;

float pid_update (pid_controler_t *pid, float error);
void pid_init (pid_controler_t* pid, float pGain, float iGain, float dGain, float iMax, float iMin);

#endif
