#ifndef PID_H
#define PID_H

typedef struct pid_t{
    float pGain;
    float iGain;
    float dGain;
    float dState;
    float iState;
    float iMax;
    float iMin;
}pid_t;

float pid_update (*pid_t pid, float error, float position);

#endif
