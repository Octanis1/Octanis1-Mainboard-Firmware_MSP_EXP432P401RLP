#include "greatest/greatest.h"
#include <peripherals/pid.h>

TEST proportional(void)
{
    pid_controler_t pid;

    pid.pGain = 2;
    pid.dGain = 0;
    pid.iGain = 0;
    
    float error = 2;
    float position = 0;

    ASSERT(pid_update(&pid, error, position) == 4);
    PASS();
}

TEST derivative(void)
{
    pid_controler_t pid;

    pid.pGain = 0;
    pid.dGain = 2;
    pid.iGain = 0;
    
    pid.dState = 0;

    float error = 0;
    float position = 2;

    ASSERT(pid_update(&pid, error, position) == -4);
    PASS();
}

TEST intergral(void)
{
    pid_controler_t pid;

    pid.pGain = 0;
    pid.dGain = 0;
    pid.iGain = 2;
    
    pid.iState = 0;
    pid.iMax = 10;
    pid.iMin = -2;

    float error = 2;
    float position = 0;

    ASSERT(pid_update(&pid, error, position) == 4);
    PASS();
    ASSERT(pid_update(&pid, 12, position) == 20);
    PASS();
    ASSERT(pid_update(&pid, -4, position) == -4);
    PASS();
}

TEST pid (void)
{
    pid_controler_t pid;

    pid.pGain = 2;
    pid.dGain = 2;
    pid.iGain = 2;
    
    pid.dState = 0;

    pid.iState = 0;
    pid.iMax = 10;
    pid.iMin = -2;

    float error = 2;
    float position = 2;

    ASSERT(pid_update(&pid, error, position) == 4);
    PASS();
}

SUITE(terms)
{
    RUN_TEST(proportional);
    RUN_TEST(derivative);
    RUN_TEST(intergral);
    RUN_TEST(pid);
}

// Test runner

GREATEST_MAIN_DEFS();

int main(int argc, char *argv[])
{
    GREATEST_MAIN_BEGIN();
    RUN_SUITE(terms);
    GREATEST_MAIN_END();
}
