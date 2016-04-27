#include "greatest/greatest.h"
#include <peripherals/navigation.h>
#include <math.h>

#define TOL 0.0001
#define M_PI 3.14159265358979323846

//dummy function
void navigation_init(){}
void navigation_move(){}
void navigation_update_position(){}
uint8_t navigation_bypass(char command, uint8_t index)
{
    (void)command;
    (void)index;
    return 0;
}
void Task_sleep(int a)
{
    (void)a;
}

float absolute (float a)
{
    if (a >= 0)
        return a;
    else
        return -a;
}

TEST correct_distance(void)
{
    //lat, lon
    float start_lon[4] = {40, 60, -12, -32};
    float start_lat[4] = {32, -10, 13, -43};
    float end_lon[4] = {23, 31, -67, -2};
    float end_lat[4] = {23, -12, 42, -66};
    float ans[4][4] ={{1949000, 4986000, 8917000, 11470000},
                    {4846000, 8796000, 4289000, 14030000},
                    {3853000, 5497000, 6176000, 8823000},
                    {7139000, 6835000, 6989000, 6415000}};
    int i, j;

    float deb = 0;

    for(i=0; i<4; i++){
        for(j=0; j<4; j++){ 
            deb = navigation_dist_to_target(start_lat[i], start_lon[i], end_lat[j], end_lon[j]);
            ASSERT (absolute(ans[i][j] - deb < TOL));
        }
    }
    PASS();
}

TEST correct_rad(void)
{
   float degree = 0;
    float a = 0;
    float i = 0;
    for(i=-4;i<=4;i++){
        degree = i*90;
        a = absolute(i*M_PI/2 - navigation_degree_to_rad(degree));
        ASSERT(a < TOL);
    }

    PASS();
}

SUITE(angle_and_dist)
{
    RUN_TEST(correct_rad);
    RUN_TEST(correct_distance);
}

// Test runner

GREATEST_MAIN_DEFS();

int main(int argc, char *argv[])
{
    GREATEST_MAIN_BEGIN();
    RUN_SUITE(angle_and_dist);
    GREATEST_MAIN_END();
}
