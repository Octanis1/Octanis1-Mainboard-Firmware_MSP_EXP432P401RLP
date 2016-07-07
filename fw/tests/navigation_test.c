#include "greatest/greatest.h"
#include <peripherals/navigation.h>
#include <math.h>

#define TOL 0.0001
#define TOL_FLOAT 1
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

TEST correct_angle(void)
{
    //lat, lon
    float start_lon[4] = {40, 60, -12, -32};
    float start_lat[4] = {32, -10, 13, -43};
    float end_lon[4] = {23, 31, -4, -2};
    float end_lat[4] = {23, -12, 42, -66};
    float ans[4][4] ={{-116.659571286, -167.465987946, -61.1681984461, -163.769226879},
                    {-47.2296973092, -96.7581853182, -43.0294108211, -157.488281539},
                    {68.2075586684, 118.586395916, 12.0034366938, 175.87878644},
                    {49.4193121596, 80.1835663392, 20.4250651976, 154.579361837}};
    int i, j;

    float deb = 0;

    for(i=0; i<4; i++){
        for(j=0; j<4; j++){ 
            deb = navigation_angle_to_target(start_lat[i], start_lon[i], end_lat[j], end_lon[j]);
            ASSERT (absolute(ans[i][j] - deb < TOL_FLOAT));
        }
    }
    PASS();
}
TEST correct_distance(void)
{
    //TODO:Numbers too big for float, re-write them with smaller distance
    //lat, lon
    float start_lat[4] = {32, -10, 13, -43};
    float start_lon[4] = {40, 60, -12, -32};
    float end_lat[4] = {23, -12, 42, -66};
    float end_lon[4] = {23, 31, -4, -2};
    float ans[4][4] ={{1944485.9, 4974393.4, 4005812.8, 11445866.1},
                    {5435772.5, 3164678.8, 8675122.8, 7734647.4},
                    {3844707.4, 5484990.6, 3308661.4, 8803720.9},
                    {9222674.0, 6899557.7, 9835526.0, 3137441.2}};
    int i, j;

    float deb = 0;

    for(i=0; i<4; i++){
        for(j=0; j<4; j++){ 
            deb = navigation_dist_to_target(start_lat[i], start_lon[i], end_lat[j], end_lon[j]);
            ASSERT (absolute(ans[i][j] - deb < TOL_FLOAT));
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

TEST go_space_needed(void)
{

    PASS();
}

TEST go_go_to_target(void)
{

    PASS();
}


TEST go_avoid_wall(void)
{

    PASS();
}


TEST go_avoid_obstacle(void)
{

    PASS();
}

TEST go_stop(void)
{

    PASS();
}


SUITE(angle_and_dist)
{
    RUN_TEST(correct_rad);
    RUN_TEST(correct_distance);
    RUN_TEST(correct_angle);
}

SUITE(state)
{
    RUN_TEST(correct_rad);
}

// Test runner

GREATEST_MAIN_DEFS();

int main(int argc, char *argv[])
{
    GREATEST_MAIN_BEGIN();
    RUN_SUITE(angle_and_dist);
    GREATEST_MAIN_END();
}
