#include "greatest/greatest.h"
#include <peripherals/pid.h>

TEST proportional(void)
{
    PASS();
}

TEST derivative(void)
{
    PASS();
}

TEST intergral(void)
{
    PASS();
}

TEST pid (void)
{
    PASS();
}

SUITE(terms)
{
    RUN_TEST(proportional);
}

// Test runner

GREATEST_MAIN_DEFS();

int main(int argc, char *argv[])
{
    GREATEST_MAIN_BEGIN();
    RUN_SUITE(terms);
    GREATEST_MAIN_END();
}
