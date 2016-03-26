#include "greatest/greatest.h"
#include "../core/log.h"

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

TEST test_name(void)
{
    
    ASSERT(1 == 1);
    PASS();
}

SUITE(suite_name)
{
    RUN_TEST(test_name);
}

GREATEST_MAIN_DEFS();

int main (int argc, char** argv)
{

    GREATEST_MAIN_BEGIN();

    RUN_TEST(test_name);
    RUN_SUITE(suite_name);

    GREATEST_MAIN_END();
}
