#include "greatest/greatest.h"
#include <peripherals/flash.h>

TEST test_can_write_and_read(void)
{
    const char *str = "hello world";
    size_t len = strlen(str) + 1;
    flash_write(0, str, len);
    char buf[100];
    memset(buf, 0, sizeof(buf));
    flash_read(0, buf, len);

    ASSERT(strcmp(buf, str) == 0);
    PASS();
}

SUITE(flash_test_suite)
{
    RUN_TEST(test_can_write_and_read);
}

GREATEST_MAIN_DEFS();

int main(int argc, char *argv[])
{
    GREATEST_MAIN_BEGIN();
    RUN_SUITE(flash_test_suite);
    GREATEST_MAIN_END();
}
