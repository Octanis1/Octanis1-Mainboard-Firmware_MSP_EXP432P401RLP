#include "greatest/greatest.h"
#include <peripherals/flash.h>

// function stubs
void flash_spi_select(void)
{
    return;
}

void flash_spi_unselect(void)
{
    return;
}

int flash_spi_send(const void *txbuf, size_t len)
{
    (void)txbuf;
    (void)len;
    return 0;
}

int flash_spi_receive(void *rxbuf, size_t len)
{
    (void)rxbuf;
    (void)len;
    return 0;
}

// test implementation
size_t flash_bytes_written;
int flash_page_program(uint32_t addr, const void *buf, size_t len)
{
    ASSERT_OR_LONGJMP(len <= FLASH_PAGE_SIZE);
    size_t bytes_till_end_of_page = FLASH_PAGE_SIZE - addr % FLASH_PAGE_SIZE;
    ASSERT_OR_LONGJMP(len <= bytes_till_end_of_page);
    flash_bytes_written += len;
    return 0;
}

TEST write_multiple_unaligned_pages(void)
{
    size_t len = 3 * FLASH_PAGE_SIZE;
    int ret = flash_write(123, NULL, len);
    ASSERT(ret == 0);
    ASSERT(flash_bytes_written == len);
    PASS();
}

TEST write_one_unaligned_pages(void)
{
    size_t len = FLASH_PAGE_SIZE;
    int ret = flash_write(123, NULL, len);
    ASSERT(ret == 0);
    ASSERT(flash_bytes_written == len);
    PASS();
}

TEST write_one_aligned_page(void)
{
    size_t len = FLASH_PAGE_SIZE;
    int ret = flash_write(0, NULL, len);
    ASSERT(ret == 0);
    ASSERT(flash_bytes_written == len);
    PASS();
}

void setup_cb(void *p)
{
    (void)p;
    flash_bytes_written = 0;
}

SUITE(flash_test_suite)
{
    SET_SETUP(setup_cb, NULL);
    RUN_TEST(write_one_aligned_page);
    RUN_TEST(write_one_unaligned_pages);
    RUN_TEST(write_multiple_unaligned_pages);
}

GREATEST_MAIN_DEFS();

int main(int argc, char *argv[])
{
    GREATEST_MAIN_BEGIN();
    RUN_SUITE(flash_test_suite);
    GREATEST_MAIN_END();
}
