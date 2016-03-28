#include "greatest/greatest.h"
#include <peripherals/flash.h>
#include <peripherals/flash_defines.h>

// function stubs
void flash_spi_select(void) {}
void flash_spi_unselect(void) {}
void flash_os_sleep_ms(uint32_t ms) {(void)ms;}
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
    // Assert that the page program operation is always inside one page (no overlapping)
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

TEST write_one_unaligned_page(void)
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

TEST write_less_than_one_page(void)
{
    size_t len = FLASH_PAGE_SIZE / 2;
    int ret = flash_write(12, NULL, len);
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
    RUN_TEST(write_less_than_one_page);
    RUN_TEST(write_one_aligned_page);
    RUN_TEST(write_one_unaligned_page);
    RUN_TEST(write_multiple_unaligned_pages);
}

GREATEST_MAIN_DEFS();

int main(int argc, char *argv[])
{
    GREATEST_MAIN_BEGIN();
    RUN_SUITE(flash_test_suite);
    GREATEST_MAIN_END();
}
