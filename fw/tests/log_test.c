#include "greatest/greatest.h"
#include <stdint.h>
#include <stdbool.h>
#include <core/log.h>
#include <core/log_internal.h>
#include <lib/crc8.h>

// mock functions
LOG_INTERNAL void logger_lock(void){}
LOG_INTERNAL void logger_unlock(void){}
LOG_INTERNAL uint32_t logger_timestamp_sec(void)
{
    return 1234;
}

uint8_t flash_array[FLASH_SIZE];

bool flash_addr_is_valid(uint32_t addr)
{
    if (addr < FLASH_SIZE) {
        return true;
    } else {
        return false;
    }
}

int flash_read(uint32_t addr, void *buf, size_t len)
{
    ASSERT_OR_LONGJMP(flash_addr_is_valid(addr));
    ASSERT_OR_LONGJMP(flash_addr_is_valid(addr+len));
    memcpy(buf, &flash_array[addr], len);
    return 0;
}

int flash_write(uint32_t addr, const void *buf, size_t len)
{
    ASSERT_OR_LONGJMP(flash_addr_is_valid(addr));
    ASSERT_OR_LONGJMP(flash_addr_is_valid(addr+len));
    memcpy(&flash_array[addr], buf, len);
    return 0;
}

int flash_block_erase(uint32_t addr)
{
    ASSERT_OR_LONGJMP(flash_addr_is_valid(addr));
    addr -= addr % FLASH_BLOCK_SIZE;
    ASSERT_OR_LONGJMP(flash_addr_is_valid(addr));
    memset(&flash_array[addr], 0xff, FLASH_BLOCK_SIZE);
    return 0;
}

struct logger test_logger;

TEST test_entry_header(void)
{
    _log_entry_create(&test_logger, "entry");

    cmp_ctx_t reader;
    cmp_mem_access_t cma;
    cmp_mem_access_ro_init(&reader, &cma, &test_logger.buffer[LOG_ENTRY_HEADER_LEN], LOG_ENTRY_DATA_LEN);
    uint32_t size = 0;
    if (!cmp_read_array(&reader, &size) || size != 3) {
        FAIL();
    }
    uint64_t timestamp = 0;
    if (!cmp_read_uinteger(&reader, &timestamp) || timestamp != logger_timestamp_sec()) {
        FAIL();
    }
    char name[10];
    size = sizeof(name);
    if (!cmp_read_str(&reader, name, &size) || strcmp(name, "entry") != 0) {
        FAIL();
    }
    PASS();
}

TEST test_entry_crc_and_length(void)
{
    test_logger.flash_write_pos = 0;
    cmp_ctx_t *ctx = _log_entry_create(&test_logger, "entry");
    cmp_write_integer(ctx, 42);
    _log_entry_write_to_flash(&test_logger);

    size_t entry_length = cmp_mem_access_get_pos(&test_logger.cma);
    uint8_t len = flash_array[0];
    uint8_t crc = flash_array[1];
    if (crc8(0, &flash_array[2], len) != crc) {
        FAILm("CRC missmatch");
    }
    if (entry_length != len) {
        FAILm("entry length missmatch");
    }
    PASS();
}

TEST test_can_read_entry(void)
{
    uint8_t buf[LOG_ENTRY_DATA_LEN];
    size_t entry_len = 4;
    memset(&flash_array[2], 42, entry_len);
    flash_array[0] = entry_len;
    flash_array[1] = crc8(0, &flash_array[2], entry_len);
    size_t len = 0;
    if (!log_read_entry(0, buf, &len)) {
        FAILm("broken CRC check");
    }
    ASSERT_EQ(entry_len, len);
    if (memcmp(&flash_array[2], buf, len) != 0) {
        FAILm("wrong entry data");
    }
    PASS();
}

TEST test_crc_mismatch(void)
{
    uint8_t buf[LOG_ENTRY_DATA_LEN];
    size_t entry_len = 4;
    memset(&flash_array[2], 42, entry_len);
    flash_array[0] = entry_len;
    // write false crc
    flash_array[1] = crc8(0, &flash_array[2], entry_len) ^ 0xff;
    size_t len;
    if (log_read_entry(0, buf, &len)) {
        FAILm("broken CRC check");
    }
    PASS();
}

TEST test_entry_write_and_readback(void)
{
    cmp_ctx_t *ctx = _log_entry_create(&test_logger, "entry");
    cmp_write_integer(ctx, 42);
    _log_entry_write_to_flash(&test_logger);

    uint8_t buf[LOG_ENTRY_DATA_LEN];
    size_t len = 0;
    if (!log_read_entry(0, buf, &len)) {
        FAILm("CRC missmatch");
    }

    // readback entry
    cmp_ctx_t reader;
    cmp_mem_access_t cma;
    cmp_mem_access_ro_init(&reader, &cma, buf, sizeof(buf));
    uint32_t size = 0;
    if (!cmp_read_array(&reader, &size) || size != 3) {
        FAIL();
    }
    uint64_t timestamp = 0;
    if (!cmp_read_uinteger(&reader, &timestamp) || timestamp != logger_timestamp_sec()) {
        FAIL();
    }
    char name[10];
    size = sizeof(name);
    if (!cmp_read_str(&reader, name, &size) || strcmp(name, "entry") != 0) {
        FAIL();
    }
    int64_t data = 0;
    if (!cmp_read_integer(&reader, &data) || data != 42) {
        FAIL();
    }
    PASS();
}

void setup_cb(void *p)
{
    (void)p;
    memset(&flash_array, 0xff, sizeof(flash_array));
    memset(&test_logger, 0, sizeof(test_logger));
}

SUITE(log_entry_test)
{
    SET_SETUP(setup_cb, NULL);
    RUN_TEST(test_entry_header);
    RUN_TEST(test_entry_crc_and_length);
    RUN_TEST(test_can_read_entry);
    RUN_TEST(test_crc_mismatch);
    RUN_TEST(test_entry_write_and_readback);
}

TEST test_page_erase_addr(void)
{
    bool erase;
    uint32_t erase_addr, addr;
    size_t len;

    erase_addr = 0;
    len = 1;
    addr = FLASH_BLOCK_SIZE;
    erase = _log_flash_erase_addr(addr, len, &erase_addr);
    ASSERT_EQ(erase, true);
    ASSERT_EQ(erase_addr, addr);

    erase_addr = 0;
    len = 3;
    addr = 2*FLASH_BLOCK_SIZE - 1;
    erase = _log_flash_erase_addr(addr, len, &erase_addr);
    ASSERT_EQ(erase, true);
    ASSERT_EQ(erase_addr, 2*FLASH_BLOCK_SIZE);

    len = 1;
    addr = FLASH_BLOCK_SIZE + 1;
    erase = _log_flash_erase_addr(addr, len, &erase_addr);
    ASSERT_EQ(erase, false);

    erase_addr = 0;
    len = 2;
    addr = FLASH_BLOCK_SIZE - 1;
    erase = _log_flash_erase_addr(addr, len, &erase_addr);
    ASSERT_EQ(erase, true);
    ASSERT_EQ(erase_addr, FLASH_BLOCK_SIZE);

    PASS();
}

SUITE(log_flash_test)
{
    SET_SETUP(setup_cb, NULL);
    RUN_TEST(test_page_erase_addr);
    // RUN_TEST(test_does_find_last_pos);
}

GREATEST_MAIN_DEFS();

int main(int argc, char *argv[])
{
    GREATEST_MAIN_BEGIN();
    RUN_SUITE(log_entry_test);
    RUN_SUITE(log_flash_test);
    GREATEST_MAIN_END();
}
