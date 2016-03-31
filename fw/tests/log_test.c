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

// helper function to debug failing tests
void flash_array_dump(uint32_t addr, size_t len)
{
    size_t i;
    for (i = 0; i < len; i++) {
        printf("%02x", flash_array[addr+i]);
    }
    printf("\n");
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

TEST test_entry_write_multiple(void)
{
    uint32_t pos_a = test_logger.flash_write_pos;
    cmp_ctx_t *ctx;
    ctx = _log_entry_create(&test_logger, "a");
    cmp_write_integer(ctx, 42);
    _log_entry_write_to_flash(&test_logger);

    uint32_t pos_b = test_logger.flash_write_pos;
    ASSERT(pos_a < pos_b);
    ctx = _log_entry_create(&test_logger, "b");
    cmp_write_integer(ctx, 23);
    _log_entry_write_to_flash(&test_logger);

    cmp_ctx_t reader;
    cmp_mem_access_t cma;
    cmp_mem_access_ro_init(&reader, &cma, &flash_array[pos_a + LOG_ENTRY_HEADER_LEN], LOG_ENTRY_DATA_LEN);
    uint32_t size = 0;
    uint64_t timestamp = 0;
    int64_t val = 0;
    char name[10];
    bool ok;
    size = sizeof(name);
    ok = true;
    ok &= cmp_read_array(&reader, &size);
    ok &= cmp_read_uinteger(&reader, &timestamp);
    ok &= cmp_read_str(&reader, name, &size);
    ok &= cmp_read_integer(&reader, &val);
    ASSERT(ok);
    ASSERT_EQ(val, 42);

    size_t pos = cmp_mem_access_get_pos(&cma);
    ASSERT_EQ(pos, flash_array[0]);
    ASSERT_EQ(pos + LOG_ENTRY_HEADER_LEN, pos_b);

    cmp_mem_access_ro_init(&reader, &cma, &flash_array[pos_b + LOG_ENTRY_HEADER_LEN], LOG_ENTRY_DATA_LEN);
    size = sizeof(name);
    ok = true;
    ok &= cmp_read_array(&reader, &size);
    ok &= cmp_read_uinteger(&reader, &timestamp);
    ok &= cmp_read_str(&reader, name, &size);
    ok &= cmp_read_integer(&reader, &val);
    ASSERT(ok);
    ASSERT_EQ(val, 23);
    ASSERT_EQ(flash_array[pos_b], cmp_mem_access_get_pos(&cma));

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
    uint32_t next_entry = 0;
    if (!log_read_entry(0, buf, &len, &next_entry)) {
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
    uint32_t next_entry = 0;
    size_t len;
    if (log_read_entry(0, buf, &len, &next_entry)) {
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
    uint32_t next_entry = 0;
    size_t len = 0;
    if (!log_read_entry(0, buf, &len, &next_entry)) {
        FAILm("CRC missmatch");
    }
    ASSERT_EQ(test_logger.flash_write_pos, next_entry);

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

TEST test_multiple_entry_and_readback(void)
{
    cmp_ctx_t *ctx;
    ctx = _log_entry_create(&test_logger, "a");
    cmp_write_integer(ctx, 42);
    _log_entry_write_to_flash(&test_logger);
    uint32_t end_a = test_logger.flash_write_pos;

    ctx = _log_entry_create(&test_logger, "b");
    cmp_write_integer(ctx, 23);
    _log_entry_write_to_flash(&test_logger);
    uint32_t end_b = test_logger.flash_write_pos;

    uint8_t buf[LOG_ENTRY_DATA_LEN];
    uint32_t next_entry = 0;
    size_t len;
    if (!log_read_entry(next_entry, buf, &len, &next_entry)) {
        FAILm("CRC missmatch");
    }
    ASSERT_EQ(end_a, next_entry);
    ASSERT_EQ(buf[len-1], 42);
    if (!log_read_entry(next_entry, buf, &len, &next_entry)) {
        FAILm("CRC missmatch");
    }
    ASSERT_EQ(end_b, next_entry);
    ASSERT_EQ(buf[len-1], 23);
    if (log_read_entry(next_entry, buf, &len, &next_entry)) {
        FAILm("dont detect last entry");
    }
    PASS();
}

void setup_cb(void *p)
{
    (void)p;
    memset(&flash_array, 0xff, sizeof(flash_array));
    memset(&test_logger, 0, sizeof(test_logger));
    test_logger.flash_write_pos = 0;
    test_logger.backup_pos = 0;
}

SUITE(log_entry_test)
{
    SET_SETUP(setup_cb, NULL);
    RUN_TEST(test_entry_header);
    RUN_TEST(test_entry_crc_and_length);
    RUN_TEST(test_entry_write_multiple);
    RUN_TEST(test_can_read_entry);
    RUN_TEST(test_crc_mismatch);
    RUN_TEST(test_entry_write_and_readback);
    RUN_TEST(test_multiple_entry_and_readback);
}

TEST test_flash_erase_at_start_of_block(void)
{
    // start of a flash block
    size_t len = 1;
    uint32_t addr = FLASH_BLOCK_SIZE;
    uint32_t erase_addr = 0;
    bool erase = _log_flash_erase_addr(addr, len, &erase_addr);
    ASSERT_EQ(erase, true);
    ASSERT_EQ(erase_addr, addr);
    PASS();
}

TEST test_flash_erase_over_end_of_block(void)
{
    // end passes over to a new flash block
    size_t len = 3;
    uint32_t addr = 2*FLASH_BLOCK_SIZE - len + 1;
    uint32_t erase_addr = 0;
    bool erase = _log_flash_erase_addr(addr, len, &erase_addr);
    ASSERT_EQ(erase, true);
    ASSERT_EQ(erase_addr, 2*FLASH_BLOCK_SIZE);

    // write over end of a flash block
    erase_addr = 0;
    len = 2;
    addr = FLASH_BLOCK_SIZE - 1;
    erase = _log_flash_erase_addr(addr, len, &erase_addr);
    ASSERT_EQ(erase, true);
    ASSERT_EQ(erase_addr, FLASH_BLOCK_SIZE);
    PASS();
}

TEST test_no_flash_erase_if_inside_block(void)
{
    // write inside a already erased flash block
    size_t len = 1;
    uint32_t addr = FLASH_BLOCK_SIZE + 1;
    uint32_t erase_addr;
    bool erase = _log_flash_erase_addr(addr, len, &erase_addr);
    ASSERT_EQ(erase, false);
    PASS();
}

TEST test_no_flash_erase_if_at_end_of_block(void)
{
    // write inside a already erased flash block
    size_t len = FLASH_BLOCK_SIZE-1;
    uint32_t addr = FLASH_BLOCK_SIZE+1;
    uint32_t erase_addr;
    bool erase = _log_flash_erase_addr(addr, len, &erase_addr);
    ASSERT_EQ(erase, false);
    PASS();
}

TEST test_find_backup_pos_in_erased_flash(void)
{
    uint32_t addr = 0xDEAD;
    bool ok = _log_backup_table_get_last(&addr);
    ASSERT(ok);
    ASSERT_EQ(addr, 0);
    PASS();
}

TEST test_detect_full_backup_table(void)
{
    memset(flash_array, 0, FLASH_BLOCK_SIZE);
    uint32_t addr;
    bool ok = _log_backup_table_get_last(&addr);
    ASSERT(!ok);
    PASS();
}

TEST test_find_backup_pos(void)
{
    const unsigned nb_entries = 3;
    memset(flash_array, 0, nb_entries*sizeof(uint32_t));
    uint32_t addr = 0;
    bool ok = _log_backup_table_get_last(&addr);
    ASSERT(ok);
    ASSERT_EQ(addr, nb_entries*sizeof(uint32_t));
    PASS();
}

TEST test_seek_at_base_address(void)
{
    // end at base address
    uint8_t seek_buffer[4];
    bool is_end;
    uint32_t end = 0;
    uint32_t end_addr = 0;
    memset(flash_array, 0, end);
    is_end = _log_seek_end(0x0000, &end_addr, seek_buffer, sizeof(seek_buffer));
    ASSERT(is_end);
    ASSERT_EQ(end, end_addr);
    PASS();
}

TEST test_seek_aligned_with_seek_buffer(void)
{
    // aligned with seek buffer
    uint8_t seek_buffer[4];
    bool is_end;
    uint32_t end = sizeof(seek_buffer);
    uint32_t end_addr = 0;
    memset(flash_array, 0, end);
    is_end = _log_seek_end(0x0000, &end_addr, seek_buffer, sizeof(seek_buffer));
    ASSERT(is_end);
    ASSERT_EQ(end, end_addr);
    PASS();
}

TEST test_seek_unaligned_plus_one(void)
{
    // not aligned with seek buffer, +1
    uint8_t seek_buffer[4];
    bool is_end;
    uint32_t end = sizeof(seek_buffer) + 1;
    uint32_t end_addr = 0;
    memset(flash_array, 0, end);
    is_end = _log_seek_end(0x0000, &end_addr, seek_buffer, sizeof(seek_buffer));
    ASSERT(is_end);
    ASSERT_EQ(end, end_addr);
    PASS();
}

TEST test_seek_unaligned_minus_one(void)
{
    // not aligned with seek buffer, -1
    uint8_t seek_buffer[4];
    bool is_end;
    uint32_t end = sizeof(seek_buffer) - 1;
    uint32_t end_addr = 0;
    memset(flash_array, 0, end);
    is_end = _log_seek_end(0x0000, &end_addr, seek_buffer, sizeof(seek_buffer));
    ASSERT(is_end);
    ASSERT_EQ(end, end_addr);
    PASS();
}

TEST test_seek_when_flash_is_full(void)
{
    // not aligned with seek buffer, -1
    uint8_t seek_buffer[4];
    bool is_end;
    uint32_t end = FLASH_SIZE;
    uint32_t end_addr;
    memset(flash_array, 0, end);
    is_end = _log_seek_end(0x0000, &end_addr, seek_buffer, sizeof(seek_buffer));
    ASSERT(!is_end);
    PASS();
}

TEST test_log_init_with_cleared_flash(void)
{
    FAIL();
}

TEST test_log_init_recover_from_reboot(void)
{
    FAIL();
}

TEST test_log_init_recover_full_backup_table(void)
{
    FAIL();
}

TEST test_log_init_detect_invalid_address(void)
{
    FAIL();
}

TEST test_position_backup(void)
{
    FAIL();
}

SUITE(log_flash_test)
{
    SET_SETUP(setup_cb, NULL);
    RUN_TEST(test_flash_erase_at_start_of_block);
    RUN_TEST(test_flash_erase_over_end_of_block);
    RUN_TEST(test_no_flash_erase_if_inside_block);
    RUN_TEST(test_no_flash_erase_if_at_end_of_block);
    RUN_TEST(test_find_backup_pos_in_erased_flash);
    RUN_TEST(test_detect_full_backup_table);
    RUN_TEST(test_find_backup_pos);
    RUN_TEST(test_seek_at_base_address);
    RUN_TEST(test_seek_aligned_with_seek_buffer);
    RUN_TEST(test_seek_unaligned_plus_one);
    RUN_TEST(test_seek_unaligned_minus_one);
    RUN_TEST(test_seek_when_flash_is_full);
    RUN_TEST(test_log_init_with_cleared_flash);
    RUN_TEST(test_log_init_recover_from_reboot);
    RUN_TEST(test_log_init_recover_full_backup_table);
    RUN_TEST(test_log_init_detect_invalid_address);
    RUN_TEST(test_position_backup);
}

GREATEST_MAIN_DEFS();

int main(int argc, char *argv[])
{
    GREATEST_MAIN_BEGIN();
    RUN_SUITE(log_entry_test);
    RUN_SUITE(log_flash_test);
    GREATEST_MAIN_END();
}
