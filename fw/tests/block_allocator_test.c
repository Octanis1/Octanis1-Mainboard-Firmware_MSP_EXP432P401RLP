#include "greatest/greatest.h"
#include <lib/block_allocator.h>

int block_count;
block_allocator_t alloc;

typedef stuct {
    int foo;
    char bar;
} object;

#define N_BLOCKS 4
object object_buffer[N_BLOCKS];

// Allocator usage

TEST can_use_entire_buffer(void)
{
    void *p;
    // allocate all available blocks
    int i;
    for (i = 0; i < N_BLOCKS; i++) {
        p = block_allocate(&allocator);
        ASSERT(p != NULL);
    }
    // allocate from empty pool
    p = block_allocate(&allocator);
    ASSERT(p == NULL);
    PASS();
}

TEST can_reuse_block(void)
{
    object *o = (object *)block_allocate(&allocator);
    o->foo = 42;
    // epmty memory pool
    while(block_allocate(&allocator) != NULL);

    // recycle an object
    block_free(o);
    o = (object *)block_allocate(&allocator);

    ASSERT(o != NULL);
    ASSERT_EQ(o->foo, 42); // should be the same object
    PASS();
}

static void setup_cb(void *data)
{
    block_count = block_allocator_init(&alloc, sizeof(object),
                                       object_buffer, sizeof(object_buffer));
}

SUITE(allocator_usage_test_suite)
{
    SET_SETUP(setup_cb, NULL);

    RUN_TEST(can_allocate_and_free);
    RUN_TEST(can_reuse_block);
}

// Allocator internals

TEST correct_block_size(void)
{
    uint8_t buffer[3 * sizeof(void *)];
    int n = block_allocator_init(&alloc, 3, buffer, sizeof(buffer));
    ASSERT_EQ(n, 3);
    ASSERT_EQ(alloc->_block_size, sizeof(void *));
    PASS();
}

TEST correct_alignment(void)
{
    const size_t ptr_size = sizeof(void *);
    void *buffer[3]; // buffer of 3 pointers

    void *p = (void *)((uintptr_t)&buffer[0] + 1); // create unaligned pointer inside buffer
    int n = block_allocator_init(&alloc, ptr_size, p, sizeof(buffer) - 2);
    ASSERT_EQ(n, 1);
    ASSERT_EQ(alloc->_block_size, ptr_size);

    // last and first element from buffer should be discarded due to alignment
    ASSERT_EQ(alloc->_list, &buffer[1]);
    ASSERT_EQ(alloc->_list->next, NULL);

    PASS();
}

SUITE(allocator_internals_test_suite)
{
    RUN_TEST(correct_block_size);
    RUN_TEST(correct_alignment);
}

// Test runner

GREATEST_MAIN_DEFS();

int main(int argc, char *argv[])
{
    GREATEST_MAIN_BEGIN();
    RUN_SUITE(allocator_usage_test_suite);
    RUN_SUITE(allocator_internals_test_suite);
    GREATEST_MAIN_END();
}