#ifndef BLOCK_ALLOCATOR_H
#define BLOCK_ALLOCATOR_H

#include <stddef.h>

struct _memory_block {
    struct _memory_block *next;
};

typedef struct {
    struct _memory_block *list;
    size_t _block_size; // objectsize + padding
} block_allocator_t;

#ifdef __cplusplus
extern "C" {
#endif

// Initialize the allocator for object size with given buffer
// Returns the number of blocks that could be created from given buffer
// Note: The memory blocks will automatically address size aligned (eg. 4byte on a 32bit system).
//       For optimal memory usage pass an aligned buffer.
int block_allocator_init(block_allocator_t *alloc, size_t object_size,
                         void *buf, size_t len);

// returns NULL if no block available
void *block_allocate(block_allocator_t *alloc);

// frees the object
void block_free(block_allocator_t *alloc, void *object);

#ifdef __cplusplus
}
#endif

#endif /* BLOCK_ALLOCATOR_H */
