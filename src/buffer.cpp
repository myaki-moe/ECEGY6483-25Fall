#include "buffer.hpp"
#include <stdlib.h>
#include <string.h>

/**
 * @file buffer.cpp
 * @brief Implementation of the mirror circular buffer.
 */

/**
 * @brief Create a mirror buffer.
 *
 * The buffer allocates 2*window_size elements. Each pushed element is written
 * twice: at index i and i+window_size. This guarantees the last window is
 * always contiguous in memory.
 *
 * @param window_size Window length (elements).
 * @param element_size Element size in bytes.
 * @return Buffer handle, or NULL on allocation failure.
 */
mirror_buffer_t* mirror_buffer_create(size_t window_size, size_t element_size) {
    mirror_buffer_t *mb = (mirror_buffer_t*)malloc(sizeof(mirror_buffer_t));
    if (!mb) return NULL;
    
    // Allocate 2x storage (mirror region).
    size_t buffer_bytes = window_size * 2 * element_size;
    mb->buffer = malloc(buffer_bytes);
    if (!mb->buffer) {
        free(mb);
        return NULL;
    }
    
    // Initialize memory to 0 for predictable startup behavior.
    memset(mb->buffer, 0, buffer_bytes);
    
    mb->window_size = window_size;
    mb->element_size = element_size;
    mb->write_index = 0;
    
    return mb;
}

/**
 * @brief Destroy a mirror buffer.
 */
void mirror_buffer_destroy(mirror_buffer_t *mb) {
    if (mb) {
        if (mb->buffer) free(mb->buffer);
        free(mb);
    }
}

/**
 * @brief Push one element into the buffer.
 * @param mb Buffer handle.
 * @param data Pointer to a single element to store.
 */
void mirror_buffer_push(mirror_buffer_t *mb, const void *data) {
    if (!mb || !data) return;
    
    // Compute byte offsets for the primary and mirrored positions.
    size_t offset1 = mb->write_index * mb->element_size;
    size_t offset2 = (mb->write_index + mb->window_size) * mb->element_size;
    
    // Write to both positions so a contiguous window is always available.
    uint8_t *buf_ptr = (uint8_t*)mb->buffer;
    memcpy(buf_ptr + offset1, data, mb->element_size);
    memcpy(buf_ptr + offset2, data, mb->element_size);
    
    // Advance write pointer (circular).
    mb->write_index = (mb->write_index + 1) % mb->window_size;
}

/**
 * @brief Get a contiguous pointer to the current window (zero-copy).
 * @param mb Buffer handle.
 * @return Pointer to the window (oldest -> newest).
 */
void* mirror_buffer_get_window(mirror_buffer_t *mb) {
    if (!mb) return NULL;
    
    // Because of mirroring, starting at write_index yields a contiguous window.
    uint8_t *buf_ptr = (uint8_t*)mb->buffer;
    size_t offset = mb->write_index * mb->element_size;
    
    return (void*)(buf_ptr + offset);
}

/**
 * @brief Get a contiguous pointer to a previous window.
 * @param mb Buffer handle.
 * @param offset Window offset in full cycles (0=current, 1=previous, ...).
 * @return Pointer to the requested window (oldest -> newest).
 */
void* mirror_buffer_get_window_offset(mirror_buffer_t *mb, uint32_t offset) {
    if (!mb) return NULL;
    
    uint32_t read_index = (mb->write_index - offset + mb->window_size) % mb->window_size;
    uint8_t *buf_ptr = (uint8_t*)mb->buffer;
    size_t byte_offset = read_index * mb->element_size;
    
    return (void*)(buf_ptr + byte_offset);
}
