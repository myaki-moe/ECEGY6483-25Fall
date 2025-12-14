#pragma once

#include <stddef.h>
#include <stdint.h>

/**
 * @file buffer.hpp
 * @brief Mirror (double-mapped) circular buffer for zero-copy sliding windows.
 *
 * This buffer stores each element twice in a contiguous memory region of
 * length 2*window_size. This makes the last `window_size` samples always
 * available as one contiguous array, which is convenient for DSP routines
 * (e.g., FFT) without doing an extra copy or wrap-around handling.
 */

/**
 * @brief Mirror buffer handle (not thread-safe).
 *
 * The caller must ensure proper synchronization if accessed from multiple
 * threads/ISRs.
 */
typedef struct mirror_buffer_t {
    void *buffer;           /**< Backing storage (2 * window_size elements). */
    size_t window_size;     /**< Window length (number of elements). */
    size_t element_size;    /**< Size of one element in bytes. */
    uint32_t write_index;   /**< Next write index (wraps 0..window_size-1). */
} mirror_buffer_t;

/**
 * @brief Create a mirror buffer.
 * @param window_size Number of elements in the sliding window.
 * @param element_size Size of one element in bytes (e.g. sizeof(float32_t)).
 * @return Pointer to buffer handle, or NULL on allocation failure.
 */
mirror_buffer_t* mirror_buffer_create(size_t window_size, size_t element_size);

/**
 * @brief Destroy a mirror buffer and free its memory.
 * @param mb Buffer handle (can be NULL).
 */
void mirror_buffer_destroy(mirror_buffer_t *mb);

/**
 * @brief Push one element into the sliding window.
 * @param mb Buffer handle.
 * @param data Pointer to a single element to copy in.
 */
void mirror_buffer_push(mirror_buffer_t *mb, const void *data);

/**
 * @brief Get a pointer to the contiguous window (oldest -> newest).
 *
 * The returned pointer is valid until the next push. No copying is performed.
 *
 * @param mb Buffer handle.
 * @return Pointer to a contiguous array of `window_size` elements.
 */
void* mirror_buffer_get_window(mirror_buffer_t *mb);

/**
 * @brief Get a pointer to a previous window.
 *
 * Offset is relative to the current newest window:
 * - offset = 0: current window
 * - offset = 1: previous window (one full cycle ago)
 *
 * @param mb Buffer handle.
 * @param offset Window offset in cycles.
 * @return Pointer to a contiguous array of `window_size` elements.
 */
void* mirror_buffer_get_window_offset(mirror_buffer_t *mb, uint32_t offset);
