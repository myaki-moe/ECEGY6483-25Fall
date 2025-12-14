#pragma once

#include <stdint.h>

/**
 * @file bool_filter.hpp
 * @brief Simple boolean debouncer / hysteresis filter.
 *
 * Many detectors can be noisy frame-to-frame. This filter requires the new
 * value to be observed consistently for N updates (threshold) before the
 * output state changes.
 */

/**
 * @brief Debounced boolean state container.
 */
typedef struct {
    bool current_state;
    bool target_state;
    uint8_t counter;
    uint8_t threshold;
} bool_filter_t;

/**
 * @brief Initialize a boolean filter.
 * @param filter Filter instance.
 * @param filter_threshold Number of consecutive updates required to switch.
 */
void bool_filter_init(bool_filter_t *filter, uint8_t filter_threshold);

/**
 * @brief Update the filter with a new raw value.
 * @param filter Filter instance.
 * @param new_value New raw boolean input.
 * @return Filtered (debounced) output state.
 */
bool bool_filter_update(bool_filter_t *filter, bool new_value);

/**
 * @brief Get the current filtered state.
 * @param filter Filter instance.
 * @return Current debounced state.
 */
bool bool_filter_get_state(const bool_filter_t *filter);

/**
 * @brief Reset the filter state immediately.
 * @param filter Filter instance.
 * @param initial_state Value to force as current and target state.
 */
void bool_filter_reset(bool_filter_t *filter, bool initial_state);
