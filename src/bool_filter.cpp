#include "bool_filter.hpp"


void bool_filter_init(bool_filter_t *filter, uint8_t filter_threshold) {
    filter->current_state = false;
    filter->target_state = false;
    filter->counter = 0;
    filter->threshold = filter_threshold;
}

bool bool_filter_update(bool_filter_t *filter, bool new_value) {
    // 如果新值与目标状态不同，重置计数器
    if (new_value != filter->target_state) {
        filter->target_state = new_value;
        filter->counter = 0;
    }
    
    // 如果目标状态与当前状态不同，累加计数器
    if (filter->target_state != filter->current_state) {
        filter->counter++;
        if (filter->counter >= filter->threshold) {
            filter->current_state = filter->target_state;
            filter->counter = 0;
        }
    }
    
    return filter->current_state;
}

bool bool_filter_get_state(const bool_filter_t *filter) {
    return filter->current_state;
}

void bool_filter_reset(bool_filter_t *filter, bool initial_state) {
    filter->current_state = initial_state;
    filter->target_state = initial_state;
    filter->counter = 0;
}
