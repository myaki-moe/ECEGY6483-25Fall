#pragma once

#include <stdint.h>

typedef struct {
    bool current_state;
    bool target_state;
    uint8_t counter;
    uint8_t threshold;
} bool_filter_t;

// 初始化滤波器
void bool_filter_init(bool_filter_t *filter, uint8_t filter_threshold);

// 更新滤波器，返回滤波后的状态
bool bool_filter_update(bool_filter_t *filter, bool new_value);

// 获取当前状态
bool bool_filter_get_state(const bool_filter_t *filter);

// 重置滤波器
void bool_filter_reset(bool_filter_t *filter, bool initial_state);
