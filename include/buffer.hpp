#pragma once

#include <stddef.h>
#include <stdint.h>

/**
 * 双倍镜像循环缓冲区结构
 */
 typedef struct mirror_buffer_t {
    void *buffer;           // 实际缓冲区（2×window_size）
    size_t window_size;     // 窗口大小（数据点数量）
    size_t element_size;    // 单个元素字节数（如sizeof(float)）
    uint32_t write_index;   // 写指针（0 ~ window_size-1 循环）
} mirror_buffer_t;

mirror_buffer_t* mirror_buffer_create(size_t window_size, size_t element_size);
void mirror_buffer_destroy(mirror_buffer_t *mb);
void mirror_buffer_push(mirror_buffer_t *mb, const void *data);
void* mirror_buffer_get_window(mirror_buffer_t *mb);
void* mirror_buffer_get_window_offset(mirror_buffer_t *mb, uint32_t offset);
