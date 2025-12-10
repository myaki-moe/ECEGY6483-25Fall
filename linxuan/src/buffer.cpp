#include "buffer.hpp"
#include <stdlib.h>
#include <string.h>

/**
 * 创建镜像缓冲区
 * 
 * @param window_size  窗口大小（需要保存多少个数据点）
 * @param element_size 单个元素大小（如 sizeof(float)）
 * @return 缓冲区句柄，失败返回NULL
 * 
 * 示例：
 *   mirror_buffer_t *buf = mirror_buffer_create(156, sizeof(float));
 */
 mirror_buffer_t* mirror_buffer_create(size_t window_size, size_t element_size) {
    mirror_buffer_t *mb = (mirror_buffer_t*)malloc(sizeof(mirror_buffer_t));
    if (!mb) return NULL;
    
    // 分配2倍空间（镜像区）
    size_t buffer_bytes = window_size * 2 * element_size;
    mb->buffer = malloc(buffer_bytes);
    if (!mb->buffer) {
        free(mb);
        return NULL;
    }
    
    // 初始化为0
    memset(mb->buffer, 0, buffer_bytes);
    
    mb->window_size = window_size;
    mb->element_size = element_size;
    mb->write_index = 0;
    
    return mb;
}

/**
 * 销毁镜像缓冲区
 */
void mirror_buffer_destroy(mirror_buffer_t *mb) {
    if (mb) {
        if (mb->buffer) free(mb->buffer);
        free(mb);
    }
}

/**
 * 写入单个数据点
 * 
 * @param mb   缓冲区句柄
 * @param data 数据指针（指向要写入的单个元素）
 * 
 * 示例：
 *   float value = 1.23f;
 *   mirror_buffer_push(buf, &value);
 */
void mirror_buffer_push(mirror_buffer_t *mb, const void *data) {
    if (!mb || !data) return;
    
    // 计算两个镜像位置的字节偏移
    size_t offset1 = mb->write_index * mb->element_size;
    size_t offset2 = (mb->write_index + mb->window_size) * mb->element_size;
    
    // 同时写入两个位置
    uint8_t *buf_ptr = (uint8_t*)mb->buffer;
    memcpy(buf_ptr + offset1, data, mb->element_size);
    memcpy(buf_ptr + offset2, data, mb->element_size);
    
    // 更新写指针（循环）
    mb->write_index = (mb->write_index + 1) % mb->window_size;
}

/**
 * 获取连续窗口指针（Zero-Copy！）
 * 
 * @param mb 缓冲区句柄
 * @return 指向最新window_size个数据点的连续指针
 * 
 * 示例：
 *   float *window = (float*)mirror_buffer_get_window(buf);
 *   // window[0]是最旧的，window[window_size-1]是最新的
 *   arm_rfft_fast_f32(&S, window, output, 0);
 */
void* mirror_buffer_get_window(mirror_buffer_t *mb) {
    if (!mb) return NULL;
    
    // 从当前写指针开始，往前数window_size个点就是完整窗口
    uint8_t *buf_ptr = (uint8_t*)mb->buffer;
    size_t offset = mb->write_index * mb->element_size;
    
    return (void*)(buf_ptr + offset);
}

/**
 * 获取指定偏移的窗口指针
 * 
 * @param mb     缓冲区句柄
 * @param offset 相对最新数据的偏移（0=最新，1=前一个周期...）
 * @return 连续窗口指针
 * 
 * 示例：
 *   // 获取上一个3秒窗口
 *   float *prev_window = (float*)mirror_buffer_get_window_offset(buf, 1);
 */
void* mirror_buffer_get_window_offset(mirror_buffer_t *mb, uint32_t offset) {
    if (!mb) return NULL;
    
    uint32_t read_index = (mb->write_index - offset + mb->window_size) % mb->window_size;
    uint8_t *buf_ptr = (uint8_t*)mb->buffer;
    size_t byte_offset = read_index * mb->element_size;
    
    return (void*)(buf_ptr + byte_offset);
}
