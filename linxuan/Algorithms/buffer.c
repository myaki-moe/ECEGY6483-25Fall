#include "buffer.h"

/**
 * @brief 初始化环形缓冲区
 */
void RingBuffer_Init(RingBuffer_t *rb) {
    rb->write_index = 0;
    rb->read_index = 0;
    rb->data_count = 0;
    rb->overflow_flag = false;
    rb->total_written = 0;
    memset((void*)rb->buffer, 0, sizeof(rb->buffer));
}

/**
 * @brief 写入单个数据
 */
bool RingBuffer_Write(RingBuffer_t *rb, SENSOR_DATA_TYPE data) {
    // 写入数据
    rb->buffer[rb->write_index] = data;

    // 更新写指针（使用模运算，如果RING_BUFFER_SIZE是2的幂次方，可优化为位运算）
    rb->write_index = (rb->write_index + 1) % RING_BUFFER_SIZE;
    // 优化版本：rb->write_index = (rb->write_index + 1) & (RING_BUFFER_SIZE - 1);

    // 更新数据计数
    if (rb->data_count < RING_BUFFER_SIZE) {
        rb->data_count++;
    } else {
        rb->overflow_flag = true;  // 标记溢出
        // 如果满了，读指针也需要移动（覆盖最旧的数据）
        rb->read_index = (rb->read_index + 1) % RING_BUFFER_SIZE;
    }

    rb->total_written++;

    return true;
}

/**
 * @brief 批量写入数据
 */
bool RingBuffer_WriteBatch(RingBuffer_t *rb, const SENSOR_DATA_TYPE *data, uint16_t len) {
    if (len == 0 || data == NULL) {
        return false;
    }

    for (uint16_t i = 0; i < len; i++) {
        rb->buffer[rb->write_index] = data[i];
        rb->write_index = (rb->write_index + 1) % RING_BUFFER_SIZE;

        if (rb->data_count < RING_BUFFER_SIZE) {
            rb->data_count++;
        } else {
            rb->overflow_flag = true;
            rb->read_index = (rb->read_index + 1) % RING_BUFFER_SIZE;
        }
        rb->total_written++;
    }

    return true;
}

/**
 * @brief 获取当前写指针位置
 */
uint16_t RingBuffer_GetWriteIndex(const RingBuffer_t *rb) {
    return rb->write_index;
}

/**
 * @brief 获取当前数据量
 */
uint16_t RingBuffer_GetDataCount(const RingBuffer_t *rb) {
    return rb->data_count;
}

/**
 * @brief 复制并组装数据
 * @param rb 环形缓冲区指针
 * @param dest 目标数组
 * @param length 需要复制的数据长度
 * @param end_index 结束位置（通常是当前写指针位置）
 * @return 成功返回true，失败返回false
 *
 * 注意：这个函数会按时间顺序复制数据，最新的数据在数组末尾
 */
bool RingBuffer_Copy(const RingBuffer_t *rb, SENSOR_DATA_TYPE *dest,
                           uint16_t length, uint16_t end_index) {
    if (dest == NULL || length == 0 || length > RING_BUFFER_SIZE) {
        return false;
    }

    // 检查是否有足够的数据
    if (rb->data_count < length) {
        return false;
    }

    // 计算起始位置（从end_index向前推length个位置）
    // start_index是最旧的数据位置
    uint16_t start_index;
    if (end_index >= length) {
        start_index = end_index - length;
    } else {
        start_index = RING_BUFFER_SIZE - (length - end_index);
    }

    // 复制数据（处理回环情况）
    if (start_index < end_index) {
        // 数据不跨越缓冲区边界，直接复制
        memcpy(dest, (void*)&rb->buffer[start_index], length * sizeof(SENSOR_DATA_TYPE));
    } else {
        // 数据跨越缓冲区边界，分两段复制
        uint16_t first_part = RING_BUFFER_SIZE - start_index;
        uint16_t second_part = length - first_part;

        memcpy(dest, (void*)&rb->buffer[start_index], first_part * sizeof(SENSOR_DATA_TYPE));
        memcpy(dest + first_part, (void*)&rb->buffer[0], second_part * sizeof(SENSOR_DATA_TYPE));
    }

    return true;
}

/**
 * @brief 清空缓冲区
 */
void RingBuffer_Clear(RingBuffer_t *rb) {
    rb->write_index = 0;
    rb->read_index = 0;
    rb->data_count = 0;
    rb->overflow_flag = false;
}