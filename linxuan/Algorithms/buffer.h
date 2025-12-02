#ifndef ECEGY6483_BUFFER_H
#define ECEGY6483_BUFFER_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// 配置参数
#define RING_BUFFER_SIZE 256  // 缓冲区大小，建议为2的幂次方，便于优化
#define SENSOR_DATA_TYPE float  // 传感器数据类型

// 环形缓冲区结构体
typedef struct {
    SENSOR_DATA_TYPE buffer[RING_BUFFER_SIZE];  // 数据缓冲区
    volatile uint16_t write_index;              // 写指针
    volatile uint16_t read_index;               // 读指针（可选，用于其他读取操作）
    volatile uint16_t data_count;               // 当前数据量
    volatile bool overflow_flag;                // 溢出标志
    uint32_t total_written;                     // 总写入计数（用于调试）
} RingBuffer_t;

void RingBuffer_Init(RingBuffer_t *rb);
bool RingBuffer_Write(RingBuffer_t *rb, SENSOR_DATA_TYPE data);
bool RingBuffer_WriteBatch(RingBuffer_t *rb, const SENSOR_DATA_TYPE *data, uint16_t len);
uint16_t RingBuffer_GetWriteIndex(const RingBuffer_t *rb);
uint16_t RingBuffer_GetDataCount(const RingBuffer_t *rb);
bool RingBuffer_Copy(const RingBuffer_t *rb, SENSOR_DATA_TYPE *dest,
                           uint16_t length, uint16_t end_index);
void RingBuffer_Clear(RingBuffer_t *rb);

#endif //ECEGY6483_BUFFER_H