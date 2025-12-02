#ifndef ECEGY6483_COMMON_H
#define ECEGY6483_COMMON_H

#include "buffer.h"

#define SENSOR_RATE 208

extern RingBuffer_t accel_x_buffer;
extern RingBuffer_t accel_y_buffer;
extern RingBuffer_t accel_z_buffer;
extern RingBuffer_t gyro_x_buffer;
extern RingBuffer_t gyro_y_buffer;
extern RingBuffer_t gyro_z_buffer;

#endif //ECEGY6483_COMMON_H