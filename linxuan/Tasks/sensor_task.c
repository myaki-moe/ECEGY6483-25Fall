#include "sensor_task.h"
#include "cmsis_os.h"
#include "semphr.h"
#include "logger.h"
#include "common.h"
#include "stm32l475e_iot01_accelero.h"
#include "stm32l475e_iot01_gyro.h"
#include <math.h>

extern osMutexId_t sensor_buffer_mutexHandle;

float gyroRaw[3];
int16_t accelRaw[3];
void SensorToBuffer(void) {
    BSP_GYRO_GetXYZ(gyroRaw);
    BSP_ACCELERO_AccGetXYZ(accelRaw);

    RingBuffer_Write(&accel_x_buffer, (float) accelRaw[0]/1000);
    RingBuffer_Write(&accel_y_buffer, (float) accelRaw[1]/1000);
    RingBuffer_Write(&accel_z_buffer, (float) accelRaw[2]/1000);
    RingBuffer_Write(&gyro_x_buffer, gyroRaw[0]/1000/360*2*M_PI);
    RingBuffer_Write(&gyro_y_buffer, gyroRaw[1]/1000/360*2*M_PI);
    RingBuffer_Write(&gyro_z_buffer, gyroRaw[2]/1000/360*2*M_PI);
}

void StartSensorTask(void *argument) {
    LOG_INFO("starting sensor acquisition task");

    RingBuffer_Init(&accel_x_buffer);
    RingBuffer_Init(&accel_y_buffer);
    RingBuffer_Init(&accel_z_buffer);
    RingBuffer_Init(&gyro_x_buffer);
    RingBuffer_Init(&gyro_y_buffer);
    RingBuffer_Init(&gyro_z_buffer);

    BSP_GYRO_Init();
    BSP_ACCELERO_Init();

    LOG_INFO("prefilling sensor readings");
    while (sensor_buffer_mutexHandle == NULL) {}
    while (xSemaphoreTake(sensor_buffer_mutexHandle, portMAX_DELAY) != pdTRUE) {}

    for (int i = 0; i < RING_BUFFER_SIZE; i++) {
        SensorToBuffer();
        osDelay(1000/SENSOR_RATE);
    }

    xSemaphoreGive(sensor_buffer_mutexHandle);
    LOG_INFO("prefilling sensor readings finished");

    while (1) {
        LOG_DEBUG("sensor acquisition start");

        if (xSemaphoreTake(sensor_buffer_mutexHandle, portMAX_DELAY) == pdTRUE) {
            SensorToBuffer();
            // 释放互斥锁
            xSemaphoreGive(sensor_buffer_mutexHandle);
            LOG_DEBUG("sensor acquisition complete");
        } else {
            LOG_DEBUG("failed to obtain buffer mutex");
        }
        osDelay(1000/SENSOR_RATE);
    }
}