#pragma once

#include <stdint.h>
#include <time.h>
#include "mbed.h"


#define IMU_SAMPLE_RATE_HZ 52.0f

typedef struct imu_data_t {
    float accel[3];
    float gyro[3];
    std::chrono::time_point<rtos::Kernel::Clock> timestamp;
} imu_data_t;

extern Mail<imu_data_t, 10> *imu_mail_box;

void imu_task();