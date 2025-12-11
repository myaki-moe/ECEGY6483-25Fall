#pragma once

#include <stdint.h>
#include <time.h>
#include "mbed.h"
#include "arm_math.h"
#include "bsp/imu.hpp"


typedef struct imu_data_t {
    float32_t accel[3];
    float32_t gyro[3];
    std::chrono::time_point<rtos::Kernel::Clock> timestamp;
} imu_data_t;

extern Mail<imu_data_t, 10> *imu_mail_box;

void imu_task();