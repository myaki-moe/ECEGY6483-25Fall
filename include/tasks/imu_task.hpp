#pragma once

/**
 * @file imu_task.hpp
 * @brief RTOS task that reads IMU data and publishes samples to a mailbox.
 */

#include <stdint.h>
#include <time.h>
#include "mbed.h"
#include "arm_math.h"
#include "bsp/imu.hpp"


/**
 * @brief One timestamped IMU sample (3-axis accel + gyro).
 *
 * Accel units depend on ACC_SENSITIVITY in the IMU BSP.
 * Gyro is converted to rad/s in `imu_read_gyro_data()`.
 */
typedef struct imu_data_t {
    float32_t accel[3];
    float32_t gyro[3];
    std::chrono::time_point<rtos::Kernel::Clock> timestamp;
} imu_data_t;

/**
 * @brief Global mailbox for IMU samples (allocated in imu_task()).
 *
 * Producer: imu_task() pushes new samples.
 * Consumers: fft_task() (and others) pop samples and must free them.
 */
extern Mail<imu_data_t, 10> *imu_mail_box;

/**
 * @brief RTOS task entry: wait for IMU data-ready and publish samples.
 */
void imu_task();
