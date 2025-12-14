/**
 * @file imu_task.cpp
 * @brief Implementation of the IMU sampling/publishing RTOS task.
 */

#include "tasks/imu_task.hpp"
#include "mbed.h"
#include "logger.hpp"
#include "main.hpp"


Mail<imu_data_t, 10> *imu_mail_box = nullptr;


void imu_task() {
    LOG_INFO("IMU Task Started");

    // Allocate the mailbox used to pass samples to other tasks (e.g., FFT).
    imu_mail_box = new Mail<imu_data_t, 10>();
    if (!imu_mail_box) {
        LOG_ERROR("Failed to create IMU mail box");
        trigger_fatal_error();
        return;
    }

    while (true) {
        // Block until the IMU raises its data-ready interrupt.
        if (imu_data_wait(1000)) {
            LOG_DEBUG("IMU data ready");

            imu_data_t *imu_data = imu_mail_box->try_alloc();
            if (imu_data != nullptr) {

                imu_data->timestamp = Kernel::Clock::now();

                if (!imu_read_acc_data(imu_data->accel)) {
                    LOG_WARN("Failed to read accel data");
                    imu_mail_box->free(imu_data);
                    continue;
                }

                if (!imu_read_gyro_data(imu_data->gyro)) {
                    LOG_WARN("Failed to read gyro data");
                    imu_mail_box->free(imu_data);
                    continue;
                }

                LOG_DEBUG("accel: %.2f, %.2f, %.2f | gyro: %.2f, %.2f, %.2f", imu_data->accel[0], imu_data->accel[1], imu_data->accel[2], imu_data->gyro[0], imu_data->gyro[1], imu_data->gyro[2]);
                // Publish sample to consumers; consumer is responsible for free().
                imu_mail_box->put(imu_data);
            } else {
                LOG_WARN("Failed to allocate IMU mail box");
                continue;
            }
        } else {
            LOG_FATAL("IMU data wait timeout");
            trigger_fatal_error();
            return;
        }
        ThisThread::sleep_for(1ms);
    }
}