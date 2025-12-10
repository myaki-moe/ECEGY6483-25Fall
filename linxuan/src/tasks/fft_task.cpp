#include "tasks/fft_task.hpp"
#include "mbed.h"
#include "arm_math.h"
#include "logger.hpp"
#include "buffer.hpp"
#include "tasks/imu_task.hpp"
#include "main.hpp"

void fft_task() {
    LOG_INFO("FFT Task Started");

    mirror_buffer_t *accel_x_buffer = mirror_buffer_create(IMU_SAMPLE_RATE_HZ * 3, sizeof(float));
    mirror_buffer_t *accel_y_buffer = mirror_buffer_create(IMU_SAMPLE_RATE_HZ * 3, sizeof(float));
    mirror_buffer_t *accel_z_buffer = mirror_buffer_create(IMU_SAMPLE_RATE_HZ * 3, sizeof(float));
    mirror_buffer_t *gyro_x_buffer = mirror_buffer_create(IMU_SAMPLE_RATE_HZ * 3, sizeof(float));
    mirror_buffer_t *gyro_y_buffer = mirror_buffer_create(IMU_SAMPLE_RATE_HZ * 3, sizeof(float));
    mirror_buffer_t *gyro_z_buffer = mirror_buffer_create(IMU_SAMPLE_RATE_HZ * 3, sizeof(float));
    if (!accel_x_buffer || !accel_y_buffer || !accel_z_buffer || !gyro_x_buffer || !gyro_y_buffer || !gyro_z_buffer) {
        LOG_ERROR("Failed to create buffer");
        trigger_fatal_error();
        return;
    }

    while (true) {
        while (!imu_mail_box->empty()) {
            imu_data_t *imu_data = imu_mail_box->try_get();
            if (imu_data != nullptr) {
                mirror_buffer_push(accel_x_buffer, &imu_data->accel[0]);
                mirror_buffer_push(accel_y_buffer, &imu_data->accel[1]);
                mirror_buffer_push(accel_z_buffer, &imu_data->accel[2]);
                mirror_buffer_push(gyro_x_buffer, &imu_data->gyro[0]);
                mirror_buffer_push(gyro_y_buffer, &imu_data->gyro[1]);
                mirror_buffer_push(gyro_z_buffer, &imu_data->gyro[2]);
                imu_mail_box->free(imu_data);
                LOG_INFO("IMU data processed");
            } else {
                LOG_ERROR("Failed to get IMU data");
                imu_mail_box->free(imu_data);
                continue;
            }
        }
        ThisThread::sleep_for(1ms);
    }
}