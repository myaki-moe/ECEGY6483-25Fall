#include "tasks/imu_task.hpp"
#include "mbed.h"
#include "bsp/imu.hpp"
#include "logger.hpp"

void imu_task() {
    LOG_INFO("IMU Task Started");

    while (true) {
        if (imu_data_wait()) {
            float acc[3];
            float gyro[3];
            if (imu_read_acc_data(acc)) {
                LOG_INFO("Acc: %.3f, %.3f, %.3f", acc[0], acc[1], acc[2]);
            }
            if (imu_read_gyro_data(gyro)) {
                LOG_INFO("Gyro: %.2f, %.2f, %.2f", gyro[0], gyro[1], gyro[2]);
            }
        }
    }
}