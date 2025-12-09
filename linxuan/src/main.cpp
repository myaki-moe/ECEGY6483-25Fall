#include "mbed.h"
#include "bsp/led.hpp"
#include "bsp/serial.hpp"
#include "bsp/imu.hpp"
#include "logger.hpp"

void hardware_error_handler() {
    while (true) {
        led_green_1_set(1);
        led_green_2_set(1);
        led_blue_yellow_on();
        ThisThread::sleep_for(250ms);
        led_green_1_set(0);
        led_green_2_set(0);
        led_blue_yellow_off();
        ThisThread::sleep_for(250ms);
    }
}

int main() {

    if (!led_init()) {
        hardware_error_handler();
    }
    if (!serial_init()) {
        hardware_error_handler();
    }

    LOG_INFO("");
    LOG_INFO("========================================");
    LOG_INFO("  Parkinson's Motion Detection System  ");
    LOG_INFO("     \"Shake, Rattle, and Roll\"       ");
    LOG_INFO("========================================");
    LOG_INFO("");
    LOG_INFO("  PROJECT: Embedded Challenge Fall 2025");
    LOG_INFO("  GROUP: 46");
    LOG_INFO("");
    LOG_INFO("  TEAM MEMBERS:");
    LOG_INFO("    - Banerjee, Janosia");
    LOG_INFO("    - Biao, Linxuan");
    LOG_INFO("    - Chang, Kyle");
    LOG_INFO("    - Xu, Lixuan");
    LOG_INFO("");
    LOG_INFO("========================================");
    LOG_INFO("Build Date: " __DATE__ " " __TIME__);
    LOG_INFO("");
    LOG_INFO("Serial initialization [OK]");
    LOG_INFO("LED initialization [OK]");

    if (!imu_init()) {
        LOG_ERROR("IMU initialization [FAIL]");
        hardware_error_handler();
    }
    LOG_INFO("IMU initialization [OK]");

    while (true) {
        if (imu_data_ready()) {
            imu_data_ready_clear();
            float acc[3];
            float gyro[3];
            if (imu_read_acc_data(acc)) {
                // LOG_INFO("Acc: %.3f, %.3f, %.3f", acc[0], acc[1], acc[2]);
            }
            if (imu_read_gyro_data(gyro)) {
                // LOG_INFO("Gyro: %.2f, %.2f, %.2f", gyro[0], gyro[1], gyro[2]);
            }
        }
        ThisThread::sleep_for(1ms);
    }
}