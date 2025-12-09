#include "main.hpp"
#include "mbed.h"
#include "bsp/led.hpp"
#include "bsp/serial.hpp"
#include "bsp/imu.hpp"
#include "logger.hpp"
#include "tasks/imu_task.hpp"
#include "tasks/led_task.hpp"
#include "tasks/test_task.hpp"

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
    LOG_INFO("");
    LOG_INFO("Version: " BUILD_VERSION " ");
    LOG_INFO("Build Date: " __DATE__ " " __TIME__);
    LOG_INFO("Hardware initialization...");
    LOG_INFO("LED initialization [OK]");
    LOG_INFO("Serial initialization [OK]");

    if (!imu_init()) {
        LOG_ERROR("IMU initialization [FAIL]");
        hardware_error_handler();
    }
    LOG_INFO("IMU initialization [OK]");

    Thread imu_thread(osPriorityHigh, OS_STACK_SIZE, nullptr, "imu_task");
    Thread fft_thread(osPriorityNormal, OS_STACK_SIZE, nullptr, "fft_task");
    Thread led_thread(osPriorityLow, OS_STACK_SIZE, nullptr, "led_task");
    Thread test_thread(osPriorityLow, OS_STACK_SIZE, nullptr, "test_task");

    imu_thread.start(imu_task);
    // fft_thread.start(fft_task);
    led_thread.start(led_task);
    test_thread.start(test_task);

    while (true) { 
        ThisThread::sleep_for(2000ms);
    }
}