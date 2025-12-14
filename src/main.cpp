/**
 * @file main.cpp
 * @brief System entry point: hardware init, task startup, fatal error handling.
 */

#include "main.hpp"
#include "mbed.h"
#include "bsp/led.hpp"
#include "bsp/serial.hpp"
#include "bsp/imu.hpp"
#include "logger.hpp"
#include "tasks/imu_task.hpp"
#include "tasks/led_task.hpp"
#include "tasks/test_task.hpp"
#include "tasks/fft_task.hpp"
#include "tasks/analysis_task.hpp"
#include "tasks/ble_task.hpp"


EventFlags *program_fatal_error_flag = nullptr;

/**
 * @brief Last-resort handler for unrecoverable failures.
 *
 * This function never returns. It provides a visible LED pattern and keeps
 * printing a fatal log message to help debugging on the bench.
 */
void fatal_error_handler() {
    while (true) {
        led_green_1_set(1);
        led_green_2_set(1);
        led_blue_yellow_on();
        ThisThread::sleep_for(500ms);
        led_green_1_set(0);
        led_green_2_set(0);
        led_blue_yellow_off();
        ThisThread::sleep_for(500ms);
        LOG_FATAL("**FATAL ERROR**");
    }
}

void trigger_fatal_error() {
    program_fatal_error_flag->set(1);
}


int main() {

    // Bring up minimal I/O first so we can signal failures early.
    if (!led_init()) {
        fatal_error_handler();
    }
    if (!serial_init()) {
        fatal_error_handler();
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
        LOG_FATAL("IMU initialization [FAIL]");
        fatal_error_handler();
    }
    LOG_INFO("IMU initialization [OK]");

    // Allocated after basic init; tasks use this to request a global shutdown.
    program_fatal_error_flag = new EventFlags();

    LOG_INFO("Starting tasks...");
    // Task priorities reflect timing sensitivity:
    // - IMU sampling must be most deterministic (Realtime).
    // - FFT and analysis should run promptly on fresh sensor data (High).
    // - LED/BLE are user-facing and can be lower priority (Normal).
    // - Test task is non-critical (Low).
    Thread imu_thread(osPriorityRealtime, OS_STACK_SIZE, nullptr, "imu_task");
    Thread fft_thread(osPriorityHigh, OS_STACK_SIZE, nullptr, "fft_task");
    Thread analysis_thread(osPriorityHigh, OS_STACK_SIZE, nullptr, "analysis_task");
    Thread led_thread(osPriorityNormal, OS_STACK_SIZE, nullptr, "led_task");
    Thread ble_thread(osPriorityNormal, OS_STACK_SIZE, nullptr, "ble_task");
    Thread test_thread(osPriorityLow, OS_STACK_SIZE, nullptr, "test_task");

    imu_thread.start(imu_task);
    fft_thread.start(fft_task);
    analysis_thread.start(analysis_task);
    led_thread.start(led_task);
    ble_thread.start(ble_task);
    test_thread.start(test_task);

    LOG_INFO("Tasks startup complete");

    // Main thread becomes the "supervisor": wait for any fatal error request,
    // then stop all tasks and fall into the fatal handler.
    if (program_fatal_error_flag->wait_all(1, osWaitForever) == 1) {
        LOG_FATAL("Program fatal error, terminating all tasks");

        Thread* threads[] = { &imu_thread, &fft_thread, &analysis_thread, &led_thread, &ble_thread, &test_thread };
        for (Thread* t : threads) {
            if (t->get_state() != Thread::Deleted && t->get_state() != Thread::Inactive) {
                t->terminate();
            }
        }
        fatal_error_handler();
    }

    return 0;
}
