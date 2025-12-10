#pragma once


#include "mbed.h"
#include "arm_math.h"
#include "tasks/imu_task.hpp"


#define ANALYSIS_WINDOW_SIZE 3
#define FFT_BUFFER_SIZE (IMU_SAMPLE_RATE_HZ * ANALYSIS_WINDOW_SIZE)
#define FFT_BUFFER_NUM 2


typedef struct fft_result_t {
    float32_t accel_magnitude[3][FFT_BUFFER_SIZE / 2];
    float32_t gyro_magnitude[3][FFT_BUFFER_SIZE / 2];
    float32_t accel_psd[3][FFT_BUFFER_SIZE / 2];
    float32_t gyro_psd[3][FFT_BUFFER_SIZE / 2];
    std::chrono::time_point<rtos::Kernel::Clock> timestamp;
    Mutex mutex;
} fft_result_t;

void fft_task();
fft_result_t *fft_find_and_lock_oldest_result();
fft_result_t *fft_find_and_lock_latest_result();