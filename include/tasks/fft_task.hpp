#pragma once

/**
 * @file fft_task.hpp
 * @brief FFT/PSD processing task and shared result buffers.
 */

#include "mbed.h"
#include "arm_math.h"
#include "tasks/imu_task.hpp"


/**
 * @brief Number of IMU samples per FFT window.
 *
 * With sampling rate IMU_SAMPLE_RATE_HZ, the frequency resolution is:
 *   df = IMU_SAMPLE_RATE_HZ / FFT_BUFFER_SIZE
 */
#define FFT_BUFFER_SIZE 256

/**
 * @brief Number of FFT result buffers (double-buffering).
 *
 * One task writes the oldest available buffer while another task reads the
 * latest available buffer. Mutexes are used to avoid partial reads/writes.
 */
#define FFT_BUFFER_NUM 2


/**
 * @brief FFT output container (per-axis) with a timestamp and mutex.
 *
 * Arrays are sized to FFT_BUFFER_SIZE/2 because the real-input FFT produces a
 * single-sided spectrum (0..Nyquist). Each PSD element corresponds to one
 * frequency bin of width (IMU_SAMPLE_RATE_HZ / FFT_BUFFER_SIZE).
 */
typedef struct fft_result_t {
    float32_t accel_magnitude[3][FFT_BUFFER_SIZE / 2];
    float32_t gyro_magnitude[3][FFT_BUFFER_SIZE / 2];
    float32_t accel_psd[3][FFT_BUFFER_SIZE / 2];
    float32_t gyro_psd[3][FFT_BUFFER_SIZE / 2];
    std::chrono::time_point<rtos::Kernel::Clock> timestamp;
    Mutex mutex;
} fft_result_t;

/**
 * @brief RTOS task that consumes IMU samples and computes FFT/PSD.
 *
 * Input: IMU mailbox (see `imu_task.hpp`). Output: fft_results buffers
 * protected by per-buffer mutexes.
 */
void fft_task();

/**
 * @brief Find a writable result buffer and lock it.
 * @return Pointer to the locked buffer, or nullptr if none available.
 */
fft_result_t *fft_find_and_lock_oldest_result();

/**
 * @brief Find the newest computed result buffer and lock it for reading.
 * @return Pointer to the locked buffer, or nullptr if none available.
 */
fft_result_t *fft_find_and_lock_latest_result();
