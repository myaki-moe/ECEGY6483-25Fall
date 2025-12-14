/**
 * @file fft_task.cpp
 * @brief Implementation of FFT/PSD processing and result-buffer management.
 *
 * Data flow (high level):
 * - `imu_task` publishes samples to `imu_mail_box`.
 * - This task maintains a sliding window of the latest FFT_BUFFER_SIZE samples
 *   per axis using mirror buffers.
 * - For each new sample, it computes a real FFT and derives single-sided
 *   magnitude spectrum and PSD (power spectral density) for accel and gyro.
 * - Results are stored in a small ring of `fft_result_t` buffers protected by
 *   per-buffer mutexes.
 */

#include "tasks/fft_task.hpp"
#include "mbed.h"
#include "arm_math.h"
#include "logger.hpp"
#include "buffer.hpp"
#include "tasks/imu_task.hpp"
#include "main.hpp"


arm_rfft_fast_instance_f32 fft_handler;
/**
 * @brief PSD normalization scale factor.
 *
 * We square the magnitude spectrum to get power and then scale it so that the
 * values are comparable across configurations. This is a simple normalization
 * based on window length and sampling rate.
 */
float32_t scale_factor = 1.0f / (FFT_BUFFER_SIZE * IMU_SAMPLE_RATE_HZ);

mirror_buffer_t *accel_sensor_data_buffer[3];
mirror_buffer_t *gyro_sensor_data_buffer[3];
float32_t fft_input[FFT_BUFFER_SIZE];
float32_t fft_output[FFT_BUFFER_SIZE];
fft_result_t fft_results[FFT_BUFFER_NUM];


/**
 * @brief RTOS task entry: compute FFT/PSD continuously from IMU samples.
 *
 * Notes:
 * - We wait until we have a full window (FFT_BUFFER_SIZE samples). After that,
 *   each incoming sample updates the sliding window and triggers a new FFT.
 * - CMSIS-DSP `arm_rfft_fast_f32` computes an efficient real-input FFT.
 * - We store only the single-sided spectrum (0..Nyquist), hence N/2 bins.
 */
void fft_task() {
    LOG_INFO("FFT Task Started");

    for (int i = 0; i < 3; i++) {
        accel_sensor_data_buffer[i] = mirror_buffer_create(FFT_BUFFER_SIZE, sizeof(float32_t));
        gyro_sensor_data_buffer[i] = mirror_buffer_create(FFT_BUFFER_SIZE, sizeof(float32_t));
        if (!accel_sensor_data_buffer[i] || !gyro_sensor_data_buffer[i]) {
            LOG_FATAL("Failed to create buffer");
            trigger_fatal_error();
            return;
        }
    }

    arm_rfft_fast_init_f32(&fft_handler, FFT_BUFFER_SIZE);

    LOG_INFO("Waiting for %d points of IMU data", FFT_BUFFER_SIZE);
    for (int i = 0; i < FFT_BUFFER_SIZE; i++) {
        imu_data_t *imu_data = imu_mail_box->try_get_for(Kernel::wait_for_u32_forever);
        if (imu_data != nullptr) {
            for (int i = 0; i < 3; i++) {
                mirror_buffer_push(accel_sensor_data_buffer[i], &imu_data->accel[i]);
                mirror_buffer_push(gyro_sensor_data_buffer[i], &imu_data->gyro[i]);
            }
            imu_mail_box->free(imu_data);
        } else {
            LOG_WARN("Failed to receive IMU data");
            continue;
        }
    }

    while (true) {
        while (!imu_mail_box->empty()) {
            imu_data_t *imu_data = imu_mail_box->try_get();
            if (imu_data != nullptr) {
                for (int i = 0; i < 3; i++) {
                    mirror_buffer_push(accel_sensor_data_buffer[i], &imu_data->accel[i]);
                    mirror_buffer_push(gyro_sensor_data_buffer[i], &imu_data->gyro[i]);
                }
                imu_mail_box->free(imu_data);

                fft_result_t *result_buffer = fft_find_and_lock_oldest_result();
                if (result_buffer == nullptr) {
                    LOG_WARN("Failed to find available FFT result buffer");
                    continue;
                }

                // Processing steps per axis:
                // 1) Copy the latest sliding-window samples into fft_input.
                // 2) Real FFT: time-domain -> frequency-domain.
                // 3) Magnitude spectrum |X[k]| for k=0..N/2-1 (single-sided).
                // 4) Power: |X[k]|^2 (simple PSD estimate).
                // 5) Scale/normalize to keep thresholds stable across configs.
                for (int i = 0; i < 3; i++) {
                    memcpy(fft_input, (float32_t*)mirror_buffer_get_window(accel_sensor_data_buffer[i]), FFT_BUFFER_SIZE * sizeof(float32_t));
                    arm_rfft_fast_f32(&fft_handler, fft_input, fft_output, 0);
                    arm_cmplx_mag_f32(fft_output, result_buffer->accel_magnitude[i], FFT_BUFFER_SIZE / 2);
                    arm_mult_f32(result_buffer->accel_magnitude[i], result_buffer->accel_magnitude[i], result_buffer->accel_psd[i], FFT_BUFFER_SIZE / 2);
                    arm_scale_f32(result_buffer->accel_psd[i], scale_factor, result_buffer->accel_psd[i], FFT_BUFFER_SIZE / 2);
                }


                for (int i = 0; i < 3; i++) {
                    memcpy(fft_input, (float32_t*)mirror_buffer_get_window(gyro_sensor_data_buffer[i]), FFT_BUFFER_SIZE * sizeof(float32_t));
                    arm_rfft_fast_f32(&fft_handler, fft_input, fft_output, 0);
                    arm_cmplx_mag_f32(fft_output, result_buffer->gyro_magnitude[i], FFT_BUFFER_SIZE / 2);
                    arm_mult_f32(result_buffer->gyro_magnitude[i], result_buffer->gyro_magnitude[i], result_buffer->gyro_psd[i], FFT_BUFFER_SIZE / 2);
                    arm_scale_f32(result_buffer->gyro_psd[i], scale_factor, result_buffer->gyro_psd[i], FFT_BUFFER_SIZE / 2);
                }

                result_buffer->timestamp = Kernel::Clock::now();
                result_buffer->mutex.unlock();

            } else {
                LOG_WARN("Failed to get IMU data");
                imu_mail_box->free(imu_data);
                continue;
            }
        }
        ThisThread::sleep_for(1ms);
    }
}

/**
 * @brief Find the oldest (least recently updated) result buffer and lock it.
 *
 * Why "oldest": the writer wants to overwrite a buffer that the reader is least
 * likely to be using. We use `trylock()` to avoid blocking if a buffer is
 * currently being read.
 *
 * @return Locked buffer pointer, or nullptr if all buffers are busy.
 */
fft_result_t *fft_find_and_lock_oldest_result() {
    int oldest_idx = -1;
    auto oldest_time = std::chrono::time_point<rtos::Kernel::Clock>::max();
    for (int i = 0; i < FFT_BUFFER_NUM; i++) {
        if (fft_results[i].mutex.trylock()) {
            if (fft_results[i].timestamp < oldest_time) {
                if (oldest_idx != -1) {
                    fft_results[oldest_idx].mutex.unlock();
                }
                oldest_idx = i;
                oldest_time = fft_results[i].timestamp;
            } else {
                fft_results[i].mutex.unlock();
            }
        }
    }
    if (oldest_idx == -1) return nullptr;
    return &fft_results[oldest_idx];
}

/**
 * @brief Find the latest (most recently updated) result buffer and lock it.
 *
 * Consumers (e.g., analysis_task) generally want the newest spectrum. We use
 * `trylock()` to avoid blocking the producer; if nothing can be locked, the
 * caller can simply retry later.
 *
 * @return Locked buffer pointer, or nullptr if none can be locked now.
 */
fft_result_t *fft_find_and_lock_latest_result() {
    int newest_idx = -1;
    auto newest_time = std::chrono::time_point<rtos::Kernel::Clock>::min();
    for (int i = 0; i < FFT_BUFFER_NUM; i++) {
        if (fft_results[i].mutex.trylock()) {
            if (fft_results[i].timestamp > newest_time) {
                if (newest_idx != -1) {
                    fft_results[newest_idx].mutex.unlock();
                }
                newest_idx = i;
                newest_time = fft_results[i].timestamp;
            } else {
                fft_results[i].mutex.unlock();
            }
        }
    }
    if (newest_idx == -1) return nullptr;
    return &fft_results[newest_idx];
}
