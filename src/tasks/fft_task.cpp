#include "tasks/fft_task.hpp"
#include "mbed.h"
#include "arm_math.h"
#include "logger.hpp"
#include "buffer.hpp"
#include "tasks/imu_task.hpp"
#include "main.hpp"


arm_rfft_fast_instance_f32 fft_handler;
float32_t scale_factor = 1.0f / (FFT_BUFFER_SIZE * IMU_SAMPLE_RATE_HZ);

mirror_buffer_t *accel_sensor_data_buffer[3];
mirror_buffer_t *gyro_sensor_data_buffer[3];
float32_t fft_input[FFT_BUFFER_SIZE];
float32_t fft_output[FFT_BUFFER_SIZE];
fft_result_t fft_results[FFT_BUFFER_NUM];


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

                // 1. 执行实数FFT
                // 2. 计算幅度谱
                // 3. 计算功率谱密度
                // 4. 归一化
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
