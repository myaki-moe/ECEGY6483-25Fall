#include "tasks/fft_task.hpp"
#include "mbed.h"
#include "arm_math.h"
#include "logger.hpp"
#include "buffer.hpp"
#include "tasks/imu_task.hpp"
#include "main.hpp"

float32_t fft_accel_x_magnitude[FFT_BUFFER_SIZE / 2];
float32_t fft_accel_y_magnitude[FFT_BUFFER_SIZE / 2];
float32_t fft_accel_z_magnitude[FFT_BUFFER_SIZE / 2];
float32_t fft_gyro_x_magnitude[FFT_BUFFER_SIZE / 2];
float32_t fft_gyro_y_magnitude[FFT_BUFFER_SIZE / 2];
float32_t fft_gyro_z_magnitude[FFT_BUFFER_SIZE / 2];

float32_t fft_accel_x_psd[FFT_BUFFER_SIZE / 2];
float32_t fft_accel_y_psd[FFT_BUFFER_SIZE / 2];
float32_t fft_accel_z_psd[FFT_BUFFER_SIZE / 2];
float32_t fft_gyro_x_psd[FFT_BUFFER_SIZE / 2];
float32_t fft_gyro_y_psd[FFT_BUFFER_SIZE / 2];
float32_t fft_gyro_z_psd[FFT_BUFFER_SIZE / 2];

float32_t scale_factor = 1.0f / (FFT_BUFFER_SIZE * IMU_SAMPLE_RATE_HZ);

void fft_task() {
    LOG_INFO("FFT Task Started");

    mirror_buffer_t *accel_x_buffer = mirror_buffer_create(FFT_BUFFER_SIZE, sizeof(float32_t));
    mirror_buffer_t *accel_y_buffer = mirror_buffer_create(FFT_BUFFER_SIZE, sizeof(float32_t));
    mirror_buffer_t *accel_z_buffer = mirror_buffer_create(FFT_BUFFER_SIZE, sizeof(float32_t));
    mirror_buffer_t *gyro_x_buffer = mirror_buffer_create(FFT_BUFFER_SIZE, sizeof(float32_t));
    mirror_buffer_t *gyro_y_buffer = mirror_buffer_create(FFT_BUFFER_SIZE, sizeof(float32_t));
    mirror_buffer_t *gyro_z_buffer = mirror_buffer_create(FFT_BUFFER_SIZE, sizeof(float32_t));
    if (!accel_x_buffer || !accel_y_buffer || !accel_z_buffer || !gyro_x_buffer || !gyro_y_buffer || !gyro_z_buffer) {
        LOG_ERROR("Failed to create buffer");
        trigger_fatal_error();
        return;
    }

    static arm_rfft_fast_instance_f32 fft_handler;
    arm_rfft_fast_init_f32(&fft_handler, FFT_BUFFER_SIZE);

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


                // 1. 执行实数FFT
                // 2. 计算幅度谱
                // 3. 计算功率谱密度
                // 4. 归一化

                float32_t fft_output_buffer[FFT_BUFFER_SIZE];

                arm_rfft_fast_f32(&fft_handler, (float32_t*)mirror_buffer_get_window(accel_x_buffer), fft_output_buffer, 0);
                arm_cmplx_mag_f32(fft_output_buffer, fft_accel_x_magnitude, FFT_BUFFER_SIZE / 2);
                arm_mult_f32(fft_accel_x_magnitude, fft_accel_x_magnitude, fft_accel_x_psd, FFT_BUFFER_SIZE / 2);
                arm_scale_f32(fft_accel_x_psd, scale_factor, fft_accel_x_psd, FFT_BUFFER_SIZE / 2);

                arm_rfft_fast_f32(&fft_handler, (float32_t*)mirror_buffer_get_window(accel_y_buffer), fft_output_buffer, 0);
                arm_cmplx_mag_f32(fft_output_buffer, fft_accel_y_magnitude, FFT_BUFFER_SIZE / 2);
                arm_mult_f32(fft_accel_y_magnitude, fft_accel_y_magnitude, fft_accel_y_psd, FFT_BUFFER_SIZE / 2);
                arm_scale_f32(fft_accel_y_psd, scale_factor, fft_accel_y_psd, FFT_BUFFER_SIZE / 2);

                arm_rfft_fast_f32(&fft_handler, (float32_t*)mirror_buffer_get_window(accel_z_buffer), fft_output_buffer, 0);
                arm_cmplx_mag_f32(fft_output_buffer, fft_accel_z_magnitude, FFT_BUFFER_SIZE / 2);
                arm_mult_f32(fft_accel_z_magnitude, fft_accel_z_magnitude, fft_accel_z_psd, FFT_BUFFER_SIZE / 2);
                arm_scale_f32(fft_accel_z_psd, scale_factor, fft_accel_z_psd, FFT_BUFFER_SIZE / 2);

                arm_rfft_fast_f32(&fft_handler, (float32_t*)mirror_buffer_get_window(gyro_x_buffer), fft_output_buffer, 0);
                arm_cmplx_mag_f32(fft_output_buffer, fft_gyro_x_magnitude, FFT_BUFFER_SIZE / 2);
                arm_mult_f32(fft_gyro_x_magnitude, fft_gyro_x_magnitude, fft_gyro_x_psd, FFT_BUFFER_SIZE / 2);
                arm_scale_f32(fft_gyro_x_psd, scale_factor, fft_gyro_x_psd, FFT_BUFFER_SIZE / 2);

                arm_rfft_fast_f32(&fft_handler, (float32_t*)mirror_buffer_get_window(gyro_y_buffer), fft_output_buffer, 0);
                arm_cmplx_mag_f32(fft_output_buffer, fft_gyro_y_magnitude, FFT_BUFFER_SIZE / 2);
                arm_mult_f32(fft_gyro_y_magnitude, fft_gyro_y_magnitude, fft_gyro_y_psd, FFT_BUFFER_SIZE / 2);
                arm_scale_f32(fft_gyro_y_psd, scale_factor, fft_gyro_y_psd, FFT_BUFFER_SIZE / 2);

                arm_rfft_fast_f32(&fft_handler, (float32_t*)mirror_buffer_get_window(gyro_z_buffer), fft_output_buffer, 0);
                arm_cmplx_mag_f32(fft_output_buffer, fft_gyro_z_magnitude, FFT_BUFFER_SIZE / 2);
                arm_mult_f32(fft_gyro_z_magnitude, fft_gyro_z_magnitude, fft_gyro_z_psd, FFT_BUFFER_SIZE / 2);
                arm_scale_f32(fft_gyro_z_psd, scale_factor, fft_gyro_z_psd, FFT_BUFFER_SIZE / 2);

            } else {
                LOG_ERROR("Failed to get IMU data");
                imu_mail_box->free(imu_data);
                continue;
            }
        }
        ThisThread::sleep_for(1ms);
    }
}