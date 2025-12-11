#include "tasks/analysis_task.hpp"
#include "mbed.h"
#include "logger.hpp"
#include "main.hpp"
#include "tasks/fft_task.hpp"


bool detectTremor(float32_t* psd, int fft_size, float32_t sampling_rate, float32_t* tremor_intensity, float32_t* tremor_frequency) {

    int tremor_min_idx = (int)(TREMOR_MIN_FREQ * fft_size / IMU_SAMPLE_RATE_HZ);
    int tremor_max_idx = (int)(TREMOR_MAX_FREQ * fft_size / IMU_SAMPLE_RATE_HZ);
    int band_min_idx = (int)(BAND_MIN_FREQ * fft_size / IMU_SAMPLE_RATE_HZ);
    int band_max_idx = (int)(BAND_MAX_FREQ * fft_size / IMU_SAMPLE_RATE_HZ);

    // ===== 步骤2: 寻找3-5Hz范围内的峰值 =====
    float peak_power = 0;
    int peak_idx = 0;

    for (int i = tremor_min_idx; i <= tremor_max_idx; i++) {
        if (psd[i] > peak_power) {
        peak_power = psd[i];
        peak_idx = i;
        }
    }

    // 计算峰值频率
    float peak_freq = (float)peak_idx * IMU_SAMPLE_RATE_HZ / fft_size;

    // ===== 步骤3: 计算峰值周围的功率（±0.5Hz窗口）=====
    float peak_window_power = 0;
    int window_size = (int)(0.5 * fft_size / IMU_SAMPLE_RATE_HZ);

    for (int i = peak_idx - window_size; i <= peak_idx + window_size; i++) {
        if (i >= 0 && i < fft_size/2) {
        peak_window_power += psd[i];
        }
    }

    // ===== 步骤4: 计算总频段功率 =====
    float total_band_power = 0;
    for (int i = band_min_idx; i <= band_max_idx; i++) {
    total_band_power += psd[i];
    }

    // ===== 步骤5: 计算相对功率 =====
    float relative_power = peak_window_power / (total_band_power + 1e-6);  // 防止除零

    // LOG_INFO("peak_power: %.2f, peak_freq: %.2f, relative_power: %.2f, total_band_power: %.2f", peak_power, peak_freq, relative_power, total_band_power);

    // ===== 步骤6: 三重判定 =====
    #define RELATIVE_POWER_THRESHOLD 0.5f  // 50%
    #define MIN_PEAK_POWER_THRESHOLD 0.05f  // 根据实际调整

    // 条件1: 频率在3-5Hz
    bool freq_check = (peak_freq >= TREMOR_MIN_FREQ && peak_freq <= TREMOR_MAX_FREQ);

    // 条件2: 相对功率 > 50%
    bool relative_power_check = (relative_power > RELATIVE_POWER_THRESHOLD);

    // 条件3: 绝对功率 > 阈值
    bool absolute_power_check = (peak_power > MIN_PEAK_POWER_THRESHOLD);

    // 综合判定
    *tremor_frequency = peak_freq;
    *tremor_intensity = sqrtf(peak_power);  // 强度 = 功率的平方根
    if (freq_check && relative_power_check && absolute_power_check) {
        return true;
    }

    return false;
}


void analysis_task() {
    LOG_INFO("Analysis Task Started");

    while (true) {
        float tremor_intensity_x = 0;
        float tremor_freq_x = 0;
        fft_result_t *result = fft_find_and_lock_latest_result();
        if (result != nullptr) {
            bool is_tremor_x = detectTremor(result->gyro_psd[0], FFT_BUFFER_SIZE, IMU_SAMPLE_RATE_HZ, &tremor_intensity_x, &tremor_freq_x);
            LOG_INFO("X axis: Tremor detected %s at frequency: %.2f Hz with intensity: %.2f", is_tremor_x ? "true" : "false", tremor_freq_x, tremor_intensity_x);
            result->mutex.unlock();
        }
        ThisThread::sleep_for(1ms);
    }
}