#include "fft_task.h"
#include "cmsis_os.h"
#include "semphr.h"
#include "logger.h"
#include "common.h"
#include "buffer.h"
#include "arm_math.h"
#include <stdio.h>
#include "main.h"


extern osMutexId_t sensor_buffer_mutexHandle;


void GPIO_SetHigh(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}

void GPIO_SetPushPullHigh(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 配置GPIO为推挽输出模式
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // 推挽输出
    GPIO_InitStruct.Pull = GPIO_NOPULL;          // 无上下拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // 高速
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);

    // 设置输出高电平
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}

void GPIO_SetPushPullLow(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 配置GPIO为推挽输出模式
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // 推挽输出
    GPIO_InitStruct.Pull = GPIO_NOPULL;          // 无上下拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // 高速
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);

    // 设置输出低电平
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}

void GPIO_SetHighZ(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 配置GPIO为浮空输入模式（高阻态）
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;      // 输入模式
    GPIO_InitStruct.Pull = GPIO_NOPULL;          // 无上下拉（浮空）
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

// 输入缓冲区 - 只需要实数
float32_t fft_inputbuf[RING_BUFFER_SIZE];      // 只存储实部！
// 输出缓冲区 - FFT结果（复数格式）
float32_t fft_outputbuf[RING_BUFFER_SIZE];     // 存储FFT复数结果
// 幅度谱缓冲区
float32_t fft_magnitude[RING_BUFFER_SIZE / 2]; // 只需要一半
// PSD
float32_t psd[RING_BUFFER_SIZE / 2];
float32_t scale_factor = 1.0f / (RING_BUFFER_SIZE * SENSOR_RATE);

arm_rfft_fast_instance_f32 fft_handler;


void FFT_Process()
{
    // 1. 执行实数FFT（输入是实数，输出是复数）
    arm_rfft_fast_f32(&fft_handler, fft_inputbuf, fft_outputbuf, 0);

    // 2. 计算幅度谱（模）
    arm_cmplx_mag_f32(fft_outputbuf, fft_magnitude, RING_BUFFER_SIZE / 2);

    // 3. 计算平方（使用CMSIS函数）
    arm_mult_f32(fft_magnitude, fft_magnitude, psd, RING_BUFFER_SIZE / 2);

    // 4. 归一化（乘以缩放因子）
    arm_scale_f32(psd, scale_factor, psd, RING_BUFFER_SIZE / 2);
}

bool detectTremor(float* psd, int fft_size, float sampling_rate,
                  float* tremor_intensity, float* tremor_frequency) {

    // ===== 步骤1: 定义频率范围 =====
    #define TREMOR_MIN_FREQ 3.0f
    #define TREMOR_MAX_FREQ 5.0f
    #define BAND_MIN_FREQ 3.5f    // 扩展频段（包含谐波）
    #define BAND_MAX_FREQ 12.0f

    int tremor_min_idx = (int)(TREMOR_MIN_FREQ * fft_size / sampling_rate);
    int tremor_max_idx = (int)(TREMOR_MAX_FREQ * fft_size / sampling_rate);
    int band_min_idx = (int)(BAND_MIN_FREQ * fft_size / sampling_rate);
    int band_max_idx = (int)(BAND_MAX_FREQ * fft_size / sampling_rate);

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
    float peak_freq = (float)peak_idx * sampling_rate / fft_size;

    // ===== 步骤3: 计算峰值周围的功率（±0.5Hz窗口）=====
    float peak_window_power = 0;
    int window_size = (int)(0.5 * fft_size / sampling_rate);

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

    // ===== 步骤6: 三重判定 =====
    #define RELATIVE_POWER_THRESHOLD 0.5f  // 50%
    #define MIN_PEAK_POWER_THRESHOLD 0.5f  // 根据实际调整

    bool is_tremor = false;

    // 条件1: 频率在3-5Hz
    bool freq_check = (peak_freq >= TREMOR_MIN_FREQ && peak_freq <= TREMOR_MAX_FREQ);

    // 条件2: 相对功率 > 50%
    bool relative_power_check = (relative_power > RELATIVE_POWER_THRESHOLD);

    // 条件3: 绝对功率 > 阈值
    bool absolute_power_check = (peak_power > MIN_PEAK_POWER_THRESHOLD);

    // 综合判定
    if (freq_check && relative_power_check && absolute_power_check) {
        is_tremor = true;
        *tremor_frequency = peak_freq;
        *tremor_intensity = sqrtf(peak_power);  // 强度 = 功率的平方根
    } else {
    }

    return is_tremor;
}


void StartFFTTask(void *argument) {
    LOG_INFO("starting fft calculation task");

    arm_rfft_fast_init_f32(&fft_handler, RING_BUFFER_SIZE);

    while (1) {
        // LOG_DEBUG("fft calculation start");
        float tremor_intensity_x,tremor_intensity_y,tremor_intensity_z;
        float tremor_freq_x,tremor_freq_y,tremor_freq_z;
        int out_x, out_y, out_z;

        if (xSemaphoreTake(sensor_buffer_mutexHandle, portMAX_DELAY) == pdTRUE) {
            uint16_t current_write_index = RingBuffer_GetWriteIndex(&gyro_x_buffer);
            while (! RingBuffer_Copy(&gyro_x_buffer, fft_inputbuf, RING_BUFFER_SIZE, current_write_index)) {}
            // 释放互斥锁
            xSemaphoreGive(sensor_buffer_mutexHandle);
            FFT_Process();
            out_x = detectTremor(psd, RING_BUFFER_SIZE, SENSOR_RATE, &tremor_intensity_x, &tremor_freq_x);
        } else {
            LOG_DEBUG("failed to obtain buffer mutex");
        }

        if (xSemaphoreTake(sensor_buffer_mutexHandle, portMAX_DELAY) == pdTRUE) {
            uint16_t current_write_index = RingBuffer_GetWriteIndex(&gyro_y_buffer);
            while (! RingBuffer_Copy(&gyro_y_buffer, fft_inputbuf, RING_BUFFER_SIZE, current_write_index)) {}
            // 释放互斥锁
            xSemaphoreGive(sensor_buffer_mutexHandle);
            FFT_Process();
            out_y = detectTremor(psd, RING_BUFFER_SIZE, SENSOR_RATE, &tremor_intensity_y, &tremor_freq_y);
        } else {
            LOG_DEBUG("failed to obtain buffer mutex");
        }

        if (xSemaphoreTake(sensor_buffer_mutexHandle, portMAX_DELAY) == pdTRUE) {
            uint16_t current_write_index = RingBuffer_GetWriteIndex(&gyro_z_buffer);
            while (! RingBuffer_Copy(&gyro_z_buffer, fft_inputbuf, RING_BUFFER_SIZE, current_write_index)) {}
            // 释放互斥锁
            xSemaphoreGive(sensor_buffer_mutexHandle);
            FFT_Process();
            out_z = detectTremor(psd, RING_BUFFER_SIZE, SENSOR_RATE, &tremor_intensity_z, &tremor_freq_z);
        } else {
            LOG_DEBUG("failed to obtain buffer mutex");
        }

        LOG_INFO("x i %f f %f y i %f f %f z i %f f %f", tremor_intensity_x, tremor_freq_x, tremor_intensity_y, tremor_freq_y, tremor_intensity_z, tremor_freq_z);
        // LOG_ERROR("x %d y %d z %d", out_x, out_y, out_z);
        if (out_x || out_y || out_z) {
            GPIO_SetPushPullLow (WIFI_BLE_GPIO_Port, WIFI_BLE_Pin);
        } else {
            GPIO_SetHighZ (WIFI_BLE_GPIO_Port, WIFI_BLE_Pin);
        }

        osDelay(1);
    }
}

