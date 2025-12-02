#include "filter.h"
#include <math.h>

/**
 * @brief 初始化滤波器
 * @param filter 滤波器结构体指针
 * @param type 滤波器类型（低通/高通/带通）
 * @param sampleRate 采样率 (Hz)
 * @param frequency 截止频率或中心频率 (Hz)
 * @param Q 品质因数，对于低通/高通通常设为0.707，带通可调整带宽
 */
void filter_init(BiquadFilter *filter, FilterType type, float sampleRate,
                 float frequency, float Q) {
    filter->type = type;
    filter->sampleRate = sampleRate;
    filter->frequency = frequency;
    filter->Q = Q;

    // 重置状态
    filter->x1 = 0.0f;
    filter->x2 = 0.0f;
    filter->y1 = 0.0f;
    filter->y2 = 0.0f;

    // 计算中间变量
    float omega = 2.0f * M_PI * frequency / sampleRate;
    float sn = sinf(omega);
    float cs = cosf(omega);
    float alpha = sn / (2.0f * Q);

    float a0, a1, a2, b0, b1, b2;

    switch (type) {
        case FILTER_TYPE_LPF:  // 低通滤波器
            b0 = (1.0f - cs) / 2.0f;
            b1 = 1.0f - cs;
            b2 = (1.0f - cs) / 2.0f;
            a0 = 1.0f + alpha;
            a1 = -2.0f * cs;
            a2 = 1.0f - alpha;
            break;

        case FILTER_TYPE_HPF:  // 高通滤波器
            b0 = (1.0f + cs) / 2.0f;
            b1 = -(1.0f + cs);
            b2 = (1.0f + cs) / 2.0f;
            a0 = 1.0f + alpha;
            a1 = -2.0f * cs;
            a2 = 1.0f - alpha;
            break;

        case FILTER_TYPE_BPF:  // 带通滤波器
            b0 = alpha;
            b1 = 0.0f;
            b2 = -alpha;
            a0 = 1.0f + alpha;
            a1 = -2.0f * cs;
            a2 = 1.0f - alpha;
            break;

        default:
            // 默认使用低通
            b0 = 1.0f;
            b1 = 0.0f;
            b2 = 0.0f;
            a0 = 1.0f;
            a1 = 0.0f;
            a2 = 0.0f;
            break;
    }

    // 归一化系数
    filter->a0 = b0 / a0;
    filter->a1 = b1 / a0;
    filter->a2 = b2 / a0;
    filter->b1 = a1 / a0;
    filter->b2 = a2 / a0;
}

/**
 * @brief 处理单个采样点
 * @param filter 滤波器结构体指针
 * @param input 输入采样值
 * @return 滤波后的输出值
 */
float filter_process(BiquadFilter *filter, float input) {
    // 差分方程: y[n] = a0*x[n] + a1*x[n-1] + a2*x[n-2] - b1*y[n-1] - b2*y[n-2]
    float output = filter->a0 * input +
                   filter->a1 * filter->x1 +
                   filter->a2 * filter->x2 -
                   filter->b1 * filter->y1 -
                   filter->b2 * filter->y2;

    // 更新状态
    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = output;

    return output;
}

/**
 * @brief 重置滤波器状态
 * @param filter 滤波器结构体指针
 */
void filter_reset(BiquadFilter *filter) {
    filter->x1 = 0.0f;
    filter->x2 = 0.0f;
    filter->y1 = 0.0f;
    filter->y2 = 0.0f;
}
