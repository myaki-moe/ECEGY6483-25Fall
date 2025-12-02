#ifndef ECEGY6483_FILTER_H
#define ECEGY6483_FILTER_H

// 滤波器类型
typedef enum {
    FILTER_TYPE_LPF,  // 低通滤波器
    FILTER_TYPE_HPF,  // 高通滤波器
    FILTER_TYPE_BPF   // 带通滤波器
} FilterType;

// Biquad滤波器结构体
typedef struct {
    // 滤波器系数
    float a0, a1, a2;  // 分子系数
    float b1, b2;      // 分母系数

    // 状态变量（存储历史数据）
    float x1, x2;      // 输入历史
    float y1, y2;      // 输出历史

    // 滤波器参数
    FilterType type;
    float sampleRate;  // 采样率
    float frequency;   // 截止频率或中心频率
    float Q;           // 品质因数（带通时使用）
} BiquadFilter;

// 函数声明
void filter_init(BiquadFilter *filter, FilterType type, float sampleRate,
                 float frequency, float Q);
float filter_process(BiquadFilter *filter, float input);
void filter_reset(BiquadFilter *filter);

#endif //ECEGY6483_FILTER_H