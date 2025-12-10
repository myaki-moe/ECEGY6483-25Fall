#pragma once

#define ANALYSIS_WINDOW_SIZE 3
#define FFT_BUFFER_SIZE (IMU_SAMPLE_RATE_HZ * ANALYSIS_WINDOW_SIZE)

void fft_task();