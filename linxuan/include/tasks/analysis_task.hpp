#pragma once

#define TREMOR_MIN_FREQ 3.0f
#define TREMOR_MAX_FREQ 5.0f
#define BAND_MIN_FREQ 3.5f    // 扩展频段（包含谐波）
#define BAND_MAX_FREQ 12.0f


void analysis_task();
