#pragma once

#define TREMOR_MIN_FREQ 3.0f
#define TREMOR_MAX_FREQ 5.0f
#define DYSKINESIA_MIN_FREQ 4.8f
#define DYSKINESIA_MAX_FREQ 7.0f
#define BAND_MIN_FREQ 3.0f
#define BAND_MAX_FREQ 12.0f

#define RELATIVE_POWER_THRESHOLD 0.75f
#define MIN_PEAK_POWER_THRESHOLD 1.0f

void analysis_task();
bool get_tremor_status();
bool get_dyskinesia_status();
bool get_fog_status();
