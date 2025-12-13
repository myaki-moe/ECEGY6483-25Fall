#pragma once

#define TREMOR_MIN_FREQ 3.0f
#define TREMOR_MAX_FREQ 5.0f
#define DYSKINESIA_MIN_FREQ 5.0f
#define DYSKINESIA_MAX_FREQ 7.0f
#define BAND_MIN_FREQ 3.0f
#define BAND_MAX_FREQ 12.0f

#define RELATIVE_POWER_THRESHOLD 0.75f
#define MIN_PEAK_POWER_THRESHOLD 1.0f


#define FOG_FREEZE_MIN_FREQ 3.0f
#define FOG_FREEZE_MAX_FREQ 8.0f
#define FOG_LOCOMOTION_MIN_FREQ 0.5f
#define FOG_LOCOMOTION_MAX_FREQ 3.0f

#define FOG_FI_THRESHOLD 2.0f
#define LOCOMOTION_POWER_THRESHOLD 0.1f
#define WALKING_STATE_HISTORY 20

void analysis_task();
bool get_tremor_status();
bool get_dyskinesia_status();
bool get_fog_status();
