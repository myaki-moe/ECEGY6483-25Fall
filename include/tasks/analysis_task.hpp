#pragma once

/**
 * @file analysis_task.hpp
 * @brief Motion classification task (tremor, dyskinesia, FOG) based on FFT PSD.
 */

/**
 * @name Frequency bands (Hz)
 * These constants define the bands used by the detection algorithms.
 * @{
 */
#define TREMOR_MIN_FREQ 3.0f
#define TREMOR_MAX_FREQ 5.0f
#define DYSKINESIA_MIN_FREQ 5.0f
#define DYSKINESIA_MAX_FREQ 7.0f
#define BAND_MIN_FREQ 3.0f
#define BAND_MAX_FREQ 12.0f
/** @} */

/**
 * @name Tremor/Dyskinesia thresholds
 * @{
 */
#define RELATIVE_POWER_THRESHOLD 0.75f
#define MIN_PEAK_POWER_THRESHOLD 1.0f
/** @} */


/**
 * @name Freezing-of-Gait (FOG) bands and thresholds
 * @{
 */
#define FOG_FREEZE_MIN_FREQ 3.0f
#define FOG_FREEZE_MAX_FREQ 8.0f
#define FOG_LOCOMOTION_MIN_FREQ 0.5f
#define FOG_LOCOMOTION_MAX_FREQ 3.0f

#define FOG_FI_THRESHOLD 2.0f
#define LOCOMOTION_POWER_THRESHOLD 0.1f
#define WALKING_STATE_HISTORY 50
/** @} */

/**
 * @brief RTOS task that analyzes FFT PSD and updates motion status flags.
 *
 * The task reads the latest FFT result (gyro PSD) and runs three detectors:
 * tremor, dyskinesia, and freezing-of-gait (FOG). Outputs are debounced/
 * smoothed using a boolean filter to avoid flickering decisions.
 */
void analysis_task();

/**
 * @brief Get the filtered tremor detection status.
 * @return true if tremor is currently detected, otherwise false.
 */
bool get_tremor_status();

/**
 * @brief Get the filtered dyskinesia detection status.
 * @return true if dyskinesia is currently detected, otherwise false.
 */
bool get_dyskinesia_status();

/**
 * @brief Get the filtered freezing-of-gait detection status.
 * @return true if FOG is currently detected, otherwise false.
 */
bool get_fog_status();
