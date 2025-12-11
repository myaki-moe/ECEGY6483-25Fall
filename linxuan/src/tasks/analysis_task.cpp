#include "tasks/analysis_task.hpp"
#include "mbed.h"
#include "logger.hpp"
#include "main.hpp"
#include "tasks/fft_task.hpp"
#include "bool_filter.hpp"


bool_filter_t tremor_filter;
bool_filter_t dyskinesia_filter;
bool_filter_t fog_filter;


void find_peak_power(float32_t* psd, uint32_t fft_size, float32_t sampling_rate, float32_t min_freq, float32_t max_freq, float32_t* peak_power, float32_t* peak_freq) {
    float32_t peak_power_temp = 0.0f;
    float32_t peak_freq_temp = 0.0f;
    uint32_t min_idx = (uint32_t)(min_freq * fft_size / sampling_rate);
    uint32_t max_idx = (uint32_t)(max_freq * fft_size / sampling_rate);
    for (uint32_t i = min_idx; i <= max_idx; i++) {
        if (psd[i] > peak_power_temp) {
            peak_power_temp = psd[i];
            peak_freq_temp = (float32_t)i * sampling_rate / fft_size;
        }
    }
    *peak_power = peak_power_temp;
    *peak_freq = peak_freq_temp;
}

float32_t find_total_band_power(float32_t* psd, uint32_t fft_size, float32_t sampling_rate, float32_t min_freq, float32_t max_freq) {
    uint32_t min_idx = (uint32_t)(min_freq * fft_size / sampling_rate);
    uint32_t max_idx = (uint32_t)(max_freq * fft_size / sampling_rate);
    float32_t total_band_power = 0.0f;
    for (uint32_t i = min_idx; i <= max_idx; i++) {
        total_band_power += psd[i];
    }
    return total_band_power;
}

bool detectTremor(float32_t* psd, uint32_t fft_size, float32_t sampling_rate) {
    float32_t band_peak_power = 0.0f;
    float32_t band_peak_freq = 0.0f;

    find_peak_power(psd, fft_size, sampling_rate, BAND_MIN_FREQ, BAND_MAX_FREQ, &band_peak_power, &band_peak_freq);
    float32_t tremor_total_power = find_total_band_power(psd, fft_size, sampling_rate, TREMOR_MIN_FREQ, TREMOR_MAX_FREQ);
    float32_t total_band_power = find_total_band_power(psd, fft_size, sampling_rate, BAND_MIN_FREQ, BAND_MAX_FREQ);

    float32_t relative_power = tremor_total_power / (total_band_power + 1e-6);

    bool freq_check = (band_peak_freq >= TREMOR_MIN_FREQ && band_peak_freq <= TREMOR_MAX_FREQ);
    bool relative_power_check = (relative_power > RELATIVE_POWER_THRESHOLD);
    bool absolute_power_check = (band_peak_power > MIN_PEAK_POWER_THRESHOLD);

    LOG_DEBUG("freq_check: %.1f <%s> , relative_power_check: %.1f <%s>, absolute_power_check: %.1f <%s>", 
        band_peak_freq, freq_check ? "true " : "false", relative_power, relative_power_check ? "true " : "false", band_peak_power, absolute_power_check ? "true " : "false");

    return (freq_check && relative_power_check && absolute_power_check);
}

bool detectDyskinesia(float32_t* psd, uint32_t fft_size, float32_t sampling_rate) {
    float32_t band_peak_power = 0.0f;
    float32_t band_peak_freq = 0.0f;

    find_peak_power(psd, fft_size, sampling_rate, BAND_MIN_FREQ, BAND_MAX_FREQ, &band_peak_power, &band_peak_freq);
    float32_t dyskinesia_total_power = find_total_band_power(psd, fft_size, sampling_rate, DYSKINESIA_MIN_FREQ, DYSKINESIA_MAX_FREQ);
    float32_t total_band_power = find_total_band_power(psd, fft_size, sampling_rate, BAND_MIN_FREQ, BAND_MAX_FREQ);

    float32_t relative_power = dyskinesia_total_power / (total_band_power + 1e-6);

    bool freq_check = (band_peak_freq >= DYSKINESIA_MIN_FREQ && band_peak_freq <= DYSKINESIA_MAX_FREQ);
    bool relative_power_check = (relative_power > RELATIVE_POWER_THRESHOLD);
    bool absolute_power_check = (band_peak_power > MIN_PEAK_POWER_THRESHOLD);

    LOG_DEBUG("freq_check: %.1f <%s> , relative_power_check: %.1f <%s>, absolute_power_check: %.1f <%s>", 
        band_peak_freq, freq_check ? "true " : "false", relative_power, relative_power_check ? "true " : "false", band_peak_power, absolute_power_check ? "true " : "false");

    return (freq_check && relative_power_check && absolute_power_check);
}


void analysis_task() {
    LOG_INFO("Analysis Task Started");

    bool_filter_init(&tremor_filter, 2);
    bool_filter_init(&dyskinesia_filter, 2);
    bool_filter_init(&fog_filter, 2);

    while (true) {
        fft_result_t *result = fft_find_and_lock_latest_result();
        if (result != nullptr) {
            bool tremor_result[3] = {false, false, false};
            bool dyskinesia_result[3] = {false, false, false};
            for (int i = 0; i < 3; i++) {
                tremor_result[i] = detectTremor(result->gyro_psd[i], FFT_BUFFER_SIZE, IMU_SAMPLE_RATE_HZ);
            }
            for (int i = 0; i < 3; i++) {
                dyskinesia_result[i] = detectDyskinesia(result->gyro_psd[i], FFT_BUFFER_SIZE, IMU_SAMPLE_RATE_HZ);
            }
            bool is_tremor = tremor_result[0] || tremor_result[1] || tremor_result[2];
            bool is_dyskinesia = dyskinesia_result[0] || dyskinesia_result[1] || dyskinesia_result[2];
            bool is_fog = false;
            LOG_DEBUG("tremor: %s %s %s, dyskinesia: %s %s %s, overall: %s %s", 
                tremor_result[0] ? "true" : "false", tremor_result[1] ? "true" : "false", tremor_result[2] ? "true" : "false", 
                dyskinesia_result[0] ? "true" : "false", dyskinesia_result[1] ? "true" : "false", dyskinesia_result[2] ? "true" : "false",
                 is_tremor ? "true" : "false", is_dyskinesia ? "true" : "false");
            result->mutex.unlock();

            bool_filter_update(&tremor_filter, is_tremor);
            bool_filter_update(&dyskinesia_filter, is_dyskinesia);
            bool_filter_update(&fog_filter, is_fog);
        } else {
            LOG_WARN("No FFT result available");
            ThisThread::sleep_for(1ms);
            continue;
        }
        ThisThread::sleep_for(100ms);
    }
}


bool get_tremor_status() {
    return bool_filter_get_state(&tremor_filter);
}

bool get_dyskinesia_status() {
    return bool_filter_get_state(&dyskinesia_filter);
}

bool get_fog_status() {
    return bool_filter_get_state(&fog_filter);
}