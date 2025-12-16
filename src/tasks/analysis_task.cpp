/**
 * @file analysis_task.cpp
 * @brief Frequency-domain motion detection (tremor, dyskinesia, FOG).
 *
 * This module consumes the latest gyro PSD produced by `fft_task` and applies
 * simple band-energy/peak heuristics to classify motion patterns:
 * - Tremor: dominant energy around 3–5 Hz
 * - Dyskinesia: dominant energy around 5–7 Hz
 * - Freezing of gait (FOG): high "freeze" energy (3–8 Hz) relative to
 *   locomotion energy (0.5–3 Hz) while the subject is walking.
 *
 * Implementation notes:
 * - PSD is indexed by FFT bin k. Bin frequency is: f_k = k * Fs / N
 *   where Fs is sampling_rate and N is fft_size.
 * - We add a small epsilon (1e-6) to denominators to avoid divide-by-zero.
 * - The boolean filters smooth results to prevent flickering.
 */

#include "tasks/analysis_task.hpp"
#include "mbed.h"
#include "logger.hpp"
#include "main.hpp"
#include "tasks/fft_task.hpp"
#include "bool_filter.hpp"
#include "bsp/serial.hpp"


bool_filter_t tremor_filter;
bool_filter_t dyskinesia_filter;
bool_filter_t fog_filter;

typedef struct {
    float32_t peak_freq;
    float32_t relative_power;
    float32_t peak_power;
    bool freq_match;
    bool relative_power_match;
    bool absolute_power_match;
} td_axis_params_t;

typedef struct {
    float32_t freeze_index;
    float32_t freeze_power;
    bool walking;
    bool fi_match;
    bool walking_match;
    bool freeze_power_match;
} fog_axis_params_t;

static inline const char* match_color(bool match) {
    return match ? COLOR_RED : COLOR_BLUE;
}

static void print_td_table(const td_axis_params_t tremor[3],
                           const td_axis_params_t dysk[3],
                           const bool tremor_detected[3],
                           const bool dysk_detected[3],
                           const fog_axis_params_t fog[3],
                           const bool fog_detected[3]) {
    static bool printed_header = false;
    const char axis_name[3] = {'X', 'Y', 'Z'};

    serial_lock();

    if (!printed_header) {
        // 表头只打印一次；后续仅刷新 3 行数据，节省串口带宽
        // 说明：
        // - f: band 内主峰频率(Hz)，Tremor=3–5Hz，Dyskinesia=5–7Hz
        // - r: band 能量占比 Pband / P(3–12Hz)，阈值 r>0.75
        // - p: band 内主峰功率，阈值 p>1.0
        // - 颜色：蓝=通过该项，红=未通过该项
        // 注意：不改动你原本表头的左侧内容，只在右边新增 FOG 栏位
        printf("\r\n   |   Tremor: (3-5Hz)   |    Dysk: (5-7Hz)    |       FOG ");
        printf("\r\nAX | Domin|Relative| Abs | Domin|Relative| Abs | Freez| WA |Freeze");
        printf("\r\nIS | Freq |  Power |Power| Freq |  Power |Power| Index| LK |Power\r\n");
        printf("--------------------------------------------------------------------------\r\n");
        printf("\r\n\r\n\r\n"); // 预留 3 行用于刷新
        // 保存数据区起始位置（避免其他日志输出后，相对移动光标失效）
        printf("\033[3A");
        printf("\033[s");
        printf("\033[3B");
        printed_header = true;
    }

    // 回到数据区起始位置，仅清理这 3 行（不影响后续日志）
    printf("\033[u");
    for (int i = 0; i < 3; i++) {
        printf("\033[2K"); // 清空整行
        if (i < 2) {
            printf("\033[1B"); // 下一行
        }
    }
    printf("\033[u");

    for (int i = 0; i < 3; i++) {
        // 固定宽度字段，避免数字位数变化导致表格跳动：
        // f: %5.1f, r: %5.2f, p: %7.2f
        // FOG: FI %5.2f, W %1d, Freeze %7.3f
        printf("%c  | %s%5.1f%s %s%5.2f%s %s%7.2f%s | %s%5.1f%s %s%5.2f%s %s%7.2f%s | %s%5.2f%s  %s%1d%s %s%7.3f%s%s%s%s\r\n",
               axis_name[i],
               match_color(tremor[i].freq_match), tremor[i].peak_freq, COLOR_RESET,
               match_color(tremor[i].relative_power_match), tremor[i].relative_power, COLOR_RESET,
               match_color(tremor[i].absolute_power_match), tremor[i].peak_power, COLOR_RESET,
               match_color(dysk[i].freq_match), dysk[i].peak_freq, COLOR_RESET,
               match_color(dysk[i].relative_power_match), dysk[i].relative_power, COLOR_RESET,
               match_color(dysk[i].absolute_power_match), dysk[i].peak_power, COLOR_RESET,
               match_color(fog[i].fi_match), fog[i].freeze_index, COLOR_RESET,
               match_color(fog[i].walking_match), fog[i].walking ? 1 : 0, COLOR_RESET,
               match_color(fog[i].freeze_power_match), fog[i].freeze_power, COLOR_RESET,
               tremor_detected[i] ? " [Tremor Detected]" : "",
               dysk_detected[i] ? " [Dyskinesia Detected]" : "",
               fog_detected[i] ? " [FOG Detected]" : "");
    }

    serial_unlock();
}


/**
 * @brief Find the peak PSD value and its corresponding frequency in a band.
 *
 * @param psd Pointer to a single-sided PSD array (length fft_size/2).
 * @param fft_size FFT size N used to compute the PSD.
 * @param sampling_rate Sampling rate Fs (Hz).
 * @param min_freq Lower band edge (Hz).
 * @param max_freq Upper band edge (Hz).
 * @param peak_power Output: peak power within the band.
 * @param peak_freq Output: frequency (Hz) of that peak.
 */
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

/**
 * @brief Sum PSD values across a frequency band (simple band-power estimate).
 *
 * @param psd Pointer to a single-sided PSD array (length fft_size/2).
 * @param fft_size FFT size N used to compute the PSD.
 * @param sampling_rate Sampling rate Fs (Hz).
 * @param min_freq Lower band edge (Hz).
 * @param max_freq Upper band edge (Hz).
 * @return Sum of PSD bins between min_freq and max_freq (inclusive).
 */
float32_t find_total_band_power(float32_t* psd, uint32_t fft_size, float32_t sampling_rate, float32_t min_freq, float32_t max_freq) {
    uint32_t min_idx = (uint32_t)(min_freq * fft_size / sampling_rate);
    uint32_t max_idx = (uint32_t)(max_freq * fft_size / sampling_rate);
    float32_t total_band_power = 0.0f;
    for (uint32_t i = min_idx; i <= max_idx; i++) {
        total_band_power += psd[i];
    }
    return total_band_power;
}

static bool compute_td_axis_params(float32_t* psd,
                                   uint32_t fft_size,
                                   float32_t sampling_rate,
                                   float32_t min_freq,
                                   float32_t max_freq,
                                   td_axis_params_t* out) {
    float32_t band_peak_power = 0.0f;
    float32_t band_peak_freq = 0.0f;

    find_peak_power(psd, fft_size, sampling_rate, BAND_MIN_FREQ, BAND_MAX_FREQ, &band_peak_power, &band_peak_freq);
    float32_t motion_total_power = find_total_band_power(psd, fft_size, sampling_rate, min_freq, max_freq);
    float32_t total_band_power = find_total_band_power(psd, fft_size, sampling_rate, BAND_MIN_FREQ, BAND_MAX_FREQ);

    float32_t relative_power = motion_total_power / (total_band_power + 1e-6f);

    bool freq_check = (band_peak_freq >= min_freq && band_peak_freq <= max_freq);
    bool relative_power_check = (relative_power > RELATIVE_POWER_THRESHOLD);
    bool absolute_power_check = (band_peak_power > MIN_PEAK_POWER_THRESHOLD);

    if (out != nullptr) {
        out->peak_freq = band_peak_freq;
        out->relative_power = relative_power;
        out->peak_power = band_peak_power;
        out->freq_match = freq_check;
        out->relative_power_match = relative_power_check;
        out->absolute_power_match = absolute_power_check;
    }

    return (freq_check && relative_power_check && absolute_power_check);
}

/**
 * @brief Detect tremor using peak frequency and relative band power.
 *
 * Decision logic (gyro PSD):
 * - Find the dominant peak in BAND_MIN_FREQ..BAND_MAX_FREQ (3–12 Hz).
 * - Compute tremor band power (3–5 Hz) and total band power (3–12 Hz).
 * - Require:
 *   1) Peak frequency lies inside tremor band (3–5 Hz)
 *   2) Tremor band contributes a large fraction of total power
 *   3) Peak power exceeds an absolute minimum threshold
 *
 * @param psd Single-sided gyro PSD (length fft_size/2).
 * @param fft_size FFT size N.
 * @param sampling_rate Sampling rate Fs (Hz).
 * @return true if tremor is detected on this axis.
 */
bool detectTremor(float32_t* psd, uint32_t fft_size, float32_t sampling_rate, td_axis_params_t* out) {
    return compute_td_axis_params(psd, fft_size, sampling_rate, TREMOR_MIN_FREQ, TREMOR_MAX_FREQ, out);
}

/**
 * @brief Detect dyskinesia using peak frequency and relative band power.
 *
 * Uses the same structure as tremor detection but with the dyskinesia band
 * (5–7 Hz). Keeping the structure consistent makes thresholds easier to tune.
 *
 * @param psd Single-sided gyro PSD (length fft_size/2).
 * @param fft_size FFT size N.
 * @param sampling_rate Sampling rate Fs (Hz).
 * @return true if dyskinesia is detected on this axis.
 */
bool detectDyskinesia(float32_t* psd, uint32_t fft_size, float32_t sampling_rate, td_axis_params_t* out) {
    return compute_td_axis_params(psd, fft_size, sampling_rate, DYSKINESIA_MIN_FREQ, DYSKINESIA_MAX_FREQ, out);
}

// Simple walking-state estimator with hysteresis (prevents rapid toggling).
static int walking_state_counter = 0;
static bool is_walking = false;

static bool compute_fog_axis_params(float32_t* psd,
                                    uint32_t fft_size,
                                    float32_t sampling_rate,
                                    fog_axis_params_t* out) {
    // 1) Compute freeze-band power (3–8 Hz).
    float32_t freeze_power = find_total_band_power(psd, fft_size, sampling_rate,
                                                   FOG_FREEZE_MIN_FREQ, FOG_FREEZE_MAX_FREQ);

    // 2) Compute locomotion-band power (0.5–3 Hz).
    float32_t locomotion_power = find_total_band_power(psd, fft_size, sampling_rate,
                                                       FOG_LOCOMOTION_MIN_FREQ, FOG_LOCOMOTION_MAX_FREQ);

    // 3) Update walking state with hysteresis based on locomotion-band power.
    if (locomotion_power > LOCOMOTION_POWER_THRESHOLD) {
        walking_state_counter++;
        if (walking_state_counter >= WALKING_STATE_HISTORY) {
            is_walking = true;
            walking_state_counter = WALKING_STATE_HISTORY; // clamp counter
        }
    } else {
        walking_state_counter--;
        if (walking_state_counter <= 0) {
            is_walking = false;
            walking_state_counter = 0;
        }
    }

    // 4) Compute Freeze Index (FI).
    float32_t freeze_index = freeze_power / (locomotion_power + 1e-6f);

    // 5) FOG decision logic.
    bool fi_check = (freeze_index > FOG_FI_THRESHOLD);
    bool walking_check = is_walking;
    bool freeze_power_check = (freeze_power > 0.05f);

    if (out != nullptr) {
        out->freeze_index = freeze_index;
        out->freeze_power = freeze_power;
        out->walking = is_walking;
        out->fi_match = fi_check;
        out->walking_match = walking_check;
        out->freeze_power_match = freeze_power_check;
    }

    LOG_DEBUG("FI: %.2f <%s>, walking: <%s>, freeze_pwr: %.3f <%s>",
              freeze_index, fi_check ? "true " : "false",
              is_walking ? "true " : "false",
              freeze_power, freeze_power_check ? "true " : "false");

    return (fi_check && walking_check && freeze_power_check);
}

/**
 * @brief Detect freezing-of-gait (FOG) using the Freeze Index (FI).
 *
 * Freeze Index is defined as:
 *   FI = P_freeze / (P_locomotion + eps)
 *
 * where:
 * - P_freeze is band power in 3–8 Hz (leg trembling/shuffling during freeze)
 * - P_locomotion is band power in 0.5–3 Hz (normal walking rhythm)
 *
 * We also require that the system believes we are in a walking state (based on
 * locomotion power over time). This avoids flagging FOG when the person is
 * simply standing still.
 *
 * @param psd Single-sided gyro PSD (length fft_size/2).
 * @param fft_size FFT size N.
 * @param sampling_rate Sampling rate Fs (Hz).
 * @return true if FOG is detected on this axis.
 */
bool detectFOG(float32_t* psd, uint32_t fft_size, float32_t sampling_rate, fog_axis_params_t* out) {
    return compute_fog_axis_params(psd, fft_size, sampling_rate, out);
}

/**
 * @brief RTOS task loop: read latest FFT result, run detectors, update filters.
 *
 * We run detectors on each gyro axis and then OR the three decisions to form an
 * overall status. The boolean filters provide temporal smoothing.
 */
void analysis_task() {
    LOG_INFO("Analysis Task Started");

    ThisThread::sleep_for(2000ms);

    bool_filter_init(&tremor_filter, 2);
    bool_filter_init(&dyskinesia_filter, 2);
    bool_filter_init(&fog_filter, 2);

    bool last_tremor_status = false;
    bool last_dyskinesia_status = false;
    bool last_fog_status = false;

    while (true) {
        fft_result_t *result = fft_find_and_lock_latest_result();
        if (result != nullptr) {
            bool tremor_result[3] = {false, false, false};
            bool dyskinesia_result[3] = {false, false, false};
            bool fog_result[3] = {false, false, false};
            td_axis_params_t tremor_params[3] = {};
            td_axis_params_t dyskinesia_params[3] = {};
            fog_axis_params_t fog_params[3] = {};

            for (int i = 0; i < 3; i++) {
                tremor_result[i] = detectTremor(result->gyro_psd[i], FFT_BUFFER_SIZE, IMU_SAMPLE_RATE_HZ, &tremor_params[i]);
            }
            for (int i = 0; i < 3; i++) {
                dyskinesia_result[i] = detectDyskinesia(result->gyro_psd[i], FFT_BUFFER_SIZE, IMU_SAMPLE_RATE_HZ, &dyskinesia_params[i]);
            }
            for (int i = 0; i < 3; i++) {
                fog_result[i] = detectFOG(result->gyro_psd[i], FFT_BUFFER_SIZE, IMU_SAMPLE_RATE_HZ, &fog_params[i]);
            }
            
            bool is_tremor = tremor_result[0] || tremor_result[1] || tremor_result[2];
            bool is_dyskinesia = dyskinesia_result[0] || dyskinesia_result[1] || dyskinesia_result[2];
            bool is_fog = fog_result[0] || fog_result[1] || fog_result[2];

            // 串口表格输出：表头只打印一次，之后每周期仅刷新 3 行数据
            print_td_table(tremor_params, dyskinesia_params, tremor_result, dyskinesia_result, fog_params, fog_result);

            result->mutex.unlock();

            bool_filter_update(&tremor_filter, is_tremor);
            bool_filter_update(&dyskinesia_filter, is_dyskinesia);
            bool_filter_update(&fog_filter, is_fog);

            // if (last_tremor_status != is_tremor && is_tremor == true) {
            //     LOG_INFO("Tremor detected!");
            // }
            // if (last_dyskinesia_status != is_dyskinesia && is_dyskinesia == true) {
            //     LOG_INFO("Dyskinesia detected!");
            // }
            // if (last_fog_status != is_fog && is_fog == true) {
            //     LOG_INFO("FOG detected!");
            // }

            last_tremor_status = is_tremor;
            last_dyskinesia_status = is_dyskinesia;
            last_fog_status = is_fog;
        } else {
            LOG_WARN("No FFT result available");
            ThisThread::sleep_for(1ms);
            continue;
        }
        ThisThread::sleep_for(200ms);
    }
}


/**
 * @brief Get the current filtered tremor status.
 * @return true if tremor is detected after filtering.
 */
bool get_tremor_status() {
    return bool_filter_get_state(&tremor_filter);
}

/**
 * @brief Get the current filtered dyskinesia status.
 * @return true if dyskinesia is detected after filtering.
 */
bool get_dyskinesia_status() {
    return bool_filter_get_state(&dyskinesia_filter);
}

/**
 * @brief Get the current filtered FOG status.
 * @return true if FOG is detected after filtering.
 */
bool get_fog_status() {
    return bool_filter_get_state(&fog_filter);
}
