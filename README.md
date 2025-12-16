## Parkinson’s Motion Detection System
Embedded Challenge Fall 2025 — Group 46

### Abstract
This project implements a real-time embedded motion detection pipeline targeting Parkinson’s-related motion patterns using an on-board LSM6DSL IMU. The system continuously samples 3-axis accelerometer and gyroscope data, performs sliding-window FFT processing, derives a power spectral density (PSD) estimate, and applies frequency-domain heuristics to classify three clinically relevant motion states: **tremor**, **dyskinesia**, and **freezing of gait (FOG)**. To improve robustness under noisy conditions, results are stabilized with hysteresis-style boolean filtering. Status is exposed through both **LED patterns** and a **BLE GATT notification** interface.

### System Overview
The firmware is organized as multiple RTOS threads with priorities aligned to timing sensitivity:

- **IMU task (Realtime priority)**: waits for IMU data-ready interrupt, reads/scales sensor data, and publishes timestamped samples to a mailbox.
- **FFT task (High priority)**: maintains a sliding window per axis, computes FFT/PSD, and publishes results into a small ring buffer protected by mutexes.
- **Analysis task (High priority)**: consumes the newest PSD result and runs tremor/dyskinesia/FOG detectors; outputs filtered state flags.
- **LED task (Normal priority)**: renders status via LEDs and indicates BLE connection state.
- **BLE task (Normal priority)**: advertises a custom service and periodically notifies the current state string.
- **Test task (Low priority)**: prints CPU usage and thread statistics for profiling.

A supervisor in `main()` initializes hardware, starts tasks, and monitors a global fatal flag. Upon fatal error, all tasks are terminated and the system enters a visible “fatal” LED loop.

### Sensor Acquisition and Scaling
The IMU is configured and uses a data-ready interrupt. Each sample provides:

- **Accelerometer**: raw 16-bit values scaled by `ACC_SENSITIVITY = 0.000061` (units depend on configured full scale).
- **Gyroscope**: raw 16-bit values scaled by `GYRO_SENSITIVITY = 0.00875`, converted from deg/s to **rad/s**.

### Core Data Processing Pipeline
#### Streaming Mailbox (Producer–Consumer)
IMU samples are passed via an RTOS mailbox (`Mail<imu_data_t, 10>`), providing bounded memory usage, decoupling between sampling and processing, and predictable behavior when processing falls behind.

#### Mirror Circular Buffer (Key Data-Structure Algorithm)
To support sliding-window DSP efficiently, the FFT task maintains **mirror buffers** per axis. Each pushed element is written twice—at index `i` and `i + window_size`—in a `2 * window_size` region so the most recent window is always contiguous in memory. This removes wrap-around handling when providing DSP routines with a window.

#### FFT + PSD Computation (Key DSP Algorithm)
The FFT task uses CMSIS-DSP `arm_rfft_fast_f32` with **FFT size N = 256**.

- **Frequency resolution**: Δf = Fs/N = 208/256 ≈ **0.8125 Hz**
- Single-sided arrays of length N/2 = **128 bins** are stored (0 to Nyquist).

Per axis:

1. Copy the current sliding-window samples into `fft_input`.
2. Real FFT: time-domain → frequency-domain.
3. Compute magnitude spectrum |X[k]| for k=0..N/2-1.
4. Compute power: |X[k]|² (simple PSD estimate).
5. Apply a normalization scale factor:

PSD[k] = |X[k]|² · (1 / (N · Fs))

#### Double-Buffered Result Ring + Try-Lock (Key Concurrency Pattern)
FFT outputs are stored in a small ring (`FFT_BUFFER_NUM = 2`) where each buffer contains magnitude/PSD arrays, a timestamp, and a mutex.

- Producer (FFT task) tries to lock the **oldest** available buffer and overwrites it.
- Consumer (analysis task) tries to lock the **newest** available buffer.

This reduces blocking and avoids partial reads/writes.

### Motion Classification Algorithms (Frequency-Domain Heuristics)
All detection is performed on **gyroscope PSD**, evaluated per-axis and then OR-combined across x/y/z.

#### Shared Feature Extractors
- **Peak-in-band**: maximum PSD and its frequency in a band [fmin, fmax].
- **Band power**: sum of PSD bins across a band (inclusive).

Bin mapping uses: fk = k·Fs/N and k = floor(f·N/Fs). A small epsilon (1e-6) is added to denominators to prevent divide-by-zero.

#### Tremor Detection (3–5 Hz Dominance)
Tremor is detected if:

- The dominant peak in **3–12 Hz** lies in **3–5 Hz**, and
- Relative power exceeds a threshold:

RelPower = P(3–5) / (P(3–12) + eps) > **0.75**

- Peak PSD exceeds an absolute threshold (noise gate):

max PSD in 3–12 Hz > **1.0**

#### Dyskinesia Detection (5–7 Hz Dominance)
Dyskinesia detection mirrors tremor but targets **5–7 Hz**:

- Peak frequency in 5–7 Hz,
- P(5–7)/(P(3–12)+eps) > **0.75**,
- Peak PSD > **1.0**.

#### Freezing of Gait (FOG) via Freeze Index + Walking Context
FOG uses a **Freeze Index (FI)**:

FI = P(3–8) / (P(0.5–3) + eps)

Decision requires:

- FI > **2.0**,
- “walking state” is true (context gating),
- freeze-band power exceeds a small absolute threshold (reject noise-only triggers).

Walking state is estimated using locomotion-band power hysteresis with a history counter (`WALKING_STATE_HISTORY = 150`) and threshold `P(0.5–3) > 0.1`.

### Temporal Smoothing: Boolean Debounce / Hysteresis Filter
Raw frame-to-frame decisions can flicker. Each class output is passed through a boolean debouncer that requires the new value to persist for N consecutive updates before switching. In this project, the threshold is **2**, providing low latency with improved stability.

### Output Interfaces
#### LED Patterns
- **FOG**: blinking blue/yellow pattern
- **Dyskinesia**: steady yellow
- **Tremor**: steady blue
- **None**: off

Additionally, a “breathing” green LED indicates liveness, and a second green LED reflects BLE connection state.

#### BLE GATT Notifications
A custom BLE service exposes a notify-only characteristic containing a null-terminated ASCII status string:

- `"FOG"`, `"DYSKINESIA"`, `"TREMOR"`, or `"NONE"`

Notifications are sent periodically (1 Hz) via an event queue.

### Practical Notes (Compute and Memory)
- FFT/PSD is computed for **6 axes** (3 accel + 3 gyro) using N=256.
- Mirror buffers and FFT ring buffers trade RAM for predictable, contiguous windows and low-latency access to results.

### Limitations and Potential Improvements
- **Windowing**: no explicit Hann/Hamming window is applied; adding one could reduce spectral leakage.
- **Compute budget**: FFT per sample-step is expensive; updating FFT every M samples could reduce CPU usage.
- **Threshold robustness**: fixed thresholds may vary with mounting/user; calibration or adaptive normalization could help.
- **Feature expansion**: adding time-domain features (RMS/entropy/autocorrelation) may reduce ambiguity.

### Conclusion
This project demonstrates an end-to-end embedded DSP pipeline for Parkinson’s-related motion classification: deterministic IMU acquisition, efficient sliding-window buffering, real-time FFT/PSD extraction, interpretable frequency-band heuristics (including Freeze Index with walking-context gating), and stable outputs via hysteresis filtering. The architecture cleanly separates sensing, DSP, classification, and user/communication interfaces.
