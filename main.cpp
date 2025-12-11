#include "mbed.h"
#include "rtos.h"
#include "arm_math.h"
#include <chrono>

// ==========================================
// 1. Define Data Packet Structure (Mail Payload)
// ==========================================
// To save memory, we transmit small chunks of data instead of the full 3-second buffer.
#define CHUNK_SIZE          52    // 1 second of data per packet
#define WINDOW_SIZE         156   // 3-second total window (Required for FFT)
#define FFT_SIZE            256 

typedef struct {
    float acc_data[CHUNK_SIZE];  // Accelerometer data for this second
    float gyro_data[CHUNK_SIZE]; // Gyroscope data for this second
} sensor_packet_t;

// Create Mail queue, capacity of 4 packets
Mail<sensor_packet_t, 4> mail_box;

// ==========================================
// 2. Hardware & Parameters
// ==========================================
I2C i2c(PB_11, PB_10);
InterruptIn int1(PD_11, PullDown);
DigitalOut led_rest(LED1);       // Green: Rest / Tremor
DigitalOut led_dyskinesia(LED2); // Blue: Dyskinesia
DigitalOut led_fog(LED3);        // Red: Freezing of Gait (FOG)

// Parameters
#define SAMPLE_RATE_HZ      52.0f
#define TREMOR_MIN          3.0f
#define TREMOR_MAX          5.0f
#define DYSKINESIA_MIN      5.0f
#define DYSKINESIA_MAX      7.0f
#define ACC_ENERGY_THRES    0.5f 
#define GYRO_ENERGY_THRES   10.0f
#define ACC_SENSITIVITY     0.000061f 
#define GYRO_SENSITIVITY    0.00875f

// FOG Specific Parameters
#define STEP_AMP_THRESHOLD  0.15f 
#define MIN_STEP_INTERVAL   200   
#define FOG_TIMEOUT_MS      1500  

// Threads
Thread acq_thread(osPriorityHigh);
Thread proc_thread(osPriorityAboveNormal);
EventQueue print_queue;

volatile bool data_ready_flag = false;
arm_rfft_fast_instance_f32 S;

// LSM6DSL Registers (Unchanged)
#define LSM6DSL_ADDR (0x6A << 1)
#define WHO_AM_I 0x0F
#define CTRL1_XL 0x10
#define CTRL2_G 0x11
#define CTRL3_C 0x12
#define DRDY_PULSE_CFG 0x0B
#define INT1_CTRL 0x0D
#define OUTX_L_G 0x22
#define OUTX_L_XL 0x28
// Threshold for detecting high-energy struggles characteristic of FOG
#define FOG_HIGH_ENERGY_THRES  0.5f 

// ==========================================
// 3. Low Level Drivers (Unchanged)
// ==========================================
void isr_drdy() { data_ready_flag = true; }

bool write_reg(uint8_t reg, uint8_t val) {
    char buf[2] = {(char)reg, (char)val};
    return (i2c.write(LSM6DSL_ADDR, buf, 2) == 0);
}
bool read_reg(uint8_t reg, uint8_t &val) {
    char r = (char)reg;
    if (i2c.write(LSM6DSL_ADDR, &r, 1, true) != 0) return false;
    if (i2c.read(LSM6DSL_ADDR, &r, 1) != 0) return false;
    val = (uint8_t)r;
    return true;
}
bool read_int16(uint8_t reg_low, int16_t &val) {
    uint8_t lo, hi;
    if (!read_reg(reg_low, lo)) return false;
    if (!read_reg(reg_low + 1, hi)) return false;
    val = (int16_t)((hi << 8) | lo);
    return true;
}

bool init_sensor() {
    uint8_t who;
    if (!read_reg(WHO_AM_I, who) || who != 0x6A) return false;
    write_reg(CTRL3_C, 0x44); 
    write_reg(CTRL1_XL, 0x30); 
    write_reg(CTRL2_G, 0x30); 
    write_reg(INT1_CTRL, 0x01); 
    write_reg(DRDY_PULSE_CFG, 0x80);
    int1.rise(&isr_drdy);
    return true;
}

// ==========================================
// 4. Acquisition Task (Producer)
// ==========================================
void acquisition_task() {
    int16_t acc_raw, gyro_raw;
    int chunk_idx = 0;
    
    // Allocate the first mail packet
    sensor_packet_t *current_mail = mail_box.alloc();

    while (true) {
        if (data_ready_flag) {
            data_ready_flag = false;
            
            // Only fill data if we have a valid packet; otherwise discard (packet loss is rare)
            if (current_mail != NULL) {
                // Read Accel
                if (read_int16(OUTX_L_XL, acc_raw)) {
                    current_mail->acc_data[chunk_idx] = (float)acc_raw * ACC_SENSITIVITY;
                }
                // Read Gyro
                if (read_int16(OUTX_L_G, gyro_raw)) {
                    current_mail->gyro_data[chunk_idx] = (float)gyro_raw * GYRO_SENSITIVITY;
                }
                
                chunk_idx++;
                
                // If the chunk is full (e.g., 1 second of data)
                if (chunk_idx >= CHUNK_SIZE) {
                    mail_box.put(current_mail); // Send it!
                    
                    // Allocate the next packet
                    current_mail = mail_box.try_alloc(); 
                    chunk_idx = 0; // Reset index
                }
            } else {
                // If allocation failed previously, try again
                current_mail = mail_box.try_alloc();
            }
        }
        ThisThread::sleep_for(1ms);
    }
}

// ==========================================
// 5. Processing Task (Consumer) - Includes Sliding Window & FOG
// ==========================================

void print_result(float freq, float en, const char* msg) {
    printf("[Analyzed] Freq: %.2f Hz | Energy: %.2f | >> %s\n", freq, en, msg);
}

// FFT Helper Function
void run_fft(float* history_buffer, float* freq_out, float* energy_out) {
    static float32_t fft_in[FFT_SIZE]; // Zero-padded buffer
    static float32_t fft_out[FFT_SIZE];

    // 1. Copy history and remove DC offset
    float sum = 0;
    for(int i=0; i<WINDOW_SIZE; i++) sum += history_buffer[i];
    float mean = sum / WINDOW_SIZE;
    
    for(int i=0; i<WINDOW_SIZE; i++) fft_in[i] = history_buffer[i] - mean;
    // 2. Zero padding
    for(int i=WINDOW_SIZE; i<FFT_SIZE; i++) fft_in[i] = 0.0f;

    // 3. FFT
    arm_rfft_fast_f32(&S, fft_in, fft_out, 0);
    arm_cmplx_mag_f32(fft_out, fft_in, FFT_SIZE/2);

    // 4. Peak Finding
    uint32_t start_idx = 5;
    float32_t max_val;
    uint32_t max_idx;
    arm_max_f32(&fft_in[start_idx], (FFT_SIZE/2)-start_idx, &max_val, &max_idx);

    *energy_out = max_val;
    *freq_out = (float)(max_idx + start_idx) * (SAMPLE_RATE_HZ / (float)FFT_SIZE);
}

void processing_task() {
    arm_rfft_fast_init_f32(&S, FFT_SIZE);
    
    // Sliding Window History Buffer (3 seconds)
    // Must be static or allocated on heap; stack might be insufficient
    static float acc_history[WINDOW_SIZE] = {0}; 
    static float gyro_history[WINDOW_SIZE] = {0};
    static bool is_in_fog_state = false;
    
    // FOG State Variables
    static uint32_t last_step_time = 0;
    static bool was_walking = false;
    
    // LED reset timer
    int led_reset_timer = 0;

    while (true) {
        // Wait for mail (blocking without timeout is most power efficient)
        osEvent evt = mail_box.get();
        
        if (evt.status == osEventMail) {
            sensor_packet_t *mail = (sensor_packet_t*)evt.value.p;

            // --- 1. Sliding Window Concatenation (Core Logic) ---
            // Shift old data to the left by CHUNK_SIZE
            // e.g., [Old_Sec1, Old_Sec2, Old_Sec3] -> [Old_Sec2, Old_Sec3, NEW_DATA]
            memmove(&acc_history[0], &acc_history[CHUNK_SIZE], (WINDOW_SIZE - CHUNK_SIZE) * sizeof(float));
            memmove(&gyro_history[0], &gyro_history[CHUNK_SIZE], (WINDOW_SIZE - CHUNK_SIZE) * sizeof(float));
            
            // Append new mail data to the end
            memcpy(&acc_history[WINDOW_SIZE - CHUNK_SIZE], mail->acc_data, CHUNK_SIZE * sizeof(float));
            memcpy(&gyro_history[WINDOW_SIZE - CHUNK_SIZE], mail->gyro_data, CHUNK_SIZE * sizeof(float));

            // --- 2. Free Mail ---
            // Data copied to history, return packet to acquisition thread
            mail_box.free(mail);

            // --- 3. FOG Step Detection (Search in history or new data) ---
            // For simplicity, iterate through DC-removed history
            float acc_sum = 0;
            for(int i=0; i<WINDOW_SIZE; i++) acc_sum += acc_history[i];
            float acc_mean = acc_sum / WINDOW_SIZE;
            uint32_t now = Kernel::get_ms_count();
            bool step_detected = false;
            
            // Focus check on the newest data chunk (last CHUNK_SIZE points)
            for(int i = WINDOW_SIZE - CHUNK_SIZE; i < WINDOW_SIZE; i++) {
                if (abs(acc_history[i] - acc_mean) > STEP_AMP_THRESHOLD) {
                    if ((now - last_step_time) > MIN_STEP_INTERVAL) {
                        last_step_time = now;
                        step_detected = true;
                    }
                }
            }

            // --- 4. Run FFT ---
            float acc_f, acc_e, gyro_f, gyro_e;
            run_fft(acc_history, &acc_f, &acc_e);
            run_fft(gyro_history, &gyro_f, &gyro_e);
            
            // --- 5. Diagnosis Logic (State Machine) ---
            bool is_moving = (acc_e > ACC_ENERGY_THRES) || (gyro_e > GYRO_ENERGY_THRES);
            bool is_walking_now = (now - last_step_time) < 2000;
            float dom_freq = (acc_e > 0.5f) ? acc_f : gyro_f;
            const char* diag = "Rest";

            // Turn off all LEDs (will be turned on based on logic)
            led_rest = 0; led_dyskinesia = 0; led_fog = 0;
            bool is_fog_level_energy = (acc_e > FOG_HIGH_ENERGY_THRES);
            
            if (!is_in_fog_state && was_walking && !is_walking_now && is_moving && is_fog_level_energy) {
                is_in_fog_state = true;
                printf(">>> ENTER FOG STATE <<<\n");
            }


            if (is_in_fog_state) {
                if (step_detected) {
                    is_in_fog_state = false; // Resumed walking!
                    was_walking = true;      // Re-flag as walking
                    printf(">>> EXIT FOG (Walked) <<<\n");
                } 
                else if (!is_moving) {
                    is_in_fog_state = false; // Stopped struggling (Resting)
                    was_walking = false;
                    printf(">>> EXIT FOG (Resting) <<<\n");
                }
            }


            if (is_in_fog_state) {
                // FOG Trigger: Was walking, now no steps, but still shaking (high energy)
                diag = "!!! FOG DETECTED !!!";
                led_fog = 1; 
                was_walking = false; // Reset
            } 
            else if (is_moving) {
                if (step_detected) was_walking = true;
                
                if (dom_freq >= TREMOR_MIN && dom_freq < TREMOR_MAX) {
                    diag = "Tremor";
                    led_rest = 1; // Using LED1 for Tremor
                } else if (dom_freq >= DYSKINESIA_MIN && dom_freq <= DYSKINESIA_MAX) {
                    diag = "Dyskinesia";
                    led_dyskinesia = 1;
                } else {
                    diag = is_walking_now ? "Walking" : "Moving";
                }
            } else {
                diag = "Resting";
                was_walking = false;
            }
            
            print_queue.call(print_result, dom_freq, is_moving ? acc_e : 0.0f, diag);
        }
    }
}

// ==========================================
// 6. MAIN
// ==========================================
int main() {
    static BufferedSerial pc(USBTX, USBRX, 115200);
    printf("--- System: Mail Queue + Sliding Window + FOG ---\n");
    
    i2c.frequency(400000); 
    if (!init_sensor()) { printf("Sensor Init Failed!\n"); while(1); }

    acq_thread.start(acquisition_task);
    proc_thread.start(processing_task);
    
    print_queue.dispatch_forever();
}