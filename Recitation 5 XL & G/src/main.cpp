#include "mbed.h"
#include "rtos.h"
#include "arm_math.h"

// ==========================================
// 1. LED 与 状态定义 [NEW]
// ==========================================
DigitalOut led_rest(LED1);       // 绿灯：静息/正常
DigitalOut led_tremor(LED2);     // 蓝灯：震颤 (Tremor)
DigitalOut led_dyskinesia(LED3); // 黄/红灯：异动 (Dyskinesia)

// 定义三种医疗状态
enum MedicalState {
    STATE_REST,
    STATE_TREMOR,
    STATE_DYSKINESIA
};

// 全局状态变量 (Proc线程写, Main线程读)
volatile MedicalState current_state = STATE_REST;

// ==========================================
// 2. 配置参数 (保持不变)
// ==========================================
#define SAMPLE_RATE_HZ      52.0f
#define WINDOW_SIZE         156 
#define STEP_SIZE           52  
#define FFT_SIZE            256 

// 诊断阈值
#define TREMOR_MIN          3.0f
#define TREMOR_MAX          5.0f
#define DYSKINESIA_MIN      5.0f
#define DYSKINESIA_MAX      7.0f
#define ACC_ENERGY_THRES    0.5f 
#define GYRO_ENERGY_THRES   10.0f
#define ACC_SENSITIVITY     0.000061f 
#define GYRO_SENSITIVITY    0.00875f

// ==========================================
// 3. 数据结构与队列
// ==========================================
typedef struct {
    float acc_chunk[STEP_SIZE];
    float gyro_chunk[STEP_SIZE]; 
} sensor_chunk_t;

Mail<sensor_chunk_t, 4> mail_box;

I2C i2c(PB_11, PB_10);
InterruptIn int1(PD_11, PullDown);

Thread acq_thread(osPriorityHigh, 4096, NULL, "Acq");
Thread proc_thread(osPriorityAboveNormal, 8192, NULL, "Proc");
EventQueue print_queue;

volatile bool data_ready_flag = false;
arm_rfft_fast_instance_f32 S;

// LSM6DSL Registers
#define LSM6DSL_ADDR (0x6A << 1)
#define WHO_AM_I 0x0F
#define CTRL1_XL 0x10
#define CTRL2_G 0x11
#define CTRL3_C 0x12
#define DRDY_PULSE_CFG 0x0B
#define INT1_CTRL 0x0D
#define OUTX_L_G 0x22
#define OUTX_L_XL 0x28

// ==========================================
// 4. LED 闪烁逻辑 [NEW]
// ==========================================
// 这个函数会被 print_queue 每 500ms 调用一次
void blink_task() {
    switch (current_state) {
        case STATE_REST:
            led_rest = !led_rest;     // 翻转 LED1
            led_tremor = 0;           // 关闭其他
            led_dyskinesia = 0;
            break;

        case STATE_TREMOR:
            led_rest = 0;
            led_tremor = !led_tremor; // 翻转 LED2
            led_dyskinesia = 0;
            break;

        case STATE_DYSKINESIA:
            led_rest = 0;
            led_tremor = 0;
            led_dyskinesia = !led_dyskinesia; // 翻转 LED3
            break;
    }
}

// ==========================================
// 5. 底层驱动 (保持不变)
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
// 6. 采集任务 (保持不变)
// ==========================================
void acquisition_task() {
    int16_t raw_val;
    int chunk_index = 0;
    sensor_chunk_t *current_mail = mail_box.alloc();

    while (true) {
        if (data_ready_flag) {
            data_ready_flag = false;
            if (current_mail != NULL) {
                if (read_int16(OUTX_L_XL, raw_val)) current_mail->acc_chunk[chunk_index] = (float)raw_val * ACC_SENSITIVITY;
                if (read_int16(OUTX_L_G, raw_val)) current_mail->gyro_chunk[chunk_index] = (float)raw_val * GYRO_SENSITIVITY;
                
                chunk_index++;
                if (chunk_index >= STEP_SIZE) {
                    mail_box.put(current_mail);
                    current_mail = mail_box.try_alloc(); 
                    chunk_index = 0;
                }
            } else {
                current_mail = mail_box.try_alloc();
            }
        }
        ThisThread::sleep_for(1ms);
    }
}

// ==========================================
// 7. 处理任务 (修改了状态更新逻辑)
// ==========================================

void print_result(float freq, float en, const char* msg) {
    // 简化打印，防止串口刷太快看不清
    printf("DOM_FREQ: %.2f Hz | ENERGY: %.2f | STATUS: %s\n", freq, en, msg);
}

void run_fft(float* history_buffer, float* freq_out, float* energy_out) {
    static float32_t fft_in[FFT_SIZE];
    static float32_t fft_out[FFT_SIZE];

    for(int i=0; i<WINDOW_SIZE; i++) fft_in[i] = history_buffer[i];
    for(int i=WINDOW_SIZE; i<FFT_SIZE; i++) fft_in[i] = 0.0f;

    float sum=0; for(int i=0; i<WINDOW_SIZE; i++) sum += fft_in[i];
    float mean = sum / WINDOW_SIZE;
    for(int i=0; i<WINDOW_SIZE; i++) fft_in[i] -= mean;

    arm_rfft_fast_f32(&S, fft_in, fft_out, 0);
    arm_cmplx_mag_f32(fft_out, fft_in, FFT_SIZE/2);

    uint32_t start_idx = 5;
    float32_t max_val;
    uint32_t max_idx;
    arm_max_f32(&fft_in[start_idx], (FFT_SIZE/2)-start_idx, &max_val, &max_idx);

    *energy_out = max_val;
    *freq_out = (float)(max_idx + start_idx) * (SAMPLE_RATE_HZ / (float)FFT_SIZE);
}

void processing_task() {
    arm_rfft_fast_init_f32(&S, FFT_SIZE);
    static float acc_history[WINDOW_SIZE] = {0};
    static float gyro_history[WINDOW_SIZE] = {0};
    float acc_f, acc_e, gyro_f, gyro_e;

    while (true) {
        osEvent evt = mail_box.get();
        if (evt.status == osEventMail) {
            sensor_chunk_t *mail = (sensor_chunk_t*)evt.value.p;

            // 1. 滑动窗口更新
            memmove(&acc_history[0], &acc_history[STEP_SIZE], (WINDOW_SIZE - STEP_SIZE) * sizeof(float));
            memmove(&gyro_history[0], &gyro_history[STEP_SIZE], (WINDOW_SIZE - STEP_SIZE) * sizeof(float));
            memcpy(&acc_history[WINDOW_SIZE - STEP_SIZE], mail->acc_chunk, STEP_SIZE * sizeof(float));
            memcpy(&gyro_history[WINDOW_SIZE - STEP_SIZE], mail->gyro_chunk, STEP_SIZE * sizeof(float));

            // 2. FFT
            run_fft(acc_history, &acc_f, &acc_e);
            run_fft(gyro_history, &gyro_f, &gyro_e);

            // 3. 诊断与状态切换 [关键修改]
            const char* diagnosis_str = "Rest";
            bool is_moving = (acc_e > ACC_ENERGY_THRES) || (gyro_e > GYRO_ENERGY_THRES);
            float dom_freq = (acc_e > 0.5f) ? acc_f : gyro_f;

            if (is_moving) {
                if (dom_freq >= TREMOR_MIN && dom_freq < TREMOR_MAX) {
                    diagnosis_str = "TREMOR";
                    current_state = STATE_TREMOR; // 切换状态
                } 
                else if (dom_freq >= DYSKINESIA_MIN && dom_freq <= DYSKINESIA_MAX) {
                    diagnosis_str = "DYSKINESIA";
                    current_state = STATE_DYSKINESIA; // 切换状态
                } 
                else {
                    // 其他运动（Voluntary Move）也算作 Rest/Wait 状态，或者你可以加个 LED
                    diagnosis_str = "Moving";
                    current_state = STATE_REST; 
                }
            } else {
                diagnosis_str = "Resting";
                current_state = STATE_REST; // 切换状态
            }

            print_queue.call(print_result, dom_freq, (acc_e > gyro_e ? acc_e : gyro_e), diagnosis_str);
            mail_box.free(mail);
        }
    }
}

// ==========================================
// 8. MAIN
// ==========================================
int main() {
    static BufferedSerial pc(USBTX, USBRX, 115200);
    printf("--- Tremor Analysis with LED Effects ---\n");
    
    ThisThread::sleep_for(200ms);
    i2c.frequency(400000); 
    if (!init_sensor()) { printf("Sensor Init Failed!\n"); while(1); }

    acq_thread.start(acquisition_task);
    proc_thread.start(processing_task);
    
    // [关键修改] 启动 LED 闪烁任务，每 500ms 运行一次
    // 500ms 翻转一次 = 1秒钟闪一下 (1Hz)
    print_queue.call_every(500ms, blink_task);

    // 开始调度
    print_queue.dispatch_forever();
}