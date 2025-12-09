#include "tasks/test_task.hpp"
#include "mbed.h"
#include "logger.hpp"


uint64_t prev_idle_time = 0;
#define SAMPLE_TIME_MS 2000


void test_task() {
    LOG_INFO("Test Task Started");

    mbed_stats_cpu_t stats;
    mbed_stats_cpu_get(&stats);
    prev_idle_time = stats.idle_time;

    ThisThread::sleep_for(SAMPLE_TIME_MS * 1ms);

    while (true) {
        mbed_stats_cpu_get(&stats);
    
        // 计算 CPU 使用率百分比
        uint64_t diff_usec = (stats.idle_time - prev_idle_time);
        uint8_t idle = (diff_usec * 100) / (SAMPLE_TIME_MS * 1000);
        uint8_t usage = 100 - idle;  // CPU 使用率
        prev_idle_time = stats.idle_time;
        
        // 打印统计信息
        LOG_INFO("Idle: %d%%   Usage: %d%%", idle, usage);     // 占用率

        ThisThread::sleep_for(SAMPLE_TIME_MS * 1ms);
    }
}