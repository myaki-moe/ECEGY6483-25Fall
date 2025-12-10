#include "tasks/test_task.hpp"
#include "mbed.h"
#include "mbed_stats.h"
#include <inttypes.h>
#include "logger.hpp"
#include "main.hpp"


uint64_t prev_idle_time = 0;
#define SAMPLE_TIME_MS 2000


void test_task() {
    LOG_INFO("Test Task Started");

    mbed_stats_thread_t *thread_stats = new mbed_stats_thread_t[8];

    mbed_stats_cpu_t cpu_stats;
    mbed_stats_cpu_get(&cpu_stats);
    prev_idle_time = cpu_stats.idle_time;

    ThisThread::sleep_for(SAMPLE_TIME_MS * 1ms);

    while (true) {
        int count = mbed_stats_thread_get_each(thread_stats, 8);
        for (int i = 0; i < count; i++) {
            LOG_DEBUG("ID: 0x%" PRIx32 " Name: %s State: %" PRId32 " Priority: %" PRId32 " Stack Size: %" PRId32 " Stack Space: %" PRId32, thread_stats[i].id, thread_stats[i].name, thread_stats[i].state, thread_stats[i].priority, thread_stats[i].stack_size, thread_stats[i].stack_space);
        }
        
        mbed_stats_cpu_get(&cpu_stats);
    
        // 计算 CPU 使用率百分比
        uint64_t diff_usec = (cpu_stats.idle_time - prev_idle_time);
        uint8_t idle = (diff_usec * 100) / (SAMPLE_TIME_MS * 1000);
        uint8_t usage = 100 - idle;  // CPU 使用率
        prev_idle_time = cpu_stats.idle_time;
        
        // 打印统计信息
        LOG_DEBUG("CPU Usage: %d%%   Idle: %d%%", usage, idle);     // 占用率

        ThisThread::sleep_for(SAMPLE_TIME_MS * 1ms);
    }
}