#include "logger.h"
#include "cmsis_os.h"
#include "semphr.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

// 互斥锁保护printf（避免多任务同时打印导致混乱）
extern osMutexId_t log_mutexHandle;

// 全局日志级别（默认DEBUG，显示所有日志）
LogLevel_t g_log_level = LOG_LEVEL_INFO;

// 日志级别字符串
static const char* log_level_strings[] = {
    "DEBUG",
    "INFO ",
    "WARN ",
    "ERROR",
    "FATAL"
};

// 日志级别颜色
static const char* log_level_colors[] = {
    COLOR_CYAN,    // DEBUG - 青色
    COLOR_GREEN,   // INFO  - 绿色
    COLOR_YELLOW,  // WARN  - 黄色
    COLOR_RED,     // ERROR - 红色
    COLOR_MAGENTA  // FATAL - 品红色
};

// 设置全局日志级别
void log_set_level(LogLevel_t level) {
    g_log_level = level;
}

// 核心日志打印函数
void log_print(LogLevel_t level, const char* format, ...) {
    // 过滤低于设定级别的日志
    if (level < g_log_level) {
        return;
    }

    // 确保互斥锁已创建
    while (log_mutexHandle == NULL) {}

    // 获取互斥锁
    if (xSemaphoreTake(log_mutexHandle, portMAX_DELAY) == pdTRUE) {
        // 获取当前任务名称
        char task_name[configMAX_TASK_NAME_LEN];
        TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
        if (current_task != NULL) {
            strncpy(task_name, pcTaskGetName(current_task), configMAX_TASK_NAME_LEN - 1);
            task_name[configMAX_TASK_NAME_LEN - 1] = '\0';
        } else {
            strcpy(task_name, "UNKNOWN");
        }

        // 获取时间戳（FreeRTOS tick count，单位ms）
        TickType_t tick = xTaskGetTickCount();
        uint32_t timestamp_ms = tick * portTICK_PERIOD_MS;
        uint32_t seconds = timestamp_ms / 1000;
        uint32_t milliseconds = timestamp_ms % 1000;

        // 打印日志头部（时间戳、级别、任务名）
        printf("%s[%5lu.%03lu] [%s] [%-15s]%s ",
               log_level_colors[level],
               seconds,
               milliseconds,
               log_level_strings[level],
               task_name,
               COLOR_RESET);

        // 打印用户消息
        va_list args;
        va_start(args, format);
        vprintf(format, args);
        va_end(args);

        // 换行
        printf("\r\n");

        // 释放互斥锁
        xSemaphoreGive(log_mutexHandle);
    }
}