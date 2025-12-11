#include "logger.hpp"
#include "mbed.h"
#include "bsp/serial.hpp"

// 全局日志级别（默认INFO，显示INFO及以上日志）
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
#ifdef COLORED_LOG
static const char* log_level_colors[] = {
    COLOR_CYAN,    // DEBUG - 青色
    COLOR_GREEN,   // INFO  - 绿色
    COLOR_YELLOW,  // WARN  - 黄色
    COLOR_RED,     // ERROR - 红色
    COLOR_MAGENTA  // FATAL - 品红色
};
char* log_level_colors_reset = COLOR_RESET;
#else
static const char* log_level_colors[] = {
    "",
    "", 
    "",
    "",
    "" 
};
const char* log_level_colors_reset = "\0";
#endif

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

    serial_lock();

    // 获取时间戳
    auto now = Kernel::Clock::now().time_since_epoch();
    uint64_t ms_total = chrono::duration_cast<chrono::milliseconds>(now).count();
    unsigned long seconds = static_cast<unsigned long>(ms_total / 1000);
    unsigned long milliseconds = static_cast<unsigned long>(ms_total % 1000);

    // 打印日志头部（时间戳、级别、任务名）
    printf("%s[%5lu.%03lu] [%s] [%-15s]%s ",
            log_level_colors[level],
            seconds,
            milliseconds,
            log_level_strings[level],
            ThisThread::get_name(),
            log_level_colors_reset);

    // 打印用户消息
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);

    // 换行
    printf("\r\n");

    serial_unlock();
}