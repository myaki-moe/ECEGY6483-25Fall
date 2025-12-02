#ifndef ECEGY6483_LOGGER_H
#define ECEGY6483_LOGGER_H

// 日志级别定义
typedef enum {
    LOG_LEVEL_DEBUG = 0,
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARN,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_FATAL
} LogLevel_t;

// ANSI颜色代码
#define COLOR_RESET   "\033[0m"
#define COLOR_RED     "\033[31m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_BLUE    "\033[34m"
#define COLOR_MAGENTA "\033[35m"
#define COLOR_CYAN    "\033[36m"
#define COLOR_WHITE   "\033[37m"
#define COLOR_GRAY    "\033[90m"

// 全局日志级别设置（只打印>=此级别的日志）
extern LogLevel_t g_log_level;

// 核心日志函数
void log_print(LogLevel_t level, const char* format, ...);

// 便捷宏定义
#define LOG_DEBUG(fmt, ...) log_print(LOG_LEVEL_DEBUG, fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...)  log_print(LOG_LEVEL_INFO, fmt, ##__VA_ARGS__)
#define LOG_WARN(fmt, ...)  log_print(LOG_LEVEL_WARN, fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) log_print(LOG_LEVEL_ERROR, fmt, ##__VA_ARGS__)
#define LOG_FATAL(fmt, ...) log_print(LOG_LEVEL_FATAL, fmt, ##__VA_ARGS__)

// 设置全局日志级别
void log_set_level(LogLevel_t level);

#endif //ECEGY6483_LOGGER_H