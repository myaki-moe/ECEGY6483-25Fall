#pragma once

/**
 * @file logger.hpp
 * @brief Lightweight printf-style logger with levels (optionally colored).
 *
 * This logger prints a timestamp, level, and current RTOS thread name, then
 * the user message. Output is serialized using the BSP serial lock.
 */

/**
 * @brief Log severity levels (increasing).
 */
typedef enum {
    LOG_LEVEL_DEBUG = 0,
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARN,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_FATAL
} LogLevel_t;

/**
 * @name ANSI color escape sequences (used when COLORED_LOG is enabled)
 * @{
 */
#define COLOR_RESET   "\033[0m"
#define COLOR_RED     "\033[31m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_BLUE    "\033[34m"
#define COLOR_MAGENTA "\033[35m"
#define COLOR_CYAN    "\033[36m"
#define COLOR_WHITE   "\033[37m"
#define COLOR_GRAY    "\033[90m"
/** @} */

/**
 * @brief Global log level filter (prints messages with level >= g_log_level).
 */
extern LogLevel_t g_log_level;

/**
 * @brief Core log function (printf-style).
 * @param level Log level.
 * @param format printf-style format string.
 */
void log_print(LogLevel_t level, const char* format, ...);

/**
 * @name Convenience macros
 * @{
 */
#define LOG_DEBUG(fmt, ...) log_print(LOG_LEVEL_DEBUG, fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...)  log_print(LOG_LEVEL_INFO, fmt, ##__VA_ARGS__)
#define LOG_WARN(fmt, ...)  log_print(LOG_LEVEL_WARN, fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) log_print(LOG_LEVEL_ERROR, fmt, ##__VA_ARGS__)
#define LOG_FATAL(fmt, ...) log_print(LOG_LEVEL_FATAL, fmt, ##__VA_ARGS__)
/** @} */

/**
 * @brief Set the global log level filter.
 * @param level Minimum level that will be printed.
 */
void log_set_level(LogLevel_t level);
