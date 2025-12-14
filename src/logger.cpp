/**
 * @file logger.cpp
 * @brief Implementation of the lightweight printf-style logger.
 */

#include "logger.hpp"
#include "mbed.h"
#include "bsp/serial.hpp"

// Global log level (default: INFO).
LogLevel_t g_log_level = LOG_LEVEL_INFO;

// Level names (fixed width helps align logs).
static const char* log_level_strings[] = {
    "DEBUG",
    "INFO ",
    "WARN ",
    "ERROR",
    "FATAL"
};

// Optional per-level color (enable by defining COLORED_LOG).
#ifdef COLORED_LOG
static const char* log_level_colors[] = {
    COLOR_CYAN,    // DEBUG
    COLOR_GREEN,   // INFO
    COLOR_YELLOW,  // WARN
    COLOR_RED,     // ERROR
    COLOR_MAGENTA  // FATAL
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

// Set global log level.
void log_set_level(LogLevel_t level) {
    g_log_level = level;
}

// Core log print function (thread-safe via serial lock).
void log_print(LogLevel_t level, const char* format, ...) {
    // Filter out messages below the configured level.
    if (level < g_log_level) {
        return;
    }

    serial_lock();

    // Timestamp from RTOS kernel clock.
    auto now = Kernel::Clock::now().time_since_epoch();
    uint64_t ms_total = chrono::duration_cast<chrono::milliseconds>(now).count();
    unsigned long seconds = static_cast<unsigned long>(ms_total / 1000);
    unsigned long milliseconds = static_cast<unsigned long>(ms_total % 1000);

    // Log header: [sec.msec] [LEVEL] [thread_name]
    printf("%s[%5lu.%03lu] [%s] [%-15s]%s ",
            log_level_colors[level],
            seconds,
            milliseconds,
            log_level_strings[level],
            ThisThread::get_name(),
            log_level_colors_reset);

    // User message (printf-style).
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);

    // Newline (CRLF for serial terminals).
    printf("\r\n");

    serial_unlock();
}
