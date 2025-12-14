#pragma once

#define BUILD_VERSION "1.0.4"

#include "mbed.h"

/**
 * @brief Global event flag used to broadcast a fatal error.
 *
 * This pointer is allocated in `main()` after basic hardware init.
 * Tasks call trigger_fatal_error() to request a system-wide shutdown.
 */
extern EventFlags *program_fatal_error_flag;

/**
 * @brief Signal a fatal error to the main thread.
 *
 * This sets bit 0 of program_fatal_error_flag. The main thread waits on this
 * flag and terminates all running tasks before entering the fatal LED blink
 * loop.
 */
void trigger_fatal_error();
