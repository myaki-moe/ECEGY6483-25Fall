#pragma once

/**
 * @file serial.hpp
 * @brief Board support for the serial console (BufferedSerial).
 *
 * The logger uses serial_lock()/serial_unlock() to serialize printf output
 * across multiple RTOS threads.
 */

/**
 * @brief Initialize the serial console port.
 * @return true on success, false on failure.
 */
bool serial_init();

/**
 * @brief Send a raw string over the serial port.
 * @param data Null-terminated string.
 */
void serial_send(const char *data);

/**
 * @brief Lock the serial output mutex.
 */
void serial_lock();

/**
 * @brief Unlock the serial output mutex.
 */
void serial_unlock();
