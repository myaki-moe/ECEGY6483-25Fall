#pragma once

/**
 * @file ble_task.hpp
 * @brief BLE task that advertises a simple status characteristic.
 */

#define BLE_DEVICE_NAME "Parkinson's-Monitor-Group-46"

/**
 * @brief RTOS task entry: initialize BLE stack and run the event queue.
 */
void ble_task();

/**
 * @brief Query whether a BLE central is currently connected.
 * @return true if connected, false otherwise.
 */
bool ble_is_connected();
