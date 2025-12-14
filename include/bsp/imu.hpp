#pragma once

/**
 * @file imu.hpp
 * @brief Board support for the LSM6DSL IMU (I2C + data-ready interrupt).
 *
 * The IMU provides 3-axis accelerometer and gyroscope measurements. This BSP
 * exposes basic init and read functions and a simple data-ready flag API.
 */

/**
 * @name LSM6DSL device address and registers
 * @{
 */
// LSM6DSL I2C address (0x6A shifted left by 1 for Mbed's 8-bit addressing)
#define LSM6DSL_ADDR        (0x6A << 1)
// Register addresses
#define WHO_AM_I            0x0F  // Device identification register
#define CTRL1_XL            0x10  // Accelerometer control register
#define CTRL2_G             0x11  // Gyroscope control register
#define CTRL3_C             0x12  // Common control register
#define DRDY_PULSE_CFG      0x0B  // Data-ready pulse configuration
#define INT1_CTRL           0x0D  // INT1 pin routing control
#define STATUS_REG          0x1E  // Status register (data ready flags)
#define OUTX_L_G            0x22  // Gyroscope X-axis low byte start address
#define OUTX_L_XL           0x28  // Accelerometer X-axis low byte start address
/** @} */

/**
 * @brief INT1 interrupt pin used for data-ready.
 */
#define LSM6DSL_INT1_PIN    PD_11

/**
 * @name Sensor scaling factors
 * These convert raw register values to physical units.
 * @{
 */
#define ACC_SENSITIVITY     0.000061f 
#define GYRO_SENSITIVITY    0.00875f
/** @} */

/**
 * @brief IMU sample rate configured in the sensor (Hz).
 */
#define IMU_SAMPLE_RATE_HZ 208

#include <time.h>
#include "arm_math.h"

/**
 * @brief Read accelerometer data (3 axes).
 * @param acc Output array of length 3 (units depend on ACC_SENSITIVITY).
 * @return true on success, false on I2C failure.
 */
bool imu_read_acc_data(float32_t* acc);

/**
 * @brief Read gyroscope data (3 axes).
 * @param gyro Output array of length 3, in rad/s.
 * @return true on success, false on I2C failure.
 */
bool imu_read_gyro_data(float32_t* gyro);

/**
 * @brief Initialize the IMU (I2C, interrupt pin, and configuration registers).
 * @return true on success, false if the device ID does not match or I2C fails.
 */
bool imu_init();

/**
 * @brief Check whether new IMU data is available (latched flag).
 * @return true if the data-ready flag is set.
 */
bool imu_data_ready();

/**
 * @brief Wait for new IMU data with timeout.
 * @param timeout Timeout (RTOS ticks / time units used by EventFlags).
 * @return true if data became ready before timeout.
 */
bool imu_data_wait(time_t timeout);

/**
 * @brief Clear the data-ready flag.
 */
void imu_data_ready_clear();
