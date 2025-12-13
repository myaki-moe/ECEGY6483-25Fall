#pragma once

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

// INT1 interrupt pin connected to PD_11
#define LSM6DSL_INT1_PIN    PD_11

#define ACC_SENSITIVITY     0.000061f 
#define GYRO_SENSITIVITY    0.00875f

#define IMU_SAMPLE_RATE_HZ 208

#include <time.h>
#include "arm_math.h"

bool imu_read_acc_data(float32_t* acc);
bool imu_read_gyro_data(float32_t* gyro);
bool imu_init();
bool imu_data_ready();
bool imu_data_wait(time_t timeout);
void imu_data_ready_clear();
