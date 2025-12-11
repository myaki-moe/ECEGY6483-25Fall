#include "bsp/imu.hpp"
#include "mbed.h"

I2C *imu_i2c = nullptr;
InterruptIn *imu_int1_pin = nullptr;
EventFlags *imu_data_ready_flag = nullptr;

// Write a single byte to a register
bool imu_write_reg(uint8_t reg, uint8_t val) {
    // Create buffer with register address and value
    char buf[2] = {(char)reg, (char)val};
    // Write to I2C and return success status
    return (imu_i2c->write(LSM6DSL_ADDR, buf, 2) == 0);
}

// Read a single byte from a register
bool imu_read_reg(uint8_t reg, uint8_t &val) {
    // Store register address to read from
    char r = (char)reg;
    // Write register address with repeated start condition
    if (imu_i2c->write(LSM6DSL_ADDR, &r, 1, true) != 0) return false;
    // Read the register value
    if (imu_i2c->read(LSM6DSL_ADDR, &r, 1) != 0) return false;
    // Store result in output parameter
    val = (uint8_t)r;
    return true;
}

// Read 16-bit signed integer from two consecutive registers
bool imu_read_int16(uint8_t reg_low, int16_t &val) {
    uint8_t lo, hi;
    // Read low byte
    if (!imu_read_reg(reg_low, lo)) return false;
    // Read high byte from next register
    if (!imu_read_reg(reg_low + 1, hi)) return false;
    // Combine bytes into 16-bit value (little-endian)
    val = (int16_t)((hi << 8) | lo);
    return true;
}

bool imu_read_acc_data(float32_t* acc) {
    int16_t acc_raw;
    // Read X, Y, Z axis in order (OUTX_L_XL, OUTY_L_XL, OUTZ_L_XL are consecutive every 2 bytes)
    for (int i = 0; i < 3; i++) {
        if (!imu_read_int16(OUTX_L_XL + i*2, acc_raw)) return false;
        acc[i] = (float32_t)acc_raw * ACC_SENSITIVITY;
    }
    return true;
}

bool imu_read_gyro_data(float32_t* gyro) {
    int16_t gyro_raw;
    // Read X, Y, Z axis in order (OUTX_L_G, OUTY_L_G, OUTZ_L_G are consecutive every 2 bytes)
    for (int i = 0; i < 3; i++) {
        if (!imu_read_int16(OUTX_L_G + i*2, gyro_raw)) return false;
        gyro[i] = (float32_t)gyro_raw * GYRO_SENSITIVITY / 360.0f * 2.0f * M_PI;
    }
    return true;
}


bool imu_init() {
    imu_i2c = new I2C(PB_11, PB_10);
    imu_i2c->frequency(400000);
    imu_int1_pin = new InterruptIn(LSM6DSL_INT1_PIN, PullDown);
    imu_int1_pin->rise([] { imu_data_ready_flag->set(1); });
    imu_data_ready_flag = new EventFlags();
    uint8_t who;
    if (!imu_read_reg(WHO_AM_I, who) || who != 0x6A) return false;
    imu_write_reg(CTRL3_C, 0x44); 
    imu_write_reg(CTRL1_XL, 0x40); 
    imu_write_reg(CTRL2_G, 0x40);
    imu_write_reg(INT1_CTRL, 0x01); 
    imu_write_reg(DRDY_PULSE_CFG, 0x80);
    return true;
}

bool imu_data_ready() {
    return (imu_data_ready_flag->get() & 1) == 1;
}

bool imu_data_wait(time_t timeout) {
    return imu_data_ready_flag->wait_all(1, timeout) == 1;
}

void imu_data_ready_clear() {
    imu_data_ready_flag->clear(1);
}
