/*
 * lis2mdl.c
 *
 * Library for the LIS2MDL magnetometer/compass, adapted for STM32L476RG
 *
 * Originally written by Bryan Siepert for Adafruit Industries.
 * Adapted for STM32 by Grok.
 * BSD license, all text above must be included in any redistribution
 */

#include "LIS2MDLSensor.h"
#include <stdio.h>

// Helper function to write a register
static HAL_StatusTypeDef lis2mdl_write_reg(lis2mdl_t *sensor, uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    return HAL_I2C_Master_Transmit(sensor->hi2c, sensor->i2c_addr << 1, buffer, 2, HAL_MAX_DELAY);
}

// Helper function to read a register
static HAL_StatusTypeDef lis2mdl_read_reg(lis2mdl_t *sensor, uint8_t reg, uint8_t *data, uint8_t len) {
    // Write register address
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(sensor->hi2c, sensor->i2c_addr << 1, &reg, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return status;
    }
    // Read data
    return HAL_I2C_Master_Receive(sensor->hi2c, sensor->i2c_addr << 1, data, len, HAL_MAX_DELAY);
}

// Read raw data from the sensor
void lis2mdl_read(lis2mdl_t *sensor) {
    uint8_t buffer[6];
    HAL_StatusTypeDef status = lis2mdl_read_reg(sensor, LIS2MDL_OUTX_L_REG, buffer, 6);
    if (status != HAL_OK) {
        printf("I2C read failed: %d\n", status);
        return;
    }
    sensor->raw.x = (int16_t)(buffer[0] | (buffer[1] << 8));
    sensor->raw.y = (int16_t)(buffer[2] | (buffer[3] << 8));
    sensor->raw.z = (int16_t)(buffer[4] | (buffer[5] << 8));
}

// Initialize the sensor
bool lis2mdl_begin(lis2mdl_t *sensor, I2C_HandleTypeDef *hi2c, uint8_t i2c_addr) {
    sensor->hi2c = hi2c;
    sensor->i2c_addr = i2c_addr;
    sensor->sensor_id = 1234;
    sensor->raw.x = 0;
    sensor->raw.y = 0;
    sensor->raw.z = 0;

    uint8_t chip_id;
    HAL_StatusTypeDef status = lis2mdl_read_reg(sensor, LIS2MDL_WHO_AM_I, &chip_id, 1);
    if (status != HAL_OK) {
        printf("WHO_AM_I read failed: %d\n", status);
        return false;
    }
    printf("WHO_AM_I: 0x%02X\n", chip_id);
    if (chip_id != LIS2MDL_CHIP_ID) {
        printf("Invalid chip ID: 0x%02X (expected 0x40)\n", chip_id);
        return false;
    }

    lis2mdl_reset(sensor);
    return true;
}

// Reset the sensor
void lis2mdl_reset(lis2mdl_t *sensor) {
    // Reset and reboot bits in CFG_REG_A
    lis2mdl_write_reg(sensor, LIS2MDL_CFG_REG_A, (1 << 5)); // Reset
    HAL_Delay(100);
    lis2mdl_write_reg(sensor, LIS2MDL_CFG_REG_A, (1 << 6)); // Reboot
    HAL_Delay(100);

    // Enable BDU and temp compensation, set continuous mode
    lis2mdl_write_reg(sensor, LIS2MDL_CFG_REG_C, (1 << 4)); // BDU
    lis2mdl_write_reg(sensor, LIS2MDL_CFG_REG_A, (1 << 7)); // Temp compensation
    lis2mdl_write_reg(sensor, LIS2MDL_CFG_REG_A, 0x00);     // Continuous mode

    lis2mdl_set_data_rate(sensor, LIS2MDL_RATE_100_HZ);
}

// Set data rate
void lis2mdl_set_data_rate(lis2mdl_t *sensor, lis2mdl_rate_t rate) {
    uint8_t reg_val;
    lis2mdl_read_reg(sensor, LIS2MDL_CFG_REG_A, &reg_val, 1);
    reg_val &= ~(0x03 << 2); // Clear data rate bits
    reg_val |= (rate << 2);  // Set new data rate
    lis2mdl_write_reg(sensor, LIS2MDL_CFG_REG_A, reg_val);
}

// Get data rate
lis2mdl_rate_t lis2mdl_get_data_rate(lis2mdl_t *sensor) {
    uint8_t reg_val;
    lis2mdl_read_reg(sensor, LIS2MDL_CFG_REG_A, &reg_val, 1);
    return (lis2mdl_rate_t)((reg_val >> 2) & 0x03);
}

// Get sensor event
bool lis2mdl_get_event(lis2mdl_t *sensor, lis2mdl_event_t *event) {
    lis2mdl_read(sensor);

    event->sensor_id = sensor->sensor_id;
    event->timestamp = HAL_GetTick(); // Use HAL tick counter for timestamp
    event->magnetic.x = (float)sensor->raw.x * LIS2MDL_MAG_LSB * LIS2MDL_MILLIGAUSS_TO_MICROTESLA;
    event->magnetic.y = (float)sensor->raw.y * LIS2MDL_MAG_LSB * LIS2MDL_MILLIGAUSS_TO_MICROTESLA;
    event->magnetic.z = (float)sensor->raw.z * LIS2MDL_MAG_LSB * LIS2MDL_MILLIGAUSS_TO_MICROTESLA;

    return true;
}

// Enable/disable interrupts
void lis2mdl_enable_interrupts(lis2mdl_t *sensor, bool enable) {
    uint8_t int_ctrl_val = enable ? 0x01 : 0x00;
    lis2mdl_write_reg(sensor, LIS2MDL_INT_CRTL_REG, int_ctrl_val);
    lis2mdl_write_reg(sensor, LIS2MDL_CFG_REG_C, enable ? (1 << 6) : 0); // INT pin output
}

// Set interrupt polarity
void lis2mdl_interrupts_active_high(lis2mdl_t *sensor, bool active_high) {
    uint8_t reg_val;
    lis2mdl_read_reg(sensor, LIS2MDL_INT_CRTL_REG, &reg_val, 1);
    reg_val = active_high ? (reg_val | (1 << 2)) : (reg_val & ~(1 << 2));
    lis2mdl_write_reg(sensor, LIS2MDL_INT_CRTL_REG, reg_val);
}
