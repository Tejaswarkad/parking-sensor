/*
 * lis2mdl.h
 *
 * Library for the LIS2MDL magnetometer/compass, adapted for STM32L476RG
 *
 * Uses I2C to communicate, 2 pins are required to interface.
 *
 * Originally written by Bryan Siepert for Adafruit Industries.
 * Adapted for STM32 by Grok.
 * BSD license, all text above must be included in any redistribution
 */

#ifndef __LIS2MDL_SENSOR_H__
#define __LIS2MDL_SENSOR_H__

#include <stdint.h>
#include <stdbool.h>
#include "stm32wlxx_hal.h"

// I2C Address and Chip ID
#define LIS2MDL_ADDRESS_MAG        0x1E   // Default I2C address (7-bit, shift left for HAL: 0x3C)
#define LIS2MDL_CHIP_ID            0x40   // Chip ID from WHO_AM_I register
#define LIS2MDL_MAG_LSB            1.5f   // Sensitivity (mgauss per LSB)
#define LIS2MDL_MILLIGAUSS_TO_MICROTESLA 0.1f // Conversion rate from Milligauss to Microtesla

// LIS2MDL Register Addresses
typedef enum {
    LIS2MDL_OFFSET_X_REG_L  = 0x45,
    LIS2MDL_OFFSET_X_REG_H  = 0x46,
    LIS2MDL_OFFSET_Y_REG_L  = 0x47,
    LIS2MDL_OFFSET_Y_REG_H  = 0x48,
    LIS2MDL_OFFSET_Z_REG_L  = 0x49,
    LIS2MDL_OFFSET_Z_REG_H  = 0x4A,
    LIS2MDL_WHO_AM_I        = 0x4F,
    LIS2MDL_CFG_REG_A       = 0x60,
    LIS2MDL_CFG_REG_B       = 0x61,
    LIS2MDL_CFG_REG_C       = 0x62,
    LIS2MDL_INT_CRTL_REG    = 0x63,
    LIS2MDL_INT_SOURCE_REG  = 0x64,
    LIS2MDL_INT_THS_L_REG   = 0x65,
    LIS2MDL_STATUS_REG      = 0x67,
    LIS2MDL_OUTX_L_REG      = 0x68,
    LIS2MDL_OUTX_H_REG      = 0x69,
    LIS2MDL_OUTY_L_REG      = 0x6A,
    LIS2MDL_OUTY_H_REG      = 0x6B,
    LIS2MDL_OUTZ_L_REG      = 0x6C,
    LIS2MDL_OUTZ_H_REG      = 0x6D,
} lis2mdl_register_t;

// Magnetometer Update Rate Settings
typedef enum {
    LIS2MDL_RATE_10_HZ  = 0x00,  // 10 Hz
    LIS2MDL_RATE_20_HZ  = 0x01,  // 20 Hz
    LIS2MDL_RATE_50_HZ  = 0x02,  // 50 Hz
    LIS2MDL_RATE_100_HZ = 0x03,  // 100 Hz
} lis2mdl_rate_t;

// Raw Magnetometer Data Structure
typedef struct {
    int16_t x;  // x-axis raw data
    int16_t y;  // y-axis raw data
    int16_t z;  // z-axis raw data
} lis2mdl_data_t;

// Sensor Event Structure
typedef struct {
    int32_t sensor_id;
    uint32_t timestamp;
    struct {
        float x;
        float y;
        float z;
    } magnetic;
} lis2mdl_event_t;

// LIS2MDL Instance Structure
typedef struct {
    I2C_HandleTypeDef *hi2c;     // Pointer to I2C handle
    uint8_t i2c_addr;            // I2C address of the sensor (7-bit)
    int32_t sensor_id;           // Unique sensor ID
    lis2mdl_data_t raw;          // Raw magnetometer data
} lis2mdl_t;

// Function Prototypes
bool lis2mdl_begin(lis2mdl_t *sensor, I2C_HandleTypeDef *hi2c, uint8_t i2c_addr);
void lis2mdl_read(lis2mdl_t *sensor);
bool lis2mdl_get_event(lis2mdl_t *sensor, lis2mdl_event_t *event);
void lis2mdl_set_data_rate(lis2mdl_t *sensor, lis2mdl_rate_t rate);
lis2mdl_rate_t lis2mdl_get_data_rate(lis2mdl_t *sensor);
void lis2mdl_reset(lis2mdl_t *sensor);
void lis2mdl_enable_interrupts(lis2mdl_t *sensor, bool enable);
void lis2mdl_interrupts_active_high(lis2mdl_t *sensor, bool active_high);

#endif /* __LIS2MDL_SENSOR_H__ */
