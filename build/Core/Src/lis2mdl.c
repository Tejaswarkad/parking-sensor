/*
 * lis2mdl.c
 *
 *  Created on: May 10, 2025
 *      Author: Dnyaneshwar
 */

#include "lis2mdl.h"
#include "i2c.h"
static I2C_HandleTypeDef *lis2mdl_i2c;

void LIS2MDL_Init(I2C_HandleTypeDef *hi2c) {
    lis2mdl_i2c = hi2c;

    HAL_Delay(100);

    LIS2MDL_WriteReg(REG_CFG_A, 0b10001100);  // TEMP_EN = 1, ODR = 100Hz, High-res, Continuous
    LIS2MDL_WriteReg(REG_CFG_B, 0b00000101);  // Offset cancel = 1, LPF = 1
    LIS2MDL_WriteReg(REG_CFG_C, 0b00010000);  // BDU = 1

    HAL_Delay(200);
    LIS2MDL_Calibrate();
}

void LIS2MDL_WriteReg(uint8_t reg, uint8_t val) {
    uint8_t data[2] = {reg, val};
    HAL_I2C_Master_Transmit(lis2mdl_i2c, LIS2MDL_ADDR, data, 2, HAL_MAX_DELAY);
}

uint8_t LIS2MDL_ReadReg(uint8_t reg) {
    uint8_t val;
    HAL_I2C_Master_Transmit(lis2mdl_i2c, LIS2MDL_ADDR, &reg, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(lis2mdl_i2c, LIS2MDL_ADDR, &val, 1, HAL_MAX_DELAY);
    return val;
}

int16_t LIS2MDL_ReadAxis(uint8_t lowReg) {
    uint8_t data[2];
    HAL_I2C_Master_Transmit(lis2mdl_i2c, LIS2MDL_ADDR, &lowReg, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(lis2mdl_i2c, LIS2MDL_ADDR, data, 2, HAL_MAX_DELAY);
    return (int16_t)(data[1] << 8 | data[0]);
}

uint8_t LIS2MDL_Calibrate() {
    int16_t x = LIS2MDL_ReadAxis(REG_OUTX_L);
    int16_t y = LIS2MDL_ReadAxis(REG_OUTY_L);
    int16_t z = LIS2MDL_ReadAxis(REG_OUTZ_L);

    LIS2MDL_WriteReg(REG_OFFSET_X_L, x & 0xFF);
    LIS2MDL_WriteReg(REG_OFFSET_X_H, (x >> 8) & 0xFF);
    LIS2MDL_WriteReg(REG_OFFSET_Y_L, y & 0xFF);
    LIS2MDL_WriteReg(REG_OFFSET_Y_H, (y >> 8) & 0xFF);
    LIS2MDL_WriteReg(REG_OFFSET_Z_L, z & 0xFF);
    LIS2MDL_WriteReg(REG_OFFSET_Z_H, (z >> 8) & 0xFF);

    return 1;
}
void LIS2MDL_PowerDown(void) {
    // Set CFG_REG_A: ODR = 0 (Power-down), MD = 00
    LIS2MDL_WriteReg(REG_CFG_A, 0x03);
}
void LIS2MDL_WakeUp_SingleConversion(bool temp_enable) {
    uint8_t reg_val = 0x01; // ODR = 10Hz (doesn't matter), MD = 01 (single-conversion)

    if (temp_enable) {
        reg_val |= 0x80;  // Set TEMP_EN = 1 (bit 7)
    }

    LIS2MDL_WriteReg(REG_CFG_A, reg_val);
}

void LIS2MDL_DeInit(I2C_HandleTypeDef *hi2c) {
    lis2mdl_i2c = hi2c;

    HAL_Delay(10);
    // CHANGE: Set MD bits to 11 (Idle Mode) to stop the sensor's internal oscillator
    LIS2MDL_WriteReg(REG_CFG_A,  0b00000011);  // MD = 11 (Idle Mode)
        LIS2MDL_WriteReg(REG_CFG_B, 0b00000101);  // Offset cancel = 1, LPF = 1
        LIS2MDL_WriteReg(REG_CFG_C, 0b00010000);  // BDU = 1

    HAL_Delay(1);
}
void LIS2MDL_ReInit(I2C_HandleTypeDef *hi2c) {
    lis2mdl_i2c = hi2c;
    HAL_Delay(1);
    LIS2MDL_WriteReg(REG_CFG_A, 0b10001100);  // TEMP_EN = 1, ODR = 100Hz, High-res, Continuous
    LIS2MDL_WriteReg(REG_CFG_B, 0b00000101);  // Offset cancel = 1, LPF = 1
    LIS2MDL_WriteReg(REG_CFG_C, 0b00010000);  // BDU = 1

    HAL_Delay(100);
}

