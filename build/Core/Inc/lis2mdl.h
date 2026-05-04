/*
 * lis2mdl.h
 *
 *  Created on: May 10, 2025
 *      Author: Dnyaneshwar
 */

#ifndef INC_LIS2MDL_H_
#define INC_LIS2MDL_H_

#include "stm32wlxx_hal.h" // or your target STM32 series
#include "stdbool.h"

#define LIS2MDL_ADDR         (0x1E << 1)  // Shifted left for HAL I2C
#define REG_WHO_AM_I         0x4F
#define REG_CFG_A            0x60
#define REG_CFG_B            0x61
#define REG_CFG_C            0x62
#define REG_OUTX_L           0x68
#define REG_OUTX_H           0x69
#define REG_OUTY_L           0x6A
#define REG_OUTY_H           0x6B
#define REG_OUTZ_L           0x6C
#define REG_OUTZ_H           0x6D
#define REG_OFFSET_X_L       0x45
#define REG_OFFSET_X_H       0x46
#define REG_OFFSET_Y_L       0x47
#define REG_OFFSET_Y_H       0x48
#define REG_OFFSET_Z_L       0x49
#define REG_OFFSET_Z_H       0x4A

void LIS2MDL_Init(I2C_HandleTypeDef *hi2c);
void LIS2MDL_WriteReg(uint8_t reg, uint8_t val);
uint8_t LIS2MDL_ReadReg(uint8_t reg);
int16_t LIS2MDL_ReadAxis(uint8_t lowReg);
uint8_t LIS2MDL_Calibrate();
void LIS2MDL_PowerDown(void);
void LIS2MDL_WakeUp_SingleConversion(bool temp_enable);
void lis2mdl_take_single_sample(float *xf, float *yf, float *zf);
void LIS2MDL_MinimalInit(void);
void LIS2MDL_DeInit(I2C_HandleTypeDef *hi2c);
void LIS2MDL_ReInit(I2C_HandleTypeDef *hi2c);
#endif /* INC_LIS2MDL_H_ */
