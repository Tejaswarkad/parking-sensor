/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wlxx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
#define TX_INTERVAL_ADDRESS 4
#define REGION_ADDRESS 8
#define LORAWAN_CLASS_ADDRESS 12
#define ADR_ADDRESS 20
#define DATA_RATE_ADDRESS 24
#define MSG_TYPE_ADDRESS 28
#define UPLINK_TIMER_ADDRESS 32
#define BATTERY_FLASH_ADDR 200
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RTC_N_PREDIV_S 10
#define RTC_PREDIV_S ((1<<RTC_N_PREDIV_S)-1)
#define RTC_PREDIV_A ((1<<(15-RTC_N_PREDIV_S))-1)
#define TILT_SENSOR_Pin GPIO_PIN_12
#define TILT_SENSOR_GPIO_Port GPIOA
#define TILT_SENSOR_EXTI_IRQn EXTI15_10_IRQn
#define V_REF_OUT_Pin GPIO_PIN_15
#define V_REF_OUT_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_15
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_9
#define LED2_GPIO_Port GPIOB
#define RF_CTR_Pin GPIO_PIN_13
#define RF_CTR_GPIO_Port GPIOC
#define SENSOR_PWR_Pin GPIO_PIN_8
#define SENSOR_PWR_GPIO_Port GPIOB
#define PROB2_Pin GPIO_PIN_13
#define PROB2_GPIO_Port GPIOB
#define VBAT_ENB_Pin GPIO_PIN_9
#define VBAT_ENB_GPIO_Port GPIOA
#define BUT3_Pin GPIO_PIN_6
#define BUT3_GPIO_Port GPIOC
#define BUT2_Pin GPIO_PIN_1
#define BUT2_GPIO_Port GPIOA
#define BUT2_EXTI_IRQn EXTI1_IRQn
#define LED3_Pin GPIO_PIN_11
#define LED3_GPIO_Port GPIOB
#define USARTx_RX_Pin GPIO_PIN_3
#define USARTx_RX_GPIO_Port GPIOA
#define USARTx_TX_Pin GPIO_PIN_2
#define USARTx_TX_GPIO_Port GPIOA
#define RF_PWR_Pin GPIO_PIN_5
#define RF_PWR_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define LED_GREEN_Pin GPIO_PIN_0
#define LED_GREEN_GPIO_Port GPIOC
#define LED_RED_Pin GPIO_PIN_1
#define LED_RED_GPIO_Port GPIOC
//
#define OUTPUT_ENB_PIN GPIO_PIN_13
#define OUTPUT_ENB_Port GPIOB
//
#define BUT4_EXTI_IRQn EXTI15_10_IRQn

void processQueuedUplinks(void);

//#define Tx_En1_Pin GPIO_PIN_8
//#define Tx_En1_GPIO_Port GPIOB
//#define Tx_En2_Pin GPIO_PIN_3
//#define Tx_En2_GPIO_Port GPIOB
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
