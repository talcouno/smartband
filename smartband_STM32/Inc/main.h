/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "debug.h"
#include "stdio.h"
#include "string.h"
#include "MAX30102.h"
#include "MPU9250.h"
#include "tm_stm32_ahrs_imu.h"
#include "uart_test.h"
#include "uart_queue.h"

typedef enum _I2C_Result_t {
    I2C_Result_Ok = 0x00,
    I2C_Result_Error,
} I2C_Result_t;

I2C_Result_t I2C_Write(I2C_HandleTypeDef* hi2c, uint8_t device_address, uint8_t register_address, uint8_t data);
I2C_Result_t I2C_ReadMulti(I2C_HandleTypeDef* hi2c, uint8_t device_address, uint8_t register_address, uint8_t* data, uint16_t count);

struct I2C_Module
{
  I2C_HandleTypeDef   instance;
  uint16_t            sdaPin;
  GPIO_TypeDef*       sdaPort;
  uint16_t            sclPin;
  GPIO_TypeDef*       sclPort;
};


#define I2C1_SCL_PIN GPIO_PIN_6
#define I2C1_SCL_PORT GPIOB
#define I2C1_SDA_PIN GPIO_PIN_7
#define I2C1_SDA_PORT GPIOB
#define I2C2_SCL_PIN GPIO_PIN_1
#define I2C2_SCL_PORT GPIOF
#define I2C2_SDA_PIN GPIO_PIN_0
#define I2C2_SDA_PORT GPIOF

typedef enum { false, true } bool;
void I2C2_ClearBusyFlagErratum(I2C_HandleTypeDef *instance, I2C_HandleTypeDef* instance2);
#define MY_PRINT(fmt, ...)\
	printf(fmt, ##__VA_ARGS__)

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
