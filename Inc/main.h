/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
void _Error_Handler(char *file, int line);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IIC1_SCL_Pin GPIO_PIN_4
#define IIC1_SCL_GPIO_Port GPIOE
#define IIC1_SDA_Pin GPIO_PIN_5
#define IIC1_SDA_GPIO_Port GPIOE
#define IIC2_SDA_Pin GPIO_PIN_6
#define IIC2_SDA_GPIO_Port GPIOE
#define I2_Pin GPIO_PIN_0
#define I2_GPIO_Port GPIOF
#define I2_EXTI_IRQn EXTI0_IRQn
#define I1_Pin GPIO_PIN_1
#define I1_GPIO_Port GPIOF
#define I1_EXTI_IRQn EXTI1_IRQn
#define IIC3_SDA_Pin GPIO_PIN_2
#define IIC3_SDA_GPIO_Port GPIOC
#define IIC4_SDA_Pin GPIO_PIN_3
#define IIC4_SDA_GPIO_Port GPIOC
#define KEY_Pin GPIO_PIN_2
#define KEY_GPIO_Port GPIOB
#define KEY_EXTI_IRQn EXTI2_IRQn
#define IIC4_SCL_Pin GPIO_PIN_1
#define IIC4_SCL_GPIO_Port GPIOB
#define IIC3_SCL_Pin GPIO_PIN_0
#define IIC3_SCL_GPIO_Port GPIOB
#define IIC2_SCL_Pin GPIO_PIN_12
#define IIC2_SCL_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
