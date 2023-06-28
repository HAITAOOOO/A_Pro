/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart7;

extern UART_HandleTypeDef huart8;

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart3;

extern UART_HandleTypeDef huart6;

/* USER CODE BEGIN Private defines */
volatile extern uint8_t rx_len_usart3;
volatile extern uint8_t recv_end_flag_usart3;
extern uint8_t rx_buffer_usart3[200];

volatile extern uint8_t rx_len_usart6;
volatile extern uint8_t recv_end_flag_usart6;
extern uint8_t rx_buffer_usart6[200];

volatile extern uint8_t rx_len_usart7;
volatile extern uint8_t recv_end_flag_usart7;
extern uint8_t rx_buffer_usart7[200];

volatile extern uint8_t rx_len_usart8;
volatile extern uint8_t recv_end_flag_usart8;
extern uint8_t rx_buffer_usart8[200];
#define BUFFER_SIZE 200
/* USER CODE END Private defines */

void MX_UART7_Init(void);
void MX_UART8_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_USART6_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

