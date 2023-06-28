#ifndef __USART_BSP_H_
#define __USART_BSP_H_

#include "main.h"
#include "stdio.h"
#include "usart.h"

#define printf(...)  HAL_UART_Transmit(&huart3,usart_tx_buf,sprintf((char *)usart_tx_buf,__VA_ARGS__),1000)

void print(UART_HandleTypeDef*huart,const char *fmt,...);
void Laser_decoding(void);
void yaw_decoding(void);

extern double info_usart6;
extern int info_usart3;
extern int info_usart7;
extern int info_usart8;

extern uint8_t usart_tx_buf[200];



#endif
