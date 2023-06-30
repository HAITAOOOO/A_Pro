#ifndef __USART_BSP_H_
#define __USART_BSP_H_

#include "main.h"
#include "stdio.h"
#include "usart.h"
#include "pid_bsp.h"

#define printf(...) HAL_UART_Transmit_IT(&huart6, usart_tx_buf, sprintf((char *)usart_tx_buf, __VA_ARGS__))


extern double info_usart6;
extern int info_usart3;
extern int info_usart7;
extern int info_usart8;

extern uint8_t usart_tx_buf[200];

void print(const char *fmt, ...); // usart3
void Laser_decoding(void);
void yaw_decoding(void);
void jy61p_read(void);//usart3
float Get_Data(void);//uart6
void adjust_pid_usart(caspid_TypeDef * caspid);

#endif
