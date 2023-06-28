#ifndef DELAY_BSP_H
#define DELAY_BSP_H

#include "main.h"

void delay_us(uint32_t us);
void delay_us_os(uint32_t nus);
void delay_us_iic(uint32_t us);

#endif
