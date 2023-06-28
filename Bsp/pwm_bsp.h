#ifndef __PWM_BSP_H_
#define __PWM_BSP_H_
#include "stm32f4xx_hal.h"

#define A			0
#define B			1
#define C			2
#define D			3
#define E			4
#define F			5
#define G			6
#define H			7
#define S			8
#define T			9
#define U			10
#define V			11
#define W			12
#define X			13
#define Y			14
#define Z			15

void PWM_init(TIM_HandleTypeDef *htim);
void PWM_Servo(uint8_t channel,float period,float duty);

#endif
