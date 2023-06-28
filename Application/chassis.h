#ifndef __CHASSIS_H_
#define __CHASSIS_H_

#include "main.h"
#include "pid_bsp.h"

/*
	将pid初始化与pid计算拆开的好处在于减少了pid重复初始化的时间，
	坏处是每一个pid结构数据不能被覆盖，即有一个电机必须初始化一个结构体（串级两个），增大了内存占用
*/
#define PI 3.14159265358979f

#define can1_14_speed 111
#define can1_14_pos 	112
#define can1_58_speed 121
#define can1_58_pos 	122
#define can2_14_speed 211
#define can2_14_pos 	212
#define can2_58_speed 221
#define can2_58_pos 	222
#define can1_6020_speed 60201
#define can1_6020_pos   60202

typedef struct
{
    double stop_ecd;				//电机停止角度
    double	real_ecd;				//底盘累计转动角度（位置环和速度环累计转动角度）
} chassis_info_t;

extern chassis_info_t  omni_wheel_info[4];
extern chassis_info_t  steering_wheel_info[8];

extern PID_TypeDef motor_pid1_speed[8],motor_pid1_pos[8];
extern PID_TypeDef motor_pid2_speed[8],motor_pid2_pos[8];
extern PID_TypeDef motor_pid_6020_speed[7],motor_pid_6020_pos[7];

extern int16_t begin_flag;

extern uint32_t start_time;
void chassis_init(void);
void PID_init(uint16_t type);
void CAN_Send_PID(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4,uint16_t type);
void omni_wheel(double x, double y, double w,uint16_t type);
void omni_wheel_t(int16_t x_earth, int16_t y_earth, int16_t w,double yaw);
void steering_wheel(void);
void laser_positioning(int16_t chassis_distance,double chassis_err_err);
long double Trape(long double rpm_max,long double a_up,long double a_down,long double s);

#endif
