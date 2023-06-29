#include "task_user.h"
#include "chassis.h"
#include "as5600.h"
#include "rc_bsp.h"
#include "pwm_bsp.h"
#include "can_bsp.h"
#include "usart_bsp.h"
#include "imu_bsp.h"
#include "as5600.h"
#include "buzzer_bsp.h"
#include "led_bsp.h"
#include "jy61p.h"
#include "iic_bsp.h"

#define START_TASK_PRIO 1
#define START_STK_SIZE 256
static TaskHandle_t StartTask_Handler;

#define IMU_TASK_PRIO 1
#define IMU_STK_SIZE 256
static TaskHandle_t imuTask_Handler;

// #define OMNI_TASK_PRIO 13
// #define OMNI_STK_SIZE 512
// static TaskHandle_t OMNITask_Handler;

#define AS5600_INIT_PRIO 15
#define AS5600_INIT_SIZE 256
static TaskHandle_t AS5600_INITTASK_Handler;

#define TEST_TASK_PRIO 13
#define TEST_STK_SIZE 512
static TaskHandle_t TestTask_Handler;

void OMNI_TASK(void *param)
{
    for (;;)
    {
        omni_wheel(-7 * rc_ctrl.ch1, -7 * rc_ctrl.ch2, -7 * rc_ctrl.ch5, can1_14_speed);
        osDelay(1);
    }
}
/**
 * @brief          获取板载陀螺仪信息和恒温控制
 * @param[in]      none
 * @retval         none
 */
void IMU_TASK(void *param)
{
    for (;;)
    {
        imu_info();
        thermostatic_control(58, imu.temp); // 恒温58度
        osDelay(1);
    }
}
/**
 * @brief          AS5600 IIC通信初始化
 * @param[in]      none
 * @retval         none
 */
void AS5600_INIT_TASK(void *param)
{
    for (;;)
    {
        as5600_init();
        vTaskDelete(AS5600_INITTASK_Handler);
        osDelay(1);
    }
}

/**
 * @brief         	用于代码测试
 * @param[in]      none
 * @retval         none
 */
float Axle_radius = 288.175f; // mm 286.175f
float yaw_err = 0, turnover_err = 0;
jy61p jy61p_offset, jy61p_read;
void TEST_TASK(void *param)
{
    // int16_t launch_flag=0;
    //		int16_t fetch_flag= 0; //转动标志位
    //    int16_t run_flag=0;
    // long double rpm_now = 0;
    // int16_t direction = forward;
    iic1_init();
    read_offset(&jy61p_offset);
    for (;;)
    {
        // 放测试代码
        read_angle(&jy61p_read, &jy61p_offset);
        // if (bt.key_0 == 1)
        // {
        //     direction = left;
        //     rpm_now = Trape(2500.0, 0.5, 0.5, 1.3);
        // }
        // else if (bt.key_1 == 1)
        // {
        //     direction = right;
        //     rpm_now = Trape(2500.0, 0.5, 0.5, 1.3);
        // }
        // else if (bt.key_0 == 0 && bt.key_1 == 0)
        // {
        //     begin_flag = 0;
        //     direction = forward;
        //     rpm_now = 0;
        // }
        // if (direction == forward)
        // {
        //     CAN_Send_PID(-rpm_now, rpm_now, rpm_now, -rpm_now, can1_14_speed);
        // }
        // else if (direction == back)
        // {
        //     CAN_Send_PID(rpm_now, -rpm_now, -rpm_now, rpm_now, can1_14_speed);
        // }
        // else if (direction == left)
        // {
        //     CAN_Send_PID(-rpm_now, -rpm_now, rpm_now, rpm_now, can1_14_speed);
        // }
        // else if (direction == right)
        // {
        //     CAN_Send_PID(rpm_now, rpm_now, -rpm_now, -rpm_now, can1_14_speed);
        // }
        // printf("%d\n", 1);
        // printf("%d\n", 2);

        //				if(rc_ctrl.sw1==MIDDLE&&rc_ctrl.sw2==MIDDLE&&run_flag==1)
        //				{
        //					run_flag=0;
        //				}

        /*手动移动 摩擦轮停止*/
        //        if(rc_ctrl.sw1==MIDDLE&&rc_ctrl.sw2==MIDDLE)
        //        {
        ////            for(int i = 0; i<4; i++)
        ////            {
        ////                pid_reset(&motor_pid1_speed[i],3,0.05,0.27);
        ////            }
        //            PID_init(can1_14_speed);

        //            omni_wheel(-7*rc_ctrl.ch1,-7*rc_ctrl.ch2,-7*rc_ctrl.ch5,can1_14_speed);
        //            //CAN_Send_PID(0,0,0,0,can2_58_speed);

        //            omni_wheel_info[0].stop_ecd=can1_motor_message[0].real_ecd;
        //            omni_wheel_info[1].stop_ecd=can1_motor_message[1].real_ecd;
        //            omni_wheel_info[2].stop_ecd=can1_motor_message[2].real_ecd;
        //            omni_wheel_info[3].stop_ecd=can1_motor_message[3].real_ecd;
        //
        //        }

        ////				if(bt.key_0==1)
        ////				{
        ////					for(int i = 0; i<4; i++)
        ////					{
        ////							pid_reset(&motor_pid1_speed[i],3,0.05,0.27);
        ////					}
        ////					while(bt.key_0==1)
        ////					{
        ////						omni_wheel(-7*bt.romte_x,-7*bt.romte_y,0,can1_14_speed);
        ////					}
        ////				}

        //        /*投环*/
        //        if(rc_ctrl.sw2==MIDDLE&&rc_ctrl.sw1==DOWN)
        //        {
        //            PID_init(can1_14_speed);
        //            omni_wheel(-4*rc_ctrl.ch1,-4*rc_ctrl.ch2,-4*rc_ctrl.ch5,can1_14_speed);
        //            if(rc_ctrl.ch4>200&&launch_flag == 0)
        //            {
        //                HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);
        //                HAL_Delay(200);
        //                HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
        //                launch_flag = 1;
        //            }
        //            if(rc_ctrl.ch4<10&&launch_flag == 1)
        //            {
        //                launch_flag = 0;
        //            }

        //            CAN_Send_PID(-2300,-2300,2300,2300,can2_58_speed);

        //        }

        ////        /*取环*/
        //////			else if(rc_ctrl.sw2==MIDDLE&&rc_ctrl.sw1==UP)
        //////			{
        //////				/*
        //////						//位置环周转
        //////						PID_init(can1_14_pos);
        //////						for(int i = 0;i<4;i++)
        //////						{
        //////							pid_reset(&motor_pid1_speed[i],3,0.05,0.27);
        //////							pid_reset(&motor_pid1_pos[i],2,0.0,0);
        //////						}
        //////
        //////						CAN_Send_PID(3.1415926f*Axle_radius*360.0f/314.15926f,3.1415926f*Axle_radius*360.0f/314.15926f,3.1415926f*Axle_radius*360.0f/314.15926f,3.1415926f*Axle_radius*360.0f/314.15926f,can1_14_pos);
        //////
        ////////						PID_init(can1_14_pos);

        ////////            CAN_Send_PID((turnover_err+180)*360.0f/314.15926f,(turnover_err+180)*360.0f/314.15926f,(turnover_err+180)*360.0f/314.15926f,(turnover_err+180)*360.0f/314.15926f,can1_14_pos);
        //////				*/
        //////			}

        /*姿态矫正*/
        // if (Power_on_record == 0) // 温漂矫正完成
        // {
        //     LED_ON();
        //     yaw_err = (yaw_start - imu.yaw);
        //     if (yaw_err < 0)
        //     {
        //         yaw_err += 360;
        //     }
        //     // turnover_err = yaw_err*3.1415926f*Axle_radius/180.0f;//理论值
        //     turnover_err = yaw_err * Axle_radius;
        // }
        // if (rc_ctrl.sw2 == MIDDLE && rc_ctrl.sw1 == DOWN)
        // {
        //     PID_init(can1_14_pos);
        //     CAN_Send_PID(turnover_err * 360.0f / 314.15926f, turnover_err * 360.0f / 314.15926f, turnover_err * 360.0f / 314.15926f, turnover_err * 360.0f / 314.15926f, can1_14_pos);
        // }
        ////        /*距离矫正*/

        //        if(rc_ctrl.sw2==MIDDLE&&rc_ctrl.sw1==UP)
        //        {
        //					PID_init(can1_14_speed);
        //					laser_positioning(1000,10);
        //
        ////             for(int i = 0; i<4; i++)
        ////            {
        ////                pid_reset(&motor_pid1_speed[i],7,0,0);
        ////								pid_reset(&motor_pid1_pos[i],1,0,0);
        ////            }
        ////            while(rc_ctrl.sw2==MIDDLE&&rc_ctrl.sw1==UP)
        ////            {
        ////                omni_wheel(200*360/314.15926,0*360/314.15926,0,can1_14_pos);
        ////            }

        //        }
        //
        osDelay(1);
    }
}

/**
 * @brief         	开始任务：用于创建其他任务
 * @param[in]      none
 * @retval         none
 */
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL(); // 进入临界保护区
    xTaskCreate((TaskFunction_t)AS5600_INIT_TASK, (const char *)"AS5600_INIT_TASK", (uint16_t)AS5600_INIT_SIZE, (void *)NULL, (UBaseType_t)AS5600_INIT_PRIO, (TaskHandle_t *)&AS5600_INITTASK_Handler);

    // xTaskCreate((TaskFunction_t)OMNI_TASK,(const char *)"OMNI_TASK",(uint16_t)OMNI_STK_SIZE,(void *)NULL,(UBaseType_t)OMNI_TASK_PRIO,(TaskHandle_t *)&OMNITask_Handler);

    xTaskCreate((TaskFunction_t)TEST_TASK, (const char *)"TEST_TASK", (uint16_t)TEST_STK_SIZE, (void *)NULL, (UBaseType_t)TEST_TASK_PRIO, (TaskHandle_t *)&TestTask_Handler);

    xTaskCreate((TaskFunction_t)IMU_TASK, (const char *)"IMU_TASK", (uint16_t)IMU_STK_SIZE, (void *)NULL, (UBaseType_t)IMU_TASK_PRIO, (TaskHandle_t *)&imuTask_Handler);

    vTaskDelete(StartTask_Handler); // 删除开始任务
    taskEXIT_CRITICAL();            // 退出临界区
}

/**
 * @brief         	创建开始任务 "start_task"
 * @param[in]      none
 * @retval         none
 */
void startTast(void)
{
    xTaskCreate((TaskFunction_t)start_task,          // 任务函数
                (const char *)"start_task",          // 任务名称
                (uint16_t)START_STK_SIZE,            // 任务堆栈大小
                (void *)NULL,                        // 传递给任务函数的参数
                (UBaseType_t)START_TASK_PRIO,        // 任务优先级
                (TaskHandle_t *)&StartTask_Handler); // 任务句柄
}
