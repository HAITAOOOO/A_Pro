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
float yaw_err = 1, turnover_err = 0;
void TEST_TASK(void *param)
{

    for (;;)
    {
        // 放测试代码
        //printf("%f\n",yaw_err);
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
