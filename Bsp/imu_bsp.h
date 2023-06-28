#ifndef __IMU_BSP_H_
#define __IMU_BSP_H_

#include "main.h"

#define MPU_DELAY(x) HAL_Delay(x)

typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t mx;
    int16_t my;
    int16_t mz;

    int16_t temp;

    int16_t gx;
    int16_t gy;
    int16_t gz;

    int16_t ax_offset;
    int16_t ay_offset;
    int16_t az_offset;

    int16_t gx_offset;
    int16_t gy_offset;
    int16_t gz_offset;
} mpu_data_t;

typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t mx;
    int16_t my;
    int16_t mz;

    float temp;

    float wx; /*!< omiga, +- 2000dps => +-32768  so gx/16.384/57.3 =	rad/s */
    float wy;
    float wz;

    float vx;
    float vy;
    float vz;

    float rol;
    float pit;
    float yaw;
} imu_t;

extern mpu_data_t mpu_data;
extern imu_t      imu;

extern float yaw_start;
extern int16_t Power_on_record;

uint8_t   mpu_device_init(void);
void init_quaternion(void);
void mpu_get_data(void);
void imu_ahrs_update(void);
void imu_attitude_update(void);
void mpu_offset_call(void);
void imu_info(void);
void imu_init(void);
void thermostatic_control(float target_temp,float measure_temp);

#endif
