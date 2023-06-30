#include "chassis.h"
#include "can_bsp.h"
#include "rc_bsp.h"
#include "usart_bsp.h"
#include <math.h>
#include "cmsis_os.h"
#include "led_bsp.h"

caspid_TypeDef motor_pid1[8];
caspid_TypeDef motor_pid2[8];
caspid_TypeDef motor_pid_6020[7];

FuzzyPID motor_fuzzypid1[8];

int32_t set_target1[4] = {1};
int32_t set_target2[4] = {1};

chassis_info_t omni_wheel_info[4];
chassis_info_t steering_wheel_info[8];

/**
 * @brief          底盘结构体参数初始化
 * @param[in]     	none
 * @retval         none
 */
void chassis_init()
{
    for (int i = 0; i < 4; i++)
    {
        omni_wheel_info[i].real_ecd = 0;
        omni_wheel_info[i].stop_ecd = 0;
    }
    for (int i = 0; i < 8; i++)
    {
        steering_wheel_info[i].real_ecd = 0;
        steering_wheel_info[i].stop_ecd = 0;
    }
}

/**
    * @brief          电机pid初始化
    * @param[in]      type：选择can总线和发送的报头
                                                    11为发送can1上报头为0X200的数据帧		12为发送can1上报头为0X1FF的数据帧
                                                    21为发送can2上报头为0X200的数据帧		22为发送can2上报头为0X1FF的数据帧
  * @retval         none
  */
void PID_init(uint16_t type)
{
    switch (type)
    {
    case can1_14_speed:
        for (int i = 0; i < 4; i++)
        {
            pid_init(&motor_pid1[i].speed);
            motor_pid1[i].speed.f_param_init(&motor_pid1[i].speed,0,0, 16383,5000,3.0,0.05,0.27); // 结构体 死区 初始期望 输出限幅 积分限幅 kp ki kd
        }
        break;
    case can1_14_pos:
        for (int i = 0; i < 4; i++)
        {
            pid_init(&motor_pid1[i].speed);
            motor_pid1[i].speed.f_param_init(&motor_pid1[i].speed,0,0, 16000,5000,3,0.05,0.27); // 结构体 死区 初始期望 输出限幅 积分限幅 kp ki kd
            pid_init(&motor_pid1[i].pos);
            motor_pid1[i].pos.f_param_init(&motor_pid1[i].pos,0,0, 16000,1000,4,0.0,25);
        }
        break;
    case can1_58_speed:
        for (int i = 4; i < 8; i++)
        {
            pid_init(&motor_pid1[i].speed);
            motor_pid1[i].speed.f_param_init(&motor_pid1[i].speed, 10, 0, 16383, 5000, 3.0, 0.05, 0.27); // 结构体 死区 初始期望 输出限幅 积分限幅 kp ki kd
        }
        break;
    case can1_58_pos:
        for (int i = 4; i < 8; i++)
        {
            pid_init(&motor_pid1[i].speed);
            motor_pid1[i].speed.f_param_init(&motor_pid1[i].speed,0,0, 16000,5000,3,0.05,0.27); // 结构体 死区 初始期望 输出限幅 积分限幅 kp ki kd
            pid_init(&motor_pid1[i].pos);
            motor_pid1[i].pos.f_param_init(&motor_pid1[i].pos,0,0, 16000,1000,0,0.0,0);
        }
        break;
    case can2_14_speed:
        for (int i = 0; i < 4; i++)
        {
            pid_init(&motor_pid2[i].speed);
            motor_pid2[i].speed.f_param_init(&motor_pid2[i].speed, 10, 0, 16383, 5000, 3.0, 0.05, 0.27); // 结构体 死区 初始期望 输出限幅 积分限幅 kp ki kd
        }
        break;
    case can2_14_pos:
        for (int i = 0; i < 4; i++)
        {
            pid_init(&motor_pid2[i].speed);
            motor_pid2[i].speed.f_param_init(&motor_pid2[i].speed,0,0, 16000,5000,0,0,0); // 结构体 死区 初始期望 输出限幅 积分限幅 kp ki kd
            pid_init(&motor_pid2[i].pos);
            motor_pid2[i].pos.f_param_init(&motor_pid2[i].pos,0,0, 16000,1000,0,0,0);
        }
        break;
    case can2_58_speed:
        for (int i = 4; i < 8; i++)
        {
            pid_init(&motor_pid2[i].speed);
            motor_pid2[i].speed.f_param_init(&motor_pid2[i].speed,0,0, 16000,5000,0,0,0); // 结构体 死区 初始期望 输出限幅 积分限幅 kp ki kd
        }
        break;
    case can2_58_pos:
        for (int i = 4; i < 8; i++)
        {
            pid_init(&motor_pid2[i].speed);
            motor_pid2[i].speed.f_param_init(&motor_pid2[i].speed,0,0, 16000,5000,0,0,0); // 结构体 死区 初始期望 输出限幅 积分限幅 kp ki kd
            pid_init(&motor_pid2[i].pos);
            motor_pid2[i].pos.f_param_init(&motor_pid2[i].pos,0,0, 16000,1000,0,0.0,0);
        }
        break;
    case can1_6020_speed:
        for (int i = 4; i < 7; i++)
        {
            pid_init(&motor_pid_6020[i].speed);
            motor_pid_6020[i].speed.f_param_init(&motor_pid_6020[i].speed, 10, 0, 30000, 30000, 500, 0, 0); // 结构体 死区 初始期望 输出限幅 积分限幅 kp ki kd
        }
        break;
    case can1_6020_pos:
        for (int i = 4; i < 7; i++)
        {
            pid_init(&motor_pid_6020[i].speed);
            motor_pid_6020[i].speed.f_param_init(&motor_pid_6020[i].speed, 10, 0, 30000, 30000, 500, 0, 0); // 结构体 死区 初始期望 输出限幅 积分限幅 kp ki kd
            pid_init(&motor_pid_6020[i].pos);
            motor_pid_6020[i].pos.f_param_init(&motor_pid_6020[i].pos, 0, 0, 360, 10, 20, 0, 0); // 结构体 死区 初始期望 输出限幅 积分限幅 kp ki kd
        }
        break;
    default:
        break;
    }
}
/**
    * @brief          can发送函数pid版
    * @param[in]      motor1,motor2,motor3,motor4：电流、角度期望值
    * @param[in]			type：选择can总线和发送的报头
                                                    11为发送can1上报头为0X200的数据帧		12为发送can1上报头为0X1FF的数据帧
                                                    21为发送can2上报头为0X200的数据帧		22为发送can2上报头为0X1FF的数据帧
  * @retval         none
  */
double test_ecd[4];

void CAN_Send_PID(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4, uint16_t type)
{
    set_target1[0] = motor1;
    set_target1[1] = motor2;
    set_target1[2] = motor3;
    set_target1[3] = motor4;
    set_target2[0] = motor1;
    set_target2[1] = motor2;
    set_target2[2] = motor3;
    set_target2[3] = motor4;

    if (type == can1_14_speed)
    {
        for (int i = 0; i < 4; i++)
        {
            motor_pid1[i].speed.f_cal_pid(&motor_pid1[i].speed, can1_motor_message[i].speed_rpm, set_target1[i]); // 根据期望值进行PID计算(电流环)  speed_rpm(当前速度)由反馈报文提供
        }
        CAN1_Send1(motor_pid1[0].speed.output, motor_pid1[1].speed.output, motor_pid1[2].speed.output, motor_pid1[3].speed.output);
    }
    if (type == can1_14_pos)
    {
        for (int i = 0; i < 4; i++)
        {
            // pid_reset(&motor_pid1_speed[i],3+fuzzy(&motor_pid1_speed[i]),0.0,0.27);
            //test_ecd[i] = can1_motor_message[i].real_ecd - omni_wheel_info[i].stop_ecd;
            motor_pid1[i].speed.f_cal_pid(&motor_pid1[i].speed, can1_motor_message[i].speed_rpm, motor_pid1[i].pos.f_cal_pid(&motor_pid1[i].pos, can1_motor_message[i].real_ecd - omni_wheel_info[i].stop_ecd, set_target1[i]));
        }
        CAN1_Send1(motor_pid1[0].speed.output, motor_pid1[1].speed.output, motor_pid1[2].speed.output, motor_pid1[3].speed.output);
    }

    if (type == can1_58_speed)
    {
        for (int i = 4; i < 8; i++)
        {
            motor_pid1[i].speed.f_cal_pid(&motor_pid1[i].speed, can1_motor_message[i].speed_rpm, set_target1[i - 4]); // 根据期望值进行PID计算(电流环)  speed_rpm(当前速度)由反馈报文提供
        }
        CAN1_Send2(motor_pid1[4].speed.output, motor_pid1[5].speed.output, motor_pid1[6].speed.output, motor_pid1[7].speed.output);
    }
    if (type == can1_58_pos)
    {
        for (int i = 4; i < 8; i++)
        {
            motor_pid1[i].speed.f_cal_pid(&motor_pid1[i].speed, can1_motor_message[i].speed_rpm, motor_pid1[i].pos.f_cal_pid(&motor_pid1[i].pos, can1_motor_message[i].real_ecd, set_target1[i - 4]));
        }
        CAN1_Send2(motor_pid1[4].speed.output, motor_pid1[5].speed.output, motor_pid1[6].speed.output, motor_pid1[7].speed.output);
    }

    if (type == can2_14_speed)
    {
        for (int i = 0; i < 4; i++)
        {
            motor_pid2[i].speed.f_cal_pid(&motor_pid2[i].speed, can2_motor_message[i].speed_rpm, set_target2[i]); // 根据期望值进行PID计算  speed_rpm(当前速度)由反馈报文提供
        }
        CAN2_Send1(motor_pid2[0].speed.output,motor_pid2[1].speed.output,motor_pid2[2].speed.output,motor_pid2[3].speed.output);
    }
    if (type == can2_14_pos)
    {
        for (int i = 0; i < 4; i++)
        {
            motor_pid2[i].speed.f_cal_pid(&motor_pid2[i].speed, can2_motor_message[i].speed_rpm, motor_pid2[i].pos.f_cal_pid(&motor_pid2[i].pos, can2_motor_message[i].real_ecd, set_target2[i]));
        }
        CAN2_Send1(motor_pid2[0].speed.output, motor_pid2[1].speed.output, motor_pid2[2].speed.output, motor_pid2[3].speed.output);
    }

    if (type == can2_58_speed)
    {
        for (int i = 4; i < 8; i++)
        {
            motor_pid2[i].speed.f_cal_pid(&motor_pid2[i].speed, can2_motor_message[i].speed_rpm, set_target2[i-4]); // 根据期望值进行PID计算  speed_rpm(当前速度)由反馈报文提供
        }
        CAN2_Send2(motor_pid2[4].speed.output, motor_pid2[5].speed.output,motor_pid2[6].speed.output,motor_pid2[7].speed.output);
    }
    if (type == can2_58_pos)
    {
        for (int i = 4; i < 8; i++)
        {
            motor_pid2[i].speed.f_cal_pid(&motor_pid2[i].speed, can2_motor_message[i].speed_rpm, motor_pid2[i].pos.f_cal_pid(&motor_pid2[i].pos, can2_motor_message[i].real_ecd, set_target2[i - 4]));
        }
        CAN2_Send2(motor_pid2[4].speed.output,motor_pid2[5].speed.output, motor_pid2[6].speed.output, motor_pid2[7].speed.output);
    }

    if (type == can1_6020_speed)
    {
        for (int i = 4; i < 7; i++)
        {
            motor_pid_6020[i].speed.f_cal_pid(&motor_pid_6020[i].speed, can_motor_message_6020[i].rotor_speed, set_target2[i - 4]); // 根据期望值进行PID计算  speed_rpm(当前速度)由反馈报文提供
        }
        CAN_Send_6020(motor_pid_6020[4].speed.output,motor_pid_6020[5].speed.output, motor_pid_6020[6].speed.output);
    }
    if (type == can1_6020_pos)
    {
        for (int i = 4; i < 7; i++)
        {
            motor_pid_6020[i].speed.f_cal_pid(&motor_pid_6020[i].speed, can_motor_message_6020[i].rotor_speed, motor_pid_6020[i].pos.f_cal_pid(&motor_pid_6020[i].pos, can_motor_message_6020[i].real_ecd, set_target2[i - 4]));
        }
        CAN_Send_6020(motor_pid_6020[4].speed.output, motor_pid_6020[5].speed.output, motor_pid_6020[6].speed.output);
    }
    osDelay(10);
    // HAL_Delay(10); // 频率100HZ,必须有阻塞，不然报文无法读取
}

/**
 * @brief          四轮全向轮控制
 * @param[in]      x：遥控器x轴数值
 * @param[in]			y：遥控器y轴数值
 * @param[in]			w：自转大小
 * @retval         none
 */
void omni_wheel(double x, double y, double w, uint16_t type)
{
    double lf, rf, lb, rb; // 左前 右前 左后 右后
    lf = -0.707106 * x - 0.707106 * y + w;
    rf = -0.707106 * x + 0.707106 * y + w;
    lb = 0.707106 * x - 0.707106 * y + w;
    rb = 0.707106 * x + 0.707106 * y + w;

    CAN_Send_PID(rf, lf, lb, rb, type); // 16384=0.707*660*2*x x约等于17.5 即系数最大不超过17.5
}

void omni_wheel_t(int16_t x_earth, int16_t y_earth, int16_t w, double yaw)
{
    // yaw为大地坐标和车坐标的夹角
    double lf, rf, lb, rb; // 左前 右前 左后 右后
    // int16_t x_robot,y_robot;
    //     x_earth=-x_earth;
    //     y_earth=-y_earth;	//视安装方向更改
    //     x_robot= x_earth*cos(yaw)+y_earth*sin(yaw);
    //     y_robot= -x_earth*sin(yaw)+y_earth*cos(yaw);//坐标旋转
    //
    //     lf = -0.707106 * x_robot - 0.707106 * y_robot + w;
    //     rf = -0.707106 * x_robot + 0.707106 * y_robot + w;
    //     lb = 0.707106 * x_robot - 0.707106 * y_robot + w;
    //     rb = 0.707106 * x_robot + 0.707106 * y_robot + w;

    lf = y_earth * cos(135 + 45) + x_earth * cos(-135 + 45);
    rf = y_earth * cos(45 + 45) + x_earth * cos(135 + 45);
    lb = y_earth * cos(-135 + 45) + x_earth * cos(-45 + 45);
    rb = y_earth * cos(-45 + 45) + x_earth * cos(45 + 45);

    CAN_Send_PID(5 * rf, 5 * lf, 5 * lb, 5 * rb, 111); // 16384=0.707*660*2*x x约等于17.5 即系数最大不超过17.5
}

/**
 * @brief          舵轮控制
 * @param[in]      none
 * @retval         none
 */
int16_t last_rc_alpha = 0, rc_alpha = 0, round_cnt = 0, total_alpha = 0;
int16_t power = 0;
void steering_wheel(void)
{

    last_rc_alpha = rc_alpha;                                   // ɏһ˲=բһ˲
    rc_alpha = atan2(rc_ctrl.ch1, rc_ctrl.ch2) * 180 / 3.14159; // arctan
    power = pow(pow(rc_ctrl.ch1, 2.0) + pow(rc_ctrl.ch2, 2.0), 0.5);

    if (rc_alpha < 0) // ½«º¯ʽͼϱ±䳉0-2¦°µķֶκ¯ʽ
    {
        rc_alpha = rc_alpha + (180 * 2);
    }
    if (rc_alpha - last_rc_alpha < -180) // Ȧʽ
    {
        round_cnt++;
    }
    else if (rc_alpha - last_rc_alpha > 180)
    {
        round_cnt--;
    }
    total_alpha = (round_cnt * 360) + rc_alpha; // ל½ǶȨң¸Щ

    if (rc_ctrl.ch1 == 0 && rc_ctrl.ch2 == 0)
    {
        CAN_Send_PID(-total_alpha * 3, -total_alpha * 3, -total_alpha * 3, -total_alpha * 3, 11);
        total_alpha = 0;
    }
    else
    {
        CAN_Send_PID(total_alpha * 3, total_alpha * 3, total_alpha * 3, total_alpha * 3, 11);
    }
}

/**
 * @brief          激光定位
 * @param[in]      x：x轴坐标            y:y轴坐标
 * @param[in]			 x_err:x轴正负误差     y_err_err：y轴正负误差
 * @retval         none
 */
double x_v, y_v;
int16_t chassis_err, pillar_err;

void laser_positioning(int16_t chassis_distance, double chassis_err_err)
{
    pillar_err = info_usart8;
    chassis_err = info_usart7 - chassis_distance;

    /*下方激光定位底盘前后  在吴哥区 chassis_err最大1400*/
    if (chassis_err > 400 || chassis_err < -400)
    {
        y_v = 700;
    }
    else if ((chassis_err <= 400 && chassis_err > 200) || (chassis_err >= -400 && chassis_err < -200))
    {
        y_v = 600;
    }
    else if ((chassis_err <= 200 && chassis_err > 100) || (chassis_err >= -200 && chassis_err < -100))
    {
        y_v = 500;
    }
    else if ((chassis_err <= 100 && chassis_err > chassis_err_err) || (chassis_err >= -100 && chassis_err < -chassis_err_err))
    {
        y_v = 400; // 600可稳
    }

    // y_v = 500;

    /* 底盘左右  立柱定位*/
    if (pillar_err > 1900) // 未打到柱子
    {
        x_v = -400; // 只能向一个方向平移 左?
    }
    else if (pillar_err <= 1500) // 打到柱子 差值155
    {
        x_v = 0;
    }

    //		x_v=-x_v;
    //		y_v=-y_v;

    //    if(chassis_err>=chassis_err_err) //底盘未到达位置
    //    {
    //        omni_wheel(x_v,-y_v,0,can1_14_speed);
    //    }
    //    else if(chassis_err<=-chassis_err_err)
    //    {
    //        omni_wheel(x_v,y_v,0,can1_14_speed);
    //    }
    //    else  																													 //到达位置
    //    {
    //        omni_wheel(x_v,0,0,can1_14_speed);
    //    }

    if (chassis_err >= chassis_err_err) // 底盘未到达位置
    {
        omni_wheel(x_v, -y_v, 0, can1_14_speed);
    }
    else if (chassis_err <= -chassis_err_err)
    {
        omni_wheel(x_v, y_v, 0, can1_14_speed);
    }
    else // 到达位置
    {
        omni_wheel(x_v, 0, 0, can1_14_speed);
    }
}

/**
 * @brief          梯形轨迹
 * @param[in]      V_max:最大速度  a_up:加速加速度   a_down:减速加速度  s：距离
 * @retval         none
 */
int16_t begin_flag = 0;
uint32_t start_time;
long double t_up, t_down, t_constant, s_up_down, v_now, now_time, v_max, rpm_now;
long double Trape(long double rpm_max, long double a_up, long double a_down, long double s)
{
    v_max = rpm_max * 0.31415926 / 60.0 / 19.0; // 单位转换
    t_up = v_max / a_up;
    t_down = v_max / a_down;
    s_up_down = (t_up + t_down) * v_max / 2;

    if (begin_flag == 0)
    {
        start_time = HAL_GetTick() / 1000.0;
        begin_flag = 1;
    }

    if (s_up_down <= s) // 加速减速时路程大于总路程   即加减速度过小 或 最大转速过大
    {
        t_constant = (s - s_up_down) / v_max;
        now_time = HAL_GetTick() / 1000.0 - start_time;
        if (0 < now_time && now_time <= t_up) // 加速阶段
        {
            v_now = now_time * a_up;
        }
        else if (t_up < now_time && now_time <= (t_up + t_constant)) // 匀速阶段
        {
            v_now = v_max;
        }
        else if ((t_up + t_constant) < now_time && now_time <= (t_up + t_constant + t_down)) // 减速阶段
        {
            v_now = v_max - (now_time - t_up - t_constant) * a_down;
        }
        //				if(now_time>(t_up+t_constant+t_down+0.5))
        //				{
        //					begin_flag = 0;
        //				}
    }
    else if (s_up_down > s)
    {
        /*等待算法上的解决*/
        // 闪红灯报警
        LED(10, 1);
        HAL_Delay(250);
        LED(10, 0);
        HAL_Delay(250);
        v_now = 0;
    }
    rpm_now = v_now * 60.0 * 19 / 0.31415926;
    return rpm_now;
}
