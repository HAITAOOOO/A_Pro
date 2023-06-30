#include "usart_bsp.h"
#include <stdarg.h>
#include "struct_typedef.h"


double yaw_usart6, x_usart6, y_usart6;
int info_usart3;
int info_usart7;
int info_usart8;

uint8_t usart_tx_buf[200];

float roll, pitch, yaw, roll_offset, pitch_offset, yaw_offset, ax, ay, az, wx, wy, wz;
/**
 * @brief          串口发送
 * @param[in]      UART_HandleTypeDef*huart：串口句柄
 * @param[in]      const char *fmt：输出格式
 * @param[in]			...：输出内容
 * @retval         none
 */
// 例：print("variable_1:%d\n",0212);
void print(const char *fmt, ...)
{
    static uint8_t tx_buf[256] = {0};
    static va_list ap;
    static uint16_t len;
    va_start(ap, fmt);
    // return length of string
    // 返回字符串长度
    len = vsprintf((char *)tx_buf, fmt, ap);
    va_end(ap);
    HAL_UART_Transmit(&huart6, tx_buf, len, 100);
}

/**
 * @brief          激光测距数据解码
 * @param[in]      none
 * @retval         none
 */
void Laser_decoding()
{
    if (rx_buffer_usart3[3] == '.')
    {
        info_usart3 = (rx_buffer_usart3[2] - '0') * 1000 + (rx_buffer_usart3[4] - '0') * 100 + (rx_buffer_usart3[5] - '0') * 10 + (rx_buffer_usart3[6] - '0');
    }
    else if (rx_buffer_usart3[4] == '.')
    {
        info_usart3 = (rx_buffer_usart3[2] - '0') * 10000 + (rx_buffer_usart3[3] - '0') * 1000 + (rx_buffer_usart3[5] - '0') * 100 + (rx_buffer_usart3[6] - '0') * 10 + (rx_buffer_usart3[7] - '0');
    }
    if (rx_buffer_usart7[3] == '.')
    {
        info_usart7 = (rx_buffer_usart7[2] - '0') * 1000 + (rx_buffer_usart7[4] - '0') * 100 + (rx_buffer_usart7[5] - '0') * 10 + (rx_buffer_usart7[6] - '0');
    }
    else if (rx_buffer_usart7[4] == '.')
    {
        info_usart7 = (rx_buffer_usart7[2] - '0') * 10000 + (rx_buffer_usart7[3] - '0') * 1000 + (rx_buffer_usart7[5] - '0') * 100 + (rx_buffer_usart7[6] - '0') * 10 + (rx_buffer_usart7[7] - '0');
    }
    if (rx_buffer_usart8[3] == '.')
    {
        info_usart8 = (rx_buffer_usart8[2] - '0') * 1000 + (rx_buffer_usart8[4] - '0') * 100 + (rx_buffer_usart8[5] - '0') * 10 + (rx_buffer_usart8[6] - '0');
    }
    else if (rx_buffer_usart8[4] == '.')
    {
        info_usart8 = (rx_buffer_usart8[2] - '0') * 10000 + (rx_buffer_usart8[3] - '0') * 1000 + (rx_buffer_usart8[5] - '0') * 100 + (rx_buffer_usart8[6] - '0') * 10 + (rx_buffer_usart8[7] - '0');
    }
}
/**
 * @brief          偏航角数据解码
 * @param[in]      none
 * @retval         none
 */
void yaw_decoding()
{

    if (rx_buffer_usart6[0] == '-') // 负数
    {
        if (rx_buffer_usart6[2] == '.') // 一位整数
        {
            yaw_usart6 = (rx_buffer_usart6[1] - '0') +
                         (rx_buffer_usart6[3] - '0') * 0.1 + (rx_buffer_usart6[4] - '0') * 0.01 +
                         (rx_buffer_usart6[5] - '0') * 0.001 + (rx_buffer_usart6[6] - '0') * 0.001 +
                         (rx_buffer_usart6[7] - '0') * 0.0001 + (rx_buffer_usart6[8] - '0') * 0.00001;
            yaw_usart6 = -yaw_usart6;
        }
        else if (rx_buffer_usart6[3] == '.') // 两位整数
        {
            yaw_usart6 = (rx_buffer_usart6[1] - '0') * 10 + (rx_buffer_usart6[2] - '0') +
                         (rx_buffer_usart6[4] - '0') * 0.1 + (rx_buffer_usart6[5] - '0') * 0.01 +
                         (rx_buffer_usart6[6] - '0') * 0.001 + (rx_buffer_usart6[7] - '0') * 0.0001 +
                         (rx_buffer_usart6[8] - '0') * 0.00001 + (rx_buffer_usart6[9] - '0') * 0.000001;
            yaw_usart6 = -yaw_usart6;
        }
        else if (rx_buffer_usart6[4] == '.')
        {
            yaw_usart6 = (rx_buffer_usart6[1] - '0') * 100 + (rx_buffer_usart6[2] - '0') * 10 + (rx_buffer_usart6[3] - '0') +
                         (rx_buffer_usart6[5] - '0') * 0.1 + (rx_buffer_usart6[6] - '0') * 0.01 +
                         (rx_buffer_usart6[7] - '0') * 0.001 + (rx_buffer_usart6[8] - '0') * 0.0001 +
                         (rx_buffer_usart6[9] - '0') * 0.00001 + (rx_buffer_usart6[10] - '0') * 0.000001;
            yaw_usart6 = -yaw_usart6;
        }
    }
    else // 正数
    {
        if (rx_buffer_usart6[1] == '.') // 一位整数
        {
            yaw_usart6 = (rx_buffer_usart6[0] - '0') +
                         (rx_buffer_usart6[2] - '0') * 0.1 + (rx_buffer_usart6[3] - '0') * 0.01 +
                         (rx_buffer_usart6[4] - '0') * 0.001 + (rx_buffer_usart6[5] - '0') * 0.001 +
                         (rx_buffer_usart6[6] - '0') * 0.0001 + (rx_buffer_usart6[7] - '0') * 0.00001;
        }
        else if (rx_buffer_usart6[2] == '.')
        {
            yaw_usart6 = (rx_buffer_usart6[0] - '0') * 10 + (rx_buffer_usart6[1] - '0') +
                         (rx_buffer_usart6[3] - '0') * 0.1 + (rx_buffer_usart6[4] - '0') * 0.01 +
                         (rx_buffer_usart6[5] - '0') * 0.001 + (rx_buffer_usart6[6] - '0') * 0.0001 +
                         (rx_buffer_usart6[7] - '0') * 0.00001 + (rx_buffer_usart6[8] - '0') * 0.000001;
        }
        else if (rx_buffer_usart6[3] == '.')
        {
            yaw_usart6 = (rx_buffer_usart6[0] - '0') * 100 + (rx_buffer_usart6[1] - '0') * 10 + (rx_buffer_usart6[2] - '0') +
                         (rx_buffer_usart6[4] - '0') * 0.1 + (rx_buffer_usart6[5] - '0') * 0.01 +
                         (rx_buffer_usart6[6] - '0') * 0.001 + (rx_buffer_usart6[7] - '0') * 0.0001 +
                         (rx_buffer_usart6[8] - '0') * 0.00001 + (rx_buffer_usart6[9] - '0') * 0.000001;
        }
    }
}
/*读取jy61p角度 波特率9600*/
int16_t angle_offset = 0;
void jy61p_read(void)
{
    if (0X51 == rx_buffer_usart3[1])
    {
        ax = (float)((float)((rx_buffer_usart3[3] << 8) | rx_buffer_usart3[2]) / 32768.0f * 16.0f * 9.8f);
        ay = (float)((float)((rx_buffer_usart3[5] << 8) | rx_buffer_usart3[4]) / 32768.0f * 16.0f * 9.8f);
        az = (float)((float)((rx_buffer_usart3[7] << 8) | rx_buffer_usart3[6]) / 32768.0f * 16.0f * 9.8f);
    }
    if (0X52 == rx_buffer_usart3[12])
    {
        wx = (float)(((rx_buffer_usart3[14] << 8) | rx_buffer_usart3[13]) / 32768.0f * 2000.0f);
        wy = (float)(((rx_buffer_usart3[16] << 8) | rx_buffer_usart3[15]) / 32768.0f * 2000.0f);
        wz = (float)(((rx_buffer_usart3[18] << 8) | rx_buffer_usart3[17]) / 32768.0f * 2000.0f);
    }
    if (0X53 == rx_buffer_usart3[23])
    {
        if (angle_offset == 0)
        {
            roll_offset = (float)(((((short)rx_buffer_usart3[25]) << 8) | rx_buffer_usart3[24]) / 32768.0f * 180.0f);
            pitch_offset = (float)(((((short)rx_buffer_usart3[27]) << 8) | rx_buffer_usart3[26]) / 32768.0f * 180.0f);
            yaw_offset = (float)(((((short)rx_buffer_usart3[29]) << 8) | rx_buffer_usart3[28]) / 32768.0f * 180.0f);
            angle_offset = 1;
        }
        else
        {
            roll = (float)(((((short)rx_buffer_usart3[25]) << 8) | rx_buffer_usart3[24]) / 32768.0f * 180.0f) - roll_offset;
            pitch = (float)(((((short)rx_buffer_usart3[27]) << 8) | rx_buffer_usart3[26]) / 32768.0f * 180.0f) - pitch_offset;
            yaw = (float)(((((short)rx_buffer_usart3[29]) << 8) | rx_buffer_usart3[28]) / 32768.0f * 180.0f) - yaw_offset;

            if (roll >= 180)
            {
                roll -= 360;
            }
            if (pitch >= 180)
            {
                pitch -= 360;
            }
            if (yaw >= 180)
            {
                yaw -= 360;
            }
        }
    }
}

/*
 * 解析出rx_buffer_usart6中的数据
 * 返回解析得到的数据
 */
float Get_Data(void)
{
    uint8_t data_Start_Num = 0; // 记录数据位开始的地方
    uint8_t data_End_Num = 0; // 记录数据位结束的地方
    uint8_t data_Num = 0; // 记录数据位数
    uint8_t minus_Flag = 0; // 判断是不是负数
    float data_return = 0; // 解析得到的数据
    for(uint8_t i=0; i<200; i++) // 查找等号和感叹号的位置
    {
        if(rx_buffer_usart6[i] == '=') data_Start_Num = i + 1; // +1是直接定位到数据起始位
        if(rx_buffer_usart6[i] == '!')
        {
            data_End_Num = i - 1;
            break;
        }
    }
    if(rx_buffer_usart6[data_Start_Num] == '-') // 如果是负数
    {
        data_Start_Num += 1; // 后移一位到数据位
        minus_Flag = 1; // 负数flag
    }
    data_Num = data_End_Num - data_Start_Num + 1;
    if(data_Num == 4) // 数据共4位
    {
        data_return = (rx_buffer_usart6[data_Start_Num]-'0')  + (rx_buffer_usart6[data_Start_Num+2]-'0')*0.1f +
                      (rx_buffer_usart6[data_Start_Num+3]-'0')*0.01f;
    }
    else if(data_Num == 5) // 数据共5位
    {
        data_return = (rx_buffer_usart6[data_Start_Num]-'0')*10 + (rx_buffer_usart6[data_Start_Num+1]-'0') + (rx_buffer_usart6[data_Start_Num+3]-'0')*0.1f +
                      (rx_buffer_usart6[data_Start_Num+4]-'0')*0.01f;
    }
    else if(data_Num == 6) // 数据共6位
    {
        data_return = (rx_buffer_usart6[data_Start_Num]-'0')*100 + (rx_buffer_usart6[data_Start_Num+1]-'0')*10+ (rx_buffer_usart6[data_Start_Num+2]-'0') +
                      (rx_buffer_usart6[data_Start_Num+4]-'0')*0.1f + (rx_buffer_usart6[data_Start_Num+5]-'0')*0.01f;
    }
    if(minus_Flag == 1)  data_return = -data_return;
    return data_return;
}

void adjust_pid_usart(caspid_TypeDef * caspid)
{
    float data_Get = Get_Data();

    if(rx_buffer_usart6[0]=='P' && rx_buffer_usart6[1]=='1') // 位置环P
    {
        caspid->pos.kp = data_Get;
    }
    else if(rx_buffer_usart6[0]=='I' && rx_buffer_usart6[1]=='1')// 位置环I
    {
        caspid->pos.ki = data_Get;
    }
    else if(rx_buffer_usart6[0]=='D' && rx_buffer_usart6[1]=='1')	// 位置环D
    {
        caspid->pos.kd = data_Get;
    }
    else if(rx_buffer_usart6[0]=='P' && rx_buffer_usart6[1]=='2')// 速度环P
    {
        caspid->speed.kp = data_Get;
    }
    else if(rx_buffer_usart6[0]=='I' && rx_buffer_usart6[1]=='2') // 速度环I
    {
        caspid->speed.ki = data_Get;
    }
    else if(rx_buffer_usart6[0]=='D' && rx_buffer_usart6[1]=='2')// 速度环D
    {
        caspid->speed.kd = data_Get;
    }
    else if((rx_buffer_usart6[0]=='S' && rx_buffer_usart6[1]=='p') && rx_buffer_usart6[2]=='e')//目标速度
    {
        caspid->speed.target = data_Get;
    }
    else if((rx_buffer_usart6[0]=='P' && rx_buffer_usart6[1]=='o') && rx_buffer_usart6[2]=='s')//目标位置
    {
        caspid->pos.target = data_Get;
    }

}
