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
    HAL_UART_Transmit(&huart3, tx_buf, len, 100);
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
void read_angle(void)
{
    if (0X51 == rx_buffer_usart3[1])
    {
        ax = (float)((float)((rx_buffer_usart3[3] << 8) | rx_buffer_usart3[2]) / 32768.0 * 16 * 9.8);
        ay = (float)((float)((rx_buffer_usart3[5] << 8) | rx_buffer_usart3[4]) / 32768.0 * 16 * 9.8);
        az = (float)((float)((rx_buffer_usart3[7] << 8) | rx_buffer_usart3[6]) / 32768.0 * 16 * 9.8);
    }
    if (0X52 == rx_buffer_usart3[12])
    {
        wx = (float)(((rx_buffer_usart3[14] << 8) | rx_buffer_usart3[13]) / 32768.0 * 2000);
        wy = (float)(((rx_buffer_usart3[16] << 8) | rx_buffer_usart3[15]) / 32768.0 * 2000);
        wz = (float)(((rx_buffer_usart3[18] << 8) | rx_buffer_usart3[17]) / 32768.0 * 2000);
    }
    if (0X53 == rx_buffer_usart3[23])
    {
        if (angle_offset == 0)
        {
            roll_offset = (float)(((((short)rx_buffer_usart3[25]) << 8) | rx_buffer_usart3[24]) / 32768.0 * 180);
            pitch_offset = (float)(((((short)rx_buffer_usart3[27]) << 8) | rx_buffer_usart3[26]) / 32768.0 * 180);
            yaw_offset = (float)(((((short)rx_buffer_usart3[29]) << 8) | rx_buffer_usart3[28]) / 32768.0 * 180);
            angle_offset = 1;
        }
        else
        {
            roll = (float)(((((short)rx_buffer_usart3[25]) << 8) | rx_buffer_usart3[24]) / 32768.0 * 180) - roll_offset;
            pitch = (float)(((((short)rx_buffer_usart3[27]) << 8) | rx_buffer_usart3[26]) / 32768.0 * 180) - pitch_offset;
            yaw = (float)(((((short)rx_buffer_usart3[29]) << 8) | rx_buffer_usart3[28]) / 32768.0 * 180) - yaw_offset;

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
