#include "jy61p.h"
#include "iic_bsp.h"

unsigned char chrTemp[30];
unsigned char chrTemp_Offset[30];
/**
 * @brief 两个char型拼接成short型
 *
 * @param cData
 * @return short
 */
short CharToShort(unsigned char cData[])
{
    return ((short)cData[1] << 8) | cData[0];
}
/**
 * @brief 获取补偿值
 * @param      none
 * @retval     none
 */
void read_offset(jy61p *jy61p_offset)
{
    HAL_Delay(1000);
    IICreadBytes(0x50, AX, 24, &chrTemp_Offset[0]);
    jy61p_offset->pitch = (float)CharToShort(&chrTemp_Offset[18]) / 32768 * 180;
    jy61p_offset->roll = (float)CharToShort(&chrTemp_Offset[20]) / 32768 * 180;
    jy61p_offset->yaw = (float)CharToShort(&chrTemp_Offset[22]) / 32768 * 180;
    /*Configure GPIO pin Output Level */

    // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
}
/**
 * @brief 读取jy61p角度
 * @param jy61p_get :储存读取角度值的结构体
 * @param jy61p_offset：上电补偿角度值的结构体
 */
void read_angle(jy61p *jy61p_read, jy61p *jy61p_offset)
{
    HAL_Delay(30);
    IICreadBytes(0x50, AX, 24, &chrTemp[0]);
    jy61p_read->pitch = ((float)CharToShort(&chrTemp[18]) / 32768 * 180) - jy61p_offset->pitch; // 俯仰
    jy61p_read->roll = ((float)CharToShort(&chrTemp[20]) / 32768 * 180) - jy61p_offset->roll;   // 横滚
    jy61p_read->yaw = ((float)CharToShort(&chrTemp[22]) / 32768 * 180) - jy61p_offset->yaw;     // 偏航
}
