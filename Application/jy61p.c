#include "jy61p.h"
#include "wit_c_sdk.h"
//extern "C"

//{

//#include "WIT_C_SDK.h"

//}  
#include "REG.h"
#include "main.h"
#include "usart.h"
// #include "iic_bsp.h"

static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;
const uint32_t c_uiBaud[10] = {0, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};

#define ACC_UPDATE 0x01
#define GYRO_UPDATE 0x02
#define ANGLE_UPDATE 0x04
#define MAG_UPDATE 0x08
#define READ_UPDATE 0x80

static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
    // Uart2Send(p_data, uiSize);
    HAL_UART_Transmit_IT(&huart3, p_data, uiSize);
}

static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
    int i;
    for (i = 0; i < uiRegNum; i++)
    {
        switch (uiReg)
        {
            //            case AX:
            //            case AY:
        case AZ:
            s_cDataUpdate |= ACC_UPDATE;
            break;
            //            case GX:
            //            case GY:
        case GZ:
            s_cDataUpdate |= GYRO_UPDATE;
            break;
            //            case HX:
            //            case HY:
        case HZ:
            s_cDataUpdate |= MAG_UPDATE;
            break;
            //            case Roll:
            //            case Pitch:
        case Yaw:
            s_cDataUpdate |= ANGLE_UPDATE;
            break;
        default:
            s_cDataUpdate |= READ_UPDATE;
            break;
        }
        uiReg++;
    }
}

static void AutoScanSensor(void)
{
    int i, iRetry;

    for (i = 1; i < 10; i++)
    {
        // Usart2Init(c_uiBaud[i]);
        iRetry = 2;
        do
        {
            s_cDataUpdate = 0;
            WitReadReg(AX, 3);
            HAL_Delay(100);
            if (s_cDataUpdate != 0)
            {
                // printf("%d baud find sensor\r\n\r\n", c_uiBaud[i]);
                // ShowHelp();
                return;
            }
            iRetry--;
        } while (iRetry);
    }
    // printf("can not find sensor\r\n");
    // printf("please check your connection\r\n");
}

static void Delayms(uint16_t ucMs)
{
    HAL_Delay(ucMs);
}

void jy61p_init(void)
{
    WitInit(WIT_PROTOCOL_NORMAL, 0x50);
    WitSerialWriteRegister(SensorUartSend);
    WitRegisterCallBack(SensorDataUpdata);
    WitDelayMsRegister(Delayms);
    AutoScanSensor();
}

float fAcc[3], fGyro[3], fAngle[3];
int i;
void get_data(void)
{
    if (s_cDataUpdate)
    {
        for (i = 0; i < 3; i++)
        {
            fAcc[i] = sReg[AX + i] / 32768.0f * 16.0f;
            fGyro[i] = sReg[GX + i] / 32768.0f * 2000.0f;
            fAngle[i] = sReg[Roll + i] / 32768.0f * 180.0f;
        }
        if (s_cDataUpdate & ACC_UPDATE)
        {
            printf("acc:%.3f %.3f %.3f\r\n", fAcc[0], fAcc[1], fAcc[2]);
            s_cDataUpdate &= ~ACC_UPDATE;
        }
        if (s_cDataUpdate & GYRO_UPDATE)
        {
            printf("gyro:%.3f %.3f %.3f\r\n", fGyro[0], fGyro[1], fGyro[2]);
            s_cDataUpdate &= ~GYRO_UPDATE;
        }
        if (s_cDataUpdate & ANGLE_UPDATE)
        {
            printf("angle:%.3f %.3f %.3f\r\n", fAngle[0], fAngle[1], fAngle[2]);
            s_cDataUpdate &= ~ANGLE_UPDATE;
        }
        if (s_cDataUpdate & MAG_UPDATE)
        {
            printf("mag:%d %d %d\r\n", sReg[HX], sReg[HY], sReg[HZ]);
            s_cDataUpdate &= ~MAG_UPDATE;
        }
    }
}
// unsigned char chrTemp[30];
// unsigned char chrTemp_Offset[30];

// /**
//  * @brief 两个char型拼接成short型
//  * @param cData
//  * @return short
//  */
// short CharToShort(unsigned char cData[])
// {
//     return ((short)cData[1] << 8) | cData[0];
// }

// /**
//  * @brief 获取补偿值
//  * @param      none
//  * @retval     none
//  */
// void read_offset(jy61p *jy61p_offset)
// {
//     HAL_Delay(1000);
//     iic1readbytes(0x50, AX, 24, &chrTemp_Offset[0]);
//     jy61p_offset->pitch = (float)CharToShort(&chrTemp_Offset[18]) / 32768 * 180;
//     jy61p_offset->roll = (float)CharToShort(&chrTemp_Offset[20]) / 32768 * 180;
//     jy61p_offset->yaw = (float)CharToShort(&chrTemp_Offset[22]) / 32768 * 180;
//     /*Configure GPIO pin Output Level */

//     // HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
// }
// /**
//  * @brief 读取jy61p角度
//  * @param jy61p_get :储存读取角度值的结构体
//  * @param jy61p_offset：上电补偿角度值的结构体
//  */
// void read_angle(jy61p *jy61p_read, jy61p *jy61p_offset)
// {
//     HAL_Delay(30);
//     iic1readbytes(0x50, AX, 24, &chrTemp[0]);
//     jy61p_read->pitch = ((float)CharToShort(&chrTemp[18]) / 32768 * 180) - jy61p_offset->pitch; // 俯仰
//     jy61p_read->roll = ((float)CharToShort(&chrTemp[20]) / 32768 * 180) - jy61p_offset->roll;   // 横滚
//     jy61p_read->yaw = ((float)CharToShort(&chrTemp[22]) / 32768 * 180) - jy61p_offset->yaw;     // 偏航
// }
