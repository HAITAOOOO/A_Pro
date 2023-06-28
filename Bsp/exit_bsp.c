#include "exit_bsp.h"
#include "led_bsp.h"

uint8_t exit_flag;
uint8_t rising_falling_flag;

/**
  * @brief          exit callback function
  * @param[in]      GPIO_Pin:gpio pin
  * @retval         none
  */
/**
  * @brief          外部中断回调
  * @param[in]      GPIO_Pin:引脚号
  * @retval         none
  */
//中断结束后自动调用
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//		if(GPIO_Pin == KEY_Pin )
//    {
//        if(exit_flag == 0)
//        {
//            exit_flag = 1;
//            rising_falling_flag = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin );
//        }
//    }
//		if(GPIO_Pin == GPIO_PIN_10)
//		{
//			LED(5,1);
//		}
//		if(GPIO_Pin == I1_Pin)
//		{
//			LED(1,1);
//		}
//		if(GPIO_Pin == I2_Pin)
//		{
//			LED(2,1);
//		}
//		if(GPIO_Pin == J1_Pin)
//		{
//			LED(3,1);
//		}
//		if(GPIO_Pin == J2_Pin)
//		{
//			LED(4,1);
//		}
//		if(GPIO_Pin == K1_Pin)
//		{
//			LED(5,1);
//		}
//		if(GPIO_Pin == K2_Pin)
//		{
//			LED(6,1);
//		}
//}

/**
  * @brief          判断按键是否按下
  * @param[in]      none
  * @retval         1：按下 0：没有
  */
/*
例:
if(if_KEY()==1)
{
	执行函数
}
*/
uint16_t if_KEY(void)			//轮询?
{
    if(exit_flag == 1)
    {
        exit_flag = 2;													//防止二次进入
        if(rising_falling_flag == GPIO_PIN_SET)//判断是否触发回调函数
        {
            //debouce
            //消抖
            HAL_Delay(20);
            if(HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET)
            {
                exit_flag = 0;
                return 1;
            }
            else
            {
                exit_flag = 0;
                return 0;
            }
        }
    }
    return 0;
}


//前台函数
/*	if(exit_flagx == 1)
	{
			exit_flagx = 2;											//防止二次进入
			if(rising_falling_flagx == GPIO_PIN_SET)//判断是否触发回调函数
			{
					//debouce
					//消抖
					HAL_Delay(20);
					if(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin_x) == GPIO_PIN_SET)
					{
							/执行函数/
							exit_flagx = 0;
					}
					else
					{
							exit_flagx = 0;
					}
			}
	}
*/
