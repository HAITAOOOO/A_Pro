#include "delay_bsp.h"
#include "cmsis_os.h"
/**
 * @brief          微妙延时__NOP版 os和非os通用
 * @param[in]      us：需延时的微秒数
 * @retval         none
 */
void delay_us_iic(uint32_t us)
{
    while(us!=0) {
        us--;
        __NOP();
    }
}

/**
 * @brief          微妙延时 非os版
 * @param[in]      us：需延时的微秒数
 * @retval         none
 */
void delay_us(uint32_t us)
{
    uint32_t ticks;
    uint32_t told,tnow,reload,tcnt=0;

    reload = SysTick->LOAD;                     //获取重装载寄存器值
    ticks = us * (SystemCoreClock / 1000000);  //计数时间值
    told=SysTick->VAL;                          //获取当前数值寄存器值（开始时数值）

    while(1)
    {
        tnow=SysTick->VAL;          //获取当前数值寄存器值
        if(tnow!=told)              //当前值不等于开始值说明已在计数
        {

            if(tnow<told)             //当前值小于开始数值，说明未计到0
                tcnt+=told-tnow;     //计数值=开始值-当前值

            else                  //当前值大于开始数值，说明已计到0并重新计数
                tcnt+=reload-tnow+told;   //计数值=重装载值-当前值+开始值（已
            //从开始值计到0）

            told=tnow;                //更新开始值
            if(tcnt>=ticks)break;     //时间超过/等于要延迟的时间,则退出.
        }
    }
}
/**
 * @brief          微妙延时 os版  注：需在任务调度启动后（滴答定时器开启）调用
 * @param[in]      us：需延时的微秒数
 * @retval         none
 */

void delay_us_os(uint32_t nus)
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;
    uint32_t reload = SysTick->LOAD;        /* LOAD的值 */
    ticks = nus * 168;                 /* 需要的节拍数 */
    vTaskSuspendAll();                    /* 阻止OS调度，防止打断us延时 */

    told = SysTick->VAL;                    /* 刚进入时的计数器值 */
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;        /* 这里注意一下SYSTICK是一个递减的计数器就可以了 */
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;                      /* 时间超过/等于要延迟的时间,则退出 */
            }
        }
    }
    xTaskResumeAll();                   /* 恢复OS调度 */
}
