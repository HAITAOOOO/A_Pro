#include "iic_bsp.h"
#include "delay_bsp.h"

/**
 * @brief       IIC延时函数,用于控制IIC读写速度
 * @param       无
 * @retval      无
 */
static void iic_delay(void)
{
    delay_us(10);    /* 10us的延时*/
}

/**
 * @brief       初始化IIC
 * @param       无
 * @retval      无
 */
void iic1_init()
{
    /*
    IIC_SCL->PE4 (J2)
    IIC_SDA->PE5 (J1)
    */
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOE, IIC1_SCL_Pin|IIC1_SDA_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = IIC1_SCL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(IIC1_SCL_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = IIC1_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(IIC1_SDA_GPIO_Port, &GPIO_InitStruct);
    /* SDA引脚模式设置,开漏输出,上拉, 这样就不用再设置IO方向了, 开漏输出的时候(=1), 也可以读取外部信号的高低电平 */

    iic1_stop();     /* 停止总线上所有设备 */
}

void iic2_init()
{
    /*
    IIC_SCL->PE4 (J2)
    IIC_SDA->PE5 (J1)
    */
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOE, IIC2_SCL_Pin|IIC2_SDA_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = IIC2_SCL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(IIC2_SCL_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = IIC2_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(IIC2_SDA_GPIO_Port, &GPIO_InitStruct);
    /* SDA引脚模式设置,开漏输出,上拉, 这样就不用再设置IO方向了, 开漏输出的时候(=1), 也可以读取外部信号的高低电平 */

    iic2_stop();     /* 停止总线上所有设备 */
}

void iic3_init()
{
    /*
    IIC_SCL->PE4 (J2)
    IIC_SDA->PE5 (J1)
    */
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, IIC3_SDA_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, IIC3_SCL_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = IIC3_SCL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(IIC3_SCL_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = IIC3_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(IIC3_SDA_GPIO_Port, &GPIO_InitStruct);
    /* SDA引脚模式设置,开漏输出,上拉, 这样就不用再设置IO方向了, 开漏输出的时候(=1), 也可以读取外部信号的高低电平 */

    iic3_stop();     /* 停止总线上所有设备 */
}

void iic4_init()
{
    /*
    IIC_SCL->PE4 (J2)
    IIC_SDA->PE5 (J1)
    */
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, IIC4_SDA_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, IIC4_SCL_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = IIC4_SCL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(IIC4_SCL_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = IIC4_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(IIC4_SDA_GPIO_Port, &GPIO_InitStruct);
    /* SDA引脚模式设置,开漏输出,上拉, 这样就不用再设置IO方向了, 开漏输出的时候(=1), 也可以读取外部信号的高低电平 */

    iic4_stop();     /* 停止总线上所有设备 */
}

/**
 * @brief       产生IIC起始信号
 * @param       无
 * @retval      无
 */
void iic1_start(void)
{
    IIC1_SDA(1);
    IIC1_SCL(1);
    iic_delay();
    IIC1_SDA(0);     /* START信号: 当SCL为高时, SDA从高变成低, 表示起始信号 */
    iic_delay();
    IIC1_SCL(0);     /* 钳住I2C总线，准备发送或接收数据 */
    iic_delay();
}

void iic2_start(void)
{
    IIC2_SDA(1);
    IIC2_SCL(1);
    iic_delay();
    IIC2_SDA(0);     /* START信号: 当SCL为高时, SDA从高变成低, 表示起始信号 */
    iic_delay();
    IIC2_SCL(0);     /* 钳住I2C总线，准备发送或接收数据 */
    iic_delay();
}

void iic3_start(void)
{
    IIC3_SDA(1);
    IIC3_SCL(1);
    iic_delay();
    IIC3_SDA(0);     /* START信号: 当SCL为高时, SDA从高变成低, 表示起始信号 */
    iic_delay();
    IIC3_SCL(0);     /* 钳住I2C总线，准备发送或接收数据 */
    iic_delay();
}

void iic4_start(void)
{
    IIC4_SDA(1);
    IIC4_SCL(1);
    iic_delay();
    IIC4_SDA(0);     /* START信号: 当SCL为高时, SDA从高变成低, 表示起始信号 */
    iic_delay();
    IIC4_SCL(0);     /* 钳住I2C总线，准备发送或接收数据 */
    iic_delay();
}

/**
 * @brief       产生IIC停止信号
 * @param       无
 * @retval      无
 */
void iic1_stop(void)
{
    IIC1_SDA(0);     /* STOP信号: 当SCL为高时, SDA从低变成高, 表示停止信号 */
    iic_delay();
    IIC1_SCL(1);
    iic_delay();
    IIC1_SDA(1);     /* 发送I2C总线结束信号 */
    iic_delay();
}

void iic2_stop(void)
{
    IIC2_SDA(0);     /* STOP信号: 当SCL为高时, SDA从低变成高, 表示停止信号 */
    iic_delay();
    IIC2_SCL(1);
    iic_delay();
    IIC2_SDA(1);     /* 发送I2C总线结束信号 */
    iic_delay();
}

void iic3_stop(void)
{
    IIC3_SDA(0);     /* STOP信号: 当SCL为高时, SDA从低变成高, 表示停止信号 */
    iic_delay();
    IIC3_SCL(1);
    iic_delay();
    IIC3_SDA(1);     /* 发送I2C总线结束信号 */
    iic_delay();
}

void iic4_stop(void)
{
    IIC4_SDA(0);     /* STOP信号: 当SCL为高时, SDA从低变成高, 表示停止信号 */
    iic_delay();
    IIC4_SCL(1);
    iic_delay();
    IIC4_SDA(1);     /* 发送I2C总线结束信号 */
    iic_delay();
}
/**
 * @brief       等待应答信号到来
 * @param       无
 * @retval      1，接收应答失败
 *              0，接收应答成功
 */
uint8_t iic1_wait_ack(void)
{
    uint8_t waittime = 0;
    uint8_t rack = 0;

    IIC1_SDA(1);     /* 主机释放SDA线(此时外部器件可以拉低SDA线) */
    iic_delay();
    IIC1_SCL(1);     /* SCL=1, 此时从机可以返回ACK */
    iic_delay();

    while (IIC1_READ_SDA)    /* 等待应答 */
    {
        waittime++;

        if (waittime > 250)
        {
            iic1_stop();
            rack = 1;
            break;
        }
    }

    IIC1_SCL(0);     /* SCL=0, 结束ACK检查 */
    iic_delay();
    return rack;
}

uint8_t iic2_wait_ack(void)
{
    uint8_t waittime = 0;
    uint8_t rack = 0;

    IIC2_SDA(1);     /* 主机释放SDA线(此时外部器件可以拉低SDA线) */
    iic_delay();
    IIC2_SCL(1);     /* SCL=1, 此时从机可以返回ACK */
    iic_delay();

    while (IIC2_READ_SDA)    /* 等待应答 */
    {
        waittime++;

        if (waittime > 250)
        {
            iic2_stop();
            rack = 1;
            break;
        }
    }

    IIC2_SCL(0);     /* SCL=0, 结束ACK检查 */
    iic_delay();
    return rack;
}

uint8_t iic3_wait_ack(void)
{
    uint8_t waittime = 0;
    uint8_t rack = 0;

    IIC3_SDA(1);     /* 主机释放SDA线(此时外部器件可以拉低SDA线) */
    iic_delay();
    IIC3_SCL(1);     /* SCL=1, 此时从机可以返回ACK */
    iic_delay();

    while (IIC3_READ_SDA)    /* 等待应答 */
    {
        waittime++;

        if (waittime > 250)
        {
            iic3_stop();
            rack = 1;
            break;
        }
    }

    IIC3_SCL(0);     /* SCL=0, 结束ACK检查 */
    iic_delay();
    return rack;
}

uint8_t iic4_wait_ack(void)
{
    uint8_t waittime = 0;
    uint8_t rack = 0;

    IIC4_SDA(1);     /* 主机释放SDA线(此时外部器件可以拉低SDA线) */
    iic_delay();
    IIC4_SCL(1);     /* SCL=1, 此时从机可以返回ACK */
    iic_delay();

    while (IIC4_READ_SDA)    /* 等待应答 */
    {
        waittime++;

        if (waittime > 250)
        {
            iic4_stop();
            rack = 1;
            break;
        }
    }

    IIC4_SCL(0);     /* SCL=0, 结束ACK检查 */
    iic_delay();
    return rack;
}

/**
 * @brief       产生ACK应答
 * @param       无
 * @retval      无
 */
void iic1_ack(void)
{
    IIC1_SDA(0);     /* SCL 0 -> 1 时 SDA = 0,表示应答 */
    iic_delay();
    IIC1_SCL(1);     /* 产生一个时钟 */
    iic_delay();
    IIC1_SCL(0);
    iic_delay();
    IIC1_SDA(1);     /* 主机释放SDA线 */
    iic_delay();
}

void iic2_ack(void)
{
    IIC2_SDA(0);     /* SCL 0 -> 1 时 SDA = 0,表示应答 */
    iic_delay();
    IIC2_SCL(1);     /* 产生一个时钟 */
    iic_delay();
    IIC2_SCL(0);
    iic_delay();
    IIC2_SDA(1);     /* 主机释放SDA线 */
    iic_delay();
}

void iic3_ack(void)
{
    IIC3_SDA(0);     /* SCL 0 -> 1 时 SDA = 0,表示应答 */
    iic_delay();
    IIC3_SCL(1);     /* 产生一个时钟 */
    iic_delay();
    IIC3_SCL(0);
    iic_delay();
    IIC3_SDA(1);     /* 主机释放SDA线 */
    iic_delay();
}

void iic4_ack(void)
{
    IIC4_SDA(0);     /* SCL 0 -> 1 时 SDA = 0,表示应答 */
    iic_delay();
    IIC4_SCL(1);     /* 产生一个时钟 */
    iic_delay();
    IIC4_SCL(0);
    iic_delay();
    IIC4_SDA(1);     /* 主机释放SDA线 */
    iic_delay();
}



/**
 * @brief       不产生ACK应答
 * @param       无
 * @retval      无
 */
void iic1_nack(void)
{
    IIC1_SDA(1);     /* SCL 0 -> 1  时 SDA = 1,表示不应答 */
    iic_delay();
    IIC1_SCL(1);     /* 产生一个时钟 */
    iic_delay();
    IIC1_SCL(0);
    iic_delay();
}

void iic2_nack(void)
{
    IIC2_SDA(1);     /* SCL 0 -> 1  时 SDA = 1,表示不应答 */
    iic_delay();
    IIC2_SCL(1);     /* 产生一个时钟 */
    iic_delay();
    IIC2_SCL(0);
    iic_delay();
}

void iic3_nack(void)
{
    IIC3_SDA(1);     /* SCL 0 -> 1  时 SDA = 1,表示不应答 */
    iic_delay();
    IIC3_SCL(1);     /* 产生一个时钟 */
    iic_delay();
    IIC3_SCL(0);
    iic_delay();
}

void iic4_nack(void)
{
    IIC4_SDA(1);     /* SCL 0 -> 1  时 SDA = 1,表示不应答 */
    iic_delay();
    IIC4_SCL(1);     /* 产生一个时钟 */
    iic_delay();
    IIC4_SCL(0);
    iic_delay();
}
/**
 * @brief       IIC发送一个字节
 * @param       data: 要发送的数据
 * @retval      无
 */
void iic1_send_byte(uint8_t data)
{
    uint8_t t;

    for (t = 0; t < 8; t++)
    {
        IIC1_SDA((data & 0x80) >> 7);    /* 高位先发送 */
        iic_delay();
        IIC1_SCL(1);
        iic_delay();
        IIC1_SCL(0);
        data <<= 1;     /* 左移1位,用于下一次发送 */
    }
    IIC1_SDA(1);         /* 发送完成, 主机释放SDA线 */
}

void iic2_send_byte(uint8_t data)
{
    uint8_t t;

    for (t = 0; t < 8; t++)
    {
        IIC2_SDA((data & 0x80) >> 7);    /* 高位先发送 */
        iic_delay();
        IIC2_SCL(1);
        iic_delay();
        IIC2_SCL(0);
        data <<= 1;     /* 左移1位,用于下一次发送 */
    }
    IIC2_SDA(1);         /* 发送完成, 主机释放SDA线 */
}

void iic3_send_byte(uint8_t data)
{
    uint8_t t;

    for (t = 0; t < 8; t++)
    {
        IIC3_SDA((data & 0x80) >> 7);    /* 高位先发送 */
        iic_delay();
        IIC3_SCL(1);
        iic_delay();
        IIC3_SCL(0);
        data <<= 1;     /* 左移1位,用于下一次发送 */
    }
    IIC3_SDA(1);         /* 发送完成, 主机释放SDA线 */
}

void iic4_send_byte(uint8_t data)
{
    uint8_t t;

    for (t = 0; t < 8; t++)
    {
        IIC4_SDA((data & 0x80) >> 7);    /* 高位先发送 */
        iic_delay();
        IIC4_SCL(1);
        iic_delay();
        IIC4_SCL(0);
        data <<= 1;     /* 左移1位,用于下一次发送 */
    }
    IIC4_SDA(1);         /* 发送完成, 主机释放SDA线 */
}
/**
 * @brief       IIC读取一个字节
 * @param       ack:  ack=1时，发送ack; ack=0时，发送nack
 * @retval      接收到的数据
 */
uint8_t iic1_read_byte(uint8_t ack)
{
    uint8_t i, receive = 0;

    for (i = 0; i < 8; i++ )    /* 接收1个字节数据 */
    {
        receive <<= 1;  /* 高位先输出,所以先收到的数据位要左移 */
        IIC1_SCL(1);
        iic_delay();

        if (IIC1_READ_SDA)
        {
            receive++;
        }

        IIC1_SCL(0);
        iic_delay();
    }

    if (!ack)
    {
        iic1_nack();     /* 发送nACK */
    }
    else
    {
        iic1_ack();      /* 发送ACK */
    }

    return receive;
}

uint8_t iic2_read_byte(uint8_t ack)
{
    uint8_t i, receive = 0;

    for (i = 0; i < 8; i++ )    /* 接收1个字节数据 */
    {
        receive <<= 1;  /* 高位先输出,所以先收到的数据位要左移 */
        IIC2_SCL(1);
        iic_delay();

        if (IIC2_READ_SDA)
        {
            receive++;
        }

        IIC2_SCL(0);
        iic_delay();
    }

    if (!ack)
    {
        iic2_nack();     /* 发送nACK */
    }
    else
    {
        iic2_ack();      /* 发送ACK */
    }

    return receive;
}

uint8_t iic3_read_byte(uint8_t ack)
{
    uint8_t i, receive = 0;

    for (i = 0; i < 8; i++ )    /* 接收1个字节数据 */
    {
        receive <<= 1;  /* 高位先输出,所以先收到的数据位要左移 */
        IIC3_SCL(1);
        iic_delay();

        if (IIC3_READ_SDA)
        {
            receive++;
        }

        IIC3_SCL(0);
        iic_delay();
    }

    if (!ack)
    {
        iic3_nack();     /* 发送nACK */
    }
    else
    {
        iic3_ack();      /* 发送ACK */
    }

    return receive;
}

uint8_t iic4_read_byte(uint8_t ack)
{
    uint8_t i, receive = 0;

    for (i = 0; i < 8; i++ )    /* 接收1个字节数据 */
    {
        receive <<= 1;  /* 高位先输出,所以先收到的数据位要左移 */
        IIC4_SCL(1);
        iic_delay();

        if (IIC4_READ_SDA)
        {
            receive++;
        }

        IIC4_SCL(0);
        iic_delay();
    }

    if (!ack)
    {
        iic4_nack();     /* 发送nACK */
    }
    else
    {
        iic4_ack();      /* 发送ACK */
    }

    return receive;
}
