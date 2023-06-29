#ifndef IIC_BSP_H
#define IIC_BSP_H

#include "main.h"

/*引脚定义*/
#define IIC1_SCL_GPIO_PORT IIC1_SCL_GPIO_Port
#define IIC1_SCL_GPIO_PIN IIC1_SCL_Pin
#define IIC1_SCL_GPIO_CLK_ENABLE()    \
    do                                \
    {                                 \
        __HAL_RCC_GPIOE_CLK_ENABLE(); \
    } while (0) /* PB口时钟使能 */

#define IIC1_SDA_GPIO_PORT IIC1_SDA_GPIO_Port
#define IIC1_SDA_GPIO_PIN IIC1_SDA_Pin
#define IIC1_SDA_GPIO_CLK_ENABLE()    \
    do                                \
    {                                 \
        __HAL_RCC_GPIOE_CLK_ENABLE(); \
    } while (0) /* PB口时钟使能 */

#define IIC2_SCL_GPIO_PORT IIC2_SCL_GPIO_Port
#define IIC2_SCL_GPIO_PIN IIC2_SCL_Pin
#define IIC2_SCL_GPIO_CLK_ENABLE()    \
    do                                \
    {                                 \
        __HAL_RCC_GPIOE_CLK_ENABLE(); \
    } while (0) /* PB口时钟使能 */

#define IIC2_SDA_GPIO_PORT IIC2_SDA_GPIO_Port
#define IIC2_SDA_GPIO_PIN IIC2_SDA_Pin
#define IIC2_SDA_GPIO_CLK_ENABLE()    \
    do                                \
    {                                 \
        __HAL_RCC_GPIOE_CLK_ENABLE(); \
    } while (0) /* PB口时钟使能 */

#define IIC3_SCL_GPIO_PORT IIC3_SCL_GPIO_Port
#define IIC3_SCL_GPIO_PIN IIC3_SCL_Pin
#define IIC3_SCL_GPIO_CLK_ENABLE()    \
    do                                \
    {                                 \
        __HAL_RCC_GPIOE_CLK_ENABLE(); \
    } while (0) /* PB口时钟使能 */

#define IIC3_SDA_GPIO_PORT IIC3_SDA_GPIO_Port
#define IIC3_SDA_GPIO_PIN IIC3_SDA_Pin
#define IIC3_SDA_GPIO_CLK_ENABLE()    \
    do                                \
    {                                 \
        __HAL_RCC_GPIOE_CLK_ENABLE(); \
    } while (0) /* PB口时钟使能 */

#define IIC4_SCL_GPIO_PORT IIC4_SCL_GPIO_Port
#define IIC4_SCL_GPIO_PIN IIC4_SCL_Pin
#define IIC4_SCL_GPIO_CLK_ENABLE()    \
    do                                \
    {                                 \
        __HAL_RCC_GPIOE_CLK_ENABLE(); \
    } while (0) /* PB口时钟使能 */

#define IIC4_SDA_GPIO_PORT IIC4_SDA_GPIO_Port
#define IIC4_SDA_GPIO_PIN IIC4_SDA_Pin
#define IIC4_SDA_GPIO_CLK_ENABLE()    \
    do                                \
    {                                 \
        __HAL_RCC_GPIOE_CLK_ENABLE(); \
    } while (0) /* PB口时钟使能 */

/* IO操作 */
#define IIC1_SCL(x)                                                                                                                                  \
    do                                                                                                                                               \
    {                                                                                                                                                \
        x ? HAL_GPIO_WritePin(IIC1_SCL_GPIO_Port, IIC1_SCL_Pin, GPIO_PIN_SET) : HAL_GPIO_WritePin(IIC1_SCL_GPIO_Port, IIC1_SCL_Pin, GPIO_PIN_RESET); \
    } while (0) /* SCL */

#define IIC1_SDA(x)                                                                                                                                  \
    do                                                                                                                                               \
    {                                                                                                                                                \
        x ? HAL_GPIO_WritePin(IIC1_SDA_GPIO_Port, IIC1_SDA_Pin, GPIO_PIN_SET) : HAL_GPIO_WritePin(IIC1_SDA_GPIO_Port, IIC1_SDA_Pin, GPIO_PIN_RESET); \
    } while (0) /* SDA */

#define IIC1_READ_SDA HAL_GPIO_ReadPin(IIC1_SDA_GPIO_Port, IIC1_SDA_Pin) /* 读取SDA */

#define IIC2_SCL(x)                                                                                                                                  \
    do                                                                                                                                               \
    {                                                                                                                                                \
        x ? HAL_GPIO_WritePin(IIC2_SCL_GPIO_Port, IIC2_SCL_Pin, GPIO_PIN_SET) : HAL_GPIO_WritePin(IIC2_SCL_GPIO_Port, IIC2_SCL_Pin, GPIO_PIN_RESET); \
    } while (0) /* SCL */

#define IIC2_SDA(x)                                                                                                                                  \
    do                                                                                                                                               \
    {                                                                                                                                                \
        x ? HAL_GPIO_WritePin(IIC2_SDA_GPIO_Port, IIC2_SDA_Pin, GPIO_PIN_SET) : HAL_GPIO_WritePin(IIC2_SDA_GPIO_Port, IIC2_SDA_Pin, GPIO_PIN_RESET); \
    } while (0) /* SDA */

#define IIC2_READ_SDA HAL_GPIO_ReadPin(IIC2_SDA_GPIO_Port, IIC2_SDA_Pin) /* 读取SDA */

#define IIC3_SCL(x)                                                                                                                                  \
    do                                                                                                                                               \
    {                                                                                                                                                \
        x ? HAL_GPIO_WritePin(IIC3_SCL_GPIO_Port, IIC3_SCL_Pin, GPIO_PIN_SET) : HAL_GPIO_WritePin(IIC3_SCL_GPIO_Port, IIC3_SCL_Pin, GPIO_PIN_RESET); \
    } while (0) /* SCL */

#define IIC3_SDA(x)                                                                                                                                  \
    do                                                                                                                                               \
    {                                                                                                                                                \
        x ? HAL_GPIO_WritePin(IIC3_SDA_GPIO_Port, IIC3_SDA_Pin, GPIO_PIN_SET) : HAL_GPIO_WritePin(IIC3_SDA_GPIO_Port, IIC3_SDA_Pin, GPIO_PIN_RESET); \
    } while (0) /* SDA */

#define IIC3_READ_SDA HAL_GPIO_ReadPin(IIC3_SDA_GPIO_Port, IIC3_SDA_Pin) /* 读取SDA */

#define IIC4_SCL(x)                                                                                                                                  \
    do                                                                                                                                               \
    {                                                                                                                                                \
        x ? HAL_GPIO_WritePin(IIC4_SCL_GPIO_Port, IIC4_SCL_Pin, GPIO_PIN_SET) : HAL_GPIO_WritePin(IIC4_SCL_GPIO_Port, IIC4_SCL_Pin, GPIO_PIN_RESET); \
    } while (0) /* SCL */

#define IIC4_SDA(x)                                                                                                                                  \
    do                                                                                                                                               \
    {                                                                                                                                                \
        x ? HAL_GPIO_WritePin(IIC4_SDA_GPIO_Port, IIC4_SDA_Pin, GPIO_PIN_SET) : HAL_GPIO_WritePin(IIC4_SDA_GPIO_Port, IIC4_SDA_Pin, GPIO_PIN_RESET); \
    } while (0) /* SDA */

#define IIC4_READ_SDA HAL_GPIO_ReadPin(IIC4_SDA_GPIO_Port, IIC4_SDA_Pin) /* 读取SDA */
/* IIC所有操作函数 */
void iic1_init(void); /* 初始化IIC的IO口 */
void iic2_init(void);
void iic3_init(void);
void iic4_init(void);
void iic1_start(void); /* 发送IIC开始信号 */
void iic2_start(void);
void iic3_start(void);
void iic4_start(void);
void iic1_stop(void); /* 发送IIC停止信号 */
void iic2_stop(void);
void iic3_stop(void);
void iic4_stop(void);
void iic1_ack(void); /* IIC发送ACK信号 */
void iic2_ack(void);
void iic3_ack(void);
void iic4_ack(void);
void iic1_nack(void); /* IIC不发送ACK信号 */
void iic2_nack(void);
void iic3_nack(void);
void iic4_nack(void);
uint8_t iic1_wait_ack(void); /* IIC等待ACK信号 */
uint8_t iic2_wait_ack(void);
uint8_t iic3_wait_ack(void);
uint8_t iic4_wait_ack(void);
void iic1_send_byte(uint8_t txd); /* IIC发送一个字节 */
void iic2_send_byte(uint8_t txd);
void iic3_send_byte(uint8_t txd);
void iic4_send_byte(uint8_t txd);
uint8_t iic1_read_byte(unsigned char ack); /* IIC读取一个字节 */
uint8_t iic2_read_byte(unsigned char ack);
uint8_t iic3_read_byte(unsigned char ack);
uint8_t iic4_read_byte(unsigned char ack);

uint8_t iic1readbytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data);

#endif
