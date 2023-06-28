/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "usart_bsp.h"
#include "rc_bsp.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern TIM_HandleTypeDef htim8;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_uart8_rx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern TIM_HandleTypeDef htim13;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
    /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

    /* USER CODE END NonMaskableInt_IRQn 0 */
    /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
    while (1)
    {
    }
    /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
    /* USER CODE BEGIN HardFault_IRQn 0 */

    /* USER CODE END HardFault_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_HardFault_IRQn 0 */
        /* USER CODE END W1_HardFault_IRQn 0 */
    }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
    /* USER CODE BEGIN MemoryManagement_IRQn 0 */

    /* USER CODE END MemoryManagement_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
        /* USER CODE END W1_MemoryManagement_IRQn 0 */
    }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
    /* USER CODE BEGIN BusFault_IRQn 0 */

    /* USER CODE END BusFault_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_BusFault_IRQn 0 */
        /* USER CODE END W1_BusFault_IRQn 0 */
    }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
    /* USER CODE BEGIN UsageFault_IRQn 0 */

    /* USER CODE END UsageFault_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
        /* USER CODE END W1_UsageFault_IRQn 0 */
    }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
    /* USER CODE BEGIN DebugMonitor_IRQn 0 */

    /* USER CODE END DebugMonitor_IRQn 0 */
    /* USER CODE BEGIN DebugMonitor_IRQn 1 */

    /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
    /* USER CODE BEGIN EXTI0_IRQn 0 */

    /* USER CODE END EXTI0_IRQn 0 */
    HAL_GPIO_EXTI_IRQHandler(I2_Pin);
    /* USER CODE BEGIN EXTI0_IRQn 1 */

    /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
    /* USER CODE BEGIN EXTI1_IRQn 0 */

    /* USER CODE END EXTI1_IRQn 0 */
    HAL_GPIO_EXTI_IRQHandler(I1_Pin);
    /* USER CODE BEGIN EXTI1_IRQn 1 */

    /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
    /* USER CODE BEGIN EXTI2_IRQn 0 */

    /* USER CODE END EXTI2_IRQn 0 */
    HAL_GPIO_EXTI_IRQHandler(KEY_Pin);
    /* USER CODE BEGIN EXTI2_IRQn 1 */

    /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
    /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

    /* USER CODE END DMA1_Stream1_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_usart3_rx);
    /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

    /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
    /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

    /* USER CODE END DMA1_Stream3_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_uart7_rx);
    /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

    /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream6 global interrupt.
  */
void DMA1_Stream6_IRQHandler(void)
{
    /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */

    /* USER CODE END DMA1_Stream6_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_uart8_rx);
    /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

    /* USER CODE END DMA1_Stream6_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
    /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

    /* USER CODE END CAN1_RX0_IRQn 0 */
    HAL_CAN_IRQHandler(&hcan1);
    /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

    /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
    /* USER CODE BEGIN USART1_IRQn 0 */
    if(huart1.Instance->SR & UART_FLAG_RXNE)//接收到数�?
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
    else if(USART1->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart1);

        if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* 当前使用的内存缓冲区为内存区0 */

            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart1_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //设定缓冲�?1
            hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;

            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
            }
        }
        else
        {
            /* 当前使用的内存缓冲区为内存区1 */

            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart1_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //设定缓冲�?0
            DMA2_Stream2->CR &= ~(DMA_SxCR_CT);

            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数�?
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
            }
        }
    }
    /* USER CODE END USART1_IRQn 0 */
    HAL_UART_IRQHandler(&huart1);
    /* USER CODE BEGIN USART1_IRQn 1 */

    /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
    /* USER CODE BEGIN USART3_IRQn 0 */
    uint32_t tmp_flag = 0;
    uint32_t temp;
    tmp_flag =__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE); //鑾峰彇IDLE鏍囧織浣?
    if((tmp_flag != RESET))//idle鏍囧織琚疆�??
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart3);//娓呴櫎鏍囧織�??
        temp = huart3.Instance->SR;  //娓呴櫎鐘舵€佸瘎瀛樺櫒SR,璇诲彇SR瀵勫瓨鍣ㄥ彲浠ュ疄鐜版竻闄R瀵勫瓨鍣ㄧ殑鍔熻�?
        temp = huart3.Instance->DR; //璇诲彇鏁版嵁瀵勫瓨鍣ㄤ腑鐨勬暟鎹?
        HAL_UART_DMAStop(&huart3); //
        temp  = hdma_uart7_rx.Instance->NDTR;// 鑾峰彇DMA涓湭浼犺緭鐨勬暟鎹釜鏁帮紝NDTR瀵勫瓨鍣ㄥ垎鏋愯涓嬮潰
        rx_len_usart7 =  BUFFER_SIZE - temp; //鎬昏鏁板噺鍘绘湭浼犺緭鐨勬暟鎹釜鏁帮紝寰�?埌宸茬粡鎺ユ敹鐨勬暟鎹釜鏁?
        recv_end_flag_usart7 = 1;	// 鎺ュ彈�?�屾垚鏍囧織浣嶇�?1
    }
    /* USER CODE END USART3_IRQn 0 */
    HAL_UART_IRQHandler(&huart3);
    /* USER CODE BEGIN USART3_IRQn 1 */
    HAL_UART_Receive_DMA(&huart3,rx_buffer_usart3,BUFFER_SIZE);//閲嶆柊鎵撳紑DMA鎺ユ�?
    if(recv_end_flag_usart3 == 1)
    {
        rx_len_usart3 = 0;
        recv_end_flag_usart3 = 0;
        for(uint8_t i=0; i<rx_len_usart3; i++)
        {
            rx_buffer_usart3[i] = 0;
        }
    }

    Laser_decoding();//婵€鍏夋暟鎹В鐮?
    /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
    /* USER CODE BEGIN EXTI15_10_IRQn 0 */

    /* USER CODE END EXTI15_10_IRQn 0 */
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
    /* USER CODE BEGIN EXTI15_10_IRQn 1 */

    /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles TIM8 update interrupt and TIM13 global interrupt.
  */
void TIM8_UP_TIM13_IRQHandler(void)
{
    /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 0 */

    /* USER CODE END TIM8_UP_TIM13_IRQn 0 */
    HAL_TIM_IRQHandler(&htim8);
    HAL_TIM_IRQHandler(&htim13);
    /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 1 */

    /* USER CODE END TIM8_UP_TIM13_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream1 global interrupt.
  */
void DMA2_Stream1_IRQHandler(void)
{
    /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */

    /* USER CODE END DMA2_Stream1_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_usart6_rx);
    /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

    /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
    /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

    /* USER CODE END DMA2_Stream2_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_usart1_rx);
    /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

    /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles CAN2 RX0 interrupts.
  */
void CAN2_RX0_IRQHandler(void)
{
    /* USER CODE BEGIN CAN2_RX0_IRQn 0 */

    /* USER CODE END CAN2_RX0_IRQn 0 */
    HAL_CAN_IRQHandler(&hcan2);
    /* USER CODE BEGIN CAN2_RX0_IRQn 1 */

    /* USER CODE END CAN2_RX0_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
    /* USER CODE BEGIN USART6_IRQn 0 */
    uint32_t tmp_flag = 0;
    uint32_t temp;
    tmp_flag =__HAL_UART_GET_FLAG(&huart6,UART_FLAG_IDLE); //鑾峰彇IDLE鏍囧織浣?
    if((tmp_flag != RESET))//idle鏍囧織琚疆�??
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart6);//娓呴櫎鏍囧織�??
        temp = huart6.Instance->SR;  //娓呴櫎鐘�??佸瘎瀛樺櫒SR,璇诲彇SR瀵勫瓨鍣ㄥ彲浠ュ疄鐜版竻闄R瀵勫瓨鍣ㄧ殑鍔熻�?
        temp = huart6.Instance->DR; //璇诲彇鏁版嵁瀵勫瓨鍣ㄤ腑鐨勬暟鎹?
        HAL_UART_DMAStop(&huart6); //
        temp  = hdma_usart6_rx.Instance->NDTR;// 鑾峰彇DMA涓湭浼犺緭鐨勬暟鎹釜鏁帮紝NDTR瀵勫瓨鍣ㄥ垎鏋愯涓嬮潰
        rx_len_usart6 =  BUFFER_SIZE - temp; //鎬昏鏁板噺鍘绘湭浼犺緭鐨勬暟鎹釜鏁帮紝寰�?埌宸茬粡鎺ユ敹鐨勬暟鎹釜鏁?
        recv_end_flag_usart6 = 1;	// 鎺ュ彈�?�屾垚鏍囧織浣嶇�?1
    }
    /* USER CODE END USART6_IRQn 0 */
    HAL_UART_IRQHandler(&huart6);
    /* USER CODE BEGIN USART6_IRQn 1 */
    HAL_UART_Receive_DMA(&huart6,rx_buffer_usart6,BUFFER_SIZE);//閲嶆柊鎵撳紑DMA鎺ユ�?
    if(recv_end_flag_usart6 == 1)
    {
        rx_len_usart6 = 0;
        recv_end_flag_usart6 = 0;
        for(uint8_t i=0; i<rx_len_usart6; i++)
        {
            rx_buffer_usart6[i] = 0;
        }
    }
		buletooth(&bt);

    /* USER CODE END USART6_IRQn 1 */
}

/**
  * @brief This function handles UART7 global interrupt.
  */
void UART7_IRQHandler(void)
{
    /* USER CODE BEGIN UART7_IRQn 0 */
    uint32_t tmp_flag = 0;
    uint32_t temp;
    tmp_flag =__HAL_UART_GET_FLAG(&huart7,UART_FLAG_IDLE); //鑾峰彇IDLE鏍囧織浣?
    if((tmp_flag != RESET))//idle鏍囧織琚疆�??
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart7);//娓呴櫎鏍囧織�??
        temp = huart7.Instance->SR;  //娓呴櫎鐘�??佸瘎瀛樺櫒SR,璇诲彇SR瀵勫瓨鍣ㄥ彲浠ュ疄鐜版竻闄R瀵勫瓨鍣ㄧ殑鍔熻�?
        temp = huart7.Instance->DR; //璇诲彇鏁版嵁瀵勫瓨鍣ㄤ腑鐨勬暟鎹?
        HAL_UART_DMAStop(&huart7); //
        temp  = hdma_uart7_rx.Instance->NDTR;// 鑾峰彇DMA涓湭浼犺緭鐨勬暟鎹釜鏁帮紝NDTR瀵勫瓨鍣ㄥ垎鏋愯涓嬮潰
        rx_len_usart7 =  BUFFER_SIZE - temp; //鎬昏鏁板噺鍘绘湭浼犺緭鐨勬暟鎹釜鏁帮紝寰楿埌宸茬粡鎺ユ敹鐨勬暟鎹釜�??
        recv_end_flag_usart7 = 1;	// 鎺ュ彈翹屾垚鏍囧織浣嶇疿1
    }
    /* USER CODE END UART7_IRQn 0 */
    HAL_UART_IRQHandler(&huart7);
    /* USER CODE BEGIN UART7_IRQn 1 */
		HAL_UART_Receive_DMA(&huart7,rx_buffer_usart7,BUFFER_SIZE);//閲嶆柊鎵撳紑DMA鎺ユ敿
    if(recv_end_flag_usart7 == 1)
    {
        rx_len_usart7 = 0;
        recv_end_flag_usart7 = 0;
        for(uint8_t i=0; i<rx_len_usart7; i++)
        {
            rx_buffer_usart7[i] = 0;
        }
    }
    Laser_decoding();
    /* USER CODE END UART7_IRQn 1 */
}

/**
  * @brief This function handles UART8 global interrupt.
  */
void UART8_IRQHandler(void)
{
    /* USER CODE BEGIN UART8_IRQn 0 */
    uint32_t tmp_flag = 0;
    uint32_t temp;
    tmp_flag =__HAL_UART_GET_FLAG(&huart8,UART_FLAG_IDLE); //鑾峰彇IDLE鏍囧織浣?
    if((tmp_flag != RESET))//idle鏍囧織琚疆�??
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart8);//娓呴櫎鏍囧織�??
        temp = huart8.Instance->SR;  //娓呴櫎鐘�??佸瘎瀛樺櫒SR,璇诲彇SR瀵勫瓨鍣ㄥ彲浠ュ疄鐜版竻闄R瀵勫瓨鍣ㄧ殑鍔熻�?
        temp = huart8.Instance->DR; //璇诲彇鏁版嵁瀵勫瓨鍣ㄤ腑鐨勬暟鎹?
        HAL_UART_DMAStop(&huart8); //
        temp  = hdma_uart8_rx.Instance->NDTR;// 鑾峰彇DMA涓湭浼犺緭鐨勬暟鎹釜鏁帮紝NDTR瀵勫瓨鍣ㄥ垎鏋愯涓嬮潰
        rx_len_usart8 =  BUFFER_SIZE - temp; //鎬昏鏁板噺鍘绘湭浼犺緭鐨勬暟鎹釜鏁帮紝寰楿埌宸茬粡鎺ユ敹鐨勬暟鎹釜�??
        recv_end_flag_usart8 = 1;	// 鎺ュ彈翹屾垚鏍囧織浣嶇疿1
    }
    /* USER CODE END UART8_IRQn 0 */
    HAL_UART_IRQHandler(&huart8);
    /* USER CODE BEGIN UART8_IRQn 1 */
    HAL_UART_Receive_DMA(&huart8,rx_buffer_usart8,BUFFER_SIZE);//閲嶆柊鎵撳紑DMA鎺ユ�?
    if(recv_end_flag_usart8 == 1)
    {
        rx_len_usart8 = 0;
        recv_end_flag_usart8 = 0;
        for(uint8_t i=0; i<rx_len_usart8; i++)
        {
            rx_buffer_usart8[i] = 0;
        }
    }
    Laser_decoding();
    //yaw_decoding();//鍋忚埅瑙掓暟鎹В�??
    /* USER CODE END UART8_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
