/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "control.h"
#include "angle.h"
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
 
  return ch;
}

#define APP_TX_DATA_SIZE  200
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

//多串口DMA数据格式化打印
void uart_printf(UART_HandleTypeDef *huart,const char *format, ...)
{
    va_list args;
    uint32_t length;
 
    va_start(args, format);
    length = vsnprintf((char *)UserTxBufferFS, APP_TX_DATA_SIZE, (char *)format, args);
    va_end(args);
		if(length>APP_TX_DATA_SIZE)
			length = APP_TX_DATA_SIZE;
    HAL_UART_Transmit_DMA(huart,UserTxBufferFS, length);   //只需要更改这儿就能一直到其他平台

}
/* USER CODE END 0 */

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart6_rx;

/* UART4 init function */
void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}
/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 38400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}
/* USART6 init function */

void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspInit 0 */

  /* USER CODE END UART4_MspInit 0 */
    /* UART4 clock enable */
    __HAL_RCC_UART4_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**UART4 GPIO Configuration
    PC10     ------> UART4_TX
    PC11     ------> UART4_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* UART4 DMA Init */
    /* UART4_RX Init */
    hdma_uart4_rx.Instance = DMA1_Stream2;
    hdma_uart4_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_uart4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart4_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart4_rx.Init.Mode = DMA_NORMAL;
    hdma_uart4_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_uart4_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart4_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_uart4_rx);

    /* UART4 interrupt Init */
    HAL_NVIC_SetPriority(UART4_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);
  /* USER CODE BEGIN UART4_MspInit 1 */

  /* USER CODE END UART4_MspInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA2_Stream7;
    hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 7, 1);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART3 DMA Init */
    /* USART3_RX Init */
    hdma_usart3_rx.Instance = DMA1_Stream1;
    hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart3_rx);

    /* USART3_TX Init */
    hdma_usart3_tx.Instance = DMA1_Stream3;
    hdma_usart3_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_tx.Init.Mode = DMA_NORMAL;
    hdma_usart3_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart3_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart3_tx);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspInit 0 */

  /* USER CODE END USART6_MspInit 0 */
    /* USART6 clock enable */
    __HAL_RCC_USART6_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART6 GPIO Configuration
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USART6 DMA Init */
    /* USART6_RX Init */
    hdma_usart6_rx.Instance = DMA2_Stream1;
    hdma_usart6_rx.Init.Channel = DMA_CHANNEL_5;
    hdma_usart6_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart6_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart6_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart6_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart6_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart6_rx.Init.Mode = DMA_NORMAL;
    hdma_usart6_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_usart6_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart6_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart6_rx);

    /* USART6 interrupt Init */
    HAL_NVIC_SetPriority(USART6_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspInit 1 */

  /* USER CODE END USART6_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspDeInit 0 */

  /* USER CODE END UART4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART4_CLK_DISABLE();

    /**UART4 GPIO Configuration
    PC10     ------> UART4_TX
    PC11     ------> UART4_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10|GPIO_PIN_11);

    /* UART4 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* UART4 interrupt Deinit */
    HAL_NVIC_DisableIRQ(UART4_IRQn);
  /* USER CODE BEGIN UART4_MspDeInit 1 */

  /* USER CODE END UART4_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

    /* USART3 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspDeInit 0 */

  /* USER CODE END USART6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART6_CLK_DISABLE();

    /**USART6 GPIO Configuration
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6|GPIO_PIN_7);

    /* USART6 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART6 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspDeInit 1 */

  /* USER CODE END USART6_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
uint8_t uart3_rx_buff[UART3_RXBUFFERSIZE];//串口3接收缓冲区
uint8_t uart6_rx_buff[UART6_RXBUFFERSIZE];//串口6接收缓冲区
uint8_t uart4_rx_buff[UART4_RXBUFFERSIZE];//串口4接收缓冲区
uint8_t uart1_rx_buff[UART1_RXBUFFERSIZE];//串口1接收缓冲区

int rx_data_correct_cnt = 0;
int rx_data_cnt = 0;
uint8_t remote_data_flash[2];
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	int i=0;
 if(huart == &huart3)
	 {
		 for(;i<(REMOTE_DATA_NUM+1);i++)
		 {
			 if(uart3_rx_buff[i]==0x5A&&uart3_rx_buff[i+6]==0xA5)//帧头帧尾正确
			 {
					switch(uart3_rx_buff[i+1])
					{
						case 0x01:
							my_remote.YG_LEFT_UD = uart3_rx_buff[i+2];
							my_remote.YG_LEFT_LR = uart3_rx_buff[i+3];
							my_remote.YG_RIGHT_UD = uart3_rx_buff[i+4];
							my_remote.YG_RIGHT_LR = uart3_rx_buff[i+5];
							remote_data_flash[0] = 1;
							remote_data_flash[1] = 1;
							break;
						case 0x02:
							my_aircraft.status=uart3_rx_buff[i+2];
							if(my_aircraft.status == 0x01)
							{
								pid_enable(&AttitudeControl.internal_pid.x,1);//开启飞机使能pid
								pid_enable(&AttitudeControl.internal_pid.y,1);//开启飞机使能pid
								pid_enable(&AttitudeControl.internal_pid.z,1);//开启飞机使能pid
								pid_enable(&AttitudeControl.external_pid.x,1);//开启飞机使能pid
								pid_enable(&AttitudeControl.external_pid.y,1);//开启飞机使能pid
								pid_enable(&AttitudeControl.external_pid.z,1);//开启飞机使能pid
							}
							else if(my_aircraft.status == 0x00)
							{
								pid_enable(&AttitudeControl.internal_pid.x,0);//关闭飞机不使能pid
								pid_enable(&AttitudeControl.internal_pid.y,0);//关闭飞机不使能pid
								pid_enable(&AttitudeControl.internal_pid.z,0);//关闭飞机不使能pid
								pid_enable(&AttitudeControl.external_pid.x,0);//关闭飞机不使能pid
								pid_enable(&AttitudeControl.external_pid.y,0);//关闭飞机不使能pid
								pid_enable(&AttitudeControl.external_pid.z,0);//关闭飞机不使能pid								
							}
							break;
						case 0x03:
							switch(uart3_rx_buff[i+2])
							{
								case 0x01:
									AttitudeControl.pitch_compensate -=0.05f;
									if(AttitudeControl.pitch_compensate<-5.0f)
											AttitudeControl.pitch_compensate =-5.0f;
									break;
								case 0x02:
									AttitudeControl.pitch_compensate +=0.05f;
									if(AttitudeControl.pitch_compensate>5.0f)
											AttitudeControl.pitch_compensate =5.0f;
									break;
								case 0x03:
									AttitudeControl.roll_compensate +=0.05f;
									if(AttitudeControl.roll_compensate>5.0f)
											AttitudeControl.roll_compensate =5.0f;
									break;
								case 0x04:
									AttitudeControl.roll_compensate -=0.05f;
									if(AttitudeControl.roll_compensate<-5.0f)
											AttitudeControl.roll_compensate =-5.0f;
									break;							
							}
							break;
						case 0x04:
							switch(uart3_rx_buff[i+2])
							{
								case 0x01:
									if(HeightControl.auto_height_control_isEnable==0)
									{										
										HeightControl.auto_height_control_isEnable = 1; //开启自动定高
										my_aircraft.status |= 0x02;
										HeightControl.base_throttle = aircraft_motor.throttle;
										HeightControl.target_height = TOF.distance_m;//设定定高高度为当前高度
										HeightControl.target_altitude = my_aircraft.Altitude;//设定定高海拔为当前海拔
										//HeightControl.pid.iout = (float)aircraft_throttle;//使i项输出为当前输出，更加平滑
										//HeightControl.pid.err = HeightControl.pid.last_err;//使偏差变化率为0，降低d项影响
										pid_enable(&HeightControl.pid,1);//开启定高使能pid
									}
									break;					
								case 0x00:
									if(HeightControl.auto_height_control_isEnable==1)
									{
										HeightControl.auto_height_control_isEnable = 0; //关闭自动定高
										my_aircraft.status &= 0xFD;
										HeightControl.mode = ALT_HOLD_DISABLED;//高度控制模式为手动
										HeightControl.target_height = 0;
										HeightControl.target_altitude =0;
										HeightControl.base_throttle =0;
										HeightControl.pid.iout = 0;
										HeightControl.pid.err = 0;
										pid_enable(&HeightControl.pid,0);//关闭定高使能pid
									}
									break;										
							}
							break;
					}
					rx_data_correct_cnt++;
					break;
			 }


		 }
		 rx_data_cnt++;
     memset(uart3_rx_buff, 0, UART3_RXBUFFERSIZE);		
   }
}
TOF_TypeDef TOF = {0,0,0,0,25,30,0,0};
void TOF_get_distance(uint8_t *uart_data,uint16_t size)
{
	char str1[9],str2[6],str3[11];
	memcpy(str1,uart_data,9);
	memcpy(str2,&uart_data[17],6);
	memcpy(str3,&uart_data[27],11);
	if(strncmp(str1,"Distance:",9)==0 && strncmp(str2,"Noise:",6)==0 && strncmp(str3,"Confidence:",11)==0 &&uart_data[size-2]==0x0D&&uart_data[size-1]==0x0A)
	{
		TOF.distance_mm=0;
		TOF.noise=0;
		TOF.confidence=0;
		if(uart_data[9]>=0x30&&uart_data[9]<=0x39)
		{
			TOF.distance_mm += (uart_data[9]-0x30) *1000;
		}
		if(uart_data[10]>=0x30&&uart_data[10]<=0x39)
		{
			TOF.distance_mm += (uart_data[10]-0x30) *100;
		}
		if(uart_data[11]>=0x30&&uart_data[11]<=0x39)
		{
			TOF.distance_mm += (uart_data[11]-0x30) *10;
		}		
		if(uart_data[12]>=0x30&&uart_data[12]<=0x39)
		{
			TOF.distance_mm += (uart_data[12]-0x30) ;
		}
		
		if(uart_data[23]>=0x30&&uart_data[23]<=0x39)
		{
			TOF.noise += (uart_data[23]-0x30)*10 ;
		}		
		if(uart_data[24]>=0x30&&uart_data[24]<=0x39)
		{
			TOF.noise += (uart_data[24]-0x30) ;
		}			
		if(size==43)
			TOF.confidence =100;
		else if(size==42)
		{
			if(uart_data[38]>=0x30&&uart_data[38]<=0x39)
			{
				TOF.noise += (uart_data[38]-0x30)*10 ;
			}		
			if(uart_data[39]>=0x30&&uart_data[39]<=0x39)
			{
				TOF.noise += (uart_data[39]-0x30) ;
			}			
		}
		else if(size==41)
		{
			if(uart_data[38]>=0x30&&uart_data[38]<=0x39)
			{
				TOF.noise += (uart_data[38]-0x30) ;
			}					
		}
		TOF.distance_mm = (TOF.distance_mm*cosf(Angle_Data.pitch*DegtoRad)*cosf(Angle_Data.roll*DegtoRad)+TOF.d_center_y_err_mm*sinf(Angle_Data.roll*DegtoRad))+TOF.distance_mm_offset;//TOF高度矫正
		TOF.distance_cm = (float)TOF.distance_mm/10.0f;
		TOF.distance_m = (float)TOF.distance_mm/1000.0f;
	}
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
       /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  UNUSED(Size);
    if (huart->Instance == UART4)
    {
       uint8_t cnt = UART4_RXBUFFERSIZE - __HAL_DMA_GET_COUNTER(&hdma_uart4_rx);
			TOF_get_distance(uart4_rx_buff,cnt)		;	
       memset(uart4_rx_buff, 0, cnt);
    }


}

uint8_t remote_data_buf[AIRCFAFT_DARA_NUM];
void aircraft_data_update()
{
	my_aircraft.Throttle = (aircraft_motor.throttle*100)/MOTOR_MAX_THROTTLE;
	
	my_aircraft.Motor_PWM_duty[0] = (aircraft_motor.duty1*100)/MOTOR_MAX_DUTY;
	my_aircraft.Motor_PWM_duty[1] = (aircraft_motor.duty2*100)/MOTOR_MAX_DUTY;
	my_aircraft.Motor_PWM_duty[2] = (aircraft_motor.duty3*100)/MOTOR_MAX_DUTY;
	my_aircraft.Motor_PWM_duty[3] = (aircraft_motor.duty4*100)/MOTOR_MAX_DUTY;

	my_aircraft.Height =(uint16_t)TOF.distance_cm;
	my_aircraft.ROLL = (int8_t)Angle_Data.roll;
	my_aircraft.PITCH = (int8_t)Angle_Data.pitch;
	my_aircraft.YAW = (int8_t)Angle_Data.yaw;
	
}
void aircraft_data_send()
{
	aircraft_data_update();
	remote_data_buf[0] = 0x5A;//帧头
	remote_data_buf[1] = 0x01;//消息ID
	remote_data_buf[2] = my_aircraft.Throttle;//油门
	remote_data_buf[3] = my_aircraft.Height>>8;//高度
	remote_data_buf[4] = my_aircraft.Height&0xff;//高度
	remote_data_buf[5] = my_aircraft.Motor_PWM_duty[0];//
	remote_data_buf[6] = my_aircraft.Motor_PWM_duty[1];//
	remote_data_buf[7] = my_aircraft.Motor_PWM_duty[2];//
	remote_data_buf[8] = my_aircraft.Motor_PWM_duty[3];//
	remote_data_buf[9] = my_aircraft.ROLL;
	remote_data_buf[10] = my_aircraft.PITCH;
	remote_data_buf[11] = my_aircraft.YAW;
	remote_data_buf[12] = my_aircraft.Battery_Volt;
	remote_data_buf[13] = (uint16_t)my_aircraft.Altitude>>8;
	remote_data_buf[14] = (uint8_t)my_aircraft.Altitude;
	remote_data_buf[15] = (int8_t)my_aircraft.Temperature;
	remote_data_buf[16] = my_aircraft.status;
	remote_data_buf[AIRCFAFT_DARA_NUM-1] = 0xA5;//帧尾
	HAL_UART_Transmit_DMA(&huart3,remote_data_buf,AIRCFAFT_DARA_NUM);//上传飞行器实时数据至遥控器
}

/* USER CODE END 1 */
